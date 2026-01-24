import threading
import numpy as np
import cv2

from ReasonData import Settings

from vision_utils import Detect_Method, DATA_DIR

class UnitedVision(Detect_Method):
    def __init__(self):

        super().__init__()

        self.PerspectiveMatrices = []
        self.P2CK = []
        self.P2CHB = []
        self.P2CVB = []

        for i in range(4):
            NumpyData = np.load(str(DATA_DIR)+"/Calibration/CalibrationData"+ str(self.CamPorts[i]) +".npz")
            PerspectiveMatrix = NumpyData['matrix']
            self.PerspectiveMatrices.append(PerspectiveMatrix)
            P2CK = NumpyData['p2c'][0]
            self.P2CK.append(P2CK)
            P2CHB = NumpyData['p2c'][1]
            self.P2CHB.append(P2CHB)
            P2CVB = NumpyData['p2c'][2]
            self.P2CVB.append(P2CVB)


        self.ChassisDetectionThread = threading.Thread(target=self.ChassisDetection)
        self.ChassisDetectionThread.daemon = True
        self.ChassisDetectionThread.start()

        self.BallDetectionThread = threading.Thread(target=self.BallDetection)
        self.BallDetectionThread.daemon = True
        self.BallDetectionThread.start()

    def ApplyPerspectiveTransform(self,X, Y, Matrix):
        # 应用透视矩阵
        Point = np.array([X, Y, 1], dtype=np.float64)
        Transformed = Matrix @ Point
        Transformed /= Transformed[2]
        return int(Transformed[1]), int(Transformed[0])

    def Pixel2CM(self,X,Y,CamIndex):
        P2CK = self.P2CK[CamIndex]
        P2CHB = self.P2CHB[CamIndex]
        P2CVB = self.P2CVB[CamIndex]
        X, Y = self.ApplyPerspectiveTransform(X, Y, self.PerspectiveMatrices[CamIndex])

        X = int(self.P2CK[CamIndex] * X + self.P2CHB[CamIndex])
        Y = int(-self.P2CK[CamIndex] * Y + self.P2CVB[CamIndex])

        return X, Y
 
    def CM2Pixel(self, X, Y, CamIndex):
        P2CK = self.P2CK[CamIndex]
        P2CHB = self.P2CHB[CamIndex]
        P2CVB = self.P2CVB[CamIndex]

        X = (X - P2CHB) / P2CK
        Y = -(Y - P2CVB) / P2CK

        X, Y = self.ApplyPerspectiveTransform(X, Y, np.linalg.inv(self.PerspectiveMatrices[CamIndex]))

        return int(X), int(Y)

    def Resize(self,Frame, TargetSize=(640, 640)):
        Height, Width = Frame.shape[:2]
        TargetHeight, TargetWidth = TargetSize
        
        Scale = min(TargetWidth/Width, TargetHeight/Height)
        
        NewWidth = int(Width * Scale)
        NewHeight = int(Height * Scale)
        
        ResizedImage = cv2.resize(Frame, (NewWidth, NewHeight))
        
        PaddedImage = np.zeros((TargetHeight, TargetWidth, 3), dtype=np.uint8)
        
        YOffset = (TargetHeight - NewHeight) // 2
        XOffset = (TargetWidth - NewWidth) // 2
        
        PaddedImage[YOffset:YOffset+NewHeight, XOffset:XOffset+NewWidth] = ResizedImage
        
        return PaddedImage
    

    def ChassisDetection(self):
        while True:
            OutputBuffer = self.ChassisQueue.get()
            # print(OutputBuffer)
            self.PostProcessTF = self.PostProcessTF + 1
            self.MergedChassisList = []
            ChassisList = []
            # Process ChassisList
            for i in range(4):
                try:
                    CenterX, BottomY, ChassisWidth, ChassisHeight, Confidence = OutputBuffer[i]
                    CameraIndex = i
                    X,Y = self.Pixel2CM(CenterX, BottomY, CameraIndex)
                    if i == 0:
                        CX = -X
                        CY = Y
                    elif i == 1:
                        CX = Y
                        CY = X
                    elif i == 2:
                        CX = X
                        CY = Y
                    elif i == 3:
                        CX = -Y
                        CY = -X

                    ChassisTuple = (CX, CY, ChassisWidth, ChassisHeight, Confidence)
                    # 20251118 already changed X forward,Y left dimension
                    
                    self.MergedChassisList.append(ChassisTuple)
                            # ChassisList.append(ChassisTuple)
                # if ChassisList:
                #     ChassisList = sorted(ChassisList, key=lambda x: x[0], reverse=False)
                #     LastMerged = False
                #     for i in range(len(ChassisList)):
                #         if not i == len(ChassisList) - 1:
                #             ChassisX = ChassisList[i][0]
                #             ChassisY = ChassisList[i][1]
                #             NextChassisX = ChassisList[i+1][0]
                #             NextChassisY = ChassisList[i+1][1]

                #         if LastMerged:
                #             Chassis = ChassisList[len(ChassisList) - 1]
                #             self.MergedChassisList.append(Chassis)

                #         elif abs(ChassisX - NextChassisX) < 20 and abs(ChassisY - NextChassisY) < 20:
                #             CX = (ChassisX + NextChassisX) / 2
                #             CY = (ChassisY + NextChassisY) / 2
                #             CW = (ChassisList[i][2] + ChassisList[i+1][2]) / 2
                #             CH = (ChassisList[i][3] + ChassisList[i+1][2]) / 2
                #             Confidence = (ChassisList[i][4] + ChassisList[i+1][4]) / 2
                #             Chassis = (CX, CY, CW, CH, Confidence)
                #             self.MergedChassisList.append(Chassis)
                #             if i == len(ChassisList) - 2:
                #                 LastMerged = True
                #         else:
                #             Chassis = ChassisList[i]
                #             self.MergedChassisList.append(Chassis)
                        

                # print(self.MergedChassisList)
                except:
                    continue

    def GetChassisPos(self):
        # 20251118 already changed X,Y dimension
        return self.MergedChassisList
    
    def BallDetection(self):
        while True:
            OutputBuffer = self.BallQueue.get()
            Balls = []
            for i in range(4):
                try:
                    CameraIndex, CenterX, BottomY, BallWidth, BallHeight, Confidence = OutputBuffer[i]
                    X,Y = self.Pixel2CM(CenterX, BottomY, CameraIndex)
                    X -= 15
                    Y -= 15
                    # X = int(1.50117*X-5.10171)
                    if CameraIndex == 0:
                        BX = -X
                        BY = Y
                    elif CameraIndex == 1:
                        BX = Y
                        BY = X
                    elif CameraIndex == 2:
                        BX = X
                        BY = Y
                    elif CameraIndex == 3:
                        BX = -Y
                        BY = -X
                    BallTuple = (BX, BY, BallWidth, BallHeight, Confidence)
                    Balls.append(BallTuple)
                except:
                    continue
            if Balls:
                Ball = Balls[0]
                BX = Ball[0]
                BY = Ball[1]
                self.BallPos = [BX, BY]
            else:
                self.BallPos = [0, 0]       

    def GetBallPos(self):
        return self.BallPos
    