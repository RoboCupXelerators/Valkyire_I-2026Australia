
from hailo_platform import VDevice, HailoSchedulingAlgorithm
HAILO = 1
# TODO transform to RDK

import threading
import queue
import time
import cv2
import numpy as np

from ReasonData import Settings, logger, Path

APP_DIR = Path(__file__).parent
DATA_DIR = APP_DIR / "ReasonData" / "data"
MODEL_DIR = APP_DIR  / "vision_utils" / "models"

class ArisuIntelligence:
    '''
    with hailo only
    '''
    def __init__(self,GetPos=None):
        self.GetPos = GetPos
        self.cfg = Settings
        self.logger = logger
        self.CamPorts = [
                    Settings.VisionVals.Front,
                    Settings.VisionVals.Right,
                    Settings.VisionVals.Back,
                    Settings.VisionVals.Left,
                    ]
        # self.CamPorts = [8,10,1,5]

        self.Cams = []
        self.Frames = []
        self.Videos = []

        self.StopRecord = 0

        self.HailoParams = VDevice.create_params()
        self.HailoParams.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN

        self.PerspectiveMatrices = []
        self.P2CK = []
        self.P2CHB = []
        self.P2CVB = []

        self.OrangeThreshold = (5, 15, 128, 255, 150, 255)
        self.BallPos = [0,0]

        self.InitCam(self.CamPorts)

        self.ReadCamTF = 0
        self.PreProcessTF = 0
        self.InferTF = 0
        self.PostProcessTF = 0

        self.YOLOQueue = queue.Queue(maxsize=1)

        self.MergedChassisList = []
        self.ChassisQueue = queue.Queue(maxsize=1)
        self.BallQueue = queue.Queue(maxsize=1)

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

        self.ReadCamsThread = threading.Thread(target=self.ReadCams)
        self.ReadCamsThread.daemon = True
        self.ReadCamsThread.start()

        # time.sleep(3) # TODO Beta test for undo this delay

        # self.VideoRecordThread = threading.Thread(target=self.VideoRecord)
        # self.VideoRecordThread.daemon = True
        # self.VideoRecordThread.start()

        self.InitConfiguredModelThread = threading.Thread(target=self.InitConfiguredModel)
        self.InitConfiguredModelThread.daemon = True
        self.InitConfiguredModelThread.start()

        # 报错会阻塞程序 try&except
        time.sleep(2)

        self.ModelPreProcessThread = threading.Thread(target=self.ModelPreProcess)
        self.ModelPreProcessThread.daemon = True
        self.ModelPreProcessThread.start()

        self.ModelInferThread = threading.Thread(target=self.ModelInfer)
        self.ModelInferThread.daemon = True
        self.ModelInferThread.start()

        self.ChassisDetectionThread = threading.Thread(target=self.ChassisDetection)
        self.ChassisDetectionThread.daemon = True
        self.ChassisDetectionThread.start()

        self.BallDetectionThread = threading.Thread(target=self.BallDetection)
        self.BallDetectionThread.daemon = True
        self.BallDetectionThread.start()


    def InitVideo(self):
        Videos = []
        if not self.cfg.VisionVals.Record:
            return
        for i in range(4):
            Time = int(time.time())
            Video = cv2.VideoWriter('./Records/' + str(i) + '/' + str(Time) + '.mp4', cv2.VideoWriter_fourcc(*'avc1'), 30, (640, 480))
            Videos.append(Video)
        self.Videos = Videos
    
    def CloseVideo(self):
        for Video in self.Videos:
            Video.release()
        self.Videos = []
    
    def VideoRecord(self):
        while True:
            if not self.Videos:
                self.InitVideo()
            elif int(time.time()) % 30 == 0:
                self.CloseVideo()
            
            for i in range(4):
                if self.Videos:
                    Frame = self.Frames[i]
                    if Frame is not None:
                        self.Videos[i].write(Frame)
            time.sleep(0.03)
    
    def InitConfiguredModel(self):
        with VDevice(self.HailoParams) as Hat:
            InferModel = Hat.create_infer_model(str(MODEL_DIR)+"/yolov8s.hef")
            InferModel.set_batch_size(4)
            self.InputShape = InferModel.input().shape
            self.OutputShape = InferModel.output().shape
            with InferModel.configure() as ConfiguredInferModel:
                self.ConfiguredInferModel = ConfiguredInferModel
                time.sleep(114514)
    
    def ModelPreProcess(self):
        while(1):
            BindingsList = []
            try:
                for i in range(4):
                    Bindings = self.ConfiguredInferModel.create_bindings()
                    OutputBuffer = np.empty(self.OutputShape, dtype=np.float32)
                    Frame = self.Frames[i]
                    if Frame is not None:
                        Frame = self.Resize(Frame, (640, 640))
                        Frame = cv2.cvtColor(Frame, cv2.COLOR_BGR2RGB)
                        # cv2.imshow(f"Frame {i}", Frame)
                        # Frame = Frame.astype(np.uint8)
                        # Frame = np.ascontiguousarray(Frame, dtype=np.uint8)
                        # print(Frame.shape)
                        Bindings.input().set_buffer(Frame)
                        Bindings.output().set_buffer(OutputBuffer)
                        BindingsList.append(Bindings)
                if self.YOLOQueue.full():
                    continue
                else:
                    self.PreProcessTF = self.PreProcessTF + 1
                    self.YOLOQueue.put(BindingsList)
            except Exception as e:
                time.sleep(0.5)
                self.ModelPreProcess()
            finally:
                time.sleep(0.01)
                
    def ModelInfer(self):
        FrameCount = 0
        LastTime = time.time()
        while(1):
            FrameCount += 1
            CurrentTime = time.time()
            if CurrentTime - LastTime >= 1.0:
                self.logger.debug(f"Hailo Process FPS: {FrameCount}")
                FrameCount = 0
                LastTime = CurrentTime
            BindingsList = self.YOLOQueue.get()
            self.InferTF = self.InferTF + 1
            # 确保在传递给 BindingsList 之前执行此操作
            try:
                self.ConfiguredInferModel.run(BindingsList, 2000)
            except Exception as e:
                self.ModelInfer()
            Outputs = []
            for Bindings in BindingsList:
                OutputBuffer = Bindings.output().get_buffer()
                Outputs.append(OutputBuffer)
            self.ChassisQueue.put(Outputs)
            self.BallQueue.put(Outputs)


    def ChassisDetection(self):
        while True:
            OutputBuffer = self.ChassisQueue.get()
            self.PostProcessTF = self.PostProcessTF + 1
            self.MergedChassisList = []
            ChassisList = []
            for i in range(4):
                List = OutputBuffer[i][3]
                if List.shape[0] > 0:
                    for Chassis in List:
                        if Chassis[4] < 0.5:
                            continue
                        YMin = int(Chassis[0] * 640) - 80
                        XMin = int(Chassis[1] * 640)
                        YMax = int(Chassis[2] * 640) - 80
                        XMax = int(Chassis[3] * 640)
                        BottomY = YMax
                        CenterX = int((XMin + XMax) / 2)
                        X,Y = self.Pixel2CM(CenterX, BottomY, i)
                        # X,Y = CenterX, BottomY
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

                        Width = XMax - XMin
                        Height = YMax - YMin

                        Confidence = Chassis[4]
                        ChassisTuple = (CY, -CX, Width, Height, Confidence)
                        # 20251017 already changed X,Y dimension
                        
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

    def GetChassisPos(self):
        # 20251017 already changed X,Y dimension
        return self.MergedChassisList
    
    def BallDetection(self):
        while True:
            OutputBuffer = self.BallQueue.get()
            Balls = []
            for i in range(4):
                List = OutputBuffer[i][0]
                if List.shape[0] > 0:
                    for Ball in List:
                        if Ball[4] < 0.4:
                            continue
                        YMin = int(Ball[0] * 640) - 80 + 15
                        XMin = int(Ball[1] * 640) + 15
                        YMax = int(Ball[2] * 640) - 80 - 15
                        XMax = int(Ball[3] * 640) - 15
                        BottomY = YMax
                        CenterX = int((XMin + XMax) / 2)
                        X,Y = self.Pixel2CM(CenterX, BottomY, i)
                        X -= 15
                        Y -= 15
                        if i == 0:
                            BX = -X
                            BY = Y
                        elif i == 1:
                            BX = Y
                            BY = X
                        elif i == 2:
                            BX = X
                            BY = Y
                        elif i == 3:
                            BX = -Y
                            BY = -X

                        Width = XMax - XMin
                        Height = YMax - YMin

                        if Ball[0] > 150 or Ball[1] > 150:
                            continue

                        Confidence = Ball[4]
                        BallTuple = (BX, BY, Width, Height, Confidence)
                        
                        Balls.append(BallTuple)
            if Balls:
                Ball = Balls[0]
                BX = Ball[0]
                BY = Ball[1]
                self.BallPos = [int(BX), int(BY)]
                # 20251221 已修改坐标 [tested]
            else:
                self.BallPos = [0, 0]

    def InitCam(self,CamPorts,Width=640, Height=480, AutoExposure=3, Exposure=157, Brightness=0, Contrast=32, Saturation=64):
        for Port in CamPorts:
            Cam = cv2.VideoCapture(Port,cv2.CAP_V4L2)
            Cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            Cam.set(cv2.CAP_PROP_FRAME_WIDTH, Width)
            Cam.set(cv2.CAP_PROP_FRAME_HEIGHT, Height)
            Cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, AutoExposure)
            Cam.set(cv2.CAP_PROP_EXPOSURE, Exposure)
            Cam.set(cv2.CAP_PROP_BRIGHTNESS, Brightness)
            Cam.set(cv2.CAP_PROP_CONTRAST, Contrast)
            Cam.set(cv2.CAP_PROP_SATURATION, Saturation)
            self.Cams.append(Cam)
    
    def ReadCams(self):
        FrameCount = 0
        LastTime = time.time()
        while True:
            Frames = []
            for Cam in self.Cams:
                Frame = Cam.read()[1]
                Frames.append(Frame)
            self.Frames = Frames
            FrameCount += 1
            CurrentTime = time.time()
            if CurrentTime - LastTime >= 1.0:
                if self.cfg.Debug.FullLog:
                    self.logger.debug("Camera FPS: "+str(FrameCount))
                FrameCount = 0
                LastTime = CurrentTime
            self.ReadCamTF = self.ReadCamTF + 1
    
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

    def Resize(Frame, TargetSize=(640, 640)):
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
    
    def FindMaxBlob(self, Blobs):
        MaxSize=0
        for Blob in Blobs:
            if Blob[2]*Blob[3] > MaxSize:
                MaxBlob=Blob
                MaxSize = Blob[2] * Blob[3]
        return MaxBlob

    def FindBlobs(self, Frame, Threshold, Format = 2, ROI = None, MinPixelCount = 5):
        if Format == 1:
            Frame = cv2.cvtColor(Frame,cv2.COLOR_BGR2LAB)

        elif Format == 2:
            Frame = cv2.cvtColor(Frame,cv2.COLOR_BGR2HSV)

        LowerThreshold = Threshold[0], Threshold[2], Threshold[4]
        UpperThreshold = Threshold[1], Threshold[3], Threshold[5]



        Mask = cv2.inRange(Frame, np.array(LowerThreshold), np.array(UpperThreshold))

        # Kernel = np.ones((5, 5), np.uint8)
        # Mask = cv2.erode(Mask, Kernel, iterations=1)
        # Mask = cv2.dilate(Mask, Kernel, iterations=2)

        Conters, _ = cv2.findContours(Mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        Blobs = []
        for Conter in Conters:
            Area = cv2.contourArea(Conter)
            if Area >= MinPixelCount:
                X, Y, W, H = cv2.boundingRect(Conter)
                CX = int(X + W / 2) 
                CY = int(Y + H / 2)
                if ROI is not None and ((CX < ROI[0] or CX > ROI[0] + ROI[2]) or (CY < ROI[1] or CY > ROI[1] + ROI[3])):
                    continue
                Blobs.append((X, Y, W, H, CX, CY))
        return Blobs
    
    def FindBall(self):
        while True:
            Balls = []
            for i in range(4):
                Blobs = []
                Frame = self.Frames[i]
                Blobs = self.FindBlobs(Frame, self.OrangeThreshold)
                if Blobs:
                    MaxBlob = self.FindMaxBlob(Blobs)
                    X = MaxBlob[4]
                    Y = MaxBlob[1] + MaxBlob[3]
                    X, Y = self.Pixel2CM(X, Y, i)
                    Balls.append((X, Y, i))
            
            if Balls:
                X = Balls[0][0]
                Y = Balls[0][1]
                CamIndex = Balls[0][2]
                if CamIndex == 0:
                    BX = X
                    BY = Y
                elif CamIndex == 1:
                    BY = -X
                    BX = Y
                elif CamIndex == 2:
                    BX = -X
                    BY = -Y
                elif CamIndex == 3:
                    BY = X
                    BX = -Y
            else:
                BX = 0
                BY = 0
            self.BallPos = [BY, BX]
            # 20251221 已修改坐标 [tested]
            time.sleep(0.03)
    
    def GetBallPos(self):
        return self.BallPos
    
    # def BinaryObjectDetection(self):
    #     二值化检测对方
    #     Frame = self.Frames[0]
    #     Pos = [self.GetPos()[0], self.GetPos()[1]]
    #     Yaw = self.GetPos()[2]
    #     DistToCorner = int(math.sqrt((Pos[0] - 10) ** 2 + (Pos[1] - 13) ** 2))
    #     VisionAngle = math.degrees(math.atan2(Pos[0] - 10, Pos[1] - 13))
    #     Theta = VisionAngle - Yaw
    #     VisionCornerX = int(DistToCorner * math.sin(math.radians(Theta)))
    #     VisionCornerY = int(DistToCorner * math.cos(math.radians(Theta)))
    #     VisionCornerX, VisionCornerY = self.CM2Pixel(VisionCornerX, VisionCornerY, 0)
    #     print(VisionCornerX, VisionCornerY)


