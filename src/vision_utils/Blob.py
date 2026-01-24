from .PreProcess import VisionPreUntil, threading, time, cv2, np

class BlobBased(VisionPreUntil):
    '''
    without hailo
    '''
    def __init__(self):

        super().__init__()
        self.OrangeThreshold = (5, 15, 128, 255, 150, 255)

        self.FindBallThread = threading.Thread(target=self.FindBall)
        self.FindBallThread.daemon = True
        self.FindBallThread.start()
        # 色块识别 设计逻辑为无HAILO时启动

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
            self.BallPos = [-BY, BX]
            # 20251118 已修改坐标 x forward, y left
            time.sleep(0.03)
