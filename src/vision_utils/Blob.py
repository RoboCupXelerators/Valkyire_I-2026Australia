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
        MaxBlob = None
        MaxSize=0
        for Blob in Blobs:
            if Blob[2]*Blob[3] > MaxSize:
                MaxBlob=Blob
                MaxSize = Blob[2] * Blob[3]
        return MaxBlob

    @staticmethod
    def RawBBoxTo640(BBox, FrameShape):
        """Map a raw-frame bbox into the 640x640 letterboxed detector space."""
        X, Y, W, H = BBox
        SrcH, SrcW = FrameShape[:2]
        if SrcW <= 0 or SrcH <= 0:
            return None

        Scale = min(640 / SrcW, 640 / SrcH)
        PadX = (640 - SrcW * Scale) / 2
        PadY = (640 - SrcH * Scale) / 2

        X1 = X * Scale + PadX
        Y1 = Y * Scale + PadY
        X2 = (X + W) * Scale + PadX
        Y2 = (Y + H) * Scale + PadY

        X1 = int(max(0, min(639, round(X1))))
        Y1 = int(max(0, min(639, round(Y1))))
        X2 = int(max(0, min(639, round(X2))))
        Y2 = int(max(0, min(639, round(Y2))))
        if X2 <= X1 or Y2 <= Y1:
            return None
        return X1, Y1, X2, Y2

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
        while not getattr(self, "_stop_event", threading.Event()).is_set():
            Balls = []
            BallBBoxes = [None, None, None, None]
            BestBBox = None
            BestArea = 0
            for i in range(4):
                Blobs = []
                Frame = self.Frames[i]
                if Frame is None:
                    continue
                Blobs = self.FindBlobs(Frame, self.OrangeThreshold)
                if Blobs:
                    MaxBlob = self.FindMaxBlob(Blobs)
                    if MaxBlob is None:
                        continue
                    X = MaxBlob[4]
                    Y = MaxBlob[1] + MaxBlob[3]
                    X, Y = self.Pixel2CM(X, Y, i)
                    Balls.append((X, Y, i))

                    Box640 = self.RawBBoxTo640(MaxBlob[:4], Frame.shape)
                    if Box640 is not None:
                        X1, Y1, X2, Y2 = Box640
                        Area = MaxBlob[2] * MaxBlob[3]
                        BBox = (i, X1, Y1, X2, Y2, 1.0)
                        BallBBoxes[i] = BBox
                        if Area > BestArea:
                            BestArea = Area
                            BestBBox = BBox
            
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
            self._ball_bboxes = BallBBoxes
            self._ball_bboxes_ts = time.time()
            if BestBBox is not None:
                self._ball_bbox = BestBBox
                self._ball_bbox_ts = time.time()
            try:
                self._update_bbox_distance_caches()
            except Exception:
                pass
            # 20251118 已修改坐标 x forward, y left
            time.sleep(0.03)
