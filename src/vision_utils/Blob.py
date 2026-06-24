from .PreProcess import VisionPreUntil, threading, time, cv2, np

class BlobBased(VisionPreUntil):
    '''
    without hailo
    '''
    def __init__(self):

        super().__init__()
        self.OrangeThreshold = (5, 15, 128, 255, 150, 255)
        self.BlobMinContourArea = float(
            self._cfg_value("BlobMinBallArea", max(80, self._cfg_value("MinBallBBoxArea", 40)))
        )
        self.BlobMinBBoxArea = float(self._cfg_value("BlobMinBallBBoxArea", self.BlobMinContourArea))
        self.BlobMinFillRatio = float(self._cfg_value("BlobMinBallFillRatio", 0.35))
        self.BlobMaxAspectRatio = float(self._cfg_value("BlobMaxBallAspectRatio", 1.8))
        self.BlobEdgeMarginPx = int(self._cfg_value("BlobEdgeMarginPx", 4))
        self.BlobMinForwardCM = float(self._cfg_value("BlobMinForwardCM", -5.0))
        self.BlobMaxAbsYCM = float(self._cfg_value("BlobMaxAbsYCM", 160.0))
        self.BlobMaxDistanceCM = float(self._cfg_value("BlobMaxDistanceCM", 260.0))

        self.FindBallThread = threading.Thread(target=self.FindBall)
        self.FindBallThread.daemon = True
        self.FindBallThread.start()
        # 色块识别 设计逻辑为无HAILO时启动

    def _cfg_value(self, Name, Default):
        for SectionName in ("VisionVals", "Vision"):
            Section = getattr(self.cfg, SectionName, None)
            if Section is not None and hasattr(Section, Name):
                return getattr(Section, Name)
        return Default

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
                Blobs.append((X, Y, W, H, CX, CY, Area))
        return Blobs

    def _camera_direction_for_blob(self, CamIndex):
        if hasattr(self, "_camera_direction"):
            try:
                return self._camera_direction(CamIndex)
            except Exception:
                pass
        return ("back", "left", "front", "right")[int(CamIndex) % 4]

    def _to_robot_frame_for_blob(self, X, Y, CamIndex):
        if hasattr(self, "_to_robot_frame"):
            try:
                return self._to_robot_frame(X, Y, CamIndex)
            except Exception:
                pass

        if CamIndex == 0:
            return -Y, X
        if CamIndex == 1:
            return X, Y
        if CamIndex == 2:
            return Y, -X
        if CamIndex == 3:
            return -X, -Y
        return X, Y

    def _blob_passes_shape_filter(self, Blob, FrameShape):
        X, Y, W, H = Blob[:4]
        Area = float(Blob[6]) if len(Blob) > 6 else float(W * H)
        BBoxArea = float(max(1, W * H))
        if Area < self.BlobMinContourArea or BBoxArea < self.BlobMinBBoxArea:
            return False

        Aspect = max(W, H) / max(1, min(W, H))
        if Aspect > self.BlobMaxAspectRatio:
            return False

        FillRatio = Area / BBoxArea
        if FillRatio < self.BlobMinFillRatio:
            return False

        FrameH, FrameW = FrameShape[:2]
        Edge = self.BlobEdgeMarginPx
        if X <= Edge or Y <= Edge or (X + W) >= (FrameW - Edge):
            return False

        return True

    def _build_ball_candidate(self, Blob, Frame, CamIndex):
        if not self._blob_passes_shape_filter(Blob, Frame.shape):
            return None

        PxX = Blob[4]
        BottomY = Blob[1] + Blob[3]
        PlaneX, PlaneY = self.Pixel2CM(PxX, BottomY, CamIndex)
        RobotX, RobotY = self._to_robot_frame_for_blob(PlaneX, PlaneY, CamIndex)
        if RobotX < self.BlobMinForwardCM:
            return None
        if abs(RobotY) > self.BlobMaxAbsYCM:
            return None
        if float(np.hypot(RobotX, RobotY)) > self.BlobMaxDistanceCM:
            return None

        Box640 = self.RawBBoxTo640(Blob[:4], Frame.shape)
        if Box640 is None:
            return None

        X1, Y1, X2, Y2 = Box640
        Area = float(Blob[6]) if len(Blob) > 6 else float(Blob[2] * Blob[3])
        BBox = (CamIndex, X1, Y1, X2, Y2, 1.0)
        Direction = self._camera_direction_for_blob(CamIndex)
        Score = (
            0 if Direction == "front" else 1,
            round(abs(RobotY), 2),
            round(float(np.hypot(RobotX, RobotY)), 2),
            -Area,
        )
        return Score, RobotX, RobotY, CamIndex, BBox
    
    def FindBall(self):
        while not getattr(self, "_stop_event", threading.Event()).is_set():
            Candidates = []
            BallBBoxes = [None, None, None, None]
            for i in range(4):
                Blobs = []
                Frame = self.Frames[i]
                if Frame is None:
                    continue
                Blobs = self.FindBlobs(Frame, self.OrangeThreshold, MinPixelCount=self.BlobMinContourArea)
                for Blob in Blobs:
                    Candidate = self._build_ball_candidate(Blob, Frame, i)
                    if Candidate is None:
                        continue
                    Candidates.append(Candidate)
                    _, _, _, _, BBox = Candidate
                    Existing = BallBBoxes[i]
                    BBoxArea = (BBox[4] - BBox[2]) * (BBox[3] - BBox[1])
                    ExistingArea = 0 if Existing is None else (Existing[4] - Existing[2]) * (Existing[3] - Existing[1])
                    if BBoxArea > ExistingArea:
                        BallBBoxes[i] = BBox
            
            if Candidates:
                Candidates.sort(key=lambda Candidate: Candidate[0])
                _, RobotX, RobotY, _, BestBBox = Candidates[0]
                self.BallPos = [round(RobotX, 2), round(RobotY, 2)]
            else:
                BestBBox = None
                self.BallPos = [0, 0]
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
