import threading
import numpy as np
import cv2
import time

# Import order is made robust so that this module can be executed either:
# - from an installed package layout (ReasonData/vision_utils are top-level)
# - directly from this repo (modules live under ./src)
try:
    from ReasonData import Settings  # type: ignore
except Exception:  # pragma: no cover
    from src.ReasonData import Settings  # type: ignore

try:
    from vision_utils import Detect_Method, DATA_DIR  # type: ignore
except Exception:  # pragma: no cover
    from src.vision_utils import Detect_Method, DATA_DIR  # type: ignore


def _norm_rel_folder(folder: str | None, default: str) -> str:
    """Normalize a folder configured like '/Calibration/' to a relative name."""
    s = (folder or default).strip()
    if not s or set(s) == {"/"}:
        s = default
    return s.strip("/") or default.strip("/")

class UnitedVision(Detect_Method):
    def __init__(self):

        super().__init__()

        # ---- Runtime caches/params (must be set before starting threads) ----
        # Ball bbox (xyxy in 640x640 resized space from detector)
        self._ball_bbox = None  # (cam, x1,y1,x2,y2,conf)
        self._ball_bbox_ts = 0.0
        self._ball_bboxes = None  # list[ (cam,x1,y1,x2,y2,conf) | None ] length=4
        self._ball_bboxes_ts = 0.0

        # Chassis bbox (xyxy in 640x640 resized space from detector)
        self._chassis_bbox = None  # (cam, x1,y1,x2,y2,conf)
        self._chassis_bbox_ts = 0.0
        self._chassis_bboxes = None  # list len=4
        self._chassis_bboxes_ts = 0.0

        # Goal bbox (xyxy in 640x640 resized space from detector)
        # Store both the best overall bbox and per-camera top1 bbox.
        self._goal_bbox = None  # (cam, cls, x1,y1,x2,y2,conf)
        self._goal_bbox_ts = 0.0
        self._goal_bboxes = None  # list len=4 of (cam, cls, x1,y1,x2,y2,conf) | None
        self._goal_bboxes_ts = 0.0

        # Ball position timestamp
        self._ball_pos_ts = 0.0
        # Hold last seen ball for a short time to avoid flickering to [0,0]
        # when the detector occasionally misses 1-2 frames.
        self.ball_hold_seconds = 0.25

        # BlobBased backend does not provide chassis/ball queues.
        # In that mode, BlobBased updates self.BallPos directly in its own thread.
        self._queue_backend = hasattr(self, "BallQueue") and hasattr(self, "ChassisQueue")

        # Calibration for Pixel2CM/CM2Pixel. BlobBased already loads calibration
        # via VisionPreUntil, but AI/TV pipelines historically relied on this class.
        self.PerspectiveMatrices = []
        self.P2CK = []
        self.P2CHB = []
        self.P2CVB = []

        # Use config folder name if provided; strip leading '/' to avoid
        # accidentally turning it into an absolute path.
        calib_sub = _norm_rel_folder(getattr(Settings.VisionVals, "CalibrationFolder", None), "Calibration")
        calib_dir = None
        try:
            if DATA_DIR is not None:
                from pathlib import Path

                calib_dir = Path(str(DATA_DIR)) / calib_sub
        except Exception:
            calib_dir = None

        for i in range(4):
            # Backward compatible path build.
            if calib_dir is not None:
                npz = str(calib_dir / f"CalibrationData{self.CamPorts[i]}.npz")
            else:
                npz = str(DATA_DIR) + "/" + calib_sub + "/CalibrationData" + str(self.CamPorts[i]) + ".npz"
            try:
                NumpyData = np.load(npz)
                PerspectiveMatrix = NumpyData['matrix']
                self.PerspectiveMatrices.append(PerspectiveMatrix)
                self.P2CK.append(float(NumpyData['p2c'][0]))
                self.P2CHB.append(float(NumpyData['p2c'][1]))
                self.P2CVB.append(float(NumpyData['p2c'][2]))
            except Exception as e:
                # Don't crash vision server if calibration is missing.
                # Pixel2CM will fall back (or behave as identity).
                self.logger.warning(f"UnitedVision: calibration load failed: {npz}: {e}")
                self.PerspectiveMatrices.append(np.eye(3, dtype=np.float64))
                self.P2CK.append(1.0)
                self.P2CHB.append(0.0)
                self.P2CVB.append(0.0)


        # For AI/TV backends, run queue consumers to compute robot-frame positions.
        # For BlobBased, its own FindBall thread updates self.BallPos directly.
        self.MergedChassisList = []
        self.MergedGoalList = []  # list of all goal detections
        self.BlueGoalPos = None    # best blue goal detection in robot frame: [x, y, conf]
        self.YellowGoalPos = None   # best yellow goal detection in robot frame: [x, y, conf]
        if self._queue_backend:
            self.ChassisDetectionThread = threading.Thread(target=self.ChassisDetection)
            self.ChassisDetectionThread.daemon = True
            self.ChassisDetectionThread.start()

            self.BallDetectionThread = threading.Thread(target=self.BallDetection)
            self.BallDetectionThread.daemon = True
            self.BallDetectionThread.start()

        # Optional: goals (BlueGoal/YellowGoal) available in AI backends.
        self._goal_backend = hasattr(self, "GoalQueue")
        if self._goal_backend:
            self.GoalDetectionThread = threading.Thread(target=self.GoalDetection)
            self.GoalDetectionThread.daemon = True
            self.GoalDetectionThread.start()

    def close(self, join_timeout: float = 1.0):
        """Stop all background threads and release resources."""
        try:
            # stop detector/camera threads (from base class)
            super().close(join_timeout=join_timeout)
        except Exception:
            pass

        # join our postprocess threads (they should exit once stop_event is set)
        for t in (
            getattr(self, "ChassisDetectionThread", None),
            getattr(self, "BallDetectionThread", None),
            getattr(self, "GoalDetectionThread", None),
        ):
            try:
                if t is not None and t.is_alive():
                    t.join(timeout=join_timeout)
            except Exception:
                pass

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

        X = round(self.P2CK[CamIndex] * X + self.P2CHB[CamIndex],2)
        Y = round(-self.P2CK[CamIndex] * Y + self.P2CVB[CamIndex],2)

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
        while not getattr(self, "_stop_event", threading.Event()).is_set():
            try:
                OutputBuffer = self.ChassisQueue.get(timeout=0.1)
            except Exception:
                continue
            # print(OutputBuffer)
            self.PostProcessTF = self.PostProcessTF + 1
            self.MergedChassisList = []
            ChassisList = []
            # Process ChassisList
            for i in range(4):
                try:
                    CameraIndex, CenterX, BottomY, ChassisWidth, ChassisHeight, Confidence = OutputBuffer[i]
                    # Ignore padded/dummy outputs
                    if Confidence is None or Confidence <= 0:
                        continue
                    X,Y = self.Pixel2CM(CenterX, BottomY, CameraIndex)
                    if CameraIndex == 0:
                        CX = -X
                        CY = Y
                    elif CameraIndex == 1:
                        CX = Y
                        CY = X
                    elif CameraIndex == 2:
                        CX = X
                        CY = Y
                    elif CameraIndex == 3:
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
            time.sleep(0.01)

    def GetChassisPos(self):
        # 20251118 already changed X,Y dimension
        return self.MergedChassisList

    def GoalDetection(self):
        """Consume GoalQueue and compute goal positions in robot frame.

        GoalQueue item formats supported:
        - TunaIntelligence: list length=4, each entry: [cam, cls, cx, bottom_y, w, h, conf]
        - ArisuIntelligence: list variable length, each entry: [cam, cls, cx, bottom_y, w, h, conf]
        """
        q = getattr(self, "GoalQueue", None)
        if q is None:
            return
        while not getattr(self, "_stop_event", threading.Event()).is_set():
            try:
                out = q.get(timeout=0.1)
            except Exception:
                continue

            goals = []
            blue_goal = None
            yellow_goal = None
            goal_bboxes = [None, None, None, None]
            best_goal = None
            # out is list of detections
            for item in (out or []):
                try:
                    if item is None:
                        continue
                    CameraIndex, cls_id, CenterX, BottomY, W, H, Confidence = item
                    if Confidence is None or Confidence <= 0:
                        continue
                    if cls_id is None or int(cls_id) < 0:
                        continue
                    cam_i = int(CameraIndex)
                    cls_i = int(cls_id)

                    # Cache bbox for dashboard overlay.
                    x1 = int(round(float(CenterX) - float(W) / 2.0))
                    y1 = int(round(float(BottomY) - float(H)))
                    x2 = int(round(float(CenterX) + float(W) / 2.0))
                    y2 = int(round(float(BottomY)))
                    bbox = (cam_i, cls_i, x1, y1, x2, y2, float(Confidence))
                    # Keep the best bbox per camera.
                    if goal_bboxes[cam_i] is None or bbox[6] > goal_bboxes[cam_i][6]:
                        goal_bboxes[cam_i] = bbox
                    if best_goal is None or bbox[6] > best_goal[6]:
                        best_goal = bbox

                    X, Y = self.Pixel2CM(CenterX, BottomY, int(CameraIndex))

                    # Match BallDetection axis conventions (x forward, y left)
                    if cam_i == 0:
                        GX = -X
                        GY = Y
                    elif cam_i == 1:
                        GX = Y
                        GY = X
                    elif cam_i == 2:
                        GX = X
                        GY = -Y
                    elif cam_i == 3:
                        GX = -Y
                        GY = -X
                    else:
                        GX = X
                        GY = Y

                    goal_item = [round(GX, 2), round(GY, 2), float(Confidence)]
                    goals.append(goal_item)
                    if int(cls_id) == 1:
                        if blue_goal is None or goal_item[2] > blue_goal[2]:
                            blue_goal = goal_item
                    elif int(cls_id) == 2:
                        if yellow_goal is None or goal_item[2] > yellow_goal[2]:
                            yellow_goal = goal_item
                except Exception:
                    continue

            self.MergedGoalList = goals
            self.BlueGoalPos = blue_goal
            self.YellowGoalPos = yellow_goal
            self._goal_bboxes = goal_bboxes
            self._goal_bboxes_ts = time.time()
            if best_goal is not None:
                self._goal_bbox = best_goal
                self._goal_bbox_ts = time.time()
            time.sleep(0.01)

    def GetGoalsPos(self):
        """Return last blue/yellow goal positions as a list.

        Returns:
            list: [blue_goal, yellow_goal]
                  where each item is [x, y, conf] or None
        """
        return [
            getattr(self, "BlueGoalPos", None),
            getattr(self, "YellowGoalPos", None),
        ]

    def GetBlueGoalPos(self):
        return getattr(self, "BlueGoalPos", None)

    def GetYellowGoalPos(self):
        return getattr(self, "YellowGoalPos", None)

    def GetGoalBBoxes(self, hold_seconds: float | None = None):
        """Return last per-camera goal bboxes.

        Returns:
            list of length 4: each is None or (cam, cls, x1, y1, x2, y2, conf)
        """
        hs = self.ball_hold_seconds if hold_seconds is None else float(hold_seconds)
        if self._goal_bboxes is None:
            return [None, None, None, None]
        if (time.time() - self._goal_bboxes_ts) > hs:
            return [None, None, None, None]
        return self._goal_bboxes

    def GetGoalBBox(self, hold_seconds: float | None = None):
        """Return last best goal bbox (cam, cls, x1, y1, x2, y2, conf)."""
        hs = self.ball_hold_seconds if hold_seconds is None else float(hold_seconds)
        if self._goal_bbox is None:
            return None
        if (time.time() - self._goal_bbox_ts) > hs:
            return None
        return self._goal_bbox
    
    def BallDetection(self):
        while not getattr(self, "_stop_event", threading.Event()).is_set():
            try:
                OutputBuffer = self.BallQueue.get(timeout=0.1)
            except Exception:
                continue

            # Update bbox cache if backend provides it (TunaIntelligence)
            self.BallBBoxUpdate()

            Balls = []
            for i in range(4):
                try:
                    CameraIndex, CenterX, BottomY, BallWidth, BallHeight, Confidence = OutputBuffer[i]
                    # Ignore padded/dummy outputs
                    if Confidence is None or Confidence <= 0:
                        continue
                    X,Y = self.Pixel2CM(CenterX, BottomY, CameraIndex)
                    X -= 12
                    Y -= 12
                    # X = int(1.50117*X-5.10171)
                    if CameraIndex == 0:
                        BX = -X
                        BY = Y
                    elif CameraIndex == 1:
                        BX = Y
                        BY = X
                    elif CameraIndex == 2:
                        BX = X
                        BY = -Y
                    elif CameraIndex == 3:
                        BX = -Y
                        BY = -X
                    BallTuple = (round(BX,2), round(BY,2), BallWidth, BallHeight, Confidence)
                    Balls.append(BallTuple)
                except:
                    continue
            if Balls:
                # pick the most confident one
                Balls = sorted(Balls, key=lambda x: x[4], reverse=True)
                Ball = Balls[0]
                BX = Ball[0]
                BY = Ball[1]
                self.BallPos = [BX, BY]
                self._ball_pos_ts = time.time()
            else:
                # hold last position briefly to reduce flicker
                if (time.time() - getattr(self, "_ball_pos_ts", 0.0)) > self.ball_hold_seconds:
                    self.BallPos = [0, 0]
            time.sleep(0.01)

    def GetBallPos(self):
        return self.BallPos

    def BallBBoxUpdate(self):
        """Read TunaIntelligence.BallBBoxQueue if available and keep latest bbox."""
        q = getattr(self, "BallBBoxQueue", None)
        if q is None:
            return
        try:
            out = q.get_nowait()
        except Exception:
            return

        # store per-cam
        bboxes = [None, None, None, None]
        for i in range(4):
            try:
                cam, x1, y1, x2, y2, conf = out[i]
                if conf is None or conf <= 0:
                    continue
                bboxes[int(cam)] = (int(cam), int(x1), int(y1), int(x2), int(y2), float(conf))
            except Exception:
                continue
        self._ball_bboxes = bboxes
        self._ball_bboxes_ts = time.time()

        # pick best conf bbox among 4 cams
        best = None
        for i in range(4):
            try:
                cam, x1, y1, x2, y2, conf = out[i]
                if conf is None or conf <= 0:
                    continue
                cand = (int(cam), int(x1), int(y1), int(x2), int(y2), float(conf))
                if best is None or cand[5] > best[5]:
                    best = cand
            except Exception:
                continue
        if best is not None:
            self._ball_bbox = best
            self._ball_bbox_ts = time.time()

    def ChassisBBoxUpdate(self):
        """Read TunaIntelligence.ChassisBBoxQueue if available and keep latest bbox."""
        q = getattr(self, "ChassisBBoxQueue", None)
        if q is None:
            return
        try:
            out = q.get_nowait()
        except Exception:
            return

        bboxes = [None, None, None, None]
        for i in range(4):
            try:
                cam, x1, y1, x2, y2, conf = out[i]
                if conf is None or conf <= 0:
                    continue
                bboxes[int(cam)] = (int(cam), int(x1), int(y1), int(x2), int(y2), float(conf))
            except Exception:
                continue
        self._chassis_bboxes = bboxes
        self._chassis_bboxes_ts = time.time()

        best = None
        for i in range(4):
            try:
                cam, x1, y1, x2, y2, conf = out[i]
                if conf is None or conf <= 0:
                    continue
                cand = (int(cam), int(x1), int(y1), int(x2), int(y2), float(conf))
                if best is None or cand[5] > best[5]:
                    best = cand
            except Exception:
                continue
        if best is not None:
            self._chassis_bbox = best
            self._chassis_bbox_ts = time.time()

    def GetChassisBBox(self, hold_seconds: float | None = None):
        """Return last chassis bbox (cam,x1,y1,x2,y2,conf)."""
        self.ChassisBBoxUpdate()
        hs = self.ball_hold_seconds if hold_seconds is None else float(hold_seconds)
        if self._chassis_bbox is None:
            return None
        if (time.time() - self._chassis_bbox_ts) > hs:
            return None
        return self._chassis_bbox

    def GetChassisBBoxes(self, hold_seconds: float | None = None):
        """Return last per-camera chassis bbox list."""
        self.ChassisBBoxUpdate()
        hs = self.ball_hold_seconds if hold_seconds is None else float(hold_seconds)
        if self._chassis_bboxes is None:
            return [None, None, None, None]
        if (time.time() - self._chassis_bboxes_ts) > hs:
            return [None, None, None, None]
        return self._chassis_bboxes

    def GetBallBBox(self, hold_seconds: float | None = None):
        """Return last ball bbox (cam,x1,y1,x2,y2,conf).

        Note: bbox is in 640x640 resized space (same as detector input).
        """
        self.BallBBoxUpdate()
        hs = self.ball_hold_seconds if hold_seconds is None else float(hold_seconds)
        if self._ball_bbox is None:
            return None
        if (time.time() - self._ball_bbox_ts) > hs:
            return None
        return self._ball_bbox

    def GetBallBBoxes(self, hold_seconds: float | None = None):
        """Return last per-camera ball bbox list.

        Returns:
            list of length 4: each is None or (cam,x1,y1,x2,y2,conf)
        """
        self.BallBBoxUpdate()
        hs = self.ball_hold_seconds if hold_seconds is None else float(hold_seconds)
        if self._ball_bboxes is None:
            return [None, None, None, None]
        if (time.time() - self._ball_bboxes_ts) > hs:
            return [None, None, None, None]
        return self._ball_bboxes
    
if __name__ == "__main__":
    import socket
    import pickle
    import time

    # UDP socket setup
    UDP_IP = "127.0.0.1"
    UDP_PORT = 5005
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    vision = UnitedVision()
    while(1):
        try:
            ball_pos = vision.GetBallPos()
            chassis_pos = vision.GetChassisPos()
            blue_goal = None
            yellow_goal = None
            try:
                goals_pos = vision.GetGoalsPos()
                if isinstance(goals_pos, (list, tuple)):
                    if len(goals_pos) >= 1:
                        blue_goal = goals_pos[0]
                    if len(goals_pos) >= 2:
                        yellow_goal = goals_pos[1]
                elif isinstance(goals_pos, dict):
                    # backward compatibility for older dict payloads
                    blue_goal = goals_pos.get("blue_goal")
                    yellow_goal = goals_pos.get("yellow_goal")
            except Exception:
                blue_goal = None
                yellow_goal = None
            data = {
                "ball_pos": ball_pos,
                "chassis_pos": chassis_pos,
                "blue_goal": blue_goal,
                "yellow_goal": yellow_goal,
            }
            vision.logger.info(data)
            pickled_data = pickle.dumps(data)
            print(data)
            sock.sendto(pickled_data, (UDP_IP, UDP_PORT))
        except Exception as e:
            print(f"Error in vision server: {e}")
        time.sleep(0.01)  # Adjust sleep time as needed for performance