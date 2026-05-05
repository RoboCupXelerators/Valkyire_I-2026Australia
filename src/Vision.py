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

        # Ensure CamPorts is available (XeleratorVision backend doesn't have it)
        if not hasattr(self, 'CamPorts'):
            self.CamPorts = [
                Settings.VisionVals.Front,
                Settings.VisionVals.Right,
                Settings.VisionVals.Back,
                Settings.VisionVals.Left,
            ]

        # ---- Runtime caches/params (must be set before starting threads) ----
        # Ball bbox (xyxy in 640x640 resized space from detector)
        self._ball_bbox = None  # (cam, x1,y1,x2,y2,conf)
        self._ball_bbox_ts = 0.0
        self._ball_bboxes = None  # list[ (cam,x1,y1,x2,y2,conf) | None ] length=4
        self._ball_bboxes_ts = 0.0
        self._ball_distance_bbox = None  # best bbox-based distance estimate in cm
        self._ball_distance_bboxes = None  # per-camera bbox distance estimates in cm
        self._ball_distance_ts = 0.0

        # Chassis bbox (xyxy in 640x640 resized space from detector)
        self._chassis_bbox = None  # (cam, x1,y1,x2,y2,conf)
        self._chassis_bbox_ts = 0.0
        self._chassis_bboxes = None  # list len=4
        self._chassis_bboxes_ts = 0.0
        self._chassis_distance_bbox = None
        self._chassis_distance_bboxes = None
        self._chassis_distance_ts = 0.0

        # Goal bbox (xyxy in 640x640 resized space from detector)
        # Store both the best overall bbox and per-camera top1 bbox.
        self._goal_bbox = None  # (cam, cls, x1,y1,x2,y2,conf)
        self._goal_bbox_ts = 0.0
        self._goal_bboxes = None  # list len=4 of (cam, cls, x1,y1,x2,y2,conf) | None
        self._goal_bboxes_ts = 0.0
        self._goal_distance_bbox = None
        self._goal_distance_bboxes = None
        self._goal_distance_ts = 0.0

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
        self.CalibrationKs = []
        self.CalibrationDists = []
        self.CalibrationModes = []
        self.EnableBBoxDistanceEstimate = bool(getattr(Settings.VisionVals, "EnableBBoxDistanceEstimate", False))
        self.BallDiameterCM = float(getattr(Settings.VisionVals, "BallDiameterCM", 4.2))
        self.ChassisWidthCM = float(getattr(Settings.VisionVals, "ChassisWidthCM", 18.0))
        self.ChassisHeightCM = float(getattr(Settings.VisionVals, "ChassisHeightCM", 3.5))
        self.BBoxFusionStrength = float(getattr(Settings.VisionVals, "BBoxFusionStrength", 0.5))
        self.BBoxLegacyReferenceAreaPx = float(getattr(Settings.VisionVals, "BBoxLegacyReferenceAreaPx", 12000.0))
        self.BlueGoalWidthCM = float(getattr(Settings.VisionVals, "BlueGoalWidthCM", 90.0))
        self.BlueGoalHeightCM = float(getattr(Settings.VisionVals, "BlueGoalHeightCM", 18.0))
        self.YellowGoalWidthCM = float(getattr(Settings.VisionVals, "YellowGoalWidthCM", 90.0))
        self.YellowGoalHeightCM = float(getattr(Settings.VisionVals, "YellowGoalHeightCM", 18.0))

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
                NumpyData = np.load(npz, allow_pickle=True)
                PerspectiveMatrix = NumpyData['matrix']
                self.PerspectiveMatrices.append(PerspectiveMatrix)
                self.P2CK.append(float(NumpyData['p2c'][0]))
                self.P2CHB.append(float(NumpyData['p2c'][1]))
                self.P2CVB.append(float(NumpyData['p2c'][2]))

                # New calibration metadata (optional, modern web calibrator)
                if 'K' in NumpyData.files:
                    self.CalibrationKs.append(np.asarray(NumpyData['K'], dtype=np.float64))
                    self.CalibrationDists.append(np.asarray(NumpyData['dist'], dtype=np.float64) if 'dist' in NumpyData.files else None)
                    self.CalibrationModes.append('modern')
                else:
                    self.CalibrationKs.append(None)
                    self.CalibrationDists.append(None)
                    self.CalibrationModes.append('legacy')
            except Exception as e:
                # Don't crash vision server if calibration is missing.
                # Pixel2CM will fall back (or behave as identity).
                self.logger.warning(f"UnitedVision: calibration load failed: {npz}: {e}")
                self.PerspectiveMatrices.append(np.eye(3, dtype=np.float64))
                self.P2CK.append(1.0)
                self.P2CHB.append(0.0)
                self.P2CVB.append(0.0)
                self.CalibrationKs.append(None)
                self.CalibrationDists.append(None)
                self.CalibrationModes.append('missing')


        # For AI/TV backends, run queue consumers to compute robot-frame positions.
        # For BlobBased, its own FindBall thread updates self.BallPos directly.
        self.MergedChassisList = []
        self.MergedGoalList = []  # list of all goal detections
        self.BlueGoalPos = None    # best blue goal detection in robot frame: [x, y, conf]
        self.YellowGoalPos = None   # best yellow goal detection in robot frame: [x, y, conf]
        self.BallPosRaw = [0, 0]
        self.ChassisPosRaw = []
        self.BlueGoalPosRaw = None
        self.YellowGoalPosRaw = None
        if self._queue_backend:
            self.ChassisDetectionThread = threading.Thread(target=self.ChassisDetection)
            self.ChassisDetectionThread.daemon = True
            self.ChassisDetectionThread.start()

            self.BallDetectionThread = threading.Thread(target=self.BallDetection)
            self.BallDetectionThread.daemon = True
            self.BallDetectionThread.start()

        # Optional: goals (BlueGoal/YellowGoal) available in AI backends.
        # self._goal_backend = hasattr(self, "GoalQueue")
        # if self._goal_backend:
        #     self.GoalDetectionThread = threading.Thread(target=self.GoalDetection)
        #     self.GoalDetectionThread.daemon = True
        #     self.GoalDetectionThread.start()

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

    def ApplyPerspectiveTransform(self, px_x, px_y, Matrix):
        # OpenCV image plane: origin is top-left, +x right, +y down.
        # This function only applies the homography; it does NOT convert to robot frame.
        Point = np.array([px_x, px_y, 1], dtype=np.float64)
        Transformed = Matrix @ Point
        Transformed /= Transformed[2]
        return int(Transformed[0]), int(Transformed[1])

    def Pixel2CM(self, px_x, px_y, CamIndex):
        # Step 1: image pixel -> board plane (still camera/board coordinates)
        plane_x_px, plane_y_px = self.ApplyPerspectiveTransform(px_x, px_y, self.PerspectiveMatrices[CamIndex])

        # Step 2: board-plane pixels -> board-plane centimeters
        # P2CK: scale, P2CHB/P2CVB: offsets. Y is flipped here so that later
        # conversion to robot frame can be named explicitly.
        plane_y_cm = -round(self.P2CK[CamIndex] * plane_x_px + self.P2CHB[CamIndex], 2)
        plane_x_cm = -round(-self.P2CK[CamIndex] * plane_y_px + self.P2CVB[CamIndex], 2)

        # IMPORTANT: return value is NOT robot-frame. It is still per-camera plane cm.
        return plane_x_cm, plane_y_cm

    def CM2Pixel(self, plane_x_cm, plane_y_cm, CamIndex):
        # Inverse of Pixel2CM: board-plane cm -> board-plane pixels -> raw image pixels.
        plane_x_px = (plane_x_cm - self.P2CHB[CamIndex]) / self.P2CK[CamIndex]
        plane_y_px = -(plane_y_cm - self.P2CVB[CamIndex]) / self.P2CK[CamIndex]

        raw_px_x, raw_px_y = self.ApplyPerspectiveTransform(
            plane_x_px,
            plane_y_px,
            np.linalg.inv(self.PerspectiveMatrices[CamIndex]),
        )

        return int(raw_px_x), int(raw_px_y)

    @staticmethod
    def _bbox_distance_from_sizes(real_w_cm: float | None, real_h_cm: float | None, bbox_w_px: float, bbox_h_px: float, K: np.ndarray | None):
        """Estimate target distance from bbox size using pinhole model.

        Returns distance in cm, or None if inputs are insufficient.
        """
        if K is None:
            return None
        try:
            fx = float(K[0, 0])
            fy = float(K[1, 1])
        except Exception:
            return None
        if fx <= 0 or fy <= 0:
            return None

        candidates = []
        if real_w_cm is not None and real_w_cm > 0 and bbox_w_px and bbox_w_px > 1:
            candidates.append(float(real_w_cm) * fx / float(bbox_w_px))
        if real_h_cm is not None and real_h_cm > 0 and bbox_h_px and bbox_h_px > 1:
            candidates.append(float(real_h_cm) * fy / float(bbox_h_px))
        if not candidates:
            return None
        return float(sum(candidates) / len(candidates))

    @staticmethod
    def _to_robot_frame(plane_x_cm: float, plane_y_cm: float, cam_index: int) -> tuple[float, float]:
        """Convert per-camera plane cm into robot-frame cm.

        Robot frame convention used across the codebase:
        - x_forward_cm: forward distance
        - y_left_cm: leftward distance

        Note: the sign/axis mapping depends on the camera mounting direction.
        """
        if cam_index == 0:
            return -plane_x_cm, plane_y_cm
        if cam_index == 1:
            return plane_y_cm, plane_x_cm
        if cam_index == 2:
            return plane_x_cm, -plane_y_cm
        if cam_index == 3:
            return -plane_y_cm, -plane_x_cm
        return plane_x_cm, plane_y_cm

    def _bbox_distance_legacy(self, bbox_w_px: float, bbox_h_px: float, real_scale_cm: float | None):
        """Fallback distance estimate when intrinsics are unavailable.

        This uses bbox area as a monotonic proxy for range. It is not metrically
        exact, but it guarantees the fusion layer still has a visible effect on
        legacy calibration files (matrix/p2c only).
        """
        try:
            area = float(max(1.0, bbox_w_px * bbox_h_px))
            ref = float(max(1.0, self.BBoxLegacyReferenceAreaPx))
            # farther -> smaller bbox -> larger distance
            scale = float(np.sqrt(ref / area))
            if real_scale_cm is not None and real_scale_cm > 0:
                return float(real_scale_cm * scale)
            return float(100.0 * scale)
        except Exception:
            return None

    @staticmethod
    def _blend_planar_with_range(planar_xy: tuple[float, float], range_cm: float | None, confidence: float, strength: float = 0.35):
        """Fuse pixel2cm planar estimate with bbox-derived range.

        The direction comes from the plane estimate, while the magnitude is softly
        pulled toward the bbox-based range. This helps reduce long-distance drift
        without destroying the existing coordinate semantics.
        """
        if range_cm is None or range_cm <= 0:
            return planar_xy

        x_cm, y_cm = float(planar_xy[0]), float(planar_xy[1])
        r_plane = float(np.hypot(x_cm, y_cm))
        if r_plane <= 1e-6:
            return planar_xy

        conf = float(np.clip(confidence, 0.0, 1.0))
        strength = float(np.clip(strength, 0.0, 1.0))
        alpha = float(np.clip(0.15 + 0.55 * conf + 0.30 * strength, 0.20, 0.85))
        r_fused = alpha * r_plane + (1.0 - alpha) * float(range_cm)
        scale = r_fused / r_plane
        return x_cm * scale, y_cm * scale

    def _fuse_detection_position(
        self,
        planar_xy: tuple[float, float],
        cam_index: int,
        conf: float,
        bbox_w_px: float,
        bbox_h_px: float,
        real_w_cm: float | None,
        real_h_cm: float | None,
        strength: float = 0.35,
    ):
        """Fuse pixel2cm planar result with bbox-size range estimate."""
        if not self.EnableBBoxDistanceEstimate:
            return planar_xy, None
        rng = None
        if 0 <= cam_index < len(self.CalibrationKs):
            rng = self._bbox_distance_from_sizes(real_w_cm, real_h_cm, bbox_w_px, bbox_h_px, self.CalibrationKs[cam_index])
        if rng is None:
            # Legacy fallback: still produce a usable correction from bbox area.
            real_scale = None
            if real_w_cm is not None and real_h_cm is not None and real_w_cm > 0 and real_h_cm > 0:
                real_scale = float(np.sqrt(float(real_w_cm) * float(real_h_cm)))
            rng = self._bbox_distance_legacy(bbox_w_px, bbox_h_px, real_scale)
        fused = self._blend_planar_with_range(planar_xy, rng, conf, strength=strength)
        return fused, rng

    def _update_bbox_distance_caches(self):
        """Compute per-camera and best bbox-based distance estimates from latest bboxes."""
        ball_bboxes = getattr(self, "_ball_bboxes", None)
        chassis_bboxes = getattr(self, "_chassis_bboxes", None)
        goal_bboxes = getattr(self, "_goal_bboxes", None)

        ball_dist = [None, None, None, None]
        chassis_dist = [None, None, None, None]
        goal_dist = [None, None, None, None]

        def _pick_best(current, candidate):
            if candidate is None:
                return current
            if current is None:
                return candidate
            return candidate if candidate[1] > current[1] else current

        best_ball = None
        best_chassis = None
        best_goal = None

        for i in range(4):
            try:
                if ball_bboxes is not None and ball_bboxes[i] is not None:
                    cam, x1, y1, x2, y2, conf = ball_bboxes[i]
                    d = self._bbox_distance_from_sizes(self.BallDiameterCM, self.BallDiameterCM, x2 - x1, y2 - y1, self.CalibrationKs[i])
                    if d is None:
                        d = self._bbox_distance_legacy(x2 - x1, y2 - y1, self.BallDiameterCM)
                    ball_dist[i] = d
                    if d is not None:
                        best_ball = _pick_best(best_ball, (d, conf, i))
                if chassis_bboxes is not None and chassis_bboxes[i] is not None:
                    cam, x1, y1, x2, y2, conf = chassis_bboxes[i]
                    d = self._bbox_distance_from_sizes(self.ChassisWidthCM, self.ChassisHeightCM, x2 - x1, y2 - y1, self.CalibrationKs[i])
                    if d is None:
                        d = self._bbox_distance_legacy(x2 - x1, y2 - y1, float(np.sqrt(self.ChassisWidthCM * self.ChassisHeightCM)))
                    chassis_dist[i] = d
                    if d is not None:
                        best_chassis = _pick_best(best_chassis, (d, conf, i))
                if goal_bboxes is not None and goal_bboxes[i] is not None:
                    cam, cls_id, x1, y1, x2, y2, conf = goal_bboxes[i]
                    if int(cls_id) == 1:
                        rw, rh = self.BlueGoalWidthCM, self.BlueGoalHeightCM
                    elif int(cls_id) == 2:
                        rw, rh = self.YellowGoalWidthCM, self.YellowGoalHeightCM
                    else:
                        rw, rh = self.BlueGoalWidthCM, self.BlueGoalHeightCM
                    d = self._bbox_distance_from_sizes(rw, rh, x2 - x1, y2 - y1, self.CalibrationKs[i])
                    if d is None:
                        d = self._bbox_distance_legacy(x2 - x1, y2 - y1, float(np.sqrt(rw * rh)))
                    goal_dist[i] = d
                    if d is not None:
                        best_goal = _pick_best(best_goal, (d, conf, i))
            except Exception:
                continue

        self._ball_distance_bboxes = ball_dist
        self._chassis_distance_bboxes = chassis_dist
        self._goal_distance_bboxes = goal_dist
        self._ball_distance_bbox = None if best_ball is None else (best_ball[2], float(best_ball[0]), float(best_ball[1]))
        self._chassis_distance_bbox = None if best_chassis is None else (best_chassis[2], float(best_chassis[0]), float(best_chassis[1]))
        self._goal_distance_bbox = None if best_goal is None else (best_goal[2], float(best_goal[0]), float(best_goal[1]))
        self._ball_distance_ts = time.time()
        self._chassis_distance_ts = time.time()
        self._goal_distance_ts = time.time()

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
                    plane_x_cm, plane_y_cm = self.Pixel2CM(CenterX, BottomY, CameraIndex)
                    (CX, CY), _ = self._fuse_detection_position(
                        self._to_robot_frame(plane_x_cm, plane_y_cm, CameraIndex),
                        int(CameraIndex),
                        float(Confidence),
                        float(ChassisWidth),
                        float(ChassisHeight),
                        self.ChassisWidthCM,
                        self.ChassisHeightCM,
                        strength=0.45,
                    )
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

                    plane_x_cm, plane_y_cm = self.Pixel2CM(CenterX, BottomY, int(CameraIndex))
                    # Match BallDetection axis conventions: output must be robot-frame (x_forward, y_left).
                    robot_xy = self._to_robot_frame(plane_x_cm, plane_y_cm, cam_i)
                    fused, _ = self._fuse_detection_position(
                        robot_xy,
                        cam_i,
                        float(Confidence),
                        float(W),
                        float(H),
                        self.BlueGoalWidthCM if cls_i == 1 else self.YellowGoalWidthCM,
                        self.BlueGoalHeightCM if cls_i == 1 else self.YellowGoalHeightCM,
                        strength=0.55,
                    )
                    robot_x_cm, robot_y_cm = fused
                    self.logger.debug(
                        f"Goal Processed ER: robot_x={robot_x_cm:.2f}, robot_y={robot_y_cm:.2f}, "
                        f"plane_x={plane_x_cm:.2f}, plane_y={plane_y_cm:.2f}, cls_id={cls_id}, conf={Confidence}"
                    )

                    goal_item = [round(robot_x_cm, 2), round(robot_y_cm, 2), float(Confidence)]
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
            self.BlueGoalPosRaw = blue_goal
            self.YellowGoalPosRaw = yellow_goal
            self._goal_bboxes = goal_bboxes
            self._goal_bboxes_ts = time.time()
            if best_goal is not None:
                self._goal_bbox = best_goal
                self._goal_bbox_ts = time.time()
            self._update_bbox_distance_caches()
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
                    plane_x_cm, plane_y_cm = self.Pixel2CM(CenterX, BottomY, CameraIndex)
                    # print(f"BallDetection raw plane_x_cm={plane_x_cm:.2f}, plane_y_cm={plane_y_cm:.2f} from pixel ({CenterX}, {BottomY}) on cam {CameraIndex}")
                    robot_x_cm, robot_y_cm = self._to_robot_frame(plane_x_cm, plane_y_cm, CameraIndex)
                    BallTuple = (round(robot_x_cm,2), round(robot_y_cm,2), BallWidth, BallHeight, Confidence)
                    Balls.append(BallTuple)
                except:
                    continue
            if Balls:
                # pick the most confident one
                Balls = sorted(Balls, key=lambda x: x[4], reverse=True)
                Ball = Balls[0]
                robot_x_cm = Ball[0]
                robot_y_cm = Ball[1]
                self.BallPosRaw = [robot_x_cm, robot_y_cm]
                # Pull the position slightly toward the bbox-derived distance.
                # This keeps the direction from pixel2cm while correcting long-range drift.
                best_idx = 0
                for idx, item in enumerate(OutputBuffer):
                    try:
                        if item[4] == Ball[2] and item[5] == Ball[3] and item[6] == Ball[4]:
                            best_idx = idx
                            break
                    except Exception:
                        continue
                cam = int(OutputBuffer[best_idx][0]) if len(OutputBuffer) > best_idx else 0
                bbox_w = float(Ball[2])
                bbox_h = float(Ball[3])
                fused, _ = self._fuse_detection_position(
                    (robot_x_cm, robot_y_cm),
                    cam,
                    float(Ball[4]),
                    bbox_w,
                    bbox_h,
                    self.BallDiameterCM,
                    self.BallDiameterCM,
                    strength=max(0.55, self.BBoxFusionStrength),
                )
                self.BallPos = self.BallPosRaw
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
        self._update_bbox_distance_caches()

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
        self._update_bbox_distance_caches()

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

    def GetBallDistanceEstimate(self, hold_seconds: float | None = None):
        """Return best ball distance estimate in cm using bbox size + calibration intrinsics."""
        self.BallBBoxUpdate()
        if not self.EnableBBoxDistanceEstimate:
            return None
        hs = self.ball_hold_seconds if hold_seconds is None else float(hold_seconds)
        if self._ball_distance_bbox is None:
            return None
        if (time.time() - self._ball_distance_ts) > hs:
            return None
        return self._ball_distance_bbox

    def GetBallDistanceEstimates(self, hold_seconds: float | None = None):
        self.BallBBoxUpdate()
        if not self.EnableBBoxDistanceEstimate:
            return [None, None, None, None]
        hs = self.ball_hold_seconds if hold_seconds is None else float(hold_seconds)
        if self._ball_distance_bboxes is None:
            return [None, None, None, None]
        if (time.time() - self._ball_distance_ts) > hs:
            return [None, None, None, None]
        return self._ball_distance_bboxes

    def GetChassisDistanceEstimate(self, hold_seconds: float | None = None):
        self.ChassisBBoxUpdate()
        if not self.EnableBBoxDistanceEstimate:
            return None
        hs = self.ball_hold_seconds if hold_seconds is None else float(hold_seconds)
        if self._chassis_distance_bbox is None:
            return None
        if (time.time() - self._chassis_distance_ts) > hs:
            return None
        return self._chassis_distance_bbox

    def GetChassisDistanceEstimates(self, hold_seconds: float | None = None):
        self.ChassisBBoxUpdate()
        if not self.EnableBBoxDistanceEstimate:
            return [None, None, None, None]
        hs = self.ball_hold_seconds if hold_seconds is None else float(hold_seconds)
        if self._chassis_distance_bboxes is None:
            return [None, None, None, None]
        if (time.time() - self._chassis_distance_ts) > hs:
            return [None, None, None, None]
        return self._chassis_distance_bboxes

    def GetGoalDistanceEstimate(self, hold_seconds: float | None = None):
        if not self.EnableBBoxDistanceEstimate:
            return None
        hs = self.ball_hold_seconds if hold_seconds is None else float(hold_seconds)
        if self._goal_distance_bbox is None:
            return None
        if (time.time() - self._goal_distance_ts) > hs:
            return None
        return self._goal_distance_bbox

    def GetGoalDistanceEstimates(self, hold_seconds: float | None = None):
        if not self.EnableBBoxDistanceEstimate:
            return [None, None, None, None]
        hs = self.ball_hold_seconds if hold_seconds is None else float(hold_seconds)
        if self._goal_distance_bboxes is None:
            return [None, None, None, None]
        if (time.time() - self._goal_distance_ts) > hs:
            return [None, None, None, None]
        return self._goal_distance_bboxes
    
if __name__ == "__main__":
    import socket
    import pickle

    # UDP socket setup
    UDP_IP = "127.0.0.1"
    UDP_PORT = 5005
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    vision = UnitedVision()
    while True:
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
            # vision.logger.info(data)
            pickled_data = pickle.dumps(data)
            # print(data)
            sock.sendto(pickled_data, (UDP_IP, UDP_PORT))
        except Exception as e:
            print(f"Error in vision server: {e}")
        time.sleep(0.01)  # Adjust sleep time as needed for performance