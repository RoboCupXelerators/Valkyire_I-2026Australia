import threading
import math
import numpy as np
import cv2
import time
import sys
from pathlib import Path

BALL_NOT_FOUND = [0xFFF, 0xFFF]  # matches LunaPre.strategy_tools.BALL_NOT_FOUND
BALL_HELD = [0xDDD, 0xDDD]  # matches LunaPre.strategy_tools.BALL_HELD

try:
    from vision_utils.ball_geometry import reconstruct_ball_contact_point  # type: ignore
except Exception:  # pragma: no cover
    try:
        from .ball_geometry import reconstruct_ball_contact_point  # type: ignore
    except Exception:  # pragma: no cover
        from src.vision_utils.ball_geometry import reconstruct_ball_contact_point  # type: ignore

if __package__ in (None, ""):
    _SRC_DIR = Path(__file__).resolve().parents[1]
    _REPO_ROOT = _SRC_DIR.parent
    for _path in (str(_SRC_DIR), str(_REPO_ROOT)):
        if _path not in sys.path:
            sys.path.insert(0, _path)

# Vision configuration is shared with the selected backend and preprocessing
# stack. Runtime data/log ownership remains in ReasonData.
try:
    from vision_utils.merged_settings import Settings  # type: ignore
except Exception:  # pragma: no cover
    from src.vision_utils.merged_settings import Settings  # type: ignore

try:
    from vision_utils import Detect_Method, DATA_DIR  # type: ignore
except Exception:  # pragma: no cover
    from src.vision_utils import Detect_Method, DATA_DIR  # type: ignore

try:
    from vision_utils.modern_calibration import ModernCalibration, ModernCalibrationError  # type: ignore
except Exception:  # pragma: no cover
    from .modern_calibration import ModernCalibration, ModernCalibrationError  # type: ignore


def _norm_rel_folder(folder: str | None, default: str) -> str:
    """Normalize a folder configured like '/Calibration/' to a relative name."""
    s = (folder or default).strip()
    if not s or set(s) == {"/"}:
        s = default
    return s.strip("/") or default.strip("/")


def _cfg(section, name: str, default=None):
    if section is None:
        return default
    return getattr(section, name, default)


def _vision_cfg(name: str, default=None):
    return _cfg(getattr(Settings, "Vision", None), name, default)


def _vision_val(name: str, default=None, *, vision_name: str | None = None):
    vision_vals = getattr(Settings, "VisionVals", None)
    if vision_vals is not None and hasattr(vision_vals, name):
        return getattr(vision_vals, name)
    if vision_name:
        return _vision_cfg(vision_name, default)
    return default


def _vision_port(slot_name: str, slot_index: int, default: int):
    vision_vals = getattr(Settings, "VisionVals", None)
    if vision_vals is not None and hasattr(vision_vals, slot_name):
        return getattr(vision_vals, slot_name)
    ports = _vision_cfg("CamPorts", None)
    try:
        return list(ports)[slot_index]
    except Exception:
        return default


class UnitedVision(Detect_Method):
    def __init__(self):

        # Must initialize parent class FIRST to get CamPorts
        super().__init__()

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
        self.PostProcessMs = float(getattr(self, "PostProcessMs", 0.0) or 0.0)
        self._postprocess_lock = threading.Lock()

        # Calibration for Pixel2CM/CM2Pixel. BlobBased already loads calibration
        # via VisionPreUntil, but AI/TI pipelines historically relied on this class.
        self.reload_calibration()
        self.EnableBBoxDistanceEstimate = False
        self.BallDiameterCM = float(_vision_val("BallDiameterCM", 4.2, vision_name="BallDiameter"))
        self.BallOffsetXCM = float(_vision_cfg("BallOffsetXCM", 0.0) or 0.0)
        self.BallOffsetYCM = float(_vision_cfg("BallOffsetYCM", 0.0) or 0.0)
        self.ChassisWidthCM = float(_vision_val("ChassisWidthCM", 18.0, vision_name="ChassisWidth"))
        self.ChassisHeightCM = float(_vision_val("ChassisHeightCM", 3.5, vision_name="ChassisHeight"))
        self.BlueGoalWidthCM = float(_vision_val("BlueGoalWidthCM", 90.0, vision_name="GoalWidth"))
        self.BlueGoalHeightCM = float(_vision_val("BlueGoalHeightCM", 18.0, vision_name="GoalHeight"))
        self.YellowGoalWidthCM = float(_vision_val("YellowGoalWidthCM", 90.0, vision_name="GoalWidth"))
        self.YellowGoalHeightCM = float(_vision_val("YellowGoalHeightCM", 18.0, vision_name="GoalHeight"))

        # For AI/TI-style queue backends, run queue consumers to compute robot-frame positions.
        # For BlobBased, its own FindBall thread updates self.BallPos directly.
        self.MergedChassisList = []
        self.MergedGoalList = []  # list of all goal detections
        self.BlueGoalPos = None    # best blue goal detection in robot frame: [x, y, conf]
        self.YellowGoalPos = None   # best yellow goal detection in robot frame: [x, y, conf]
        self.BallPos = [0, 0]
        self.BallPosRaw = [0, 0]
        self.BallPixelPoint = None
        self.BallOcclusion = {"reconstructed": False, "kind": "none", "bbox_wh": [0.0, 0.0]}
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

    def reload_calibration(self):
        projectors = []
        calibration_errors = []

        calib_sub = _norm_rel_folder(_vision_val("CalibrationFolder", None, vision_name="CalibrationFolder"), "Calibration")
        calib_dir = None
        try:
            if DATA_DIR is not None:
                from pathlib import Path

                calib_dir = Path(str(DATA_DIR)) / calib_sub
        except Exception:
            calib_dir = None

        for i in range(4):
            if calib_dir is not None:
                npz = str(calib_dir / f"CalibrationData{self.CamPorts[i]}.npz")
            else:
                npz = str(DATA_DIR) + "/" + calib_sub + "/CalibrationData" + str(self.CamPorts[i]) + ".npz"
            try:
                projectors.append(ModernCalibration.load(npz))
            except Exception as e:
                self.logger.error(f"UnitedVision: modern calibration required for camera {self.CamPorts[i]}: {e}")
                projectors.append(None)
                calibration_errors.append(str(e))

        self.CalibrationProjectors = projectors
        self.CalibrationModes = ["modern" if projector is not None else "missing" for projector in projectors]
        self.CalibrationImageSizes = [None if projector is None else projector.image_size for projector in projectors]
        self.CalibrationErrors = calibration_errors
        return {"modes": list(self.CalibrationModes), "errors": list(calibration_errors)}

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

    def _record_postprocess_ms(self, start_time):
        elapsed_ms = (time.perf_counter() - start_time) * 1000.0
        lock = getattr(self, "_postprocess_lock", None)
        if lock is None:
            self.PostProcessMs = elapsed_ms
            return

        with lock:
            previous = float(getattr(self, "PostProcessMs", 0.0) or 0.0)
            if previous <= 0.0:
                self.PostProcessMs = elapsed_ms
            else:
                self.PostProcessMs = previous * 0.8 + elapsed_ms * 0.2

    def Pixel2CM(self, px_x, px_y, CamIndex, frame_size: tuple[int, int] | None = None):
        cam_i = int(CamIndex)
        try:
            projector = self.CalibrationProjectors[cam_i]
            if projector is None:
                raise ModernCalibrationError(f"camera {CamIndex} has no modern calibration")
            return projector.pixel_to_cm(px_x, px_y, frame_size=frame_size)
        except Exception as e:
            raise ModernCalibrationError(f"Pixel2CM failed for camera {CamIndex}: {e}") from e

    def CM2Pixel(self, plane_x_cm, plane_y_cm, CamIndex):
        cam_i = int(CamIndex)
        try:
            projector = self.CalibrationProjectors[cam_i]
            if projector is None:
                raise ModernCalibrationError(f"camera {CamIndex} has no modern calibration")
            return projector.cm_to_pixel(plane_x_cm, plane_y_cm)
        except Exception as e:
            raise ModernCalibrationError(f"CM2Pixel failed for camera {CamIndex}: {e}") from e

    def _camera_direction(self, cam_index: int) -> str:
        try:
            port = int(self.CamPorts[int(cam_index)])
        except Exception:
            port = int(cam_index)

        direction_by_port = {
            int(_vision_port("Back", 0, 0)): "back",
            int(_vision_port("Left", 1, 2)): "left",
            int(_vision_port("Front", 2, 4)): "front",
            int(_vision_port("Right", 3, 6)): "right",
        }
        if port in direction_by_port:
            return direction_by_port[port]

        # Default CamPorts order is [back, left, front, right].
        return ("back", "left", "front", "right")[int(cam_index) % 4]

    def _to_robot_frame(self, plane_x_cm: float, plane_y_cm: float, cam_index: int) -> tuple[float, float]:
        """Convert per-camera plane cm into robot-frame cm.

        Robot frame convention used across the codebase:
        - x_forward_cm: forward distance
        - y_left_cm: leftward distance

        Note: the sign/axis mapping depends on the camera mounting direction.
        """
        # CameraIndex is the inference batch slot. Resolve it through CamPorts,
        # whose physical order is back/left/front/right on the real machine.
        # Pixel2CM returns a camera-local plane: plane_y is outward distance and
        # plane_x is lateral displacement for the mounted camera.  Robot-frame Y
        # is y_left; keep the second component positive on the robot's left.
        direction = self._camera_direction(cam_index)
        if direction == "front":
            return plane_y_cm, -plane_x_cm
        if direction == "left":
            return plane_x_cm, plane_y_cm
        if direction == "back":
            return -plane_y_cm, plane_x_cm
        if direction == "right":
            return -plane_x_cm, -plane_y_cm
        return plane_x_cm, -plane_y_cm

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
        """Use the calibrated ground anchor without a bbox-size range heuristic."""
        return planar_xy, None

    def _update_bbox_distance_caches(self):
        """Disable bbox-size distance estimates for heterogeneous field objects."""
        self._ball_distance_bboxes = [None, None, None, None]
        self._chassis_distance_bboxes = [None, None, None, None]
        self._goal_distance_bboxes = [None, None, None, None]
        self._ball_distance_bbox = None
        self._chassis_distance_bbox = None
        self._goal_distance_bbox = None
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
            post_start = time.perf_counter()
            self.PostProcessTF = self.PostProcessTF + 1
            self.MergedChassisList = []
            ChassisList = []
            # Process ChassisList
            for i in range(4):
                try:
                    item = OutputBuffer[i]
                    CameraIndex, CenterX, BottomY, ChassisWidth, ChassisHeight, Confidence = item[:6]
                    frame_size = (int(item[6]), int(item[7])) if len(item) >= 8 else None
                    # Ignore padded/dummy outputs and clipped edge boxes.
                    if Confidence is None or Confidence <= 0 or ChassisWidth <= 0 or ChassisHeight <= 0:
                        continue
                    plane_x_cm, plane_y_cm = self.Pixel2CM(CenterX, BottomY, CameraIndex, frame_size=frame_size)
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
            self._record_postprocess_ms(post_start)
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

            post_start = time.perf_counter()
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
                    CameraIndex, cls_id, CenterX, BottomY, W, H, Confidence = item[:7]
                    frame_size = (int(item[7]), int(item[8])) if len(item) >= 9 else None
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

                    plane_x_cm, plane_y_cm = self.Pixel2CM(CenterX, BottomY, int(CameraIndex), frame_size=frame_size)
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
            self._record_postprocess_ms(post_start)
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
        self.GoalBBoxUpdate()
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

            post_start = time.perf_counter()
            # Update bbox cache if backend provides it (TunaIntelligence)
            self.BallBBoxUpdate()

            Balls = []
            for i in range(4):
                try:
                    item = OutputBuffer[i]
                    CameraIndex, CenterX, BottomY, BallWidth, BallHeight, Confidence = item[:6]
                    frame_size = (int(item[6]), int(item[7])) if len(item) >= 8 else None
                    # Ignore padded/dummy outputs and clipped edge boxes.
                    min_ball_bbox_area = _vision_cfg("MinBallBBoxArea", 40)
                    if (
                        Confidence is None
                        or Confidence <= 0
                        or BallWidth <= 0
                        or BallHeight <= 0
                        or BallWidth * BallHeight < min_ball_bbox_area
                    ):
                        continue
                    contact_x, contact_y, geom = reconstruct_ball_contact_point(
                        CenterX,
                        BottomY,
                        BallWidth,
                        BallHeight,
                    )
                    plane_x_cm, plane_y_cm = self.Pixel2CM(contact_x, contact_y, CameraIndex, frame_size=frame_size)
                    # print(f"BallDetection raw plane_x_cm={plane_x_cm:.2f}, plane_y_cm={plane_y_cm:.2f} from pixel ({CenterX}, {BottomY}) on cam {CameraIndex}")
                    robot_x_cm, robot_y_cm = self._to_robot_frame(plane_x_cm, plane_y_cm, CameraIndex)
                    conf = float(Confidence)
                    if geom.get("reconstructed"):
                        # Occluded boxes are useful but slightly less trustworthy.
                        conf = max(0.05, conf * 0.92)
                    BallTuple = (
                        round(robot_x_cm, 2),
                        round(robot_y_cm, 2),
                        BallWidth,
                        BallHeight,
                        conf,
                        int(CameraIndex),
                        int(round(contact_x)),
                        int(round(contact_y)),
                        bool(geom.get("reconstructed")),
                        str(geom.get("occlusion") or "none"),
                    )
                    Balls.append(BallTuple)
                except:
                    continue
            if Balls:
                # Pick the closest plausible ball, with a small continuity bias.
                # Confidence alone hops between different balls in cluttered scenes.
                last_pos = getattr(self, "BallPosRaw", [0, 0])
                last_recent = (time.time() - getattr(self, "_ball_pos_ts", 0.0)) <= self.ball_hold_seconds
                def ball_score(ball):
                    robot_dist = math.hypot(float(ball[0]), float(ball[1]))
                    continuity = 0.0
                    if last_recent and last_pos not in ([0, 0], BALL_NOT_FOUND, BALL_HELD):
                        continuity = math.hypot(float(ball[0]) - float(last_pos[0]), float(ball[1]) - float(last_pos[1]))
                    return robot_dist + continuity * 0.4 - float(ball[4]) * 5.0
                Balls = sorted(Balls, key=ball_score)
                Ball = Balls[0]
                raw_robot_x_cm = float(Ball[0])
                raw_robot_y_cm = float(Ball[1])
                robot_x_cm = raw_robot_x_cm + self.BallOffsetXCM
                robot_y_cm = raw_robot_y_cm + self.BallOffsetYCM
                self.BallPosRaw = [round(raw_robot_x_cm, 2), round(raw_robot_y_cm, 2)]
                self.BallPixelPoint = [int(Ball[5]), int(Ball[6]), int(Ball[7])]
                self.BallOcclusion = {
                    "reconstructed": bool(Ball[8]) if len(Ball) > 8 else False,
                    "kind": str(Ball[9]) if len(Ball) > 9 else "none",
                    "bbox_wh": [float(Ball[2]), float(Ball[3])],
                }
                # Pull the position slightly toward the bbox-derived distance.
                # This keeps the direction from pixel2cm while correcting long-range drift.
                cam = int(Ball[5])
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
                self.BallPos = [round(float(fused[0]), 2), round(float(fused[1]), 2)]
                self._ball_pos_ts = time.time()
            else:
                # hold last position briefly to reduce flicker
                if (time.time() - getattr(self, "_ball_pos_ts", 0.0)) > self.ball_hold_seconds:
                    self.BallPos = [0, 0]
            self._record_postprocess_ms(post_start)
            time.sleep(0.01)

    def GetBallPos(self):
        return getattr(self, "BallPos", [0, 0])

    def GetBallPosRaw(self):
        return getattr(self, "BallPosRaw", [0, 0])

    def GetBallPixelPoint(self):
        return getattr(self, "BallPixelPoint", None)

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

    def GoalBBoxUpdate(self):
        """Read GoalBBoxQueue if available and keep latest goal bboxes."""
        q = getattr(self, "GoalBBoxQueue", None)
        if q is None:
            return
        try:
            out = q.get_nowait()
        except Exception:
            return

        bboxes = [None, None, None, None]
        best = None
        for i in range(4):
            try:
                # Goal bbox format: [cam, cls, x1, y1, x2, y2, conf]
                # cls: 1=BlueGoal, 2=YellowGoal
                item = out[i]
                if item is None or len(item) < 7:
                    continue
                cam, cls, x1, y1, x2, y2, conf = item[0], item[1], item[2], item[3], item[4], item[5], item[6]
                if conf is None or conf <= 0:
                    continue
                if cls is None or int(cls) < 0:
                    continue
                bbox = (int(cam), int(cls), int(x1), int(y1), int(x2), int(y2), float(conf))
                bboxes[int(cam)] = bbox
                if best is None or conf > best[6]:
                    best = bbox
            except Exception:
                continue
        self._goal_bboxes = bboxes
        self._goal_bboxes_ts = time.time()
        if best is not None:
            self._goal_bbox = best
            self._goal_bbox_ts = time.time()
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
    import signal

    # UDP socket setup
    UDP_IP = "127.0.0.1"
    UDP_PORT = 5005
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    vision = UnitedVision()
    running = {"value": True}

    def _shutdown(*_args):
        running["value"] = False

    try:
        signal.signal(signal.SIGINT, _shutdown)
        signal.signal(signal.SIGTERM, _shutdown)
    except Exception:
        pass

    try:
        while running["value"]:
            try:
                ball_pos = vision.GetBallPos()
                chassis_pos = vision.GetChassisPos()
                blue_goal = None
                yellow_goal = None
                goal_bboxes = [None, None, None, None]
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
                try:
                    goal_bboxes = vision.GetGoalBBoxes()
                except Exception:
                    goal_bboxes = [None, None, None, None]
                data = {
                    "ball_pos": ball_pos,
                    "chassis_pos": chassis_pos,
                    "blue_goal": blue_goal,
                    "yellow_goal": yellow_goal,
                    "goal_bboxes": goal_bboxes,
                }
                # vision.logger.info(data)
                pickled_data = pickle.dumps(data)
                # print(data)
                sock.sendto(pickled_data, (UDP_IP, UDP_PORT))
            except Exception as e:
                print(f"Error in vision server: {e}")
            time.sleep(0.01)  # Adjust sleep time as needed for performance
    finally:
        # Always release cameras / background threads so the next process can open V4L2 devices.
        try:
            close = getattr(vision, "close", None)
            if callable(close):
                close(join_timeout=1.0)
        except Exception as e:
            print(f"Vision cleanup failed: {e}")
        try:
            sock.close()
        except Exception:
            pass
