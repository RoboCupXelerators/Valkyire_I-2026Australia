from __future__ import annotations

import threading
import time
from pathlib import Path

from .PreProcess import MODEL_DIR, Settings, VisionPreUntil, cv2, np
from .hybrid.blob_tracker import BlobTracker
from .hybrid.fusion import FusionEngine
from .hybrid.perf import HybridPerf
from .hybrid.profiles import HybridConfig, load_hybrid_config
from .hybrid.scheduler import RateGate
from .hybrid.yolo_detector import YoloDetector


def _norm_model_path(base: Path, value) -> str:
    text = str(value or "")
    if not text:
        return ""
    path = Path(text)
    if path.is_absolute():
        # Legacy configs store model paths as "/name.ext" while meaning MODEL_DIR/name.ext.
        candidate = base / text.lstrip("/")
        if candidate.exists():
            return str(candidate)
        return str(path)
    return str(base / path)


def _first_existing_path(*paths: str) -> str:
    for path in paths:
        if path and Path(path).exists():
            return path
    return ""


def _configured_model_paths() -> tuple[str, str]:
    vision_cfg = getattr(Settings, "Vision", object())
    configured_model = _norm_model_path(MODEL_DIR, getattr(vision_cfg, "ModelPath", ""))
    configured_engine = _norm_model_path(MODEL_DIR, getattr(vision_cfg, "EnginePath", ""))

    model_path = _first_existing_path(
        configured_model,
        str(MODEL_DIR / "ArisuModel_v1.2s.pt"),
        str(MODEL_DIR / "yolov8s.pt"),
    )
    engine_path = _first_existing_path(
        configured_engine,
        str(MODEL_DIR / "ArisuModel_v1.2s.engine"),
        str(MODEL_DIR / "yolov8s.engine"),
    )
    return model_path, engine_path


class HybridVision(VisionPreUntil):
    """YOLO tick + Blob/ROI subtick vision component.

    The public methods mirror `UnitedVision` so strategy and the control panel
    can consume HybridVision without API churn.
    """

    def __init__(
        self,
        init_cams: bool = True,
        start_threads: bool = True,
        config_overrides: dict | None = None,
        yolo_enabled: bool | None = None,
    ):
        self.cfg_hybrid: HybridConfig = load_hybrid_config(Settings, config_overrides)
        if yolo_enabled is not None:
            self.cfg_hybrid.yolo_enabled = bool(yolo_enabled)

        self._lock = threading.RLock()
        self._frame_lock = threading.Lock()
        self._latest_timestamps = [0.0, 0.0, 0.0, 0.0]
        self._latest_output = None
        self._last_yolo_detections = []

        self.perf = HybridPerf()
        self.BlobMs = 0.0
        self.FusionMs = 0.0
        self.OutputHz = 0.0
        self.YoloHz = 0.0
        self.TrackerHz = 0.0
        self.VisionConfidence = 0.0

        self.BallPos = [0, 0]
        self.BallPosRaw = [0, 0]
        self.MergedChassisList = []
        self.BlueGoalPos = None
        self.YellowGoalPos = None
        self._ball_bbox = None
        self._ball_bboxes = [None, None, None, None]
        self._chassis_bboxes = [None, None, None, None]
        self._goal_bboxes = [None, None, None, None]
        self._ball_bbox_ts = 0.0
        self._ball_bboxes_ts = 0.0
        self._chassis_bboxes_ts = 0.0
        self._goal_bboxes_ts = 0.0

        super().__init__(init_cams=init_cams)

        self.blob_tracker = BlobTracker(
            cv2,
            np,
            pixel_to_cm=self.Pixel2CM,
            to_robot_frame=self._to_robot_frame,
            raw_bbox_to_640=self.RawBBoxTo640,
            roi_scale=self.cfg_hybrid.tracker_roi_scale,
            lost_ms=self.cfg_hybrid.tracker_lost_ms,
            min_area=self.cfg_hybrid.min_ball_area,
        )
        self.fusion = FusionEngine(
            lost_ms=self.cfg_hybrid.tracker_lost_ms,
            predict_ms=self.cfg_hybrid.tracker_predict_ms,
        )

        model_path, engine_path = _configured_model_paths()
        prefer_engine = self.cfg_hybrid.profile in ("orin", "cuda")
        self.yolo = YoloDetector(
            model_path=model_path,
            engine_path=engine_path,
            input_size=self.cfg_hybrid.yolo_input_size,
            prefer_engine=prefer_engine,
            device=0 if self.cfg_hybrid.profile in ("orin", "cuda") else None,
            enabled=self.cfg_hybrid.yolo_enabled,
            pixel_to_cm=self.Pixel2CM,
            to_robot_frame=self._to_robot_frame,
        )
        self.perf.yolo_backend = self.yolo.backend
        self.perf.yolo_error = self.yolo.error

        self._tracker_thread = None
        self._yolo_thread = None
        self._output_thread = None
        if start_threads:
            self._tracker_thread = threading.Thread(target=self._tracker_loop, name="HybridBlobTracker", daemon=True)
            self._tracker_thread.start()
            self._yolo_thread = threading.Thread(target=self._yolo_loop, name="HybridYoloTick", daemon=True)
            self._yolo_thread.start()
            self._output_thread = threading.Thread(target=self._output_loop, name="HybridFusionOutput", daemon=True)
            self._output_thread.start()

    @staticmethod
    def RawBBoxTo640(bbox, frame_shape):
        x, y, w, h = bbox
        src_h, src_w = frame_shape[:2]
        if src_w <= 0 or src_h <= 0:
            return None
        scale = min(640 / src_w, 640 / src_h)
        pad_x = (640 - src_w * scale) / 2
        pad_y = (640 - src_h * scale) / 2
        x1 = int(max(0, min(639, round(x * scale + pad_x))))
        y1 = int(max(0, min(639, round(y * scale + pad_y))))
        x2 = int(max(0, min(639, round((x + w) * scale + pad_x))))
        y2 = int(max(0, min(639, round((y + h) * scale + pad_y))))
        if x2 <= x1 or y2 <= y1:
            return None
        return x1, y1, x2, y2

    def _camera_direction(self, cam_index: int) -> str:
        try:
            port = int(self.CamPorts[int(cam_index)])
        except Exception:
            port = int(cam_index)
        vals = getattr(Settings, "VisionVals", object())
        direction_by_port = {
            int(getattr(vals, "Back", 0)): "back",
            int(getattr(vals, "Left", 2)): "left",
            int(getattr(vals, "Front", 4)): "front",
            int(getattr(vals, "Right", 6)): "right",
        }
        return direction_by_port.get(port, ("back", "left", "front", "right")[int(cam_index) % 4])

    def _to_robot_frame(self, plane_x_cm: float, plane_y_cm: float, cam_index: int) -> tuple[float, float]:
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

    def _copy_frames(self):
        with self._frame_lock:
            frames = [frame.copy() if frame is not None else None for frame in list(self.Frames)]
            timestamps = list(self._latest_timestamps)
        now = time.time()
        for i, frame in enumerate(frames):
            if frame is not None and (i >= len(timestamps) or timestamps[i] <= 0):
                if i >= len(timestamps):
                    timestamps.append(now)
                else:
                    timestamps[i] = now
        return frames, timestamps

    def ReadCams(self):
        frame_count = 0
        last_time = time.time()
        fail_counts = [0] * len(self.Cams)
        while not self._stop_event.is_set():
            frames = []
            timestamps = []
            any_ok = False
            for idx, cam in enumerate(self.Cams):
                ret, frame = cam.read()
                ts = time.time()
                if (not ret) or frame is None:
                    fail_counts[idx] += 1
                    frames.append(None)
                    timestamps.append(0.0)
                    if fail_counts[idx] in (10, 30, 100):
                        self.logger.warning(f"HybridVision camera read failed /dev/video{self.CamPorts[idx]} count={fail_counts[idx]}")
                else:
                    fail_counts[idx] = 0
                    any_ok = True
                    frames.append(frame)
                    timestamps.append(ts)

            if any_ok:
                with self._frame_lock:
                    self.Frames = frames
                    self._latest_timestamps = timestamps
                self.ReadCamTF += 1
            frame_count += 1
            current = time.time()
            if getattr(Settings.Debug, "FullLog", False) and current - last_time >= 1.0:
                self.logger.debug(f"HybridVision camera loop FPS: {frame_count}")
                frame_count = 0
                last_time = current
            self._stop_event.wait(0.0005)

    def _tracker_loop(self):
        gate = RateGate(self.cfg_hybrid.tracker_hz)
        while not self._stop_event.is_set():
            if not gate.wait_next(self._stop_event):
                continue
            now = time.time()
            t0 = time.perf_counter()
            frames, timestamps = self._copy_frames()
            detections = []
            for cam, frame in enumerate(frames[:4]):
                ts = timestamps[cam] if cam < len(timestamps) and timestamps[cam] > 0 else now
                det = self.blob_tracker.track_frame(cam, frame, ts)
                if det is not None:
                    detections.append(det)
            with self._lock:
                self.fusion.update_blob(detections)
            elapsed_ms = (time.perf_counter() - t0) * 1000.0
            self.perf.record_ms("blob_ms", elapsed_ms)
            self.perf.tracker_rate.tick()
            self.InferTF += 1
            self.InferMs = self.perf.blob_ms

    def _yolo_loop(self):
        gate = RateGate(self.cfg_hybrid.yolo_hz)
        while not self._stop_event.is_set():
            if not self.yolo.available():
                self._stop_event.wait(0.1)
                continue
            if not gate.wait_next(self._stop_event):
                continue
            now = time.time()
            t0 = time.perf_counter()
            frames, timestamps = self._copy_frames()
            pp_ms = (time.perf_counter() - t0) * 1000.0
            self.perf.record_ms("preprocess_ms", pp_ms)
            try:
                detections = self.yolo.detect_batch(frames[:4], timestamps[:4], cv2, np)
                with self._lock:
                    self._last_yolo_detections = detections
                    self.fusion.update_yolo(detections)
                    for det in detections:
                        if det.label == "ball":
                            self.blob_tracker.reset_roi(det)
                self.perf.yolo_error = ""
            except Exception as exc:
                detections = []
                self.perf.yolo_error = str(exc)
                self.logger.warning(f"HybridVision YOLO tick failed: {exc}")
            elapsed_ms = (time.perf_counter() - t0) * 1000.0
            self.perf.record_ms("yolo_ms", elapsed_ms)
            self.perf.yolo_rate.tick()
            self.PreProcessTF += 1
            self.PreProcessMs = self.perf.preprocess_ms
            self.InferBatchMs = self.perf.yolo_ms
            self.InferBatchSize = min(4, len([f for f in frames[:4] if f is not None]))

    def _output_loop(self):
        gate = RateGate(self.cfg_hybrid.output_hz)
        while not self._stop_event.is_set():
            if not gate.wait_next(self._stop_event):
                continue
            now = time.time()
            t0 = time.perf_counter()
            with self._lock:
                out = self.fusion.output(now)
                self._latest_output = out
                self.BallPos = [float(out.ball_pos_cm[0]), float(out.ball_pos_cm[1])]
                self.BallPosRaw = list(self.BallPos)
                self.MergedChassisList = list(out.chassis_positions_cm)
                self.BlueGoalPos = out.blue_goal_pos_cm
                self.YellowGoalPos = out.yellow_goal_pos_cm
                self._ball_bbox = out.ball_bbox
                self._ball_bboxes = list(out.ball_bboxes)
                self._chassis_bboxes = list(out.chassis_bboxes)
                self._goal_bboxes = list(out.goal_bboxes)
                self._ball_bbox_ts = now
                self._ball_bboxes_ts = now
                self._chassis_bboxes_ts = now
                self._goal_bboxes_ts = now
                self.perf.vision_confidence = float(out.ball_confidence)
            elapsed_ms = (time.perf_counter() - t0) * 1000.0
            self.perf.record_ms("fusion_ms", elapsed_ms)
            self.perf.output_rate.tick()
            snap = self.perf.snapshot()
            self.PostProcessTF += 1
            self.PostProcessMs = snap.fusion_ms
            self.BlobMs = snap.blob_ms
            self.FusionMs = snap.fusion_ms
            self.OutputHz = snap.output_hz
            self.YoloHz = snap.yolo_hz
            self.TrackerHz = snap.tracker_hz
            self.VisionConfidence = snap.vision_confidence

    def close(self, join_timeout: float = 1.0):
        try:
            self._stop_event.set()
            for thread in (self._tracker_thread, self._yolo_thread, self._output_thread):
                if thread is not None and thread.is_alive():
                    thread.join(timeout=join_timeout)
        finally:
            try:
                super().close(join_timeout=join_timeout)
            except Exception:
                pass

    def GetBallPos(self):
        with self._lock:
            return list(self.BallPos)

    def GetChassisPos(self):
        with self._lock:
            return list(self.MergedChassisList)

    def GetBallBBox(self, hold_seconds: float | None = None):
        with self._lock:
            return self._ball_bbox

    def GetBallBBoxes(self, hold_seconds: float | None = None):
        with self._lock:
            return list(self._ball_bboxes)

    def GetChassisBBoxes(self, hold_seconds: float | None = None):
        with self._lock:
            return list(self._chassis_bboxes)

    def GetGoalBBoxes(self, hold_seconds: float | None = None):
        with self._lock:
            return list(self._goal_bboxes)

    def GetBlueGoalPos(self):
        with self._lock:
            return self.BlueGoalPos

    def GetYellowGoalPos(self):
        with self._lock:
            return self.YellowGoalPos

    def GetGoalsPos(self):
        with self._lock:
            return [self.BlueGoalPos, self.YellowGoalPos]

    def GetHybridPerf(self) -> dict:
        return self.perf.snapshot().to_dict()


# Keep the conventional class name available for direct backend import tests.
UnitedVision = HybridVision


__all__ = ["HybridVision", "UnitedVision"]
