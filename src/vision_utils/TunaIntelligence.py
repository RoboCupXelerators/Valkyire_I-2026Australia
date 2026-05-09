"""TunaIntelligence

Jetson Nano (super) oriented inference backend.

This module mirrors the thread/queue interface of `ArisuIntelligence` so that
`src/Vision.py` can consume the same `ChassisQueue` and `BallQueue` outputs.

Model path (recommended): `src/vision_utils/models/yolov8nForX5.pt`

Recommended backend on Jetson
-----------------------------
TensorRT engine exported from Ultralytics:

1) Export (on Jetson) .pt -> .engine (FP16 by default; INT8 requires calibration data)
2) Load .engine with Ultralytics YOLO, which will dispatch to TensorRT runtime.

Notes
-----
- This repository does not pin `torch/ultralytics` because Jetson requires
  JetPack-compatible PyTorch wheels.
- This project uses a custom 4-class detect model by default:
    0 Ball, 1 BlueGoal, 2 YellowGoal, 3 Chassis
  (see yolov8nForX5.pt). If you load a COCO model (80 classes), the class IDs
  will differ and postprocess must be updated accordingly.
"""

from __future__ import annotations

from .PreProcess import Settings, logger, MODEL_DIR, VisionPreUntil, threading, queue, time, cv2, np

from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass
class _Det:
    cls: int
    conf: float
    xyxy: Tuple[float, float, float, float]  # x1,y1,x2,y2 in 640x640 space


class TunaIntelligence(VisionPreUntil):
    """YOLOv8s(pt) inference on Jetson.

    Output format (matches ArisuIntelligence):
        BallOutputs:    [[cam, center_x, bottom_y, w, h, conf], ...]
        ChassisOutputs: [[cam, center_x, bottom_y, w, h, conf], ...]

    Important:
        `src/Vision.py` currently *assumes* OutputBuffer[i] exists for i=0..3.
        Therefore we always output at most 1 detection per camera for ball and
        chassis (top-1 by confidence), and we pad missing cameras with dummy
        entries (conf=0).
    """

    def __init__(
        self,
        model_path: Optional[str] = None,
        engine_path: Optional[str] = None,
        device: Optional[str] = None,
        prefer_engine: bool = True,
        # Ball conf in real camera feed tends to be ~0.35-0.45 for small/blur balls.
        # Use a slightly lower default to reduce flicker.
        conf_ball: float = 0.30,
        conf_chassis: float = 0.50,
        init_cams: bool = True,
        export_engine_if_missing: bool = True,
        engine_half: bool = True,
        start_threads: bool = True,
    ):
        self.logger = logger

        self.conf_ball = float(conf_ball)
        self.conf_chassis = float(conf_chassis)

        self.PreProcessTF = 0
        self.InferTF = 0

        # Queues consistent with ArisuIntelligence
        self.YOLOQueue = queue.Queue(maxsize=1)
        self.MergedChassisList = []
        self.ChassisQueue = queue.Queue(maxsize=1)
        self.BallQueue = queue.Queue(maxsize=1)
        # Goal outputs: per-camera top1 for blue/yellow goal.
        # Format: [[cam, cls, center_x, bottom_y, w, h, conf], ...] padded to 4.
        # cls: 1 BlueGoal, 2 YellowGoal (matches model class IDs / mapping).
        self.GoalQueue = queue.Queue(maxsize=1)
        # Optional: bbox outputs (xyxy in 640x640 resized space)
        self.BallBBoxQueue = queue.Queue(maxsize=1)
        self.ChassisBBoxQueue = queue.Queue(maxsize=1)

        # Init cameras/record threads from base class (can be disabled for tests)
        super().__init__(init_cams=init_cams)

        # Model paths (config-driven)
        # Priority:
        #   1) explicit args: model_path/engine_path
        #   2) config: Settings.Vision.ModelPath / EnginePath
        #   3) fallback: repo default yolov8nForX5
        cfg_model = str(MODEL_DIR) + getattr(getattr(Settings, "Vision", object()), "ModelPath", None)
        cfg_engine = str(MODEL_DIR) + getattr(getattr(Settings, "Vision", object()), "EnginePath", None)

        self.model_path = cfg_model
        self.engine_path = cfg_engine
        self.device = device  # None => auto
        # Default behavior: prefer TensorRT engine if present (Jetson-oriented).
        # Set prefer_engine=False in offline CPU tests.
        self.prefer_engine = bool(prefer_engine)

        # If user explicitly requests CPU, never try TensorRT engine (it will fail
        # or even segfault on some builds).
        if isinstance(self.device, str) and self.device.lower().startswith("cpu"):
            self.prefer_engine = False
        self.export_engine_if_missing = bool(export_engine_if_missing)
        self.engine_half = bool(engine_half)

        self._backend = None
        self._yolo = None  # ultralytics.YOLO
        self._names = None  # class-id -> name
        self._cls_ball = 0
        self._cls_chassis = 3
        self._cls_blue_goal = 1
        self._cls_yellow_goal = 2

        # thread lifecycle
        # NOTE: VisionPreUntil already defines self._stop_event for camera threads.
        # We reuse the same event so that one close() can stop everything.
        if not hasattr(self, "_stop_event"):
            self._stop_event = threading.Event()
        self._start_threads = bool(start_threads)

        self._init_backend()

        self.ModelPreProcessThread = None
        self.ModelInferThread = None
        if self._start_threads:
            self.ModelPreProcessThread = threading.Thread(target=self.ModelPreProcess)
            self.ModelPreProcessThread.daemon = True
            self.ModelPreProcessThread.start()

            self.ModelInferThread = threading.Thread(target=self.ModelInfer)
            self.ModelInferThread.daemon = True
            self.ModelInferThread.start()

    def close(self, join_timeout: float = 1.0):
        """Stop background threads.

        Useful for offline tests to avoid interpreter abort caused by active
        threads while torch/TensorRT is shutting down.
        """
        try:
            self._stop_event.set()
            # unblock/flush queues (best-effort)
            try:
                while True:
                    _ = self.YOLOQueue.get_nowait()
            except Exception:
                pass
            try:
                self.YOLOQueue.put_nowait([None, None, None, None])
            except Exception:
                pass

            for t in (self.ModelPreProcessThread, self.ModelInferThread):
                if t is not None and t.is_alive():
                    t.join(timeout=join_timeout)

            # also stop/release camera threads/resources
            try:
                super().close(join_timeout=join_timeout)
            except Exception:
                pass
        except Exception:
            pass

    def __del__(self):
        # best-effort cleanup
        try:
            self.close(join_timeout=0.2)
        except Exception:
            pass

    # ------------------------- util -------------------------
    def Resize(self, Frame, TargetSize=(640, 640)):
        """Letterbox-ish resize to 640x640 with padding (same as Arisu)."""
        Height, Width = Frame.shape[:2]
        TargetHeight, TargetWidth = TargetSize

        Scale = min(TargetWidth / Width, TargetHeight / Height)
        NewWidth = int(Width * Scale)
        NewHeight = int(Height * Scale)

        ResizedImage = cv2.resize(Frame, (NewWidth, NewHeight))
        PaddedImage = np.zeros((TargetHeight, TargetWidth, 3), dtype=np.uint8)
        YOffset = (TargetHeight - NewHeight) // 2
        XOffset = (TargetWidth - NewWidth) // 2
        PaddedImage[YOffset : YOffset + NewHeight, XOffset : XOffset + NewWidth] = ResizedImage
        return PaddedImage

    def _init_backend(self):
        """Initialize TensorRT(ENGINE) backend if possible.

        Priority:
        1) If `engine_path` exists: load it via ultralytics (TensorRT runtime)
        2) Else if `.pt` exists and `export_engine_if_missing=True`: export engine on-device
        3) Else: try running .pt directly (PyTorch) if ultralytics available
        """
        try:
            from ultralytics import YOLO  # type: ignore

            # Avoid task auto-guess warnings and ensure detect pipeline.
            task = "detect"

            # device selection: ultralytics accepts 'cpu', 'cuda:0', 0, etc.
            if self.device is None:
                # On Jetson we prefer GPU.
                self.device = 0

            # 1) Prefer engine
            import os

            if self.engine_path and os.path.exists(self.engine_path):
                if self.prefer_engine:
                    self._yolo = YOLO(self.engine_path, task=task)
                    self._backend = "ultralytics-tensorrt"
                    self._init_class_mapping()
                    self.logger.success(f"TunaIntelligence backend=tensorrt engine={self.engine_path}")
                    return
                else:
                    self.logger.info(
                        f"TensorRT engine exists but prefer_engine=False, skipping engine load: {self.engine_path}"
                    )

            # 2) Export engine if requested
            if self.export_engine_if_missing and self.model_path and os.path.exists(self.model_path):
                self.logger.info(
                    f"TensorRT engine not found, exporting: {self.model_path} -> {self.engine_path} (half={self.engine_half})"
                )
                y = YOLO(self.model_path, task=task)
                # Export will create engine near weights by default. We accept returned path.
                exported = y.export(
                    format="engine",
                    imgsz=640,
                    half=self.engine_half,
                    int8=False,  # user: ignore quantization for now
                    device=self.device,
                )
                # exported is a path-like (str) in most ultralytics versions
                if isinstance(exported, (str, bytes)):
                    self.engine_path = str(exported)
                # load engine
                self._yolo = YOLO(self.engine_path, task=task)
                self._backend = "ultralytics-tensorrt"
                self._init_class_mapping()
                self.logger.success(f"TunaIntelligence backend=tensorrt engine={self.engine_path}")
                return

            # 3) Fallback to pytorch .pt
            if self.model_path and os.path.exists(self.model_path):
                self._yolo = YOLO(self.model_path, task=task)
                self._backend = "ultralytics-pytorch"
                self._init_class_mapping()
                self.logger.warning(
                    f"TunaIntelligence backend=pytorch model={self.model_path} (TensorRT engine missing: {self.engine_path})"
                )
                return

            raise FileNotFoundError(f"Model not found: pt={self.model_path}, engine={self.engine_path}")
        except Exception as e:
            self.logger.warning(f"Ultralytics/TensorRT backend not available: {e}")

        raise ImportError(
            "No available backend for TunaIntelligence. "
            "On Jetson: install torch+ultralytics and ensure TensorRT is available."
        )

    def _init_class_mapping(self):
        """Initialize class-id mapping from model names.

        Expected custom model names (case-insensitive):
            Ball, BlueGoal, YellowGoal, Chassis

        Fallback to (ball=0, chassis=3) if names are unavailable.
        """
        try:
            if self._yolo is None:
                return
            names = getattr(self._yolo, "names", None)
            if not isinstance(names, dict):
                return
            self._names = names

            inv = {str(v).lower(): int(k) for k, v in names.items()}
            # support common variants
            for key in ("ball", "sports ball"):
                if key in inv:
                    self._cls_ball = inv[key]
                    break
            for key in ("chassis", "car"):
                if key in inv:
                    self._cls_chassis = inv[key]
                    break

            # goals
            for key in ("bluegoal", "blue goal"):
                if key in inv:
                    self._cls_blue_goal = inv[key]
                    break
            for key in ("yellowgoal", "yellow goal"):
                if key in inv:
                    self._cls_yellow_goal = inv[key]
                    break

            self.logger.info(
                "TunaIntelligence class map: "
                f"ball={self._cls_ball} chassis={self._cls_chassis} "
                f"blue_goal={self._cls_blue_goal} yellow_goal={self._cls_yellow_goal} names={self._names}"
            )
        except Exception as e:
            self.logger.warning(f"TunaIntelligence: failed to init class mapping: {e}")

    # ------------------------- threads -------------------------
    def ModelPreProcess(self):
        """Collect latest frames and push to queue."""
        while not self._stop_event.is_set():
            frames_640: List[Optional[np.ndarray]] = [None, None, None, None]
            for i in range(4):
                Frame = self.Frames[i] if i < len(self.Frames) else None
                if Frame is None:
                    continue
                Frame = self.Resize(Frame, (640, 640))
                frames_640[i] = Frame

            self.PreProcessTF += 1
            # Non-blocking put (avoid deadlock at shutdown)
            try:
                if self.YOLOQueue.full():
                    _ = self.YOLOQueue.get_nowait()
                self.YOLOQueue.put_nowait(frames_640)
            except Exception:
                # queue full race, just drop this frame
                pass
            time.sleep(0.005)

    def _infer_ultralytics(self, imgs: List[np.ndarray]) -> List[List[_Det]]:
        """Run ultralytics YOLO on a list of images.

        Returns: list (per image) of detections.
        """
        assert self._yolo is not None
        # imgs: list of BGR numpy arrays
        # ultralytics returns list[Results]
        # NOTE:
        # - If engine was exported with batch=1, TensorRT will error on a 4-image list.
        # - If engine supports batch>=len(imgs), we should batch to improve throughput.
        if self._backend == "ultralytics-tensorrt" and len(imgs) > 1:
            try:
                results = self._yolo.predict(
                    imgs,
                    verbose=False,
                    conf=min(self.conf_ball, self.conf_chassis),
                    device=self.device,
                    imgsz=640,
                )
            except Exception as e:
                # fallback to image-by-image
                self.logger.warning(f"TRT batch predict failed, fallback to per-image: {e}")
                results = []
                for im in imgs:
                    r1 = self._yolo.predict(
                        im,
                        verbose=False,
                        conf=min(self.conf_ball, self.conf_chassis),
                        device=self.device,
                        imgsz=640,
                    )
                    results.extend(list(r1))
        else:
            results = self._yolo.predict(
                imgs,
                verbose=False,
                conf=min(self.conf_ball, self.conf_chassis),
                device=self.device,
                imgsz=640,
            )
        dets_by_img: List[List[_Det]] = []
        for r in results:
            dets: List[_Det] = []
            if r.boxes is None or len(r.boxes) == 0:
                dets_by_img.append(dets)
                continue

            # Boxes are xyxy in original image scale (here 640x640)
            b = r.boxes
            xyxy = b.xyxy.detach().cpu().numpy()
            conf = b.conf.detach().cpu().numpy()
            cls = b.cls.detach().cpu().numpy().astype(int)
            for (x1, y1, x2, y2), c, k in zip(xyxy, conf, cls):
                dets.append(_Det(cls=int(k), conf=float(c), xyxy=(float(x1), float(y1), float(x2), float(y2))))
            dets_by_img.append(dets)
        return dets_by_img

    def ModelInfer(self):
        FrameCount = 0
        LastTime = time.time()
        while not self._stop_event.is_set():
            FrameCount += 1
            CurrentTime = time.time()
            if Settings.Debug.FullLog and (CurrentTime - LastTime >= 1.0):
                self.logger.debug(f"TunaIntelligence FPS: {FrameCount}")
                FrameCount = 0
                LastTime = CurrentTime

            try:
                frames_640: List[Optional[np.ndarray]] = self.YOLOQueue.get(timeout=0.1)
            except Exception:
                continue
            if self._stop_event.is_set():
                break
            self.InferTF += 1

            # Prepare batch list only for available frames; keep mapping
            imgs: List[np.ndarray] = []
            cam_map: List[int] = []
            for cam_i, f in enumerate(frames_640):
                if f is None:
                    continue
                imgs.append(f)
                cam_map.append(cam_i)

            # Defaults: 4 dummy outputs (keeps Vision.py safe)
            ball_out = [[i, 0, 0, 0, 0, 0.0] for i in range(4)]
            chassis_out = [[i, 0, 0, 0, 0, 0.0] for i in range(4)]
            ball_bbox_out = [[i, 0, 0, 0, 0, 0.0] for i in range(4)]  # [cam,x1,y1,x2,y2,conf]
            chassis_bbox_out = [[i, 0, 0, 0, 0, 0.0] for i in range(4)]  # [cam,x1,y1,x2,y2,conf]
            # goals: [cam, cls, cx, bottom_y, w, h, conf]
            goal_out = [[i, -1, 0, 0, 0, 0, 0.0] for i in range(4)]

            if imgs:
                try:
                    dets_by_img = self._infer_ultralytics(imgs)
                except Exception as e:
                    self.logger.error(f"Inference error: {e}")
                    time.sleep(0.05)
                    continue

                for local_idx, dets in enumerate(dets_by_img):
                    cam_i = cam_map[local_idx]

                    # Custom model classes by default:
                    # 0 Ball, 1 BlueGoal, 2 YellowGoal, 3 Chassis
                    # (mapping is initialized from model.names when available)
                    if not dets:
                        continue

                    # Sort by confidence
                    dets_sorted = sorted(dets, key=lambda d: d.conf, reverse=True)

                    # Choose chassis
                    chassis_candidate = next(
                        (d for d in dets_sorted if d.cls == self._cls_chassis and d.conf >= self.conf_chassis),
                        None,
                    )
                    if chassis_candidate is not None:
                        x1, y1, x2, y2 = chassis_candidate.xyxy
                        chassis_bbox_out[cam_i] = [cam_i, int(x1), int(y1), int(x2), int(y2), float(chassis_candidate.conf)]
                        cx = int((x1 + x2) / 2)
                        bottom_y = int(y2)
                        w = int(x2 - x1)
                        h = int(y2 - y1)
                        chassis_out[cam_i] = [cam_i, cx, bottom_y, w, h, float(chassis_candidate.conf)]

                    # Choose ball
                    ball_candidate = next(
                        (d for d in dets_sorted if d.cls == self._cls_ball and d.conf >= self.conf_ball),
                        None,
                    )
                    if ball_candidate is not None:
                        x1, y1, x2, y2 = ball_candidate.xyxy
                        ball_bbox_out[cam_i] = [cam_i, int(x1), int(y1), int(x2), int(y2), float(ball_candidate.conf)]
                        # Keep Arisu's small biases for ball.
                        ball_y_min = int(y1) - 80 + 15
                        ball_x_min = int(x1) + 15
                        ball_y_max = int(y2) - 80 - 15
                        ball_x_max = int(x2) - 15
                        bottom_y = int(ball_y_max)
                        w = int(ball_x_max - ball_x_min)
                        h = int(ball_y_max - ball_y_min)
                        cx = int((ball_x_min + ball_x_max) / 2)
                        ball_out[cam_i] = [cam_i, cx, bottom_y, w, h, float(ball_candidate.conf)]

                    # Choose goal (prefer the more confident between blue/yellow)
                    goal_candidate = next(
                        (
                            d
                            for d in dets_sorted
                            if d.cls in (self._cls_blue_goal, self._cls_yellow_goal)
                            and d.conf >= 0.40
                        ),
                        None,
                    )
                    if goal_candidate is not None:
                        x1, y1, x2, y2 = goal_candidate.xyxy
                        cx = int((x1 + x2) / 2)
                        bottom_y = int(y2)
                        w = int(x2 - x1)
                        h = int(y2 - y1)
                        goal_out[cam_i] = [cam_i, int(goal_candidate.cls), cx, bottom_y, w, h, float(goal_candidate.conf)]

            # Queues are size=1, keep latest
            if self.ChassisQueue.full():
                try:
                    _ = self.ChassisQueue.get_nowait()
                except Exception:
                    pass
            if self.BallQueue.full():
                try:
                    _ = self.BallQueue.get_nowait()
                except Exception:
                    pass
            if self.BallBBoxQueue.full():
                try:
                    _ = self.BallBBoxQueue.get_nowait()
                except Exception:
                    pass
            if self.ChassisBBoxQueue.full():
                try:
                    _ = self.ChassisBBoxQueue.get_nowait()
                except Exception:
                    pass
            if self.GoalQueue.full():
                try:
                    _ = self.GoalQueue.get_nowait()
                except Exception:
                    pass

            self.ChassisQueue.put(chassis_out)
            self.BallQueue.put(ball_out)
            self.BallBBoxQueue.put(ball_bbox_out)
            self.ChassisBBoxQueue.put(chassis_bbox_out)
            self.GoalQueue.put(goal_out)

            time.sleep(0.001)


__all__ = ["TunaIntelligence"]
