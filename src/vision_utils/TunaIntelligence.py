"""TunaIntelligence

Jetson Nano (super) oriented inference backend.

This module mirrors the thread/queue interface of `ArisuIntelligence` so that
`src/Vision.py` can consume the same `ChassisQueue` and `BallQueue` outputs.

Model path (given): `src/vision_utils/models/yolov8s.pt`

Recommended backend on Jetson
-----------------------------
TensorRT engine exported from Ultralytics:

1) Export (on Jetson) .pt -> .engine (FP16 by default; INT8 requires calibration data)
2) Load .engine with Ultralytics YOLO, which will dispatch to TensorRT runtime.

Notes
-----
- This repository does not pin `torch/ultralytics` because Jetson requires
  JetPack-compatible PyTorch wheels.
- Class mapping (as per user):
    class 0 = ball
    class 1 = chassis
    class 2/3 = ignore
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
        conf_ball: float = 0.40,
        conf_chassis: float = 0.50,
        init_cams: bool = True,
        export_engine_if_missing: bool = True,
        engine_half: bool = True,
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

        # Init cameras/record threads from base class (can be disabled for tests)
        super().__init__(init_cams=init_cams)

        self.model_path = model_path or str(MODEL_DIR / "yolov8s.pt")
        self.engine_path = engine_path or str(MODEL_DIR / "yolov8s.engine")
        self.device = device  # None => auto
        self.export_engine_if_missing = bool(export_engine_if_missing)
        self.engine_half = bool(engine_half)

        self._backend = None
        self._yolo = None  # ultralytics.YOLO

        self._init_backend()

        self.ModelPreProcessThread = threading.Thread(target=self.ModelPreProcess)
        self.ModelPreProcessThread.daemon = True
        self.ModelPreProcessThread.start()

        self.ModelInferThread = threading.Thread(target=self.ModelInfer)
        self.ModelInferThread.daemon = True
        self.ModelInferThread.start()

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

            # device selection: ultralytics accepts 'cpu', 'cuda:0', 0, etc.
            if self.device is None:
                # On Jetson we prefer GPU.
                self.device = 0

            # 1) Prefer engine
            import os

            if self.engine_path and os.path.exists(self.engine_path):
                self._yolo = YOLO(self.engine_path)
                self._backend = "ultralytics-tensorrt"
                self.logger.success(f"TunaIntelligence backend=tensorrt engine={self.engine_path}")
                return

            # 2) Export engine if requested
            if self.export_engine_if_missing and self.model_path and os.path.exists(self.model_path):
                self.logger.info(
                    f"TensorRT engine not found, exporting: {self.model_path} -> {self.engine_path} (half={self.engine_half})"
                )
                y = YOLO(self.model_path)
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
                self._yolo = YOLO(self.engine_path)
                self._backend = "ultralytics-tensorrt"
                self.logger.success(f"TunaIntelligence backend=tensorrt engine={self.engine_path}")
                return

            # 3) Fallback to pytorch .pt
            if self.model_path and os.path.exists(self.model_path):
                self._yolo = YOLO(self.model_path)
                self._backend = "ultralytics-pytorch"
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

    # ------------------------- threads -------------------------
    def ModelPreProcess(self):
        """Collect latest frames and push to queue."""
        while True:
            frames_640: List[Optional[np.ndarray]] = [None, None, None, None]
            for i in range(4):
                Frame = self.Frames[i] if i < len(self.Frames) else None
                if Frame is None:
                    continue
                Frame = self.Resize(Frame, (640, 640))
                frames_640[i] = Frame

            if self.YOLOQueue.full():
                # drop old
                try:
                    _ = self.YOLOQueue.get_nowait()
                except Exception:
                    pass

            self.PreProcessTF += 1
            self.YOLOQueue.put(frames_640)
            time.sleep(0.005)

    def _infer_ultralytics(self, imgs: List[np.ndarray]) -> List[List[_Det]]:
        """Run ultralytics YOLO on a list of images.

        Returns: list (per image) of detections.
        """
        assert self._yolo is not None
        # imgs: list of BGR numpy arrays
        # ultralytics returns list[Results]
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
        while True:
            FrameCount += 1
            CurrentTime = time.time()
            if Settings.Debug.FullLog and (CurrentTime - LastTime >= 1.0):
                self.logger.debug(f"TunaIntelligence FPS: {FrameCount}")
                FrameCount = 0
                LastTime = CurrentTime

            frames_640: List[Optional[np.ndarray]] = self.YOLOQueue.get()
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

            if imgs:
                try:
                    dets_by_img = self._infer_ultralytics(imgs)
                except Exception as e:
                    self.logger.error(f"Inference error: {e}")
                    time.sleep(0.05)
                    continue

                for local_idx, dets in enumerate(dets_by_img):
                    cam_i = cam_map[local_idx]

                    # Class mapping (given by user):
                    # 0=ball, 1=chassis, 2/3 ignore
                    if not dets:
                        continue

                    # Sort by confidence
                    dets_sorted = sorted(dets, key=lambda d: d.conf, reverse=True)

                    # Choose chassis: best class==1
                    chassis_candidate = next((d for d in dets_sorted if d.cls == 1 and d.conf >= self.conf_chassis), None)
                    if chassis_candidate is not None:
                        x1, y1, x2, y2 = chassis_candidate.xyxy
                        cx = int((x1 + x2) / 2)
                        bottom_y = int(y2)
                        w = int(x2 - x1)
                        h = int(y2 - y1)
                        chassis_out[cam_i] = [cam_i, cx, bottom_y, w, h, float(chassis_candidate.conf)]

                    # Choose ball: best class==0
                    ball_candidate = next((d for d in dets_sorted if d.cls == 0 and d.conf >= self.conf_ball), None)
                    if ball_candidate is not None:
                        x1, y1, x2, y2 = ball_candidate.xyxy
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

            self.ChassisQueue.put(chassis_out)
            self.BallQueue.put(ball_out)

            time.sleep(0.001)


__all__ = ["TunaIntelligence"]
