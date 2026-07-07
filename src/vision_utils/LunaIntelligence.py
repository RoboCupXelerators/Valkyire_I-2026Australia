"""SimYoloVision

模拟器环境下的 YOLO 视觉检测方案。

设计目标：
1) 通过模拟器获取摄像头画面（复用 XeleratorVision 的帧获取逻辑）
2) 推理 YOLO 模型（复用 TunaIntelligence 的推理逻辑）
3) 输出 BBox 结果

使用场景：
- 在 Unity 模拟器中运行时，需要真实的视觉检测能力
- 不依赖真实摄像头，而是从模拟器的 MJPEG 服务器获取画面
- 适用于需要在模拟环境中测试视觉算法的场景

配置方式：
- 在 config.json 中设置 "Rec": "SY" 或 "SIMYOLO"
- 或直接实例化 SimYoloVision 类使用
"""

from __future__ import annotations

BALL_NOT_FOUND = [0xFFF, 0xFFF]  # matches LunaPre.strategy_tools.BALL_NOT_FOUND

import os
import threading
import queue
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, List, Optional, Tuple

try:
    import cv2  # type: ignore
    import numpy as np
except Exception as e:
    raise ImportError(f"SimYoloVision requires OpenCV and NumPy: {e}")

try:
    from ReasonData import Settings, logger  # type: ignore
except Exception:  # pragma: no cover
    try:
        from src.ReasonData import Settings, logger  # type: ignore
    except Exception:  # pragma: no cover
        from .merged_settings import Settings  # type: ignore
        import logging
        logger = logging.getLogger(__name__)


@dataclass
class _Det:
    """Detection result container."""
    cls: int
    conf: float
    xyxy: Tuple[float, float, float, float]  # x1,y1,x2,y2 in 640x640 space


class SimYoloVision:
    """模拟器环境下的 YOLO 视觉检测方案。

    结合了：
    - XeleratorVision 的模拟器摄像头获取能力
    - TunaIntelligence 的 YOLO 推理能力

    输出格式（与 TunaIntelligence 兼容）：
        BallQueue:     [[cam, center_x, bottom_y, w, h, conf], ...]
        ChassisQueue:  [[cam, center_x, bottom_y, w, h, conf], ...]
        BallBBoxQueue: [[cam, x1, y1, x2, y2, conf], ...]
        ChassisBBoxQueue: [[cam, x1, y1, x2, y2, conf], ...]
        GoalQueue:     [[cam, cls, center_x, bottom_y, w, h, conf], ...]
    """

    def __init__(
        self,
        model_path: Optional[str] = None,
        engine_path: Optional[str] = None,
        device: Optional[str] = None,
        prefer_engine: bool = True,
        conf_ball: float = 0.30,
        conf_chassis: float = 0.50,
        robot_id: Optional[int] = None,
        camera_slots: Optional[List[int]] = None,
        server_port: Optional[int] = None,
        host: Optional[str] = None,
        start_threads: bool = True,
    ):
        self.logger = logger

        # Configuration
        self.conf_ball = float(conf_ball)
        self.conf_chassis = float(conf_chassis)

        # Performance counters
        self.PreProcessTF = 0
        self.InferTF = 0
        self.ReadCamTF = 0

        # Camera configuration
        self._robot_id = robot_id
        self._camera_slots = camera_slots or self._get_default_camera_slots()
        self._server_port = server_port or int(os.environ.get("XEL_CAM_SERVER_PORT", "20801"))
        self._host = host or os.environ.get("XEL_CAM_HOST", "127.0.0.1")

        # Frames storage (4 camera slots)
        self.Frames: List[Optional[np.ndarray]] = [None, None, None, None]
        self.CamPorts = self._camera_slots

        # Queues for detection results (compatible with TunaIntelligence)
        self.YOLOQueue = queue.Queue(maxsize=1)
        self.BallQueue = queue.Queue(maxsize=1)
        self.ChassisQueue = queue.Queue(maxsize=1)
        self.GoalQueue = queue.Queue(maxsize=1)
        self.BallBBoxQueue = queue.Queue(maxsize=1)
        self.ChassisBBoxQueue = queue.Queue(maxsize=1)

        # Model paths (config-driven, same logic as TunaIntelligence)
        # Priority:
        #   1) explicit args: model_path/engine_path
        #   2) config: Settings.Vision.ModelName (统一模型名称，自动拼接后缀)
        #   3) legacy config: Settings.Vision.ModelPath / EnginePath (向后兼容)
        #   4) fallback: repo default yolov8s
        vision_cfg = getattr(Settings, "Vision", object())
        model_name = getattr(vision_cfg, "ModelName", None)
        cfg_model_legacy = getattr(vision_cfg, "ModelPath", None)
        cfg_engine_legacy = getattr(vision_cfg, "EnginePath", None)
        
        # Get MODEL_DIR
        try:
            from .PreProcess import MODEL_DIR
        except Exception:
            MODEL_DIR = Path(__file__).parent / "models"
        
        if model_name:
            cfg_model = str(MODEL_DIR / f"{model_name}.pt")
            cfg_engine = str(MODEL_DIR / f"{model_name}.engine")
        elif cfg_model_legacy:
            cfg_model = str(MODEL_DIR) + cfg_model_legacy
            cfg_engine = str(MODEL_DIR) + (cfg_engine_legacy or cfg_model_legacy.replace(".pt", ".engine"))
        else:
            cfg_model = str(MODEL_DIR / "yolov8s.pt")
            cfg_engine = str(MODEL_DIR / "yolov8s.engine")

        self.model_path = model_path or cfg_model
        self.engine_path = engine_path or cfg_engine
        self.device = device
        self.prefer_engine = prefer_engine

        # YOLO model
        self._yolo = None
        self._backend = None
        self._names = None

        # Class mapping (default: custom 4-class model)
        self._cls_ball = 0
        self._cls_chassis = 3
        self._cls_blue_goal = 1
        self._cls_yellow_goal = 2

        # Thread control
        self._stop_event = threading.Event()
        self._start_threads = start_threads

        # Camera capture objects
        self._cameras: List[Any] = [None, None, None, None]

        # Initialize model
        self._init_backend()

        # Start threads
        if self._start_threads:
            self._frame_thread = threading.Thread(target=self._read_frames_loop, daemon=True)
            self._frame_thread.start()

            self._preprocess_thread = threading.Thread(target=self._preprocess_loop, daemon=True)
            self._preprocess_thread.start()

            self._infer_thread = threading.Thread(target=self._infer_loop, daemon=True)
            self._infer_thread.start()

    # ==================== Camera Helpers (from XeleratorVision) ====================

    @staticmethod
    def _get_default_camera_slots() -> List[int]:
        """Get default camera slots from config."""
        try:
            cam_ports = list(getattr(Settings.Vision, "CamPorts", [0, 2, 4, 6]))
            if cam_ports:
                return cam_ports
        except Exception:
            pass
        return [0, 2, 4, 6]

    @staticmethod
    def _parse_int_list(text: str) -> List[int]:
        """Parse comma-separated integers."""
        out: List[int] = []
        for part in (text or "").split(","):
            part = part.strip()
            if not part:
                continue
            try:
                out.append(int(part))
            except ValueError:
                continue
        return out

    @staticmethod
    def build_stream_id(robot_id: int, camera_slot: int) -> str:
        """Build stream ID for Unity camera server."""
        return f"robot{int(robot_id)}_{int(camera_slot)}"

    @staticmethod
    def build_camera_url(robot_id: int, camera_slot: int, server_port: int = 20801, host: str = "127.0.0.1") -> str:
        """Build MJPEG stream URL."""
        stream_id = SimYoloVision.build_stream_id(robot_id, camera_slot)
        return f"http://{host}:{int(server_port)}/stream?id={stream_id}"

    def _get_robot_id(self) -> int:
        """Get robot ID from config or environment."""
        if self._robot_id is not None:
            return self._robot_id
        try:
            return int(getattr(Settings.RoboInfo, "number", 1))
        except Exception:
            return 1

    def _build_camera_urls(self) -> List[str]:
        """Build all camera URLs."""
        robot_id = self._get_robot_id()
        return [
            self.build_camera_url(robot_id, slot, self._server_port, self._host)
            for slot in self._camera_slots
        ]

    # ==================== Model Initialization (from TunaIntelligence) ====================

    def _get_model_dir(self) -> str:
        """Get model directory."""
        return str(os.path.dirname(__file__) + "/models")

    def _init_backend(self):
        """Initialize YOLO backend (TensorRT or PyTorch)."""
        try:
            from ultralytics import YOLO  # type: ignore
        except ImportError as e:
            raise ImportError(
                "SimYoloVision requires ultralytics. "
                "Install with: pip install ultralytics"
            ) from e

        # Determine model paths
        model_dir = self._get_model_dir()

        if self.model_path is None:
            cfg_model = getattr(getattr(Settings, "Vision", object()), "ModelPath", None)
            if cfg_model:
                self.model_path = str(model_dir) + cfg_model
            else:
                self.model_path = os.path.join(model_dir, "yolov8nForX5.pt")

        if self.engine_path is None:
            cfg_engine = getattr(getattr(Settings, "Vision", object()), "EnginePath", None)
            if cfg_engine:
                self.engine_path = str(model_dir) + cfg_engine
            else:
                self.engine_path = os.path.join(model_dir, "yolov8nForX5.engine")

        if self.device is None:
            self.device = 0  # Default to GPU

        # If user explicitly requests CPU, disable engine preference
        if isinstance(self.device, str) and self.device.lower().startswith("cpu"):
            self.prefer_engine = False

        task = "detect"

        # 1) Try TensorRT engine first
        if self.engine_path and os.path.exists(self.engine_path) and self.prefer_engine:
            try:
                self._yolo = YOLO(self.engine_path, task=task)
                self._backend = "ultralytics-tensorrt"
                self._init_class_mapping()
                self.logger.info(f"SimYoloVision backend=tensorrt engine={self.engine_path}")
                return
            except Exception as e:
                self.logger.warning(f"Failed to load TensorRT engine: {e}")

        # 2) Fallback to PyTorch .pt
        if self.model_path and os.path.exists(self.model_path):
            try:
                self._yolo = YOLO(self.model_path, task=task)
                self._backend = "ultralytics-pytorch"
                self._init_class_mapping()
                self.logger.info(f"SimYoloVision backend=pytorch model={self.model_path}")
                return
            except Exception as e:
                self.logger.warning(f"Failed to load PyTorch model: {e}")

        raise FileNotFoundError(
            f"No valid YOLO model found. pt={self.model_path}, engine={self.engine_path}"
        )

    def _init_class_mapping(self):
        """Initialize class ID mapping from model names."""
        if self._yolo is None:
            return

        names = getattr(self._yolo, "names", None)
        if not isinstance(names, dict):
            return

        self._names = names
        inv = {str(v).lower(): int(k) for k, v in names.items()}

        # Ball class
        for key in ("ball", "sports ball"):
            if key in inv:
                self._cls_ball = inv[key]
                break

        # Chassis class
        for key in ("chassis", "car"):
            if key in inv:
                self._cls_chassis = inv[key]
                break

        # Goal classes
        for key in ("bluegoal", "blue goal"):
            if key in inv:
                self._cls_blue_goal = inv[key]
                break
        for key in ("yellowgoal", "yellow goal"):
            if key in inv:
                self._cls_yellow_goal = inv[key]
                break

        self.logger.info(
            f"SimYoloVision class map: ball={self._cls_ball} chassis={self._cls_chassis} "
            f"blue_goal={self._cls_blue_goal} yellow_goal={self._cls_yellow_goal}"
        )

    # ==================== Frame Processing ====================

    def _resize(self, frame: np.ndarray, target_size: Tuple[int, int] = (640, 640)) -> np.ndarray:
        """Letterbox resize to target size with padding."""
        height, width = frame.shape[:2]
        target_height, target_width = target_size

        scale = min(target_width / width, target_height / height)
        new_width = int(width * scale)
        new_height = int(height * scale)

        resized = cv2.resize(frame, (new_width, new_height))
        padded = np.zeros((target_height, target_width, 3), dtype=np.uint8)

        y_offset = (target_height - new_height) // 2
        x_offset = (target_width - new_width) // 2
        padded[y_offset:y_offset + new_height, x_offset:x_offset + new_width] = resized

        return padded

    # ==================== Background Threads ====================

    def _read_frames_loop(self):
        """Background thread to read frames from Unity MJPEG server using requests library."""
        import requests
        
        urls = self._build_camera_urls()
        reconnect_delay = 5.0
        last_error_log = [0.0, 0.0, 0.0, 0.0]
        
        self.logger.info(f"SimYoloVision connecting to cameras: {urls}")
        
        # Create session for each camera
        sessions = [None, None, None, None]
        responses = [None, None, None, None]
        
        while not self._stop_event.is_set():
            for i, url in enumerate(urls):
                try:
                    # Initialize session if needed
                    if sessions[i] is None:
                        sessions[i] = requests.Session()
                        self.logger.info(f"SimYoloVision camera {i} session created: {url}")
                    
                    # Start stream if not active
                    if responses[i] is None:
                        try:
                            responses[i] = sessions[i].get(url, stream=True, timeout=5.0)
                            if responses[i].status_code == 200:
                                self.logger.info(f"SimYoloVision camera {i} stream started: {url}")
                            else:
                                self.logger.warning(f"SimYoloVision camera {i} stream failed: status={responses[i].status_code}")
                                responses[i] = None
                        except Exception as e:
                            now = time.time()
                            if now - last_error_log[i] > reconnect_delay:
                                self.logger.warning(f"SimYoloVision camera {i} connection error: {e}")
                                last_error_log[i] = now
                            responses[i] = None
                    
                    # Read frames from stream
                    if responses[i] is not None:
                        try:
                            # Parse MJPEG stream - look for JPEG boundaries
                            for chunk in responses[i].iter_content(chunk_size=8192):
                                if self._stop_event.is_set():
                                    break
                                
                                # Look for JPEG start marker (0xFFD8)
                                chunk_data = chunk
                                if b'\xff\xd8' in chunk_data:
                                    # Found JPEG start, try to extract frame
                                    # Find JPEG end marker (0xFFD9)
                                    jpeg_start = chunk_data.find(b'\xff\xd8')
                                    jpeg_end = chunk_data.find(b'\xff\xd9', jpeg_start)
                                    
                                    if jpeg_end > jpeg_start:
                                        # Complete JPEG in this chunk
                                        jpeg_data = chunk_data[jpeg_start:jpeg_end+2]
                                        frame = cv2.imdecode(np.frombuffer(jpeg_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                                        if frame is not None:
                                            self.Frames[i] = frame
                                            self.ReadCamTF += 1
                                            break
                                    # else: JPEG spans multiple chunks - skip for now
                        except requests.exceptions.RequestException as e:
                            now = time.time()
                            if now - last_error_log[i] > reconnect_delay:
                                self.logger.warning(f"SimYoloVision camera {i} read error: {e}")
                                last_error_log[i] = now
                            try:
                                responses[i].close()
                            except Exception:
                                pass
                            responses[i] = None
                except Exception as e:
                    now = time.time()
                    if now - last_error_log[i] > reconnect_delay:
                        self.logger.warning(f"SimYoloVision camera {i} error: {e}")
                        last_error_log[i] = now
                    if responses[i] is not None:
                        try:
                            responses[i].close()
                        except Exception:
                            pass
                        responses[i] = None

            time.sleep(0.03)  # ~30 FPS

        # Cleanup
        for resp in responses:
            if resp is not None:
                try:
                    resp.close()
                except Exception:
                    pass
        for sess in sessions:
            if sess is not None:
                try:
                    sess.close()
                except Exception:
                    pass

    def _preprocess_loop(self):
        """Background thread to preprocess frames for inference."""
        while not self._stop_event.is_set():
            frames_640: List[Optional[np.ndarray]] = [None, None, None, None]

            for i in range(4):
                frame = self.Frames[i] if i < len(self.Frames) else None
                if frame is None:
                    continue
                frames_640[i] = self._resize(frame, (640, 640))

            self.PreProcessTF += 1

            # Non-blocking put
            try:
                if self.YOLOQueue.full():
                    _ = self.YOLOQueue.get_nowait()
                self.YOLOQueue.put_nowait(frames_640)
            except Exception:
                pass

            time.sleep(0.005)

    def _infer_ultralytics(self, imgs: List[np.ndarray]) -> List[List[_Det]]:
        """Run YOLO inference on a batch of images."""
        assert self._yolo is not None

        try:
            results = self._yolo.predict(
                imgs,
                verbose=False,
                conf=min(self.conf_ball, self.conf_chassis),
                device=self.device,
                imgsz=640,
            )
        except Exception as e:
            # Fallback to per-image inference
            self.logger.warning(f"Batch inference failed, fallback to per-image: {e}")
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

        dets_by_img: List[List[_Det]] = []
        for r in results:
            dets: List[_Det] = []
            if r.boxes is None or len(r.boxes) == 0:
                dets_by_img.append(dets)
                continue

            b = r.boxes
            xyxy = b.xyxy.detach().cpu().numpy()
            conf = b.conf.detach().cpu().numpy()
            cls = b.cls.detach().cpu().numpy().astype(int)

            for (x1, y1, x2, y2), c, k in zip(xyxy, conf, cls):
                dets.append(_Det(cls=int(k), conf=float(c), xyxy=(float(x1), float(y1), float(x2), float(y2))))

            dets_by_img.append(dets)

        return dets_by_img

    def _infer_loop(self):
        """Background thread to run YOLO inference."""
        frame_count = 0
        last_time = time.time()

        while not self._stop_event.is_set():
            frame_count += 1
            current_time = time.time()

            # Log FPS periodically
            if current_time - last_time >= 1.0:
                if getattr(getattr(Settings, "Debug", object()), "FullLog", False):
                    self.logger.debug(f"SimYoloVision FPS: {frame_count}")
                frame_count = 0
                last_time = current_time

            try:
                frames_640: List[Optional[np.ndarray]] = self.YOLOQueue.get(timeout=0.1)
            except Exception:
                continue

            if self._stop_event.is_set():
                break

            self.InferTF += 1

            # Prepare batch
            imgs: List[np.ndarray] = []
            cam_map: List[int] = []
            for cam_i, f in enumerate(frames_640):
                if f is None:
                    continue
                imgs.append(f)
                cam_map.append(cam_i)

            # Default outputs (4 dummy entries)
            ball_out = [[i, 0, 0, 0, 0, 0.0] for i in range(4)]
            chassis_out = [[i, 0, 0, 0, 0, 0.0] for i in range(4)]
            ball_bbox_out = [[i, 0, 0, 0, 0, 0.0] for i in range(4)]
            chassis_bbox_out = [[i, 0, 0, 0, 0, 0.0] for i in range(4)]
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

                    if not dets:
                        continue

                    # Sort by confidence
                    dets_sorted = sorted(dets, key=lambda d: d.conf, reverse=True)

                    # Chassis detection
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

                    # Ball detection
                    ball_candidate = next(
                        (d for d in dets_sorted if d.cls == self._cls_ball and d.conf >= self.conf_ball),
                        None,
                    )
                    if ball_candidate is not None:
                        x1, y1, x2, y2 = ball_candidate.xyxy
                        ball_bbox_out[cam_i] = [cam_i, int(x1), int(y1), int(x2), int(y2), float(ball_candidate.conf)]
                        # Apply small biases (same as TunaIntelligence)
                        ball_y_min = int(y1) - 80 + 15
                        ball_x_min = int(x1) + 15
                        ball_y_max = int(y2) - 80 - 15
                        ball_x_max = int(x2) - 15
                        bottom_y = int(ball_y_max)
                        w = int(ball_x_max - ball_x_min)
                        h = int(ball_y_max - ball_y_min)
                        cx = int((ball_x_min + ball_x_max) / 2)
                        ball_out[cam_i] = [cam_i, cx, bottom_y, w, h, float(ball_candidate.conf)]

                    # Goal detection
                    goal_candidate = next(
                        (d for d in dets_sorted if d.cls in (self._cls_blue_goal, self._cls_yellow_goal) and d.conf >= 0.40),
                        None,
                    )
                    if goal_candidate is not None:
                        x1, y1, x2, y2 = goal_candidate.xyxy
                        cx = int((x1 + x2) / 2)
                        bottom_y = int(y2)
                        w = int(x2 - x1)
                        h = int(y2 - y1)
                        goal_out[cam_i] = [cam_i, int(goal_candidate.cls), cx, bottom_y, w, h, float(goal_candidate.conf)]

            # Update queues (non-blocking)
            for q, data in [
                (self.ChassisQueue, chassis_out),
                (self.BallQueue, ball_out),
                (self.BallBBoxQueue, ball_bbox_out),
                (self.ChassisBBoxQueue, chassis_bbox_out),
                (self.GoalQueue, goal_out),
            ]:
                try:
                    if q.full():
                        _ = q.get_nowait()
                    q.put_nowait(data)
                except Exception:
                    pass

            time.sleep(0.001)

    # ==================== Public API (compatible with Vision.py) ====================

    def get_ball_pos(self) -> List[float]:
        """Get ball position from the best detection."""
        try:
            ball_data = self.BallQueue.get_nowait()
            # Find best detection by confidence
            best = max(ball_data, key=lambda x: x[5] if x else 0)
            if best and best[5] > 0:
                # Return center position (cx, bottom_y)
                return [float(best[1]), float(best[2])]
        except Exception:
            pass
        return list(BALL_NOT_FOUND)

    def get_chassis_pos(self) -> List[List[float]]:
        """Get chassis detections."""
        try:
            chassis_data = self.ChassisQueue.get_nowait()
            return [c for c in chassis_data if c and c[5] > 0]
        except Exception:
            pass
        return []

    def get_goals_pos(self) -> dict:
        """Get goal detections."""
        try:
            goal_data = self.GoalQueue.get_nowait()
            blue_goal = None
            yellow_goal = None

            for g in goal_data:
                if g is None or g[5] <= 0:
                    continue
                if g[1] == self._cls_blue_goal:
                    if blue_goal is None or g[6] > blue_goal[2]:
                        blue_goal = [float(g[2]), float(g[3]), float(g[6])]
                elif g[1] == self._cls_yellow_goal:
                    if yellow_goal is None or g[6] > yellow_goal[2]:
                        yellow_goal = [float(g[2]), float(g[3]), float(g[6])]

            return {"blue_goal": blue_goal, "yellow_goal": yellow_goal}
        except Exception:
            pass
        return {"blue_goal": None, "yellow_goal": None}

    def get_vision_data(self) -> dict:
        """Get all vision data in a single dict (compatible with LunaPre)."""
        return {
            "ball_pos": self.get_ball_pos(),
            "chassis_pos": self.get_chassis_pos(),
            "blue_goal": self.get_goals_pos().get("blue_goal"),
            "yellow_goal": self.get_goals_pos().get("yellow_goal"),
        }

    def GetBallBBoxes(self) -> List[Optional[tuple]]:
        """Get ball bounding boxes for all cameras.

        Returns:
            List of (cam, x1, y1, x2, y2, conf) or None for each camera.
        """
        try:
            bbox_data = self.BallBBoxQueue.get_nowait()
            result = []
            for b in bbox_data:
                if b and b[5] > 0:
                    result.append(tuple(b))
                else:
                    result.append(None)
            return result
        except Exception:
            pass
        return [None, None, None, None]

    def GetChassisBBoxes(self) -> List[Optional[tuple]]:
        """Get chassis bounding boxes for all cameras."""
        try:
            bbox_data = self.ChassisBBoxQueue.get_nowait()
            result = []
            for b in bbox_data:
                if b and b[5] > 0:
                    result.append(tuple(b))
                else:
                    result.append(None)
            return result
        except Exception:
            pass
        return [None, None, None, None]

    def GetGoalBBoxes(self) -> List[Optional[tuple]]:
        """Get goal bounding boxes for all cameras."""
        try:
            goal_data = self.GoalQueue.get_nowait()
            result = []
            for g in goal_data:
                if g and g[6] > 0:
                    result.append(tuple(g))
                else:
                    result.append(None)
            return result
        except Exception:
            pass
        return [None, None, None, None]

    # ==================== Lifecycle ====================

    def close(self, join_timeout: float = 1.0):
        """Stop all background threads and release resources."""
        self._stop_event.set()

        # Unblock queues
        for q in [self.YOLOQueue, self.BallQueue, self.ChassisQueue, self.GoalQueue,
                  self.BallBBoxQueue, self.ChassisBBoxQueue]:
            try:
                while True:
                    _ = q.get_nowait()
            except Exception:
                pass

        # Wait for threads
        for t in [getattr(self, "_frame_thread", None),
                  getattr(self, "_preprocess_thread", None),
                  getattr(self, "_infer_thread", None)]:
            if t is not None and t.is_alive():
                try:
                    t.join(timeout=join_timeout)
                except Exception:
                    pass

        # Release cameras
        for cap in self._cameras:
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass

    def __del__(self):
        try:
            self.close(join_timeout=0.2)
        except Exception:
            pass


__all__ = ["SimYoloVision"]