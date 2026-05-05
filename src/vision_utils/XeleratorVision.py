"""XeleratorVision

给 SoccerSoftware 提供一个“视觉数据”统一接口。

设计目标：
1) 在 Unity 模拟器中：不依赖外部 Vision.py 子进程，直接从 XeleratorSimBits
   的状态包里拿到坐标（位姿/球相对坐标）。
2) 在真实机/后续版本：保留接口以接入 OpenCV 摄像头 + 本地推理 .pt 模型。

当前实现（SIM）：
- ball_pos: 机器人坐标系下的球相对坐标 [x, y]，单位 cm
           与 LunaPre.Positions.RobotBallPos 的语义一致。
- chassis_pos: 对方机器人检测框列表（暂时返回空列表）。
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Any

try:
    from ReasonData import Settings  # type: ignore
except Exception:  # pragma: no cover
    try:
        from src.ReasonData import Settings  # type: ignore
    except Exception:  # pragma: no cover
        Settings = None  # type: ignore


@dataclass
class VisionData:
    # relative ball position in robot frame (cm)
    ball_pos: list[float] | list[int]
    # list of detections, keep same semantic as existing vision server
    # each item typically: [x, y, w, h, conf]
    chassis_pos: list[list[float]]
    # goal detections, direct one-blue/one-yellow output
    blue_goal: list[float] | list[int] | None
    yellow_goal: list[float] | list[int] | None

    def as_dict(self) -> dict[str, Any]:
        return {
            "ball_pos": self.ball_pos,
            "chassis_pos": self.chassis_pos,
            "blue_goal": self.blue_goal,
            "yellow_goal": self.yellow_goal,
        }


class XeleratorVision:
    """Vision adapter.

    For simulator, you should pass `bits` = XeleratorSimBits instance.
    """

    def __init__(self, bits=None):
        self.bits = bits

        # reserved for future OpenCV/PT pipeline
        self._camera = None
        self._cameras: list[Any] = []
        self._camera_urls: list[str] = []
        self._model = None
        
        # Frames list for dashboard compatibility (like VisionPreUntil)
        self.Frames = [None, None, None, None]
        
        # Start background thread to pull frames from Unity MJPEG server
        self._frame_thread = None
        self._frame_running = False
        self._frame_stop_event = None
        try:
            import threading
            self._frame_stop_event = threading.Event()
            self._frame_running = True
            self._frame_thread = threading.Thread(target=self._pull_frames_loop, daemon=True)
            self._frame_thread.start()
        except Exception:
            pass
        
        # Simple logger fallback
        try:
            from ReasonData import logger  # type: ignore
        except Exception:
            import logging
            logger = logging.getLogger(__name__)

    # ===== Unity camera URL helpers =====
    @staticmethod
    def _parse_int_list(text: str) -> list[int]:
        out: list[int] = []
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
    def _camera_slot_defaults() -> list[int]:
        # Keep compatibility with existing vision config and also normalize to
        # the user-friendly 0/2/4/6 slot set.
        if Settings is not None:
            try:
                vv = Settings.VisionVals
                slots = [
                    int(getattr(vv, "Back", 0)),
                    int(getattr(vv, "Left", 2)),
                    int(getattr(vv, "Front", 4)),
                    int(getattr(vv, "Right", 6)),
                ]
                ordered: list[int] = []
                for s in slots:
                    if s not in ordered:
                        ordered.append(s)
                if ordered:
                    return ordered
            except Exception:
                pass
        return [0, 2, 4, 6]

    @staticmethod
    def build_stream_id(robot_id: int, camera_slot: int) -> str:
        return f"robot{int(robot_id)}_{int(camera_slot)}"

    @staticmethod
    def build_camera_url(robot_id: int, camera_slot: int, server_port: int = 20801, host: str = "127.0.0.1") -> str:
        stream_id = XeleratorVision.build_stream_id(robot_id, camera_slot)
        return f"http://{host}:{int(server_port)}/stream?id={stream_id}"

    def build_camera_urls(
        self,
        robot_id: int | None = None,
        server_port: int | None = None,
        camera_slots: list[int] | None = None,
        host: str | None = None,
    ) -> list[str]:
        if robot_id is None:
            if self.bits is not None and hasattr(self.bits, "robot_id"):
                robot_id = int(getattr(self.bits, "robot_id", 1))
            elif Settings is not None and hasattr(Settings, "RoboInfo"):
                robot_id = int(getattr(Settings.RoboInfo, "number", 1))
            else:
                robot_id = 1

        if server_port is None:
            server_port = int(os.environ.get("XEL_CAM_SERVER_PORT", "20801"))
        if host is None:
            host = os.environ.get("XEL_CAM_HOST", "127.0.0.1")

        if camera_slots is None:
            slots_env = os.environ.get("XEL_CAM_PORTS", "").strip()
            camera_slots = self._parse_int_list(slots_env) if slots_env else self._camera_slot_defaults()

        return [self.build_camera_url(robot_id, slot, server_port=server_port, host=host) for slot in camera_slots]

    def open_camera(
        self,
        index: int | None = None,
        width: int | None = None,
        height: int | None = None,
        *,
        robot_id: int | None = None,
        server_port: int | None = None,
        camera_slots: list[int] | None = None,
        host: str | None = None,
    ):
        """Open Unity MJPEG camera streams.

        index=None  -> open all four camera streams and return a list of captures
        index=int   -> open only the selected camera slot
        """

        try:
            import cv2  # type: ignore
        except Exception as e:  # pragma: no cover
            raise RuntimeError("OpenCV is required to open Unity camera streams") from e

        urls = self.build_camera_urls(robot_id=robot_id, server_port=server_port, camera_slots=camera_slots, host=host)
        if not urls:
            raise RuntimeError("No camera URLs could be constructed")

        self.close_camera()

        if index is None:
            caps = []
            for url in urls:
                cap = cv2.VideoCapture(url)
                if not cap.isOpened():
                    cap.release()
                    raise RuntimeError(f"Failed to open camera stream: {url}")
                caps.append(cap)
            self._cameras = caps
            self._camera_urls = list(urls)
            return caps

        idx = int(index)
        if idx < 0 or idx >= len(urls):
            raise IndexError(f"camera index out of range: {idx} (available={len(urls)})")

        url = urls[idx]
        cap = cv2.VideoCapture(url)
        if not cap.isOpened():
            cap.release()
            raise RuntimeError(f"Failed to open camera stream: {url}")
        self._camera_urls = [url]
        self._cameras = [cap]
        return cap

    def close_camera(self) -> None:
        for cap in getattr(self, "_cameras", []):
            try:
                cap.release()
            except Exception:
                pass
        self._cameras = []
        self._camera_urls = []

    # ===== SIM fast-path =====
    def get_ball_pos(self):
        if self.bits is None:
            return [0xFFF, 0xFFF]
        if hasattr(self.bits, "GetBallPos"):
            try:
                return self.bits.GetBallPos()
            except Exception:
                return [0xFFF, 0xFFF]
        return [0xFFF, 0xFFF]

    def get_chassis_pos(self):
        # Unity simulator currently doesn't provide opponent detections.
        return []

    def get_goals_pos(self):
        # Unity simulator currently doesn't provide goal detections.
        return {"blue_goal": None, "yellow_goal": None}

    def get_vision_data(self) -> dict[str, Any]:
        return VisionData(
            ball_pos=self.get_ball_pos(),
            chassis_pos=self.get_chassis_pos(),
            blue_goal=None,
            yellow_goal=None,
        ).as_dict()

    # ===== Future hooks (OpenCV + PT inference) =====
    def open_camera_legacy(self, index: int = 0, width: int | None = None, height: int | None = None) -> None:
        """Backward-compatible placeholder retained for older callers."""
        raise NotImplementedError("Use open_camera(..., robot_id=..., server_port=...) for Unity streams.")

    def load_pt_model(self, pt_path: str) -> None:
        """Placeholder for future local inference (.pt)."""
        raise NotImplementedError("PT model inference is not implemented yet.")

    def infer(self, frame) -> dict[str, Any]:
        """Placeholder for future inference output -> VisionData."""
        raise NotImplementedError("Vision inference is not implemented yet.")

    # ===== Frame pulling for dashboard =====
    def _pull_frames_loop(self):
        """Background thread to pull frames from Unity MJPEG server using OpenCV."""
        import time
        
        # Get robot ID and camera slots from config
        robot_id = 1
        if Settings is not None and hasattr(Settings, "RoboInfo"):
            try:
                robot_id = int(getattr(Settings.RoboInfo, "number", 1))
            except Exception:
                pass
        
        camera_slots = self._camera_slot_defaults()
        server_port = int(os.environ.get("XEL_CAM_SERVER_PORT", "20801"))
        host = os.environ.get("XEL_CAM_HOST", "127.0.0.1")
        
        # Build URLs for each camera
        urls = [self.build_camera_url(robot_id, slot, server_port, host) for slot in camera_slots]
        
        # Wait a bit for Unity server to start
        time.sleep(1.0)
        
        # OpenCV VideoCapture can handle MJPEG streams
        import cv2  # type: ignore
        
        caps = [None, None, None, None]
        
        while self._frame_running and not self._frame_stop_event.is_set():
            for i, url in enumerate(urls):
                try:
                    # Open or reopen capture if needed
                    if caps[i] is None or not caps[i].isOpened():
                        caps[i] = cv2.VideoCapture(url)
                        caps[i].set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    
                    if caps[i] is not None and caps[i].isOpened():
                        ret, frame = caps[i].read()
                        if ret and frame is not None:
                            self.Frames[i] = frame
                except Exception:
                    if caps[i] is not None:
                        try:
                            caps[i].release()
                        except Exception:
                            pass
                        caps[i] = None
            
            time.sleep(0.03)  # ~30 FPS
        
        # Cleanup
        for cap in caps:
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass
    
    def close(self, join_timeout: float = 1.0):
        """Stop background frame thread."""
        self._frame_running = False
        if self._frame_stop_event:
            self._frame_stop_event.set()
        if self._frame_thread and self._frame_thread.is_alive():
            try:
                self._frame_thread.join(timeout=join_timeout)
            except Exception:
                pass
        self.close_camera()
