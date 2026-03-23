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

from dataclasses import dataclass
from typing import Any


@dataclass
class VisionData:
    # relative ball position in robot frame (cm)
    ball_pos: list[float] | list[int]
    # list of detections, keep same semantic as existing vision server
    # each item typically: [x, y, w, h, conf]
    chassis_pos: list[list[float]]

    def as_dict(self) -> dict[str, Any]:
        return {"ball_pos": self.ball_pos, "chassis_pos": self.chassis_pos}


class XeleratorVision:
    """Vision adapter.

    For simulator, you should pass `bits` = XeleratorSimBits instance.
    """

    def __init__(self, bits=None):
        self.bits = bits

        # reserved for future OpenCV/PT pipeline
        self._camera = None
        self._model = None

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

    def get_vision_data(self) -> dict[str, Any]:
        return VisionData(ball_pos=self.get_ball_pos(), chassis_pos=self.get_chassis_pos()).as_dict()

    # ===== Future hooks (OpenCV + PT inference) =====
    def open_camera(self, index: int = 0, width: int | None = None, height: int | None = None) -> None:
        """Placeholder for future OpenCV capture."""
        raise NotImplementedError("OpenCV camera pipeline is not implemented yet.")

    def load_pt_model(self, pt_path: str) -> None:
        """Placeholder for future local inference (.pt)."""
        raise NotImplementedError("PT model inference is not implemented yet.")

    def infer(self, frame) -> dict[str, Any]:
        """Placeholder for future inference output -> VisionData."""
        raise NotImplementedError("Vision inference is not implemented yet.")
