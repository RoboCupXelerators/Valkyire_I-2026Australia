from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal, Optional, Sequence, Tuple


SourceName = Literal["blob", "yolo", "prediction"]


@dataclass(slots=True)
class FramePacket:
    camera_index: int
    frame: object
    timestamp: float
    sequence: int = 0


@dataclass(slots=True)
class Detection:
    camera_index: int
    label: str
    confidence: float
    bbox_xyxy: Tuple[int, int, int, int]
    timestamp: float
    source: SourceName
    cls_id: int = -1
    position_cm: Optional[Tuple[float, float]] = None
    raw_bbox_xyxy: Optional[Tuple[int, int, int, int]] = None
    raw_frame_size: Optional[Tuple[int, int]] = None

    @property
    def center_xy(self) -> Tuple[float, float]:
        x1, y1, x2, y2 = self.bbox_xyxy
        return (float(x1 + x2) * 0.5, float(y1 + y2) * 0.5)

    @property
    def width(self) -> int:
        return max(0, int(self.bbox_xyxy[2]) - int(self.bbox_xyxy[0]))

    @property
    def height(self) -> int:
        return max(0, int(self.bbox_xyxy[3]) - int(self.bbox_xyxy[1]))

    def as_bbox_tuple(self):
        x1, y1, x2, y2 = self.bbox_xyxy
        return (int(self.camera_index), int(x1), int(y1), int(x2), int(y2), float(self.confidence))

    def as_goal_bbox_tuple(self):
        x1, y1, x2, y2 = self.bbox_xyxy
        return (
            int(self.camera_index),
            int(self.cls_id),
            int(x1),
            int(y1),
            int(x2),
            int(y2),
            float(self.confidence),
        )


@dataclass(slots=True)
class TrackState:
    position_cm: Optional[Tuple[float, float]] = None
    velocity_cm_s: Tuple[float, float] = (0.0, 0.0)
    confidence: float = 0.0
    source: SourceName = "prediction"
    last_observed_ts: float = 0.0
    last_output_ts: float = 0.0
    camera_index: int = -1


@dataclass(slots=True)
class VisionOutput:
    ball_pos_cm: Tuple[float, float] = (0.0, 0.0)
    ball_confidence: float = 0.0
    ball_source: SourceName = "prediction"
    ball_bbox: Optional[Tuple[int, int, int, int, int, float]] = None
    ball_bboxes: Sequence[Optional[Tuple[int, int, int, int, int, float]]] = field(
        default_factory=lambda: [None, None, None, None]
    )
    chassis_positions_cm: Sequence[Sequence[float]] = field(default_factory=list)
    chassis_bboxes: Sequence[Optional[Tuple[int, int, int, int, int, float]]] = field(
        default_factory=lambda: [None, None, None, None]
    )
    goal_bboxes: Sequence[Optional[Tuple[int, int, int, int, int, int, float]]] = field(
        default_factory=lambda: [None, None, None, None]
    )
    blue_goal_pos_cm: Optional[Sequence[float]] = None
    yellow_goal_pos_cm: Optional[Sequence[float]] = None
    timestamp: float = 0.0


@dataclass(slots=True)
class PerfSnapshot:
    preprocess_ms: float = 0.0
    yolo_ms: float = 0.0
    blob_ms: float = 0.0
    fusion_ms: float = 0.0
    output_hz: float = 0.0
    yolo_hz: float = 0.0
    tracker_hz: float = 0.0
    vision_confidence: float = 0.0
    yolo_backend: str = "disabled"
    yolo_error: str = ""

    def to_dict(self) -> dict:
        return {
            "preprocess_ms": round(float(self.preprocess_ms), 3),
            "yolo_ms": round(float(self.yolo_ms), 3),
            "blob_ms": round(float(self.blob_ms), 3),
            "fusion_ms": round(float(self.fusion_ms), 3),
            "output_hz": round(float(self.output_hz), 2),
            "yolo_hz": round(float(self.yolo_hz), 2),
            "tracker_hz": round(float(self.tracker_hz), 2),
            "vision_confidence": round(float(self.vision_confidence), 3),
            "yolo_backend": self.yolo_backend,
            "yolo_error": self.yolo_error,
        }
