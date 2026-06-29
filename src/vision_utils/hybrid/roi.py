from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple


Rect = Tuple[int, int, int, int]


def clamp_rect(rect: Rect, width: int, height: int) -> Optional[Rect]:
    x, y, w, h = rect
    x1 = max(0, min(width - 1, int(round(x))))
    y1 = max(0, min(height - 1, int(round(y))))
    x2 = max(0, min(width, int(round(x + w))))
    y2 = max(0, min(height, int(round(y + h))))
    if x2 <= x1 or y2 <= y1:
        return None
    return x1, y1, x2 - x1, y2 - y1


def grow_around_bbox(
    bbox_xyxy: tuple[int, int, int, int],
    frame_width: int,
    frame_height: int,
    scale: float,
    extra_px: float = 0.0,
) -> Rect:
    x1, y1, x2, y2 = bbox_xyxy
    cx = (float(x1) + float(x2)) * 0.5
    cy = (float(y1) + float(y2)) * 0.5
    bw = max(4.0, float(x2 - x1))
    bh = max(4.0, float(y2 - y1))
    size = max(bw, bh) * float(scale) + float(extra_px) * 2.0
    rect = (int(round(cx - size * 0.5)), int(round(cy - size * 0.5)), int(round(size)), int(round(size)))
    return clamp_rect(rect, frame_width, frame_height) or (0, 0, frame_width, frame_height)


@dataclass(slots=True)
class RoiState:
    rect: Optional[Rect] = None
    last_ts: float = 0.0
    lost_count: int = 0

    def clear(self) -> None:
        self.rect = None
        self.last_ts = 0.0
        self.lost_count = 0
