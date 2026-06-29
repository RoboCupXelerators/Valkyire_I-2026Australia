from __future__ import annotations

import time
from dataclasses import dataclass, field

from .types import PerfSnapshot


@dataclass(slots=True)
class RateMeter:
    alpha: float = 0.25
    rate_hz: float = 0.0
    _last_ts: float = 0.0

    def tick(self, now: float | None = None) -> float:
        now = time.perf_counter() if now is None else float(now)
        if self._last_ts > 0.0:
            dt = max(1e-6, now - self._last_ts)
            rate = 1.0 / dt
            self.rate_hz = rate if self.rate_hz <= 0.0 else self.rate_hz * (1.0 - self.alpha) + rate * self.alpha
        self._last_ts = now
        return self.rate_hz


@dataclass(slots=True)
class HybridPerf:
    alpha: float = 0.25
    preprocess_ms: float = 0.0
    yolo_ms: float = 0.0
    blob_ms: float = 0.0
    fusion_ms: float = 0.0
    output_rate: RateMeter = field(default_factory=RateMeter)
    yolo_rate: RateMeter = field(default_factory=RateMeter)
    tracker_rate: RateMeter = field(default_factory=RateMeter)
    vision_confidence: float = 0.0
    yolo_backend: str = "disabled"
    yolo_error: str = ""

    def record_ms(self, name: str, elapsed_ms: float) -> None:
        previous = float(getattr(self, name, 0.0) or 0.0)
        value = float(max(0.0, elapsed_ms))
        setattr(self, name, value if previous <= 0.0 else previous * (1.0 - self.alpha) + value * self.alpha)

    def snapshot(self) -> PerfSnapshot:
        return PerfSnapshot(
            preprocess_ms=self.preprocess_ms,
            yolo_ms=self.yolo_ms,
            blob_ms=self.blob_ms,
            fusion_ms=self.fusion_ms,
            output_hz=self.output_rate.rate_hz,
            yolo_hz=self.yolo_rate.rate_hz,
            tracker_hz=self.tracker_rate.rate_hz,
            vision_confidence=self.vision_confidence,
            yolo_backend=self.yolo_backend,
            yolo_error=self.yolo_error,
        )
