from __future__ import annotations

import bisect
import math
import threading
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class RobotPose:
    x_cm: float
    y_cm: float
    yaw_deg: float
    timestamp: float


def robot_to_field(position_cm, pose: RobotPose) -> tuple[float, float]:
    """Rotate/translate ``(forward, left)`` from robot into field coordinates."""

    forward, left = map(float, position_cm[:2])
    yaw = math.radians(float(pose.yaw_deg))
    c, s = math.cos(yaw), math.sin(yaw)
    return pose.x_cm + forward * c - left * s, pose.y_cm + forward * s + left * c


def field_to_robot(position_cm, pose: RobotPose) -> tuple[float, float]:
    """Convert a field point into the robot's current ``(forward, left)`` frame."""

    dx = float(position_cm[0]) - pose.x_cm
    dy = float(position_cm[1]) - pose.y_cm
    yaw = math.radians(float(pose.yaw_deg))
    c, s = math.cos(yaw), math.sin(yaw)
    return dx * c + dy * s, -dx * s + dy * c


def _interpolate_yaw(start: float, end: float, ratio: float) -> float:
    delta = (float(end) - float(start) + 180.0) % 360.0 - 180.0
    return (float(start) + delta * ratio) % 360.0


class PoseHistory:
    """Small timestamped pose buffer used to compensate camera capture latency."""

    def __init__(self, max_samples: int = 128, max_age_s: float = 0.25):
        self.max_samples = max(2, int(max_samples))
        self.max_age_s = max(0.01, float(max_age_s))
        self._poses: list[RobotPose] = []
        self._lock = threading.RLock()

    def add(self, pose, timestamp: float) -> bool:
        try:
            x_cm, y_cm, yaw_deg = map(float, pose[:3])
            ts = float(timestamp)
        except (TypeError, ValueError, IndexError):
            return False
        if not all(math.isfinite(value) for value in (x_cm, y_cm, yaw_deg, ts)):
            return False

        sample = RobotPose(x_cm, y_cm, yaw_deg % 360.0, ts)
        with self._lock:
            if self._poses and ts < self._poses[-1].timestamp:
                index = bisect.bisect_left([item.timestamp for item in self._poses], ts)
                self._poses.insert(index, sample)
            elif self._poses and ts == self._poses[-1].timestamp:
                self._poses[-1] = sample
            else:
                self._poses.append(sample)
            if len(self._poses) > self.max_samples:
                del self._poses[:-self.max_samples]
        return True

    def resolve(self, timestamp: float) -> RobotPose | None:
        ts = float(timestamp)
        with self._lock:
            poses = list(self._poses)
        if not poses:
            return None

        times = [item.timestamp for item in poses]
        right = bisect.bisect_left(times, ts)
        if right == 0:
            return poses[0] if abs(poses[0].timestamp - ts) <= self.max_age_s else None
        if right >= len(poses):
            return poses[-1] if abs(ts - poses[-1].timestamp) <= self.max_age_s else None

        before, after = poses[right - 1], poses[right]
        if min(ts - before.timestamp, after.timestamp - ts) > self.max_age_s:
            return None
        span = after.timestamp - before.timestamp
        ratio = 0.0 if span <= 1e-9 else (ts - before.timestamp) / span
        return RobotPose(
            x_cm=before.x_cm + (after.x_cm - before.x_cm) * ratio,
            y_cm=before.y_cm + (after.y_cm - before.y_cm) * ratio,
            yaw_deg=_interpolate_yaw(before.yaw_deg, after.yaw_deg, ratio),
            timestamp=ts,
        )

    def clear(self) -> None:
        with self._lock:
            self._poses.clear()
