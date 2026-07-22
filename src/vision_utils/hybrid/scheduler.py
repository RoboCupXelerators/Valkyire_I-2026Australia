from __future__ import annotations

import time
from dataclasses import dataclass
from threading import RLock


class RateGate:
    def __init__(self, hz: float):
        self.period = 0.0 if hz <= 0.0 else 1.0 / float(hz)
        self.next_ts = time.perf_counter()

    def ready(self, now: float) -> bool:
        if self.period <= 0.0:
            return False
        if now < self.next_ts:
            return False
        self.next_ts = now + self.period
        return True

    def wait_next(self, stop_event) -> bool:
        """Wait until the next deadline using a monotonic clock.

        Returns False when disabled or when `stop_event` is set. If the loop
        falls behind, stale ticks are skipped instead of running a catch-up
        burst that would add latency.
        """
        if self.period <= 0.0:
            stop_event.wait(0.1)
            return False

        now = time.perf_counter()
        remaining = self.next_ts - now
        if remaining > 0.0 and stop_event.wait(remaining):
            return False

        now = time.perf_counter()
        if now - self.next_ts > self.period:
            missed = int((now - self.next_ts) // self.period)
            self.next_ts += (missed + 1) * self.period
        else:
            self.next_ts += self.period
        return not stop_event.is_set()


@dataclass(slots=True)
class _CameraFrameState:
    last_frame_ts: float = 0.0
    last_selected_mono: float = float("-inf")
    focus_until_mono: float = 0.0
    focus_score: float = 0.0
    selected_count: int = 0


class AdaptiveFrameDistributor:
    """Select the newest camera frames that deserve the next YOLO tick.

    Every camera keeps a low-rate search budget. A camera that reports a likely
    ball gets a temporary high-rate budget, while the other cameras continue to
    receive occasional frames so a moving ball can be reacquired elsewhere.
    """

    def __init__(
        self,
        camera_count: int = 4,
        pump_hz: float = 20.0,
        min_hz: float = 4.0,
        focus_hz: float = 20.0,
        focus_hold_ms: float = 250.0,
        max_frames_per_tick: int = 4,
        preferred_camera_index: int | None = None,
    ):
        self.camera_count = max(1, int(camera_count))
        self.pump_hz = max(0.0, float(pump_hz))
        self.min_hz = max(0.0, float(min_hz))
        self.focus_hz = max(self.min_hz, float(focus_hz))
        self.focus_hold_seconds = max(0.0, float(focus_hold_ms) / 1000.0)
        self.max_frames_per_tick = max(1, min(self.camera_count, int(max_frames_per_tick)))
        self.preferred_camera_index = (
            int(preferred_camera_index)
            if preferred_camera_index is not None and 0 <= int(preferred_camera_index) < self.camera_count
            else None
        )
        self._states = [_CameraFrameState() for _ in range(self.camera_count)]
        self._lock = RLock()
        self._started_mono = time.monotonic()

    def observe(self, camera_index: int, confidence: float, now: float | None = None) -> None:
        """Raise a camera's priority after a cheap or YOLO detector sees a ball."""
        cam = int(camera_index)
        if not 0 <= cam < self.camera_count or float(confidence) <= 0.0:
            return
        now = time.monotonic() if now is None else float(now)
        score = max(0.0, min(1.0, float(confidence)))
        with self._lock:
            state = self._states[cam]
            state.focus_score = max(state.focus_score * 0.75, score)
            state.focus_until_mono = max(state.focus_until_mono, now + self.focus_hold_seconds)

    def _is_due(self, state: _CameraFrameState, now: float) -> bool:
        focused = now < state.focus_until_mono
        rate = self.focus_hz if focused else self.min_hz
        if rate <= 0.0:
            return False
        return now - state.last_selected_mono >= (1.0 / rate) - 1e-6

    def select(self, frames, timestamps, now: float | None = None):
        """Return a camera-slot-preserving list with only selected frames."""
        now = time.monotonic() if now is None else float(now)
        selected = [None] * self.camera_count
        selected_ts = [0.0] * self.camera_count
        candidates = []

        with self._lock:
            for cam in range(self.camera_count):
                frame = frames[cam] if cam < len(frames) else None
                ts = float(timestamps[cam]) if cam < len(timestamps) else 0.0
                state = self._states[cam]
                if frame is None or ts <= 0.0 or ts <= state.last_frame_ts:
                    continue
                if not self._is_due(state, now):
                    continue

                focused = now < state.focus_until_mono
                age = now - state.last_selected_mono
                # Focused cameras win first; otherwise oldest due camera wins.
                priority = (
                    2 if focused else 0,
                    state.focus_score if focused else 0.0,
                    1 if cam == self.preferred_camera_index else 0,
                    age,
                )
                candidates.append((priority, cam, frame, ts))

            candidates.sort(key=lambda item: item[0], reverse=True)
            for _, cam, frame, ts in candidates[: self.max_frames_per_tick]:
                state = self._states[cam]
                selected[cam] = frame
                selected_ts[cam] = ts
                state.last_frame_ts = ts
                state.last_selected_mono = now
                state.selected_count += 1

        return selected, selected_ts

    def snapshot(self, now: float | None = None) -> dict:
        now = time.monotonic() if now is None else float(now)
        with self._lock:
            elapsed = max(1e-6, now - self._started_mono)
            focused = [
                i for i, state in enumerate(self._states) if now < state.focus_until_mono
            ]
            return {
                "camera_yolo_hz": [round(state.selected_count / elapsed, 2) for state in self._states],
                "camera_yolo_frames": [state.selected_count for state in self._states],
                "focused_cameras": focused,
                "min_hz": round(self.min_hz, 2),
                "focus_hz": round(self.focus_hz, 2),
                "max_frames_per_tick": self.max_frames_per_tick,
            }
