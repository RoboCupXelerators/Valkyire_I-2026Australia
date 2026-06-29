from __future__ import annotations

import time


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
