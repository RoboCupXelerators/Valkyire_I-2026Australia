import threading
import time


class RobotStateSync:
    """Explicit robot_state telemetry loop for control-panel side-channel data."""

    def __init__(self, chassis, period: float = 0.05):
        self.chassis = chassis
        self.period = float(period)
        self._thread = None
        self._stop = threading.Event()

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self):
        if self.running:
            return False
        self._stop.clear()
        self._thread = threading.Thread(target=self._update_loop, daemon=True)
        self._thread.start()
        return True

    def stop(self, timeout: float | None = 0.2):
        if self._thread is None:
            return
        self._stop.set()
        self._thread.join(timeout=timeout)
        if not self._thread.is_alive():
            self._thread = None

    def _update_loop(self):
        while not self._stop.is_set():
            try:
                from robot_state import set_robot_state

                yaw = self.chassis.GetYaw() if self.chassis.GetYaw else 0.0
                set_robot_state(
                    yaw_deg=yaw,
                    vx=getattr(self.chassis, "_last_vx", 0.0),
                    vy=getattr(self.chassis, "_last_vy", 0.0),
                    wz=getattr(self.chassis, "_last_wz", 0.0),
                )
            except Exception:
                pass
            time.sleep(self.period)
