from __future__ import annotations

import numpy as np


class BallKalman:
    """Small constant-velocity Kalman filter for ball x/y in robot-frame cm."""

    def __init__(self, process_var: float = 360.0, measurement_var: float = 50.0):
        self.x = np.zeros((4, 1), dtype=np.float64)
        self.p = np.eye(4, dtype=np.float64) * 500.0
        self.q_base = float(process_var)
        self.r = np.eye(2, dtype=np.float64) * float(measurement_var)
        self.initialized = False
        self.last_ts = 0.0

    def reset(self, px: float, py: float, ts: float) -> None:
        self.x[:, 0] = [float(px), float(py), 0.0, 0.0]
        self.p = np.eye(4, dtype=np.float64) * 80.0
        self.initialized = True
        self.last_ts = float(ts)

    def predict(self, ts: float) -> tuple[float, float]:
        if not self.initialized:
            self.last_ts = float(ts)
            return 0.0, 0.0
        dt = max(1e-3, min(0.25, float(ts) - float(self.last_ts)))
        f = np.array(
            [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        q = np.eye(4, dtype=np.float64) * self.q_base * dt * dt
        self.x = f @ self.x
        self.p = f @ self.p @ f.T + q
        self.last_ts = float(ts)
        return float(self.x[0, 0]), float(self.x[1, 0])

    def forecast(self, ts: float) -> tuple[float, float]:
        """Predict position at ``ts`` without advancing the filter state."""
        if not self.initialized:
            return 0.0, 0.0
        dt = max(0.0, min(0.25, float(ts) - float(self.last_ts)))
        return float(self.x[0, 0] + self.x[2, 0] * dt), float(self.x[1, 0] + self.x[3, 0] * dt)

    def set_position(self, px: float, py: float) -> None:
        if not self.initialized:
            return
        self.x[0, 0] = float(px)
        self.x[1, 0] = float(py)

    def set_velocity(self, vx: float, vy: float) -> None:
        if not self.initialized:
            return
        self.x[2, 0] = float(vx)
        self.x[3, 0] = float(vy)

    def update(self, px: float, py: float, ts: float, confidence: float = 1.0) -> tuple[float, float]:
        if not self.initialized:
            self.reset(px, py, ts)
            return float(px), float(py)
        self.predict(ts)
        z = np.array([[float(px)], [float(py)]], dtype=np.float64)
        h = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]], dtype=np.float64)
        conf = max(0.05, min(1.0, float(confidence)))
        r = self.r / conf
        y = z - h @ self.x
        s = h @ self.p @ h.T + r
        k = self.p @ h.T @ np.linalg.inv(s)
        self.x = self.x + k @ y
        self.p = (np.eye(4, dtype=np.float64) - k @ h) @ self.p
        return float(self.x[0, 0]), float(self.x[1, 0])

    @property
    def velocity(self) -> tuple[float, float]:
        return float(self.x[2, 0]), float(self.x[3, 0])
