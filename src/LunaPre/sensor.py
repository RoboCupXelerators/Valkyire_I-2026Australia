"""Bits pose validation and robot-frame wall distances.

SIM and IceLoong share this observation boundary. Native scan drivers and
their localization pipeline are maintained on the legacy branch.
"""
from __future__ import annotations

import math

from ReasonData import Settings
from .strategy_tools import BALL_NOT_FOUND

LIDAR_DISABLED_DISTS = [BALL_NOT_FOUND[0]] * 4

def coerce_pose(pose):
    """Return a finite ``(x_cm, y_cm, yaw_deg)`` tuple or ``None``.

    Bits implementations are allowed to report an unavailable pose with
    ``None``.  Keeping validation here gives runtime and sensor callers one
    contract and prevents malformed payloads from turning into a false origin.
    """

    if pose is None:
        return None
    try:
        x_cm, y_cm, yaw_deg = pose
        values = (float(x_cm), float(y_cm), float(yaw_deg))
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in values):
        return None
    return values



def _pose_wall_distances(pose, *, bounds_long_cm: float, bounds_short_cm: float) -> list[float] | None:
    """Ray-cast a valid Bits pose against the rectangular field in millimetres."""

    normalized = coerce_pose(pose)
    if normalized is None:
        return None
    x_cm, y_cm, yaw_deg = normalized
    half_long = float(bounds_long_cm) / 2.0
    half_short = float(bounds_short_cm) / 2.0
    if not all(math.isfinite(value) and value > 0.0 for value in (half_long, half_short)):
        return None

    def _ray_distance_cm(dx: float, dy: float) -> float:
        candidates = []
        epsilon = 1e-9
        if abs(dx) > epsilon:
            for bound_x in (-half_long, half_long):
                t = (bound_x - x_cm) / dx
                hit_y = y_cm + t * dy
                if t >= 0.0 and -half_short - epsilon <= hit_y <= half_short + epsilon:
                    candidates.append(t)
        if abs(dy) > epsilon:
            for bound_y in (-half_short, half_short):
                t = (bound_y - y_cm) / dy
                hit_x = x_cm + t * dx
                if t >= 0.0 and -half_long - epsilon <= hit_x <= half_long + epsilon:
                    candidates.append(t)
        return max(0.0, min(candidates)) if candidates else 0.0

    heading = math.radians(yaw_deg)
    forward = (math.cos(heading), math.sin(heading))
    left = (-forward[1], forward[0])
    directions = (forward, left, (-forward[0], -forward[1]), (forward[1], -forward[0]))
    return [_ray_distance_cm(dx, dy) * 10.0 for dx, dy in directions]



class Sensor:
    """Expose validated Bits pose and [front, left, back, right] wall ranges."""

    def __init__(self, *, pose_provider=None, bounds_long_cm=None, bounds_short_cm=None):
        self._pose_provider = pose_provider
        self.bounds_long_cm = float(
            bounds_long_cm if bounds_long_cm is not None else Settings.Bounds.Long
        )
        self.bounds_short_cm = float(
            bounds_short_cm if bounds_short_cm is not None else Settings.Bounds.Short
        )

    def GetPose(self):
        """Read one observation; unavailable or malformed poses stay None."""
        try:
            return coerce_pose(self._pose_provider())
        except Exception:
            return None

    def DirectDistance(self):
        """Derive robot-frame wall ranges in mm from one fresh pose."""
        distances = _pose_wall_distances(
            self.GetPose(), bounds_long_cm=self.bounds_long_cm,
            bounds_short_cm=self.bounds_short_cm,
        )
        return distances if distances is not None else list(LIDAR_DISABLED_DISTS)

    # Observation aliases used by existing strategies and MLP.
    DirectDistanceSnapshot = DirectDistance
    GetDists = DirectDistance
