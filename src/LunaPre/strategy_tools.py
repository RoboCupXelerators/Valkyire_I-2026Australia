"""Small shared helpers for strategy modules."""

from __future__ import annotations

import math

BALL_NOT_FOUND = [0xFFF, 0xFFF]
BALL_HELD = [0xDDD, 0xDDD]
POSE_NOT_FOUND = [0xFFF, 0xFFF, 0xFFF]


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def angle_error_deg(current: float, target: float) -> float:
    return ((float(target) - float(current) + 180.0) % 360.0) - 180.0


def distance(p1, p2) -> float:
    dx = float(p1[0]) - float(p2[0])
    dy = float(p1[1]) - float(p2[1])
    return math.hypot(dx, dy)


def valid_pose(pose) -> bool:
    return bool(pose) and len(pose) >= 3 and list(pose[:3]) != POSE_NOT_FOUND


def known_robot_ball(ball) -> bool:
    return bool(ball) and ball not in (BALL_NOT_FOUND, BALL_HELD)


def robot_ball_distance(ball):
    if not known_robot_ball(ball):
        return None
    try:
        return math.hypot(float(ball[0]), float(ball[1]))
    except Exception:
        return None


def safe_stop(chassis, peripheral=None, position=None) -> None:
    if peripheral is not None:
        try:
            peripheral.Dribble(False)
        except Exception:
            pass
    try:
        chassis.stop()
    except Exception:
        pass
    if position is not None:
        try:
            position.Stop()
        except AttributeError:
            try:
                position.stop()
            except Exception:
                pass
        except Exception:
            pass
