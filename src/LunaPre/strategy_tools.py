"""Small shared helpers for strategy modules."""

from __future__ import annotations

import math
import time
import weakref

BALL_NOT_FOUND = [0xFFF, 0xFFF]
BALL_HELD = [0xDDD, 0xDDD]
POSE_NOT_FOUND = [0xFFF, 0xFFF, 0xFFF]

# Controllers whose integral term must not survive a referee permission loss.
# The chassis resets them on the permission True->False edge, so a controller
# that is simply not called while motion is denied cannot carry stale integral
# action into the first command after re-enable.
_PERMISSION_GATED_PIDS: "weakref.WeakSet" = weakref.WeakSet()


def register_permission_pid(pid) -> None:
    """Register a PID whose state must be cleared when motion is disallowed."""

    if pid is None:
        return
    try:
        _PERMISSION_GATED_PIDS.add(pid)
    except TypeError:
        # A non-weakref-able controller cannot be tracked; callers that use it
        # are still gated at their command entry points.
        pass


def reset_permission_pids() -> None:
    """Reset every registered PID; used on the referee permission-loss edge."""

    for pid in list(_PERMISSION_GATED_PIDS):
        try:
            pid.reset()
        except Exception:
            pass

# ``transition`` produces one interpolated target per control cycle at roughly
# this rate.  The caller drives iteration; the generator only paces itself so a
# consumer that already kept up is not blocked.
TRANSITION_RATE_HZ = 30.0


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def angle_error_deg(current: float, target: float) -> float:
    return ((float(target) - float(current) + 180.0) % 360.0) - 180.0


def nearest_angle(candidates, target: float):
    return min(candidates, key=lambda angle: abs(angle_error_deg(angle, target)))


def distance(p1, p2) -> float:
    dx = float(p1[0]) - float(p2[0])
    dy = float(p1[1]) - float(p2[1])
    return math.hypot(dx, dy)


def transition(current, aim, time_s):
    """Yield linearly interpolated scalars from ``current`` to ``aim``.

    The generator yields ``current`` immediately, then steps toward ``aim`` at
    about :data:`TRANSITION_RATE_HZ`.  The interpolation is measured against a
    monotonic clock instead of a presumed fixed sample period, so a late caller
    snaps forward rather than drifting.  The final yielded value is always the
    exact ``aim``.

    ``time_s <= 0`` yields ``aim`` once, with no starting value.
    """

    start_value = float(current)
    aim_value = float(aim)
    duration = float(time_s)
    if duration <= 0.0:
        yield aim_value
        return
    yield start_value

    interval = 1.0 / max(1.0, float(TRANSITION_RATE_HZ))
    started = time.monotonic()
    next_sample = started
    while True:
        now = time.monotonic()
        if now - started >= duration:
            break
        next_sample += interval
        wait_s = next_sample - now
        if wait_s > 0.0:
            time.sleep(wait_s)
            now = time.monotonic()
        if now - started >= duration:
            break
        ratio = min(1.0, (now - started) / duration)
        yield start_value + (aim_value - start_value) * ratio
    yield aim_value


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

class timer:
    def __init__(self):
        self._time_stamp_cache = None
        self.tagged = False

    def started(self) -> bool:
        return False if not self.tagged or not self._time_stamp_cache else True

    def countdown(self, seconds):
        '''
        it will record the time when created.
        `seconds` int: after ~ second, this will return true
        '''
        if not self.tagged or not self._time_stamp_cache:
            self._time_stamp_cache = time.time()
            self.tagged = True
            return self._time_stamp_cache
        return True if time.time() - self._time_stamp_cache >= seconds else False

    def reset(self):
        self.tagged = False
        return True