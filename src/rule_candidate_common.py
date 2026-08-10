"""Small, nonblocking rule-strategy kernel used by the tournament candidates.

The entrypoint modules select a profile; this module owns only sensor guards,
bounded actions, and the common safe-stop path. State is kept in a plain dict so
each process has an isolated controller and no robot state is shared.
"""

from __future__ import annotations

import math
import os
import time

from BasicFuc import lockball
from LunaPre import Settings, UnitedPosition, chassis, peripheral
from LunaPre.strategy_tools import BALL_HELD, BALL_NOT_FOUND, clamp, safe_stop, valid_pose


PROFILES = {
    "lane_runner": {
        "shot_progress": 58.0, "shot_y": 18.0, "carry_speed": 0.62,
        "search_gain": 1.4, "defence_x": 54.0, "press_distance": 72.0,
    },
    "possession_guard": {
        "shot_progress": 78.0, "shot_y": 14.0, "carry_speed": 0.42,
        "search_gain": 1.0, "defence_x": 62.0, "press_distance": 52.0,
    },
    "counter_press": {
        "shot_progress": 42.0, "shot_y": 28.0, "carry_speed": 0.70,
        "search_gain": 1.8, "defence_x": 48.0, "press_distance": 98.0,
    },
    "predictive_sweeper": {
        "shot_progress": 68.0, "shot_y": 20.0, "carry_speed": 0.42,
        "search_gain": 1.2, "defence_x": 70.0, "press_distance": 64.0,
        "anti_stall": True,
        "field_controller": True, "approach_standoff": 24.0,
        "capture_distance": 18.0, "intercept_threshold": 12.0,
        "intercept_offset": 18.0, "lane_limit": 0.20,
    },
    # Sadr United observation-derived hypothesis: keep the ball carrier in
    # the central lane, leave the second robot as a deep anchor, and only
    # collapse into a short press when the ball is already in our half.
    "sadr_united": {
        "shot_progress": 55.0, "shot_y": 24.0, "carry_speed": 0.58,
        "search_gain": 1.35, "defence_x": 58.0, "press_distance": 76.0,
        "anti_stall": True,
        "field_controller": True, "approach_standoff": 18.0,
        "capture_distance": 22.0, "intercept_threshold": 0.0,
        "intercept_offset": 12.0, "lane_limit": 0.25,
    },
}


def new_state(profile: str) -> dict:
    if profile not in PROFILES:
        raise ValueError(f"unknown candidate profile: {profile}")
    return {"profile": profile, "state": "boot", "last_shot": -999.0, "last_ball": None,
            "last_ball_ts": 0.0, "shot_release_until": 0.0,
            "progress_pose": None, "progress_ts": 0.0, "last_escape_ts": -999.0,
            "escape_side": _physical_escape_side(), "escape_ticks": 0,
            "last_ball_kind": None, "free_ball_ts": 0.0,
            "pocket_clear_ticks": 0, "pocket_clear_complete": False}


def attack_direction() -> float:
    value = os.environ.get("LEAGUE_ATTACK_DIRECTION", "").strip()
    if value in {"1", "-1"}:
        return float(value)
    try:
        robot_id = int(getattr(getattr(Settings, "Sim", object()), "RobotId", 1))
        return -1.0 if str(getattr(Settings.RoboInfo, "Bit", "")).upper() == "SIM" and robot_id >= 3 else 1.0
    except Exception:
        return 1.0


def _physical_escape_side() -> float:
    try:
        robot_id = int(os.environ.get("XEL_ROBOT_ID", "1"))
    except ValueError:
        robot_id = 1
    return 1.0 if robot_id <= 2 else -1.0


def _ball_state(ball) -> str:
    try:
        state = UnitedPosition.BallState(ball)
        if state in {"lost", "held", "free"}:
            return state
    except Exception:
        pass
    if ball == BALL_NOT_FOUND:
        return "lost"
    if ball == BALL_HELD:
        return "held"
    return "free" if _valid_ball(ball) else "lost"


def _valid_ball(ball) -> bool:
    if ball in (BALL_NOT_FOUND, BALL_HELD) or not ball or len(ball) < 2:
        return False
    try:
        x, y = float(ball[0]), float(ball[1])
    except (TypeError, ValueError):
        return False
    return math.isfinite(x) and math.isfinite(y) and math.hypot(x, y) <= 240.0


def _home(direction: float, profile: dict) -> list[float]:
    x = -direction * profile["defence_x"]
    yaw = 0.0 if direction > 0 else 180.0
    return [x, 0.0, yaw]


def _stop_tick() -> str:
    chassis.stop()
    peripheral.Dribble(False)
    return "restricted_stop"


def _goal_yaw(direction: float) -> float:
    return 0.0 if direction > 0 else 180.0


def _corner_recovery_target(profile: dict, field_ball, direction: float):
    if not profile.get("anti_stall") or not _valid_ball(field_ball):
        return None
    try:
        field_x_half = float(Settings.Bounds.Long) / 2.0
        field_y_half = float(Settings.Bounds.Short) / 2.0
        restricted = Settings.Motion.RestrictedArea
        x_limit = min(float(getattr(restricted, "XLimit")), field_x_half)
        y_limit = min(float(getattr(restricted, "YLimit")), field_y_half)
        clearance = float(getattr(restricted, "Clearance"))
        recovery_clearance = float(getattr(restricted, "RecoveryClearance"))
    except (AttributeError, TypeError, ValueError):
        return None

    recovery_x = max(0.0, x_limit - recovery_clearance)
    recovery_y = max(0.0, y_limit - recovery_clearance)
    inset = recovery_clearance + clearance
    bx, by = float(field_ball[0]), float(field_ball[1])
    if abs(bx) < x_limit - inset or abs(by) < y_limit - inset:
        return None

    target_x = clamp(bx - math.copysign(inset, bx), -recovery_x, recovery_x)
    target_y = clamp(by - math.copysign(inset, by), -recovery_y, recovery_y)
    return [target_x, target_y, _goal_yaw(direction)]


def _safe_target(target):
    try:
        return UnitedPosition.ProjectSafePose(target) or target
    except Exception:
        return target


def _lost_patrol_target(state: dict, profile: dict, role: str, direction: float, now: float) -> list[float]:
    """Return a short, bounded visual-search route without using ball position."""
    try:
        robot_id = int(os.environ.get("XEL_ROBOT_ID", "1"))
    except ValueError:
        robot_id = 1
    escape_side = state["escape_side"]
    if role == "defender":
        lane = 10.0 if (int(now / 1.5) + robot_id - 1) % 2 == 0 else -10.0
        return [-direction * min(profile["defence_x"], 42.0), escape_side * lane, _goal_yaw(direction)]

    phases = ((-18.0, 22.0), (8.0, 34.0), (20.0, -26.0), (-6.0, -34.0))
    progress, lateral = phases[(int(now / 1.25) + robot_id - 1) % len(phases)]
    return [direction * progress, escape_side * lateral, _goal_yaw(direction)]


def _restricted_geometry():
    """Return the legal interior shoulder bounds around either goal box."""
    try:
        field_x_half = float(Settings.Bounds.Long) / 2.0
        field_y_half = float(Settings.Bounds.Short) / 2.0
        restricted = Settings.Motion.RestrictedArea
        x_limit = min(float(restricted.XLimit), field_x_half)
        y_limit = min(float(restricted.YLimit), field_y_half)
        goal_box_x_min = max(0.0, x_limit - float(restricted.GoalBoxWidth))
        goal_box_y_half = max(0.0, min(float(restricted.GoalBoxYHalf), y_limit))
        clearance = max(0.0, float(restricted.Clearance))
        recovery = max(clearance, float(restricted.RecoveryClearance))
    except (AttributeError, TypeError, ValueError):
        return None
    return {
        "goal_box_x_min": goal_box_x_min,
        "goal_box_y_half": goal_box_y_half,
        "clearance": clearance,
        "recovery_x": max(0.0, x_limit - recovery),
        "recovery_y": max(0.0, y_limit - recovery),
        "pocket_y": min(max(0.0, y_limit - recovery), goal_box_y_half + clearance),
    }


def _own_goal_pocket(field_ball, direction: float):
    """Identify a free ball beside the own goal box and its escape side."""
    geometry = _restricted_geometry()
    if geometry is None or not _valid_ball(field_ball) or geometry["pocket_y"] <= geometry["goal_box_y_half"]:
        return None
    bx, by = float(field_ball[0]), float(field_ball[1])
    in_own_depth = -direction * bx >= geometry["goal_box_x_min"] - geometry["clearance"]
    beside_goal_box = abs(by) <= geometry["goal_box_y_half"] + geometry["clearance"]
    if not in_own_depth or not beside_goal_box:
        return None
    return geometry, (1.0 if by >= 0.0 else -1.0)


def _pocket_shoulder(field_ball, direction: float, side: float):
    geometry = _restricted_geometry()
    if geometry is None:
        return None
    bx = float(field_ball[0])
    return [
        clamp(bx, -geometry["recovery_x"], geometry["recovery_x"]),
        side * geometry["pocket_y"],
        _goal_yaw(direction),
    ]


def _at_target(pose, target, radius: float = 12.0) -> bool:
    return math.hypot(float(pose[0]) - target[0], float(pose[1]) - target[1]) <= radius


def _active_field_step(state: dict, profile: dict, role: str, pose, ball, ball_kind: str, direction: float, now: float) -> str:
    goal_yaw = _goal_yaw(direction)
    field_ball = UnitedPosition.FieldBallPos() if ball_kind == "free" else None
    pocket = _own_goal_pocket(field_ball, direction) if ball_kind == "free" else None
    if pocket is None:
        state["pocket_clear_ticks"] = 0
        state["pocket_clear_complete"] = False
    if role == "defender":
        if ball_kind == "held":
            peripheral.Dribble(0.55)
            chassis.FieldMoveVector(direction * 0.45, 0.0, goal_yaw, KpW=0.7)
            state["state"] = "defender_clear"
            return state["state"]

        if ball_kind == "lost":
            peripheral.Dribble(False)
            UnitedPosition.Pos2Pos(
                _safe_target(_lost_patrol_target(state, profile, role, direction, now)),
                profile["search_gain"],
            )
            state["state"] = "defender_home"
            return state["state"]

        if ball_kind == "free" and _valid_ball(field_ball):
            bx, by = float(field_ball[0]), float(field_ball[1])
            if pocket is not None:
                _, side = pocket
                shoulder = _pocket_shoulder(field_ball, direction, side)
                if state["pocket_clear_complete"] and shoulder is not None:
                    peripheral.Dribble(False)
                    UnitedPosition.Pos2Pos(_safe_target(shoulder), profile["search_gain"])
                    state["state"] = "pocket_clear_hold"
                    return state["state"]
                if shoulder is not None and not _at_target(pose, shoulder):
                    peripheral.Dribble(False)
                    UnitedPosition.Pos2Pos(_safe_target(shoulder), profile["search_gain"])
                    state["pocket_clear_ticks"] = 0
                    state["state"] = "pocket_shoulder"
                    return state["state"]
                if state["pocket_clear_ticks"] <= 0:
                    state["pocket_clear_ticks"] = 4
                peripheral.Dribble(0.7)
                chassis.FieldMoveVector(direction * 0.50, -side * 0.20, goal_yaw, KpW=0.75)
                state["pocket_clear_ticks"] -= 1
                if state["pocket_clear_ticks"] == 0:
                    state["pocket_clear_complete"] = True
                state["state"] = "pocket_clear"
                return state["state"]
            own_progress = -direction * bx
            if own_progress >= profile["intercept_threshold"]:
                target = [
                    bx - direction * profile["intercept_offset"],
                    clamp(by, -55.0, 55.0),
                    goal_yaw,
                ]
                UnitedPosition.Pos2Pos(_safe_target(target), profile["search_gain"])
                state["state"] = "defender_intercept"
                return state["state"]
        UnitedPosition.Pos2Pos(_safe_target(_home(direction, profile)), profile["search_gain"])
        state["state"] = "defender_home"
        return state["state"]

    if ball_kind == "lost":
        peripheral.Dribble(False)
        UnitedPosition.Pos2Pos(
            _safe_target(_lost_patrol_target(state, profile, role, direction, now)),
            profile["search_gain"],
        )
        state["state"] = "search_ball"
        return state["state"]

    if ball_kind == "free":
        if not _valid_ball(field_ball):
            peripheral.Dribble(False)
            UnitedPosition.Pos2Pos(_safe_target(_home(direction, profile)), profile["search_gain"])
            state["state"] = "search_ball"
            return state["state"]
        if pocket is not None:
            bx, by = float(field_ball[0]), float(field_ball[1])
            target = [
                bx + direction * (profile["approach_standoff"] + 14.0),
                clamp(by * 0.55, -45.0, 45.0),
                goal_yaw,
            ]
            peripheral.Dribble(False)
            UnitedPosition.Pos2Pos(_safe_target(target), profile["search_gain"])
            state["state"] = "pocket_support"
            return state["state"]
        if _valid_ball(ball) and math.hypot(float(ball[0]), float(ball[1])) <= profile["capture_distance"]:
            peripheral.Dribble(0.75)
            lockball.LockBallSlip()
            state["state"] = "close_capture"
            return state["state"]
        bx, by = float(field_ball[0]), float(field_ball[1])
        target = [
            bx - direction * profile["approach_standoff"],
            clamp(by, -55.0, 55.0),
            goal_yaw,
        ]
        peripheral.Dribble(False)
        UnitedPosition.Pos2Pos(_safe_target(target), profile["search_gain"])
        state["state"] = "approach_ball"
        return state["state"]

    progress = direction * float(pose[0])
    lateral = abs(float(pose[1]))
    try:
        yaw = float(chassis.GetYaw())
    except Exception:
        yaw = float(pose[2])
    yaw_error = abs(((goal_yaw - yaw + 180.0) % 360.0) - 180.0)
    if progress >= profile["shot_progress"] and lateral <= profile["shot_y"] and yaw_error <= 65.0 and now - state["last_shot"] >= 1.0:
        peripheral.Dribble(0.9)
        chassis.stop()
        peripheral.ShootBall()
        peripheral.Dribble(False)
        state["last_shot"] = now
        state["shot_release_until"] = now + 0.20
        state["state"] = "shoot"
        return state["state"]

    target_y = clamp(-float(pose[1]) * 0.006, -profile["lane_limit"], profile["lane_limit"])
    peripheral.Dribble(0.9)
    chassis.FieldMoveVector(direction * profile["carry_speed"], target_y, goal_yaw, KpW=0.8)
    state["state"] = "carry_lane"
    return state["state"]


def _escape_tick(state: dict, profile: dict, direction: float) -> bool:
    if not profile.get("anti_stall") or state["escape_ticks"] <= 0:
        return False
    peripheral.Dribble(False)
    chassis.FieldMoveVector(
        -direction * 0.18,
        state["escape_side"] * 0.22,
        _goal_yaw(direction),
        KpW=0.7,
    )
    state["escape_ticks"] -= 1
    state["state"] = "anti_stall_escape"
    return True


def _anti_stall_escape(state: dict, profile: dict, pose, ball, ball_kind: str, direction: float, now: float) -> bool:
    """Start a short reverse/lateral escape after pose or free-ball stagnation."""
    if not profile.get("anti_stall"):
        return False

    current = (float(pose[0]), float(pose[1]))
    previous = state["progress_pose"]
    if previous is None:
        state["progress_pose"] = current
        state["progress_ts"] = now
        return False
    if math.hypot(current[0] - previous[0], current[1] - previous[1]) >= 2.0:
        state["progress_pose"] = current
        state["progress_ts"] = now
    pose_stuck = now - state["progress_ts"] >= 1.5
    free_ball_stuck = ball_kind == "free" and now - state["free_ball_ts"] >= 0.9
    if not pose_stuck and not free_ball_stuck:
        return False
    if now - state["last_escape_ts"] < 1.5:
        return False

    state["escape_ticks"] = 5
    state["progress_pose"] = current
    state["progress_ts"] = now
    state["last_escape_ts"] = now
    return _escape_tick(state, profile, direction)


def step(state: dict, role: str = "attacker") -> str:
    """Execute one bounded control tick and return a traceable state name."""
    profile = PROFILES[state["profile"]]
    direction = attack_direction()
    pose = UnitedPosition.FieldRobotPose()
    if not valid_pose(pose):
        state["state"] = _stop_tick()
        return state["state"]
    if not UnitedPosition.RestrictedArea():
        # RestrictedArea already dispatched Pos2Pos toward a projected legal pose.
        # Do not overwrite that nonblocking recovery command with a stop.
        peripheral.Dribble(False)
        state["state"] = "restricted_recover"
        return state["state"]

    ball = UnitedPosition.RobotBallPos()
    ball_kind = _ball_state(ball)
    now = time.monotonic()
    if state["last_ball_kind"] != ball_kind:
        state["last_ball_kind"] = ball_kind
        state["progress_pose"] = (float(pose[0]), float(pose[1]))
        state["progress_ts"] = now
        state["free_ball_ts"] = now if ball_kind == "free" else 0.0
        state["escape_ticks"] = 0
        state["pocket_clear_ticks"] = 0
        state["pocket_clear_complete"] = False
        state["last_escape_ts"] = -999.0
    if _valid_ball(ball):
        state["last_ball"] = [float(ball[0]), float(ball[1])]
        state["last_ball_ts"] = now

    if _escape_tick(state, profile, direction):
        return state["state"]
    if ball_kind in {"free", "held"} and _anti_stall_escape(state, profile, pose, ball, ball_kind, direction, now):
        return state["state"]

    if ball_kind == "free" and role == "attacker" and profile.get("anti_stall"):
        corner_target = _corner_recovery_target(profile, UnitedPosition.FieldBallPos(), direction)
        if corner_target is not None:
            peripheral.Dribble(False)
            UnitedPosition.Pos2Pos(_safe_target(corner_target), profile["search_gain"])
            state["state"] = "corner_recovery"
            return state["state"]

    if profile.get("field_controller"):
        return _active_field_step(state, profile, role, pose, ball, ball_kind, direction, now)

    if role == "defender":
        if ball_kind == "held":
            peripheral.Dribble(0.55)
            chassis.RobotMoveVector(0.45, 0.0, 0.0)
            state["state"] = "defender_clear"
            return state["state"]
        if ball_kind == "lost":
            peripheral.Dribble(False)
            UnitedPosition.Pos2Pos(
                _safe_target(_lost_patrol_target(state, profile, role, direction, now)),
                profile["search_gain"],
            )
            state["state"] = "defender_home"
            return state["state"]
        field_ball = UnitedPosition.FieldBallPos()
        if ball_kind == "free" and _valid_ball(field_ball):
            bx, by = float(field_ball[0]), float(field_ball[1])
            own_progress = -direction * bx
            if math.hypot(float(ball[0]), float(ball[1])) <= profile["press_distance"] and own_progress < 30.0:
                lockball.LockBallSlip()
                state["state"] = "defender_press"
                return state["state"]
            target = [-direction * profile["defence_x"], clamp(by * 0.72, -48.0, 48.0), _goal_yaw(direction)]
            UnitedPosition.Pos2Pos(target, profile["search_gain"])
            state["state"] = "defender_cover"
            return state["state"]
        UnitedPosition.Pos2Pos(_home(direction, profile), profile["search_gain"])
        state["state"] = "defender_home"
        return state["state"]

    if state["shot_release_until"] > now:
        chassis.FieldMoveVector(-direction * 0.18, 0.24, _goal_yaw(direction), KpW=0.7)
        state["state"] = "shot_release"
        return state["state"]

    if ball_kind == "lost":
        if state["last_ball"] is not None and now - state["last_ball_ts"] <= 0.30:
            lockball.LockBallSlip()
            state["state"] = "stale_chase"
            return state["state"]
        peripheral.Dribble(False)
        UnitedPosition.Pos2Pos(
            _safe_target(_lost_patrol_target(state, profile, role, direction, now)),
            profile["search_gain"],
        )
        state["state"] = "search_ball"
        return state["state"]

    if ball_kind == "free":
        peripheral.Dribble(0.85)
        lockball.LockBallSlip()
        state["state"] = "acquire_ball"
        return state["state"]

    progress = direction * float(pose[0])
    lateral = abs(float(pose[1]))
    try:
        yaw = float(chassis.GetYaw())
    except Exception:
        yaw = float(pose[2])
    target_yaw = _goal_yaw(direction)
    yaw_error = abs(((target_yaw - yaw + 180.0) % 360.0) - 180.0)
    if progress >= profile["shot_progress"] and lateral <= profile["shot_y"] and yaw_error <= 65.0 and now - state["last_shot"] >= 1.0:
        peripheral.Dribble(0.9)
        chassis.stop()
        peripheral.ShootBall()
        peripheral.Dribble(False)
        state["last_shot"] = now
        state["shot_release_until"] = now + 0.20
        state["state"] = "shoot"
        return state["state"]

    target_y = clamp(-float(pose[1]) * 0.006, -0.25, 0.25)
    chassis.FieldMoveVector(direction * profile["carry_speed"], target_y, target_yaw, KpW=0.8)
    state["state"] = "carry_lane"
    return state["state"]


def stop() -> None:
    safe_stop(chassis, peripheral, UnitedPosition)
