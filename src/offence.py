from BasicFuc import *
import math
import os
import time

from LunaPre.strategy_tools import BALL_HELD, BALL_NOT_FOUND, angle_error_deg, clamp, safe_stop, valid_pose


def _env_float(name, default):
    try:
        return float(os.environ.get(name, default))
    except Exception:
        return float(default)


OFFENCE_DT = _env_float("OFFENCE_DT", "0.03")
ATTACK_DIR = 1 if _env_float("OFFENCE_ATTACK_SIGN", "1") >= 0 else -1
SEARCH_SPEED_GAIN = 1.6
ROBOT_BALL_MAX_DISTANCE_CM = 240.0
ROBOT_BALL_MAX_ABS_X_CM = 260.0
ROBOT_BALL_MAX_ABS_Y_CM = 220.0
HELD_DEBOUNCE_S = 0.18
SHOT_COOLDOWN_S = 1.2

QUICK_CAPTURE_FORWARD_MIN_CM = -1.5
QUICK_CAPTURE_FORWARD_MAX_CM = 12.5
QUICK_CAPTURE_LATERAL_CM = 8.0

state = "boot"
last_decision = {}
_last_free_ball = None
_last_free_ball_ts = 0.0
_drive_intent = {}
_held_since = None
_last_shot_ts = -999999.0


def _search_home():
    home = list(Settings.CheckPoints.Home)
    while len(home) < 3:
        home.append(0)
    return [home[0] + 20, home[1], 0]


def _decision_phase(action) -> str:
    if action in {"search_ball", "quick_capture", "lockballslip_chase", "lockballslip_last_ball"}:
        return "acquire_ball"
    if action in {"own_half_carry", "advance_lane", "corner_recover", "sideline_drive"}:
        return "advance_lane"
    if action in {"direct_shot", "back_shot", "sideline_shot", "quick_shot", "shot_guard_hold"}:
        return "shoot"
    if action in {"post_goal_reset_hold", "shot_release"}:
        return "recover_lane"
    return "observe"


def _set_decision(**fields):
    global last_decision
    fields.setdefault("phase", _decision_phase(fields.get("action")))
    last_decision = fields
    return fields


def _remember_drive_intent(action: str, duration_s: float = 0.32, **fields) -> None:
    global _drive_intent
    _drive_intent = {"action": action, "until": time.monotonic() + duration_s, **fields}


def _active_drive_intent(action: str | None = None):
    if not _drive_intent:
        return None
    if time.monotonic() > float(_drive_intent.get("until", 0.0)):
        return None
    if action is not None and _drive_intent.get("action") != action:
        return None
    return _drive_intent


def _clear_drive_intent() -> None:
    global _drive_intent
    _drive_intent = {}


def _observe_ball_state(ball) -> None:
    global _held_since
    if ball == BALL_HELD:
        if _held_since is None:
            _held_since = time.monotonic()
    else:
        _held_since = None


def _held_is_stable() -> bool:
    if HELD_DEBOUNCE_S <= 0.0:
        return True
    return _held_since is not None and time.monotonic() - float(_held_since) >= HELD_DEBOUNCE_S


def _shot_guard_reason():
    if not _held_is_stable():
        return "held_debounce"
    if time.monotonic() - float(_last_shot_ts) < SHOT_COOLDOWN_S:
        return "shot_cooldown"
    return None


def _goal_angle() -> float:
    return UnitedPosition.Local2Angle([ATTACK_DIR * 110.0, 0.0]) % 360


def _fire_now() -> str:
    global _last_shot_ts

    guard_reason = _shot_guard_reason()
    if guard_reason:
        blocked = dict(last_decision)
        peripheral.Dribble(0.9)
        chassis.stop()
        _set_decision(
            action="shot_guard_hold",
            mode=guard_reason,
            blocked_action=blocked.get("action"),
            blocked_gate=blocked.get("shot_gate"),
        )
        return "shot_guard_hold"

    _clear_drive_intent()
    peripheral.Dribble(0.85)
    peripheral.ShootBall()
    _last_shot_ts = time.monotonic()
    peripheral.Dribble(False)
    chassis.stop()
    _remember_drive_intent("shot_release", 0.18)
    return "quick_shot"


def _shot_release_step() -> str:
    try:
        pose = UnitedPosition.FieldRobotPos()
    except Exception:
        pose = [0.0, 0.0, 0.0]

    pose_x = float(pose[0])
    pose_y = float(pose[1])
    side = 1.0 if pose_y <= 0.0 else -1.0
    target_yaw = 0.0 if ATTACK_DIR > 0 else 180.0
    speed_x = -ATTACK_DIR * 18.0
    speed_y = side * 32.0

    peripheral.Dribble(False)
    chassis.FieldMoveVector(speed_x, speed_y, target_yaw, KpW=0.7)
    _set_decision(
        action="shot_release",
        mode="post_shot_peel",
        pose=[round(pose_x, 3), round(pose_y, 3), round(float(pose[2]), 3)],
        speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(target_yaw), 3)],
    )
    return "shot_release"


def _carry_own_half(goal_angle, pose) -> str:
    x, y, _ = pose
    progress = ATTACK_DIR * float(x)
    speed_x = ATTACK_DIR * (62.0 if progress > -35.0 else 54.0)
    speed_y = clamp(-float(y) * 0.55, -24.0, 24.0)
    peripheral.Dribble(0.9)
    chassis.FieldMoveVector(speed_x, speed_y, goal_angle, KpW=0.8)
    _set_decision(
        action="own_half_carry",
        progress=round(float(progress), 3),
        y=round(float(y), 3),
        speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(goal_angle), 3)],
    )
    return "own_half_carry"


def _valid_free_ball(ball) -> bool:
    if ball in (BALL_NOT_FOUND, BALL_HELD):
        return False
    if not ball or len(ball) < 2:
        return False
    try:
        bx = float(ball[0])
        by = float(ball[1])
    except Exception:
        return False
    if not math.isfinite(bx) or not math.isfinite(by):
        return False
    if abs(bx) > ROBOT_BALL_MAX_ABS_X_CM or abs(by) > ROBOT_BALL_MAX_ABS_Y_CM:
        return False
    return math.hypot(bx, by) <= ROBOT_BALL_MAX_DISTANCE_CM


def _remember_free_ball(ball) -> None:
    global _last_free_ball, _last_free_ball_ts
    if not _valid_free_ball(ball):
        return
    _last_free_ball = [float(ball[0]), float(ball[1])]
    _last_free_ball_ts = time.monotonic()


def _recent_free_ball():
    if _last_free_ball is None:
        return None
    if time.monotonic() - _last_free_ball_ts > 0.35:
        return None
    return _last_free_ball


def _direct_chase_ball(ball, *, stale=False) -> str:
    bx = float(ball[0])
    by = float(ball[1])
    dist = math.hypot(bx, by)
    angle = math.degrees(math.atan2(by, bx)) if dist > 1e-6 else 0.0

    if QUICK_CAPTURE_FORWARD_MIN_CM <= bx <= QUICK_CAPTURE_FORWARD_MAX_CM and abs(by) <= QUICK_CAPTURE_LATERAL_CM:
        try:
            pose = UnitedPosition.FieldRobotPos()
            field_long = float(getattr(Settings.Bounds, "Long", 260))
            field_short = float(getattr(Settings.Bounds, "Short", 200))
            if valid_pose(pose) and abs(float(pose[0])) <= field_long * 0.5 + 20.0 and abs(float(pose[1])) <= field_short * 0.5 + 20.0:
                peripheral.Dribble(0.95)
                controlled_state = _controlled_ball_step(pose)
                last_decision["control_source"] = "quick_capture"
                last_decision["contact_ball"] = [round(bx, 3), round(by, 3)]
                last_decision["contact_distance"] = round(float(dist), 3)
                return controlled_state
        except Exception:
            pass

        peripheral.Dribble(0.95)
        speed_x = 28.0
        speed_y = clamp(by * 1.2, -12.0, 12.0)
        speed_w = clamp(angle * 0.35, -24.0, 24.0)
        chassis.RobotMoveVector(speed_x, speed_y, speed_w)
        _set_decision(
            action="quick_capture",
            mode="dribbler_window",
            ball=[round(bx, 3), round(by, 3)],
            distance=round(float(dist), 3),
            angle=round(float(angle), 3),
            speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(speed_w), 3)],
        )
        return "quick_capture"

    peripheral.Dribble(0.85)
    locked = bool(lockball.LockBallSlip())
    action = "lockballslip_last_ball" if stale else "lockballslip_chase"
    _set_decision(
        action=action,
        mode="lockballslip_chase",
        ball=[round(bx, 3), round(by, 3)],
        distance=round(float(dist), 3),
        angle=round(float(angle), 3),
        locked=locked,
    )
    return action


def _lock_or_chase_free_ball(ball) -> str:
    _remember_free_ball(ball)
    bx = float(ball[0])
    by = float(ball[1])
    try:
        pose = UnitedPosition.FieldRobotPos()
        field_ball = UnitedPosition.FieldBallPos()
        field_long = float(getattr(Settings.Bounds, "Long", 260))
        field_short = float(getattr(Settings.Bounds, "Short", 200))
        field_pose_valid = (
            valid_pose(pose)
            and field_ball not in (BALL_NOT_FOUND, BALL_HELD)
            and len(field_ball) >= 2
            and abs(float(pose[0])) <= field_long * 0.5 + 20.0
            and abs(float(pose[1])) <= field_short * 0.5 + 20.0
            and abs(float(field_ball[0])) <= field_long * 0.5 + 20.0
            and abs(float(field_ball[1])) <= field_short * 0.5 + 20.0
        )
        yaw_rad = math.radians(float(pose[2]))
        expected_ball_x = float(pose[0]) + bx * math.cos(yaw_rad) - by * math.sin(yaw_rad)
        expected_ball_y = float(pose[1]) + bx * math.sin(yaw_rad) + by * math.cos(yaw_rad)
        field_ball_matches_robot = math.hypot(expected_ball_x - float(field_ball[0]), expected_ball_y - float(field_ball[1])) <= 20.0
        restricted_cfg = getattr(getattr(Settings, "Motion", None), "RestrictedArea", None)
        drivable_half_y = float(
            getattr(restricted_cfg, "YLimit", min(70.0, float(getattr(Settings.Bounds, "Short", 200)) / 2.0 - 30.0))
        )

        if (
            field_pose_valid
            and field_ball_matches_robot
            and ATTACK_DIR * float(field_ball[0]) >= 108.0
            and abs(float(field_ball[1])) >= drivable_half_y + 12.0
        ):
            _clear_drive_intent()
            target_x = ATTACK_DIR * 46.0
            target_y = 0.0
            target_yaw = 0.0 if ATTACK_DIR > 0 else 180.0
            speed_x = clamp((target_x - float(pose[0])) * 1.2, -46.0, 46.0)
            speed_y = clamp((target_y - float(pose[1])) * 1.2, -44.0, 44.0)
            peripheral.Dribble(False)
            chassis.FieldMoveVector(speed_x, speed_y, target_yaw, KpW=0.65)
            _set_decision(
                action="post_goal_reset_hold",
                mode="net_ball",
                ball=[round(bx, 3), round(by, 3)],
                field_ball=[round(float(field_ball[0]), 3), round(float(field_ball[1]), 3)],
                pose=[round(float(pose[0]), 3), round(float(pose[1]), 3), round(float(pose[2]), 3)],
                target=[round(float(target_x), 3), round(float(target_y), 3), round(float(target_yaw), 3)],
                speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(target_yaw), 3)],
            )
            return "post_goal_reset_hold"
    except Exception:
        pass
    return _direct_chase_ball(ball)


def _controlled_ball_step(pose) -> str:
    x, y, _ = pose
    progress = ATTACK_DIR * float(x)
    abs_y = abs(float(y))
    goal_angle = _goal_angle()
    yaw_error = angle_error_deg(chassis.GetYaw(), goal_angle)
    abs_yaw_error = abs(float(yaw_error))

    if progress < -45.0 and abs_y >= 58.0:
        recover_angle = UnitedPosition.Local2Angle([ATTACK_DIR * 85.0, 0.0]) % 360
        speed_x = ATTACK_DIR * 46.0
        speed_y = clamp(-float(y) * 0.65, -44.0, 44.0)
        peripheral.Dribble(0.9)
        chassis.FieldMoveVector(speed_x, speed_y, recover_angle, KpW=0.9)
        _set_decision(
            action="corner_recover",
            progress=round(float(progress), 3),
            y=round(float(y), 3),
            speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(recover_angle), 3)],
        )
        return "corner_recover"

    if progress < 0.0:
        return _carry_own_half(goal_angle, pose)

    if progress >= 24.0 and abs_y >= 34.0 and UnitedPosition.RobotBallPos() == BALL_HELD:
        shot = bool(Slipsideshot(attack_dir=ATTACK_DIR))
        action = "sideline_shot" if shot else "sideline_drive"
        _set_decision(action=action, progress=round(float(progress), 3), y=round(float(y), 3))
        return action

    back_angle = (goal_angle + 180.0) % 360.0
    back_error = angle_error_deg(chassis.GetYaw(), back_angle)
    if progress >= 72.0 and abs_y <= 20.0 and abs(float(back_error)) <= 28.0:
        _set_decision(
            action="back_shot",
            shot_family="back_shot",
            progress=round(float(progress), 3),
            y=round(float(y), 3),
            yaw_error=round(float(yaw_error), 3),
            back_error=round(float(back_error), 3),
            goal_angle=round(float(goal_angle), 3),
        )
        return _fire_now()

    shot_allowed = False
    if abs_y > 69.75:
        shot_reason = "too_wide"
    elif abs_y > 45.0 and progress < 50.0:
        shot_reason = "wide_before_pressure_zone"
    elif progress >= 38.0 and abs_y <= 12.0:
        shot_allowed = abs_yaw_error <= 46.0
        shot_reason = "open_goal" if shot_allowed else "open_goal_yaw_not_ready"
    elif progress >= 65.0:
        shot_allowed = abs_yaw_error <= 70.0
        shot_reason = "quick_zone" if shot_allowed else "quick_zone_not_ready"
    elif progress >= 50.0:
        shot_allowed = abs_yaw_error <= 70.0
        shot_reason = "pressure_zone" if shot_allowed else "pressure_zone_not_ready"
    elif progress >= 38.0:
        shot_allowed = abs_yaw_error <= 36.0
        shot_reason = "opportunity" if shot_allowed else "opportunity_not_ready"
    elif progress >= 35.0:
        shot_allowed = abs_yaw_error <= 25.0
        shot_reason = "fast_attack_window" if shot_allowed else "fast_attack_not_ready"
    else:
        shot_reason = "advance_more"

    if shot_allowed:
        shot_family = "direct_shot" if shot_reason == "open_goal" else "quick_shot"
        _set_decision(
            action=shot_family,
            shot_family=shot_family,
            shot_gate=shot_reason,
            progress=round(float(progress), 3),
            y=round(float(y), 3),
            yaw_error=round(float(yaw_error), 3),
            goal_angle=round(float(goal_angle), 3),
        )
        return _fire_now()

    speed_x = 65.0 if progress >= 35.0 else 52.0
    speed_y = clamp(-float(y), -44.0, 44.0)
    peripheral.Dribble(0.85)
    chassis.FieldMoveVector(speed_x * ATTACK_DIR, speed_y, goal_angle, KpW=0.9)
    _set_decision(
        action="advance_lane",
        mode="fast" if progress >= 35.0 else "setup",
        shot_gate=shot_reason,
        progress=round(float(progress), 3),
        y=round(float(y), 3),
        yaw_error=round(float(yaw_error), 3),
        speed=[round(float(speed_x * ATTACK_DIR), 3), round(float(speed_y), 3), round(float(goal_angle), 3)],
    )
    return "advance_lane"


def OffenceStep():
    global state

    ball = UnitedPosition.RobotBallPos()
    if ball != BALL_HELD and ball != BALL_NOT_FOUND and not _valid_free_ball(ball):
        ball = BALL_NOT_FOUND
    _observe_ball_state(ball)
    search_home = _search_home()

    if _active_drive_intent("shot_release"):
        state = _shot_release_step()
        return state

    if ball == BALL_NOT_FOUND or (ball != BALL_HELD and not _valid_free_ball(ball)):
        recent_ball = _recent_free_ball()
        if recent_ball is not None:
            state = _direct_chase_ball(recent_ball, stale=True)
            return state
        peripheral.Dribble(0.7)
        UnitedPosition.Pos2Pos(search_home, SEARCH_SPEED_GAIN)
        _set_decision(action="search_ball", target=search_home, speed_gain=round(float(SEARCH_SPEED_GAIN), 3))
        state = "search_ball"
        return state

    if ball != BALL_HELD:
        state = _lock_or_chase_free_ball(ball)
        return state

    try:
        pose = UnitedPosition.FieldRobotPos()
    except Exception:
        pose = [0, 0, 0]
    state = _controlled_ball_step(pose)
    return state


def OffenceStop():
    global state
    safe_stop(chassis, peripheral, UnitedPosition)
    state = "stopped"


def main():
    try:
        while True:
            OffenceStep()
            time.sleep(OFFENCE_DT)
    finally:
        OffenceStop()


if __name__ == "__main__":
    main()
