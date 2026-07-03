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

SIDELINE_BOUNDARY_RESERVE_CM = _env_float("OFFENCE_SIDELINE_BOUNDARY_RESERVE_CM", "18.0")
SIDELINE_WIDE_SHOT_MARGIN_CM = 14.0
SIDELINE_EARLY_SHOT_PROGRESS_CM = 82.0
SIDELINE_EARLY_SHOT_MIN_ABS_Y_CM = 24.0
SIDELINE_EARLY_SHOT_MAX_ABS_Y_CM = 45.0
SIDELINE_EARLY_SHOT_YAW_DEG = 12.0
SIDELINE_GOAL_HALF_WIDTH_CM = 42.0

QUICK_CAPTURE_FORWARD_MIN_CM = -1.5
QUICK_CAPTURE_FORWARD_MAX_CM = 12.5
QUICK_CAPTURE_LATERAL_CM = 8.0

state = "boot"
last_decision = {}
_last_free_ball = None
_last_free_ball_ts = 0.0
_drive_intent = {}


def _search_home():
    home = list(Settings.CheckPoints.Home)
    while len(home) < 3:
        home.append(0)
    return [home[0] + 20, home[1], 0]


def _decision_phase(action) -> str:
    if action in {"search_ball", "quick_capture", "lockballslip_chase", "lockballslip_last_ball"}:
        return "acquire_ball"
    if action in {"own_half_carry", "advance_lane", "corner_recover", "sideline_hide", "sideline_drive"}:
        return "advance_lane"
    if action in {"direct_shot", "back_shot", "sideline_shot", "quick_shot"}:
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


def _goal_angle() -> float:
    return UnitedPosition.Local2Angle([ATTACK_DIR * 110.0, 0.0]) % 360


def _fire_now() -> str:
    _clear_drive_intent()
    peripheral.Dribble(0.85)
    peripheral.ShootBall()
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


def _sideline_goal_target(y: float, side: float | None = None) -> list[float]:
    side = (1.0 if float(y) >= 0.0 else -1.0) if side is None else (1.0 if float(side) >= 0.0 else -1.0)
    return [ATTACK_DIR * 110.0, -side * 18.0]


def _sideline_lane_abs_y() -> float:
    half_y = float(getattr(Settings.Bounds, "Short", 200)) * 0.5
    reserve = max(16.0, float(SIDELINE_BOUNDARY_RESERVE_CM))
    return max(50.0, half_y - reserve)


def _sideline_lane_y(side: float) -> float:
    return (1.0 if float(side) >= 0.0 else -1.0) * _sideline_lane_abs_y()


def _sideline_outside_yaw(y: float) -> float:
    return 90.0 if float(y) >= 0.0 else 270.0


def _sideline_locked_side(y: float, locked_sideline) -> float:
    if locked_sideline:
        if "side" in locked_sideline:
            return 1.0 if float(locked_sideline["side"]) >= 0.0 else -1.0
        if abs(float(locked_sideline.get("target_y", 0.0))) >= 30.0:
            return 1.0 if float(locked_sideline["target_y"]) >= 0.0 else -1.0
    return 1.0 if float(y) >= 0.0 else -1.0


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


def _sideline_lane_step(pose) -> str | None:
    x, y, _ = pose
    progress = ATTACK_DIR * float(x)
    abs_y = abs(float(y))
    lane_abs_y = _sideline_lane_abs_y()
    locked_sideline = _active_drive_intent()
    if locked_sideline and locked_sideline.get("action") not in {"sideline_drive", "sideline_hide"}:
        locked_sideline = None
    if not locked_sideline and not (progress >= 24.0 and 34.0 <= abs_y <= lane_abs_y + 4.0):
        return None

    side = _sideline_locked_side(float(y), locked_sideline)
    corner_y = _sideline_lane_y(side)
    restore_width = bool(locked_sideline and locked_sideline.get("action") == "sideline_drive") and abs_y < 50.0
    needs_corner_entry = progress < 68.0 or (progress < 74.0 and abs_y < lane_abs_y - 8.0) or restore_width
    hide_locked = bool(locked_sideline and locked_sideline.get("action") == "sideline_hide")

    if ((not locked_sideline or hide_locked) and needs_corner_entry) or restore_width:
        outward_yaw = _sideline_outside_yaw(side)
        if progress >= SIDELINE_EARLY_SHOT_PROGRESS_CM:
            speed_x = 0.0 if restore_width else ATTACK_DIR * 8.0
        else:
            speed_x = ATTACK_DIR * (38.0 if progress < 60.0 else 28.0)
        if abs_y < lane_abs_y - 20.0:
            speed_y = side * (14.0 if restore_width else 22.0)
        elif abs_y < lane_abs_y - 2.0:
            speed_y = side * 10.0
        else:
            speed_y = -side * 8.0

        peripheral.Dribble(0.92)
        chassis.FieldMoveVector(speed_x, speed_y, outward_yaw, KpW=0.85)
        _remember_drive_intent("sideline_hide", 0.55, target_y=corner_y, side=side)
        _set_decision(
            action="sideline_hide",
            shot_gate="restore_width" if restore_width else "corner_entry",
            progress=round(float(progress), 3),
            y=round(float(y), 3),
            target=[round(float(ATTACK_DIR * 68.0), 3), round(float(corner_y), 3), round(float(outward_yaw), 3)],
            speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(outward_yaw), 3)],
        )
        return "sideline_hide"

    sideline_target = _sideline_goal_target(float(y), side=side)
    sideline_angle = UnitedPosition.Local2Angle(sideline_target) % 360
    yaw = float(chassis.GetYaw())
    sideline_error = angle_error_deg(yaw, sideline_angle)
    sideline_error_abs = abs(float(sideline_error))
    yaw_rad = math.radians(yaw)
    heading_x = math.cos(yaw_rad)
    goal_dx = ATTACK_DIR * 110.0 - float(x)
    shot_y_at_goal = None
    if ATTACK_DIR * heading_x > 0.12 and ATTACK_DIR * goal_dx > 0.0:
        shot_y_at_goal = float(y) + math.sin(yaw_rad) * (goal_dx / heading_x)
    shot_line_hits_goal = shot_y_at_goal is not None and abs(float(shot_y_at_goal)) <= SIDELINE_GOAL_HALF_WIDTH_CM

    wide_sideline_window = (
        progress >= 68.0
        and abs_y >= lane_abs_y - SIDELINE_WIDE_SHOT_MARGIN_CM
        and sideline_error_abs <= 18.0
        and shot_line_hits_goal
    )
    early_shot_window = (
        progress >= SIDELINE_EARLY_SHOT_PROGRESS_CM
        and SIDELINE_EARLY_SHOT_MIN_ABS_Y_CM <= abs_y <= SIDELINE_EARLY_SHOT_MAX_ABS_Y_CM
        and sideline_error_abs <= SIDELINE_EARLY_SHOT_YAW_DEG
        and shot_line_hits_goal
    )

    if wide_sideline_window or early_shot_window:
        _set_decision(
            action="sideline_shot",
            shot_gate="early_sideline_shot_window" if early_shot_window and not wide_sideline_window else "decisive_sideline_window",
            progress=round(float(progress), 3),
            y=round(float(y), 3),
            yaw_error=round(float(sideline_error), 3),
            shot_y_at_goal=None if shot_y_at_goal is None else round(float(shot_y_at_goal), 3),
            target=[round(float(sideline_target[0]), 3), round(float(sideline_target[1]), 3)],
        )
        return _fire_now()

    if needs_corner_entry:
        shot_gate = "sideline_build_progress"
    elif abs_y < lane_abs_y - SIDELINE_WIDE_SHOT_MARGIN_CM:
        shot_gate = "sideline_width_not_ready"
    elif not shot_line_hits_goal:
        shot_gate = "sideline_goal_line_not_ready"
    else:
        shot_gate = "sideline_yaw_not_ready"

    _remember_drive_intent("sideline_drive", 0.65, target_y=sideline_target[1], side=side)
    speed_x = ATTACK_DIR * (46.0 if progress < 58.0 else (28.0 if progress < 78.0 else 18.0))
    if progress >= 86.0:
        speed_x = -ATTACK_DIR * 8.0
    speed_y = side * clamp(lane_abs_y - abs_y, -8.0, 12.0)
    peripheral.Dribble(0.85)
    chassis.FieldMoveVector(speed_x, speed_y, sideline_angle, KpW=0.9)
    _set_decision(
        action="sideline_drive",
        shot_gate=shot_gate,
        progress=round(float(progress), 3),
        y=round(float(y), 3),
        yaw_error=round(float(sideline_error), 3),
        shot_y_at_goal=None if shot_y_at_goal is None else round(float(shot_y_at_goal), 3),
        target=[round(float(sideline_target[0]), 3), round(float(sideline_target[1]), 3)],
        speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(sideline_angle), 3)],
    )
    return "sideline_drive"


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
    return math.isfinite(bx) and math.isfinite(by)


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

    sideline_state = _sideline_lane_step(pose)
    if sideline_state is not None:
        return sideline_state

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
        UnitedPosition.Pos2Pos(search_home, 4.5)
        _set_decision(action="search_ball", target=search_home)
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
