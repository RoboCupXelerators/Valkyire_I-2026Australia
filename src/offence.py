from BasicFuc import *
import math
import os
import time

from LunaPre.simple_strategy import choose_attack_plan
from LunaPre.strategy_tools import BALL_HELD, BALL_NOT_FOUND, angle_error_deg, clamp, safe_stop, valid_pose

# 只保留跨 tick/运行方式有关的参数；具体策略阈值直接写在对应判断旁边。
def _env_float(name, default):
    try:
        return float(os.environ.get(name, default))
    except Exception:
        return float(default)


OFFENCE_DT = _env_float("OFFENCE_DT", "0.03")
ATTACK_DIR = 1 if _env_float("OFFENCE_ATTACK_SIGN", "1") >= 0 else -1
ROBOT_ID = int(_env_float("XEL_ROBOT_ID", "1"))
OFFENCE_SUPPORT_MODE = str(os.environ.get("OFFENCE_SUPPORT_MODE", "0")).strip().lower() in {"1", "true", "yes"}
OFFENCE_SUPPORT_SIDE = 1.0 if _env_float("OFFENCE_SUPPORT_SIDE", "1") >= 0 else -1.0
SIDELINE_BOUNDARY_RESERVE_CM = _env_float("OFFENCE_SIDELINE_BOUNDARY_RESERVE_CM", "18.0")
SIDELINE_COMPONENT_CENTER_FORWARD_MAX_CM = 17.0
SIDELINE_COMPONENT_ACQUIRE_OFFSET_CM = 8.0
SIDELINE_COMPONENT_RECENTER_FORWARD_CM = 8.5
SIDELINE_COMPONENT_RECENTER_BACKOFF_CM = 14.0
SIDELINE_COMPONENT_CENTERED_LATERAL_CM = 4.0

state = "boot"
last_decision = {}
_last_free_ball = None
_last_free_ball_ts = 0.0
_drive_intent = {}


def _component_mode() -> str:
    return str(os.environ.get("OFFENCE_COMPONENT_MODE", "")).strip().lower().replace("-", "_")


def _sideline_component_enabled() -> bool:
    return _component_mode() in {"sideline", "sideline_shot"}


def _search_home():
    home = list(Settings.CheckPoints.Home)
    while len(home) < 3:
        home.append(0)
    return [home[0] + 20, home[1], 0]


def _decision_phase(action) -> str:
    if action in {
        "search_ball",
        "chase_ball",
        "chase_last_ball",
        "turn_to_ball",
        "near_ball_control",
        "sideline_free_ball_approach",
        "sideline_free_ball_recover",
        "ball_rear_reposition",
    }:
        return "acquire_ball"
    if action in {
        "own_half_carry",
        "advance_lane",
        "sideline_hide",
        "sideline_drive",
        "wall_setup",
        "penalty_mouth_peel",
    }:
        return "advance_lane"
    if action in {"attack_dodge", "pressure_release_shot"}:
        return "handle_pressure"
    if action in {
        "quick_shot",
        "direct_shot",
        "back_shot",
        "wall_shot",
        "sideline_shot",
        "oh_my_back_shot",
    }:
        return "shoot"
    if action in {
        "corner_recover",
        "deep_corner_release",
        "oh_my_back",
        "own_backline_escape",
        "own_half_wide_clear",
        "post_goal_reset_hold",
        "shot_release",
    }:
        return "recover_lane"
    return "observe"


def _set_decision(**fields):
    """记录本 tick 的关键判断，供真实场景 trace 复盘。"""
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


def _relevant_enemies(pose, enemies):
    """过滤掉后方和远侧 chassis，避免队友或静态占位干扰进攻判断。"""
    try:
        x, y = float(pose[0]), float(pose[1])
    except Exception:
        return []

    result = []
    for enemy in enemies or []:
        if not enemy or len(enemy) < 2:
            continue
        ex, ey = float(enemy[0]), float(enemy[1])
        forward_gap = ATTACK_DIR * (ex - x)
        lateral_gap = ey - y
        if forward_gap < -10:
            continue
        if forward_gap > 130:
            continue
        if abs(lateral_gap) > 70:
            continue
        result.append(enemy)
    return result


def _blocking_enemy(pose, enemies):
    """返回当前推进路径上最需要处理的拦截者。"""
    try:
        x, y = float(pose[0]), float(pose[1])
    except Exception:
        return None

    nearest = None
    nearest_dist = 9999.0
    for enemy in enemies or []:
        if not enemy or len(enemy) < 2:
            continue
        ex, ey = float(enemy[0]), float(enemy[1])
        forward_gap = ATTACK_DIR * (ex - x)
        lateral_gap = ey - y
        if not 12 < forward_gap <= 82:
            continue
        if abs(lateral_gap) > 34:
            continue

        dist = math.hypot(forward_gap, lateral_gap)
        if dist < nearest_dist:
            nearest_dist = dist
            nearest = {
                "x": ex,
                "y": ey,
                "forward_gap": forward_gap,
                "lateral_gap": lateral_gap,
                "dist": dist,
            }
    return nearest


def _front_pressure_enemies(pose, enemies, *, max_gap: float = 36.0, max_lateral: float = 26.0):
    """Return blockers close enough to cause a pushing call if we keep carrying."""
    try:
        x, y = float(pose[0]), float(pose[1])
    except Exception:
        return []

    blockers = []
    for enemy in enemies or []:
        if not enemy or len(enemy) < 2:
            continue
        ex, ey = float(enemy[0]), float(enemy[1])
        forward_gap = ATTACK_DIR * (ex - x)
        lateral_gap = ey - y
        if 10.0 < forward_gap <= max_gap and abs(lateral_gap) <= max_lateral:
            blockers.append(
                {
                    "x": ex,
                    "y": ey,
                    "forward_gap": forward_gap,
                    "lateral_gap": lateral_gap,
                    "dist": math.hypot(forward_gap, lateral_gap),
                }
            )
    blockers.sort(key=lambda item: item["dist"])
    return blockers


def _lane_score(pose, target, enemies, block_width=12.0) -> float:
    """给从机器人到目标点的射门线路打分，越高表示越不容易被挡。"""
    sx, sy = float(pose[0]), float(pose[1])
    ex, ey = float(target[0]), float(target[1])
    vx, vy = ex - sx, ey - sy
    denom = vx * vx + vy * vy
    if denom <= 1e-6:
        return 0.0

    score = 1.0
    for enemy in enemies or []:
        if not enemy or len(enemy) < 2:
            continue
        px, py = float(enemy[0]), float(enemy[1])
        t = ((px - sx) * vx + (py - sy) * vy) / denom
        if not 0.05 <= t <= 0.98:
            continue

        closest_x = sx + t * vx
        closest_y = sy + t * vy
        distance_to_line = math.hypot(px - closest_x, py - closest_y)
        if distance_to_line < block_width:
            score -= 1.0
        else:
            score -= max(0.0, (block_width * 1.8 - distance_to_line) / 100.0)
    return score


def _best_goal_target(pose, enemies):
    """在球门左中右三个点里选一个最值得立即瞄准的目标。"""
    goal_x = ATTACK_DIR * 110
    aim_ys = [-42 * 0.9, 0.0, 42 * 0.9]
    scored = []
    for aim_y in aim_ys:
        target = [goal_x, aim_y]
        score = _lane_score(pose, target, enemies)
        scored.append((score - abs(aim_y) * 0.003, score, target))
    _, raw_score, target = max(scored, key=lambda item: item[0])
    return target, raw_score


def _fire_now() -> str:
    """执行一次果断射门，并关闭吸球避免射门后继续拖住球。"""
    _clear_drive_intent()
    peripheral.Dribble(0.85)
    peripheral.ShootBall()
    peripheral.Dribble(False)
    chassis.stop()
    _remember_drive_intent("shot_release", 0.18)
    return "quick_shot"


def _shot_release_step() -> str:
    """After a shot, peel away briefly so we do not re-catch the ball into a defender."""
    try:
        pose = UnitedPosition.FieldRobotPos()
    except Exception:
        pose = [0.0, 0.0, 0.0]
    try:
        enemies = UnitedPosition.FieldChassisPos()
    except Exception:
        enemies = []

    pose_x = float(pose[0])
    pose_y = float(pose[1])
    nearest_enemy = None
    if enemies:
        nearest_enemy = min(
            enemies,
            key=lambda enemy: math.hypot(float(enemy[0]) - pose_x, float(enemy[1]) - pose_y),
        )
    if nearest_enemy is not None and math.hypot(float(nearest_enemy[0]) - pose_x, float(nearest_enemy[1]) - pose_y) <= 54.0:
        side = 1.0 if pose_y >= float(nearest_enemy[1]) else -1.0
    else:
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
        blocker=nearest_enemy,
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


def _carry_own_half(goal_angle, pose, lane_score) -> str:
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
        lane_score=round(float(lane_score), 3),
        speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(goal_angle), 3)],
    )
    return "own_half_carry"


def _sideline_lane_step(pose, shot_lane_score: float, *, component_mode: bool = False) -> str | None:
    """Own the sideline decision with one clear gate: drive until ready, then shoot."""
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
    if not component_mode and progress >= 90.0 and abs_y >= 50.0:
        _clear_drive_intent()
        infield_y = side * 34.0
        recover_angle = UnitedPosition.Pos2Angle([float(x), float(y)], [ATTACK_DIR * 82.0, infield_y]) % 360
        speed_x = -ATTACK_DIR * 20.0
        speed_y = clamp((infield_y - float(y)) * 1.2, -34.0, 34.0)
        peripheral.Dribble(0.65)
        chassis.FieldMoveVector(speed_x, speed_y, recover_angle, KpW=0.85)
        _set_decision(
            action="deep_corner_release",
            shot_gate="sideline_too_deep_release",
            progress=round(float(progress), 3),
            y=round(float(y), 3),
            target=[round(float(ATTACK_DIR * 82.0), 3), round(float(infield_y), 3), round(float(recover_angle), 3)],
            speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(recover_angle), 3)],
        )
        return "deep_corner_release"

    corner_y = _sideline_lane_y(side)
    restore_width = (
        component_mode
        and bool(locked_sideline and locked_sideline.get("action") == "sideline_drive")
        and abs_y < 50.0
    )
    needs_corner_entry = progress < 68.0 or (progress < 74.0 and abs_y < lane_abs_y - 8.0) or restore_width
    hide_locked = bool(locked_sideline and locked_sideline.get("action") == "sideline_hide")
    if ((not locked_sideline or hide_locked) and needs_corner_entry) or restore_width:
        outward_yaw = _sideline_outside_yaw(side)
        if component_mode and progress >= 82.0:
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
            lane_score=round(float(shot_lane_score), 3),
            target=[round(float(ATTACK_DIR * 68.0), 3), round(float(corner_y), 3), round(float(outward_yaw), 3)],
            speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(outward_yaw), 3)],
        )
        return "sideline_hide"

    sideline_target = _sideline_goal_target(float(y), side=side)
    sideline_angle = UnitedPosition.Local2Angle(sideline_target) % 360
    sideline_error = angle_error_deg(chassis.GetYaw(), sideline_angle)
    sideline_error_abs = abs(float(sideline_error))

    if (
        progress >= 68.0
        and abs_y >= lane_abs_y - 10.0
        and shot_lane_score >= -0.55
        and sideline_error_abs <= 18.0
    ):
        _set_decision(
            action="sideline_shot",
            shot_gate="decisive_sideline_window",
            progress=round(float(progress), 3),
            y=round(float(y), 3),
            yaw_error=round(float(sideline_error), 3),
            lane_score=round(float(shot_lane_score), 3),
            target=[round(float(sideline_target[0]), 3), round(float(sideline_target[1]), 3)],
        )
        return _fire_now()

    if needs_corner_entry:
        shot_gate = "sideline_build_progress"
    elif shot_lane_score < -0.55:
        shot_gate = "sideline_lane_blocked"
    else:
        shot_gate = "sideline_yaw_not_ready"

    _remember_drive_intent("sideline_drive", 0.65, target_y=sideline_target[1], side=side)
    speed_x = ATTACK_DIR * (46.0 if progress < 58.0 else (28.0 if progress < 78.0 else 18.0))
    if component_mode and progress >= 86.0:
        speed_x = -ATTACK_DIR * 8.0
    if component_mode:
        speed_y = side * clamp(lane_abs_y - abs_y, -8.0, 12.0)
    else:
        speed_y = -side * (18.0 if progress < 58.0 else 10.0)
    peripheral.Dribble(0.85)
    chassis.FieldMoveVector(speed_x, speed_y, sideline_angle, KpW=0.9)
    _set_decision(
        action="sideline_drive",
        shot_gate=shot_gate,
        progress=round(float(progress), 3),
        y=round(float(y), 3),
        yaw_error=round(float(sideline_error), 3),
        lane_score=round(float(shot_lane_score), 3),
        target=[round(float(sideline_target[0]), 3), round(float(sideline_target[1]), 3)],
        speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(sideline_angle), 3)],
    )
    return "sideline_drive"


def _valid_free_ball(ball) -> bool:
    """判断球坐标是否是可追的普通观测，而不是持球/丢球哨兵值。"""
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
    """缓存最近一次可追球坐标，用来覆盖短暂丢帧。"""
    global _last_free_ball, _last_free_ball_ts
    if not _valid_free_ball(ball):
        return
    _last_free_ball = [float(ball[0]), float(ball[1])]
    _last_free_ball_ts = time.monotonic()


def _recent_free_ball():
    """返回短时间内的上一帧球坐标；超过窗口就视为不可追。"""
    if _last_free_ball is None:
        return None
    if time.monotonic() - _last_free_ball_ts > 0.35:
        return None
    return _last_free_ball


def _is_dribbler_contact_ball(bx: float, by: float) -> bool:
    return -1.5 <= bx <= 12.5 and abs(by) <= 8.0


def _direct_chase_ball(ball, *, stale=False) -> str:
    """用当前球坐标直接追球，避免可追球被搜索/锁球分支拖慢。"""
    bx = float(ball[0])
    by = float(ball[1])
    dist = math.hypot(bx, by)
    angle = math.degrees(math.atan2(by, bx)) if dist > 1e-6 else 0.0

    peripheral.Dribble(0.75)
    if bx < -8.0:
        if abs(by) < 4.0:
            turn_dir = 1.0
        else:
            turn_dir = 1.0 if by > 0 else -1.0
        speed_x = 0.0
        speed_y = clamp(by * 0.25, -10.0, 10.0)
        speed_w = turn_dir * clamp(abs(angle) * 0.22, 22.0, 48.0)
        chassis.RobotMoveVector(speed_x, speed_y, speed_w)
        _set_decision(
            action="turn_to_last_ball" if stale else "turn_to_ball",
            ball=[round(bx, 3), round(by, 3)],
            distance=round(float(dist), 3),
            angle=round(float(angle), 3),
            speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(speed_w), 3)],
        )
        return "turn_to_last_ball" if stale else "turn_to_ball"

    if dist >= 72.0:
        max_x = 145.0 if abs(by) <= 22.0 else 126.0
        lateral_limit = 88.0
    else:
        max_x = 112.0 if abs(by) <= 18.0 else 96.0
        lateral_limit = 72.0
    speed_x = clamp((bx - 8.5) * 2.45, -18.0, max_x)
    if 0 <= bx <= 8.5 + 5 and abs(by) <= 10:
        speed_x = max(speed_x, 22.0)
    speed_y = clamp(by * 2.05, -lateral_limit, lateral_limit)
    speed_w = clamp(angle * 1.05, -120.0, 120.0)
    chassis.RobotMoveVector(speed_x, speed_y, speed_w)
    _set_decision(
        action="chase_last_ball" if stale else "chase_ball",
        ball=[round(bx, 3), round(by, 3)],
        distance=round(float(dist), 3),
        angle=round(float(angle), 3),
        speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(speed_w), 3)],
    )
    return "chase_last_ball" if stale else "chase_ball"


def _support_free_ball(pose, field_ball) -> str:
    """Move a secondary attacker into a passing lane instead of double-rushing."""
    ball_x = float(field_ball[0])
    ball_y = float(field_ball[1])
    pose_x = float(pose[0])
    pose_y = float(pose[1])

    side = OFFENCE_SUPPORT_SIDE
    if abs(ball_y) > 36.0:
        side = -1.0 if ball_y > 0.0 else 1.0

    target_x = clamp(ball_x - ATTACK_DIR * 30.0, -86.0, 86.0)
    target_y = clamp(ball_y + side * 34.0, -58.0, 58.0)
    target_yaw = UnitedPosition.Pos2Angle([target_x, target_y], [ball_x, ball_y]) % 360
    speed_x = clamp((target_x - pose_x) * 1.15, -44.0, 44.0)
    speed_y = clamp((target_y - pose_y) * 1.15, -44.0, 44.0)

    peripheral.Dribble(0.2)
    chassis.FieldMoveVector(speed_x, speed_y, target_yaw, KpW=0.75)
    _set_decision(
        action="support_free_ball",
        robot_id=ROBOT_ID,
        field_ball=[round(ball_x, 3), round(ball_y, 3)],
        pose=[round(pose_x, 3), round(pose_y, 3), round(float(pose[2]), 3)],
        target=[round(float(target_x), 3), round(float(target_y), 3), round(float(target_yaw), 3)],
        speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(target_yaw), 3)],
    )
    return "support_free_ball"


def _lock_or_chase_free_ball(ball) -> str:
    """Free balls stay active: close balls still get forward/lateral pressure."""
    _remember_free_ball(ball)
    bx = float(ball[0])
    by = float(ball[1])
    field_pose_valid = False
    try:
        pose = UnitedPosition.FieldRobotPos()
        field_ball = UnitedPosition.FieldBallPos()
        robot_progress = ATTACK_DIR * float(pose[0])
        ball_progress = ATTACK_DIR * float(field_ball[0])
        restricted_cfg = getattr(getattr(Settings, "Motion", None), "RestrictedArea", None)
        drivable_half_y = float(
            getattr(
                restricted_cfg,
                "YLimit",
                min(70.0, float(getattr(Settings.Bounds, "Short", 200)) / 2.0 - 30.0),
            )
        )
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
        field_ball_matches_robot = (
            math.hypot(expected_ball_x - float(field_ball[0]), expected_ball_y - float(field_ball[1])) <= 20.0
        )
        if (
            field_pose_valid
            and field_ball_matches_robot
            and ball_progress >= 108.0
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
        if OFFENCE_SUPPORT_MODE and field_pose_valid:
            return _support_free_ball(pose, field_ball)
        if (
            field_pose_valid
            and field_ball_matches_robot
            and robot_progress >= 68.0
            and abs(float(pose[1])) >= 48.0
            and -4.0 <= bx <= 12.0
            and abs(by) <= 12.0
        ):
            side = 1.0 if float(pose[1]) >= 0.0 else -1.0
            sideline_target = _sideline_goal_target(float(pose[1]))
            sideline_angle = UnitedPosition.Local2Angle(sideline_target) % 360
            sideline_error = angle_error_deg(chassis.GetYaw(), sideline_angle)
            if abs(float(sideline_error)) <= 22.0:
                _set_decision(
                    action="sideline_shot",
                    mode="free_ball_contact",
                    shot_gate="decisive_sideline_window",
                    ball=[round(bx, 3), round(by, 3)],
                    field_ball=[round(float(field_ball[0]), 3), round(float(field_ball[1]), 3)],
                    pose=[round(float(pose[0]), 3), round(float(pose[1]), 3), round(float(pose[2]), 3)],
                    yaw_error=round(float(sideline_error), 3),
                    target=[round(float(sideline_target[0]), 3), round(float(sideline_target[1]), 3)],
                )
                return _fire_now()
            speed_x = ATTACK_DIR * 18.0
            speed_y = -side * 10.0
            peripheral.Dribble(0.88)
            chassis.FieldMoveVector(speed_x, speed_y, sideline_angle, KpW=0.9)
            _set_decision(
                action="sideline_drive",
                mode="free_ball_contact",
                shot_gate="sideline_yaw_not_ready",
                ball=[round(bx, 3), round(by, 3)],
                field_ball=[round(float(field_ball[0]), 3), round(float(field_ball[1]), 3)],
                pose=[round(float(pose[0]), 3), round(float(pose[1]), 3), round(float(pose[2]), 3)],
                yaw_error=round(float(sideline_error), 3),
                target=[round(float(sideline_target[0]), 3), round(float(sideline_target[1]), 3)],
                speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(sideline_angle), 3)],
            )
            return "sideline_drive"
        if (
            field_pose_valid
            and field_ball_matches_robot
            and abs(float(field_ball[1])) >= drivable_half_y - 4.0
            and abs(float(pose[1])) < drivable_half_y - 18.0
        ):
            side = 1.0 if float(field_ball[1]) >= 0.0 else -1.0
            safe_y = side * (drivable_half_y - 16.0)
            target_x = clamp(float(field_ball[0]) - ATTACK_DIR * 26.0, -86.0, 86.0)
            target_y = clamp(safe_y, -62.0, 62.0)
            target_yaw = UnitedPosition.Pos2Angle(
                [target_x, target_y],
                [float(field_ball[0]), clamp(float(field_ball[1]), -drivable_half_y, drivable_half_y)],
            ) % 360
            speed_x = clamp((target_x - float(pose[0])) * 1.35, -58.0, 58.0)
            speed_y = clamp((target_y - float(pose[1])) * 1.35, -52.0, 52.0)
            peripheral.Dribble(0.78)
            chassis.FieldMoveVector(speed_x, speed_y, target_yaw, KpW=0.9)
            _set_decision(
                action="sideline_free_ball_approach",
                ball=[round(bx, 3), round(by, 3)],
                field_ball=[round(float(field_ball[0]), 3), round(float(field_ball[1]), 3)],
                pose=[round(float(pose[0]), 3), round(float(pose[1]), 3), round(float(pose[2]), 3)],
                target=[round(float(target_x), 3), round(float(target_y), 3), round(float(target_yaw), 3)],
                speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(target_yaw), 3)],
            )
            return "sideline_free_ball_approach"
        if (
            field_pose_valid
            and field_ball_matches_robot
            and abs(float(field_ball[1])) >= drivable_half_y - 3.0
            and abs(float(pose[1])) >= drivable_half_y - 18.0
        ):
            side = 1.0 if float(field_ball[1]) >= 0.0 else -1.0
            pull_angle = UnitedPosition.Local2Angle([ATTACK_DIR * 70.0, -side * 44.0]) % 360
            speed_x = ATTACK_DIR * 30.0
            speed_y = -side * 46.0
            peripheral.Dribble(0.85)
            chassis.FieldMoveVector(speed_x, speed_y, pull_angle, KpW=0.9)
            _set_decision(
                action="sideline_free_ball_recover",
                ball=[round(bx, 3), round(by, 3)],
                field_ball=[round(float(field_ball[0]), 3), round(float(field_ball[1]), 3)],
                pose=[round(float(pose[0]), 3), round(float(pose[1]), 3), round(float(pose[2]), 3)],
                speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(pull_angle), 3)],
            )
            return "sideline_free_ball_recover"
        if ATTACK_DIR * float(pose[0]) < -68.0 and ball_progress < -88.0:
            clear_angle = UnitedPosition.Local2Angle([ATTACK_DIR * 95, 0]) % 360
            speed_x = ATTACK_DIR * 36
            speed_y = clamp(-float(field_ball[1]) * 0.45, -34.0, 34.0)
            peripheral.Dribble(0.75)
            chassis.FieldMoveVector(speed_x, speed_y, clear_angle, KpW=0.9)
            _set_decision(
                action="own_backline_escape",
                ball=[round(bx, 3), round(by, 3)],
                field_ball=[round(float(field_ball[0]), 3), round(float(field_ball[1]), 3)],
                speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(clear_angle), 3)],
            )
            return "own_backline_escape"
        if (
            field_pose_valid
            and field_ball_matches_robot
            and robot_progress < -22.0
            and ball_progress < -32.0
            and abs(float(field_ball[1])) >= 42.0
            and bx <= 46.0
            and abs(by) >= 4.0
        ):
            angle = math.degrees(math.atan2(by, bx)) if abs(bx) + abs(by) > 1e-6 else 0.0
            speed_x = 28.0 if abs(by) <= 18.0 else 18.0
            speed_y = clamp(by * 2.45, -44.0, 44.0)
            speed_w = clamp(angle * 0.15, -12.0, 12.0)
            peripheral.Dribble(0.9)
            chassis.RobotMoveVector(speed_x, speed_y, speed_w)
            _set_decision(
                action="own_half_wide_clear",
                ball=[round(bx, 3), round(by, 3)],
                field_ball=[round(float(field_ball[0]), 3), round(float(field_ball[1]), 3)],
                progress=round(float(robot_progress), 3),
                speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(speed_w), 3)],
            )
            return "own_half_wide_clear"
        if field_pose_valid and field_ball_matches_robot:
            behind_gap = ATTACK_DIR * (float(field_ball[0]) - float(pose[0]))
            close_ball = math.hypot(bx, by) <= 34.0
            pressure_enemy = None
            pressure_contact = -1.5 <= bx <= 12.5 and abs(by) <= 10.5
            if close_ball and pressure_contact and robot_progress >= -10.0:
                for enemy in _relevant_enemies(pose, UnitedPosition.FieldChassisPos()):
                    ex, ey = float(enemy[0]), float(enemy[1])
                    pressure_dist = math.hypot(ex - float(pose[0]), ey - float(pose[1]))
                    forward_gap = ATTACK_DIR * (ex - float(pose[0]))
                    lateral_gap = ey - float(pose[1])
                    if pressure_dist <= 38.0 and -4.0 <= forward_gap <= 38.0 and abs(lateral_gap) <= 32.0:
                        pressure_enemy = {
                            "x": round(ex, 3),
                            "y": round(ey, 3),
                            "dist": round(pressure_dist, 3),
                            "forward_gap": round(float(forward_gap), 3),
                            "lateral_gap": round(float(lateral_gap), 3),
                        }
                        break
            if pressure_enemy is not None:
                _set_decision(
                    action="pressure_release_shot",
                    mode="free_ball_contact",
                    ball=[round(bx, 3), round(by, 3)],
                    field_ball=[round(float(field_ball[0]), 3), round(float(field_ball[1]), 3)],
                    pose=[round(float(pose[0]), 3), round(float(pose[1]), 3), round(float(pose[2]), 3)],
                    pressure=pressure_enemy,
                )
                return _fire_now()
            if (
                close_ball
                and behind_gap < -6.0
                and robot_progress >= -10.0
                and abs(float(field_ball[1])) < drivable_half_y - 8.0
            ):
                # The robot is already past the ball along the attack axis.
                # Move back to the ball's rear side before pushing, otherwise
                # the higher-friction ball gets rubbed sideways in place.
                target_x = float(field_ball[0]) - ATTACK_DIR * 18.0
                target_y = clamp(float(field_ball[1]) - clamp(by * 0.65, -14.0, 14.0), -62.0, 62.0)
                target_yaw = UnitedPosition.Pos2Angle([target_x, target_y], [float(field_ball[0]), float(field_ball[1])]) % 360
                peripheral.Dribble(0.82)
                chassis.FieldMoveVector(
                    clamp((target_x - float(pose[0])) * 1.35, -38.0, 38.0),
                    clamp((target_y - float(pose[1])) * 1.35, -34.0, 34.0),
                    target_yaw,
                    KpW=0.9,
                )
                _set_decision(
                    action="ball_rear_reposition",
                    ball=[round(bx, 3), round(by, 3)],
                    field_ball=[round(float(field_ball[0]), 3), round(float(field_ball[1]), 3)],
                    pose=[round(float(pose[0]), 3), round(float(pose[1]), 3), round(float(pose[2]), 3)],
                    behind_gap=round(float(behind_gap), 3),
                    target=[round(float(target_x), 3), round(float(target_y), 3), round(float(target_yaw), 3)],
                )
                return "ball_rear_reposition"
            if _is_dribbler_contact_ball(bx, by) and robot_progress > -55.0:
                controlled_state = _controlled_ball_step(pose, _search_home())
                last_decision["control_source"] = "visible_dribbler_contact"
                last_decision["contact_ball"] = [round(bx, 3), round(by, 3)]
                return controlled_state
    except Exception:
        pass
    if math.hypot(bx, by) >= 30 or bx < -5:
        return _direct_chase_ball(ball)

    angle = math.degrees(math.atan2(by, bx)) if abs(bx) + abs(by) > 1e-6 else 0.0
    if -2.0 <= bx <= 12.0 and abs(by) <= 10.0:
        peripheral.Dribble(0.95)
        speed_x = clamp(34.0 + max(bx, 0.0) * 1.15, 32.0, 48.0)
        speed_y = clamp(by * 0.72, -14.0, 14.0)
        speed_w = clamp(angle * 0.38, -28.0, 28.0)
        chassis.RobotMoveVector(speed_x, speed_y, speed_w)
        _set_decision(
            action="near_ball_control",
            mode="capture",
            ball=[round(bx, 3), round(by, 3)],
            speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(speed_w), 3)],
        )
        return "near_ball_control"

    if bx <= 22.0 and abs(by) > 9.0:
        peripheral.Dribble(0.95)
        speed_x = clamp(24.0 + (bx - 8.5) * 0.9, 16.0, 40.0)
        speed_y = clamp(by * 2.2, -40.0, 40.0)
        speed_w = clamp(angle * 0.35, -30.0, 30.0)
        chassis.RobotMoveVector(speed_x, speed_y, speed_w)
        _set_decision(
            action="near_ball_control",
            mode="align",
            ball=[round(bx, 3), round(by, 3)],
            distance=round(float(math.hypot(bx, by)), 3),
            angle=round(float(angle), 3),
            speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(speed_w), 3)],
        )
        return "near_ball_control"

    peripheral.Dribble(0.9)
    speed_x = clamp(30.0 + (bx - 8.5) * 1.7, 28.0, 62.0)
    speed_y = clamp(by * 1.35, -28.0, 28.0)
    speed_w = clamp(angle * 0.7, -58.0, 58.0)
    chassis.RobotMoveVector(speed_x, speed_y, speed_w)
    _set_decision(
        action="near_ball_control",
        mode="acquire",
        ball=[round(bx, 3), round(by, 3)],
        distance=round(float(math.hypot(bx, by)), 3),
        angle=round(float(angle), 3),
        speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(speed_w), 3)],
    )
    return "near_ball_control"


def _attack_with_ball(goal_angle, pose, enemies, lane_score) -> str:
    """持球后持续推进；遇到单个拦截者时横向绕开，不停在原地等。"""
    x, y, _ = pose
    progress = ATTACK_DIR * float(x)
    blocker = _blocking_enemy(pose, enemies)
    locked_dodge = _active_drive_intent("attack_dodge")
    yaw_error = angle_error_deg(chassis.GetYaw(), goal_angle)
    abs_y = abs(float(y))
    abs_yaw_error = abs(float(yaw_error))
    front_pressure = _front_pressure_enemies(pose, enemies)
    if progress >= 30.0 and abs_y <= 24.0 and lane_score >= 0.72 and abs_yaw_error <= 32.0:
        _set_decision(
            action="direct_shot",
            shot_family="direct_shot",
            shot_gate="decisive_midfield_lane",
            progress=round(float(progress), 3),
            y=round(float(y), 3),
            yaw_error=round(float(yaw_error), 3),
            lane_score=round(float(lane_score), 3),
            goal_angle=round(float(goal_angle), 3),
        )
        return _fire_now()

    if progress >= 58.0 and abs_y >= 42.0:
        side = -1.0 if float(y) > 0 else 1.0
        peel_angle = UnitedPosition.Local2Angle([ATTACK_DIR * 76.0, side * 28.0]) % 360
        speed_x = ATTACK_DIR * 24.0
        speed_y = clamp(side * 34.0, -34.0, 34.0)
        peripheral.Dribble(0.85)
        chassis.FieldMoveVector(speed_x, speed_y, peel_angle, KpW=0.85)
        _set_decision(
            action="penalty_mouth_peel",
            progress=round(float(progress), 3),
            y=round(float(y), 3),
            yaw_error=round(float(yaw_error), 3),
            speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(peel_angle), 3)],
        )
        return "penalty_mouth_peel"

    if len(front_pressure) >= 2 and progress >= 10.0 and abs_y <= 34.0 and abs_yaw_error <= 95.0:
        _set_decision(
            action="pressure_release_shot",
            progress=round(float(progress), 3),
            y=round(float(y), 3),
            yaw_error=round(float(yaw_error), 3),
            lane_score=round(float(lane_score), 3),
            blockers=front_pressure[:2],
        )
        return _fire_now()

    force_allowed = False
    if abs_y > 45 * 1.35:
        force_reason = "too_wide_for_forced_shot"
    elif blocker is None:
        force_reason = "no_close_blocker"
    elif progress >= 30 and float(blocker["forward_gap"]) <= 28 and abs_y <= 45 * 1.1:
        if abs_yaw_error <= 60 and lane_score >= 0.0:
            force_allowed = True
            force_reason = "contested_window"
        else:
            force_reason = "contested_window_not_ready"
    elif progress < 50:
        force_reason = "forced_shot_before_threat_zone"
    elif float(blocker["forward_gap"]) > 28:
        force_reason = "blocker_not_urgent"
    elif abs_yaw_error <= 70 and lane_score >= -0.05:
        force_allowed = True
        force_reason = "pressure_blocker"
    else:
        force_reason = "pressure_blocker_not_ready"

    if force_allowed:
        _set_decision(
            action="quick_shot",
            force_gate=force_reason,
            progress=round(float(progress), 3),
            y=round(float(y), 3),
            yaw_error=round(float(yaw_error), 3),
            lane_score=round(float(lane_score), 3),
            blocker=blocker,
        )
        return _fire_now()

    plan_action, plan_x, plan_y, plan_speed_x, plan_speed_y = choose_attack_plan(
        pose,
        enemies,
        ATTACK_DIR,
        lane_score,
    )
    if plan_action == "wall_shot":
        target = [plan_x, plan_y]
        plan_angle = UnitedPosition.Local2Angle(target) % 360
        yaw_error = angle_error_deg(chassis.GetYaw(), plan_angle)
        if abs(float(yaw_error)) <= 72:
            _set_decision(
                action="wall_shot",
                force_gate=force_reason,
                progress=round(float(progress), 3),
                y=round(float(y), 3),
                yaw_error=round(float(yaw_error), 3),
                lane_score=round(float(lane_score), 3),
                blocker=blocker,
                target=[round(float(plan_x), 3), round(float(plan_y), 3)],
            )
            return _fire_now()

    if plan_action == "wall_setup":
        plan_angle = UnitedPosition.Local2Angle([plan_x, plan_y]) % 360
        peripheral.Dribble(0.85)
        chassis.FieldMoveVector(plan_speed_x, plan_speed_y, plan_angle, KpW=0.9)
        _set_decision(
            action="wall_setup",
            force_gate=force_reason,
            progress=round(float(progress), 3),
            y=round(float(y), 3),
            lane_score=round(float(lane_score), 3),
            target=[round(float(plan_x), 3), round(float(plan_y), 3)],
            speed=[round(float(plan_speed_x), 3), round(float(plan_speed_y), 3), round(float(plan_angle), 3)],
        )
        return "wall_setup"

    peripheral.Dribble(0.85)
    if blocker or locked_dodge:
        if blocker:
            lateral_gap = float(blocker["lateral_gap"])
            if abs(lateral_gap) < 6:
                side = -1.0 if float(y) > 0 else 1.0
            else:
                side = -1.0 if lateral_gap > 0 else 1.0
            forward_gap = float(blocker["forward_gap"])
            _remember_drive_intent("attack_dodge", 0.42, side=side, start_progress=progress)
        else:
            side = float(locked_dodge.get("side", -1.0 if float(y) > 0 else 1.0))
            forward_gap = 34.0
            lateral_gap = None

        dodge_limit = max(8.0, 45 - 6.0)
        target_y = clamp(float(y) + side * 34, -dodge_limit, dodge_limit)
        speed_y = clamp((target_y - float(y)) * 1.1, -34.0, 34.0)
        speed_x = ATTACK_DIR * (34 if forward_gap < 35 else 44)
        chassis.FieldMoveVector(speed_x, speed_y, goal_angle, KpW=0.95)
        _set_decision(
            action="attack_dodge",
            mode="blocker" if blocker else "locked",
            force_gate=force_reason,
            progress=round(float(progress), 3),
            y=round(float(y), 3),
            yaw_error=round(float(yaw_error), 3),
            lane_score=round(float(lane_score), 3),
            blocker=blocker,
            target_y=round(float(target_y), 3),
            speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(goal_angle), 3)],
        )
        return "attack_dodge"

    speed_x = 65 if progress >= 35 else 52
    chassis.FieldMoveVector(speed_x * ATTACK_DIR, -float(y) * 1.0, goal_angle, KpW=0.9)
    action = "advance_lane"
    _set_decision(
        action=action,
        mode="fast" if progress >= 35 else "setup",
        force_gate=force_reason,
        progress=round(float(progress), 3),
        y=round(float(y), 3),
        yaw_error=round(float(yaw_error), 3),
        lane_score=round(float(lane_score), 3),
        speed=[round(float(speed_x * ATTACK_DIR), 3), round(float(-float(y)), 3), round(float(goal_angle), 3)],
    )
    return action


def _controlled_ball_step(pose, search_home) -> str:
    x, y, _ = pose
    progress = ATTACK_DIR * float(x)
    abs_y = abs(float(y))
    if progress < -45 and abs_y >= 58:
        recover_angle = UnitedPosition.Local2Angle([ATTACK_DIR * 85, 0]) % 360
        speed_x = ATTACK_DIR * 46
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

    if progress >= 72 and 24 <= abs_y <= 38:
        safe_angle = UnitedPosition.Local2Angle([ATTACK_DIR * 88, 48 if float(y) >= 0 else -48]) % 360
        speed_x = -ATTACK_DIR * 16
        speed_y = 34 if float(y) >= 0 else -34
        peripheral.Dribble(0.85)
        chassis.FieldMoveVector(speed_x, speed_y, safe_angle, KpW=0.85)
        _set_decision(
            action="penalty_box_escape",
            progress=round(float(progress), 3),
            y=round(float(y), 3),
            speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(safe_angle), 3)],
        )
        return "penalty_box_escape"

    enemies = _relevant_enemies(pose, UnitedPosition.FieldChassisPos())
    goal_target, shot_lane_score = _best_goal_target(pose, enemies)
    goal_angle = UnitedPosition.Local2Angle(goal_target) % 360
    yaw_error = angle_error_deg(chassis.GetYaw(), goal_angle)
    abs_yaw_error = abs(float(yaw_error))

    if progress < 0:
        return _carry_own_half(goal_angle, pose, shot_lane_score)

    sideline_state = _sideline_lane_step(pose, shot_lane_score)
    if sideline_state is not None:
        return sideline_state

    back_angle = (goal_angle + 180.0) % 360.0
    back_error = angle_error_deg(chassis.GetYaw(), back_angle)
    if progress >= 72 and abs_y <= 20 and shot_lane_score >= 0.72 and abs(float(back_error)) <= 28:
        _set_decision(
            action="back_shot",
            shot_family="back_shot",
            progress=round(float(progress), 3),
            y=round(float(y), 3),
            yaw_error=round(float(yaw_error), 3),
            back_error=round(float(back_error), 3),
            lane_score=round(float(shot_lane_score), 3),
            goal_angle=round(float(goal_angle), 3),
        )
        return _fire_now()

    shot_allowed = False
    if abs_y > 45 * 1.55:
        shot_reason = "too_wide"
    elif abs_y > 45 and progress < 50:
        shot_reason = "wide_before_pressure_zone"
    elif progress >= 38 and abs_y <= 12 and shot_lane_score >= 0.88:
        if abs_yaw_error <= 46:
            shot_allowed = True
            shot_reason = "open_goal"
        else:
            shot_reason = "open_goal_yaw_not_ready"
    elif progress >= 65:
        if abs_yaw_error <= 70 and shot_lane_score >= -0.05:
            shot_allowed = True
            shot_reason = "quick_zone"
        else:
            shot_reason = "quick_zone_not_ready"
    elif progress >= 50:
        if abs_yaw_error <= 70 and shot_lane_score >= -0.05:
            shot_allowed = True
            shot_reason = "pressure_zone"
        else:
            shot_reason = "pressure_zone_not_ready"
    elif progress >= 38:
        if abs_yaw_error <= 36 and shot_lane_score >= 0.15:
            shot_allowed = True
            shot_reason = "opportunity"
        else:
            shot_reason = "opportunity_not_ready"
    elif progress >= 35:
        if abs_yaw_error <= 25 and shot_lane_score >= 0.65:
            shot_allowed = True
            shot_reason = "fast_attack_window"
        else:
            shot_reason = "fast_attack_not_ready"
    else:
        shot_reason = "advance_more"

    _set_decision(
        action="hold_ball",
        shot_gate=shot_reason,
        progress=round(float(progress), 3),
        y=round(float(y), 3),
        yaw_error=round(float(yaw_error), 3),
        lane_score=round(float(shot_lane_score), 3),
        goal_angle=round(float(goal_angle), 3),
    )
    if shot_allowed:
        shot_family = "direct_shot" if shot_reason == "open_goal" else "quick_shot"
        _set_decision(
            action=shot_family,
            shot_family=shot_family,
            shot_gate=shot_reason,
            progress=round(float(progress), 3),
            y=round(float(y), 3),
            yaw_error=round(float(yaw_error), 3),
            lane_score=round(float(shot_lane_score), 3),
            goal_angle=round(float(goal_angle), 3),
        )
        return _fire_now()

    if abs(float(y)) >= 45 and progress < 50:
        if OhMyBackStep(HomePos=search_home, attack_dir=ATTACK_DIR):
            _set_decision(action="oh_my_back_shot", progress=round(float(progress), 3), y=round(float(y), 3))
            return "oh_my_back_shot"
        _set_decision(action="oh_my_back", progress=round(float(progress), 3), y=round(float(y), 3))
        return "oh_my_back"

    return _attack_with_ball(goal_angle, pose, enemies, shot_lane_score)


def _mark_sideline_component_decision(**fields) -> None:
    last_decision["component_mode"] = "sideline_shot"
    last_decision.update(fields)


def _sideline_component_controlled_step(pose, *, control_source: str | None = None, contact_ball=None) -> str:
    x, y, _ = pose
    progress = ATTACK_DIR * float(x)
    abs_y = abs(float(y))
    side = 1.0 if float(y) >= 0.0 else -1.0

    if progress >= 0.0:
        enemies = _relevant_enemies(pose, UnitedPosition.FieldChassisPos())
        _, shot_lane_score = _best_goal_target(pose, enemies)
        sideline_state = _sideline_lane_step(pose, max(0.0, float(shot_lane_score)), component_mode=True)
        if sideline_state is not None:
            fields = {}
            if control_source:
                fields["control_source"] = control_source
            if contact_ball is not None:
                fields["contact_ball"] = [round(float(contact_ball[0]), 3), round(float(contact_ball[1]), 3)]
            _mark_sideline_component_decision(**fields)
            return sideline_state

    outward_yaw = _sideline_outside_yaw(float(y))
    target_y = _sideline_lane_y(side)
    speed_x = ATTACK_DIR * (42.0 if progress < 54.0 else 30.0)
    speed_y = clamp((target_y - float(y)) * 1.05, -34.0, 34.0)
    peripheral.Dribble(0.92)
    chassis.FieldMoveVector(speed_x, speed_y, outward_yaw, KpW=0.85)
    _remember_drive_intent("sideline_hide", 0.55, target_y=target_y)
    _set_decision(
        action="sideline_hide",
        mode="component_to_sideline",
        shot_gate="component_corner_entry",
        progress=round(float(progress), 3),
        y=round(float(y), 3),
        target=[round(float(ATTACK_DIR * 68.0), 3), round(float(target_y), 3), round(float(outward_yaw), 3)],
        speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(outward_yaw), 3)],
    )
    fields = {}
    if control_source:
        fields["control_source"] = control_source
    if contact_ball is not None:
        fields["contact_ball"] = [round(float(contact_ball[0]), 3), round(float(contact_ball[1]), 3)]
    _mark_sideline_component_decision(**fields)
    return "sideline_hide"


def _sideline_component_free_ball_step(ball) -> str:
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
        field_ball_matches_robot = field_pose_valid and (
            math.hypot(expected_ball_x - float(field_ball[0]), expected_ball_y - float(field_ball[1])) <= 24.0
        )

        if field_ball_matches_robot:
            component_recovery_contact = (
                -1.5 <= bx <= 12.5
                and abs(by) <= SIDELINE_COMPONENT_CENTERED_LATERAL_CM
            )
            centered_contact = (
                _is_dribbler_contact_ball(bx, by)
                and abs(by) <= SIDELINE_COMPONENT_CENTERED_LATERAL_CM
            )
            if centered_contact or component_recovery_contact:
                return _sideline_component_controlled_step(
                    pose,
                    control_source="visible_dribbler_contact",
                    contact_ball=[bx, by],
                )

            side = 1.0 if float(field_ball[1]) >= 0.0 else -1.0
            lane_abs_y = _sideline_lane_abs_y()
            lane_y = _sideline_lane_y(side)
            if -2.0 <= bx <= SIDELINE_COMPONENT_CENTER_FORWARD_MAX_CM and abs(by) <= 12.0:
                if abs(by) > SIDELINE_COMPONENT_CENTERED_LATERAL_CM:
                    speed_x = clamp((bx - SIDELINE_COMPONENT_RECENTER_BACKOFF_CM) * 1.8, -18.0, -6.0)
                    speed_y = clamp(by * 1.8, -24.0, 24.0)
                    speed_w = 0.0
                    peripheral.Dribble(0.98)
                    chassis.RobotMoveVector(speed_x, speed_y, speed_w)
                    _remember_drive_intent("sideline_hide", 0.35, target_y=lane_y, side=side)
                    _set_decision(
                        action="sideline_hide",
                        mode="component_local_recenter",
                        shot_gate="recover_ball_center",
                        ball=[round(bx, 3), round(by, 3)],
                        field_ball=[round(float(field_ball[0]), 3), round(float(field_ball[1]), 3)],
                        pose=[round(float(pose[0]), 3), round(float(pose[1]), 3), round(float(pose[2]), 3)],
                        speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(speed_w), 3)],
                    )
                    _mark_sideline_component_decision()
                    return "sideline_hide"

                speed_x = clamp(5.0 + max(bx, 0.0) * 0.25, 4.0, 8.5)
                speed_y = clamp(by * 0.45, -6.0, 6.0)
                speed_w = 0.0
                peripheral.Dribble(0.98)
                chassis.RobotMoveVector(speed_x, speed_y, speed_w)
                _remember_drive_intent("sideline_hide", 0.3, target_y=lane_y, side=side)
                _set_decision(
                    action="sideline_hide",
                    mode="component_local_center",
                    shot_gate="center_ball_in_dribbler",
                    ball=[round(bx, 3), round(by, 3)],
                    field_ball=[round(float(field_ball[0]), 3), round(float(field_ball[1]), 3)],
                    pose=[round(float(pose[0]), 3), round(float(pose[1]), 3), round(float(pose[2]), 3)],
                    speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(speed_w), 3)],
                )
                _mark_sideline_component_decision()
                return "sideline_hide"

            target_x = clamp(float(field_ball[0]), -86.0, 86.0)
            target_y = clamp(float(field_ball[1]) - side * SIDELINE_COMPONENT_ACQUIRE_OFFSET_CM, -lane_abs_y, lane_abs_y)
            target_yaw = _sideline_outside_yaw(float(field_ball[1]))
            speed_x = clamp((target_x - float(pose[0])) * 1.55, -46.0, 46.0)
            speed_y = clamp((target_y - float(pose[1])) * 1.55, -42.0, 42.0)
            peripheral.Dribble(0.95)
            chassis.FieldMoveVector(speed_x, speed_y, target_yaw, KpW=0.95)
            _remember_drive_intent("sideline_hide", 0.5, target_y=lane_y, side=side)
            _set_decision(
                action="sideline_hide",
                mode="component_acquire",
                shot_gate="approach_ball_from_infield",
                ball=[round(bx, 3), round(by, 3)],
                field_ball=[round(float(field_ball[0]), 3), round(float(field_ball[1]), 3)],
                pose=[round(float(pose[0]), 3), round(float(pose[1]), 3), round(float(pose[2]), 3)],
                target=[round(float(target_x), 3), round(float(target_y), 3), round(float(target_yaw), 3)],
                speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(target_yaw), 3)],
            )
            _mark_sideline_component_decision()
            return "sideline_hide"
    except Exception:
        pass

    angle = math.degrees(math.atan2(by, bx)) if abs(bx) + abs(by) > 1e-6 else 0.0
    peripheral.Dribble(0.92)
    speed_x = clamp(30.0 + max(bx, 0.0) * 1.1, 24.0, 52.0)
    speed_y = clamp(by * 1.45, -34.0, 34.0)
    speed_w = clamp(angle * 0.55, -42.0, 42.0)
    chassis.RobotMoveVector(speed_x, speed_y, speed_w)
    _set_decision(
        action="sideline_hide",
        mode="component_local_acquire",
        shot_gate="local_ball_approach",
        ball=[round(bx, 3), round(by, 3)],
        distance=round(float(math.hypot(bx, by)), 3),
        angle=round(float(angle), 3),
        speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(speed_w), 3)],
    )
    _mark_sideline_component_decision()
    return "sideline_hide"


def _sideline_component_step(ball, search_home) -> str:
    if ball == BALL_NOT_FOUND or (ball != BALL_HELD and not _valid_free_ball(ball)):
        recent_ball = _recent_free_ball()
        if recent_ball is not None:
            return _sideline_component_free_ball_step(recent_ball)
        peripheral.Dribble(0.7)
        UnitedPosition.Pos2Pos(search_home, 4.5)
        _set_decision(action="search_ball", mode="component_sideline", target=search_home)
        return "search_ball"

    if ball != BALL_HELD:
        return _sideline_component_free_ball_step(ball)

    try:
        pose = UnitedPosition.FieldRobotPos()
    except Exception:
        pose = [0, 0, 0]
    return _sideline_component_controlled_step(pose, control_source="strict_possession")


def OffenceStep():
    """执行进攻策略的一个非阻塞 tick，并返回当前状态名。"""
    global state

    ball = UnitedPosition.RobotBallPos()
    search_home = _search_home()

    if _active_drive_intent("shot_release"):
        state = _shot_release_step()
        return state

    if _sideline_component_enabled():
        state = _sideline_component_step(ball, search_home)
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
    state = _controlled_ball_step(pose, search_home)
    return state


def OffenceStop():
    """停止进攻策略相关运动和外设，并标记状态为 stopped。"""
    global state
    safe_stop(chassis, peripheral, UnitedPosition)
    state = "stopped"


def main():
    """直接运行本文件时持续执行进攻策略循环。"""
    try:
        while True:
            OffenceStep()
            time.sleep(OFFENCE_DT)
    finally:
        OffenceStop()


if __name__ == "__main__":
    main()
