from BasicFuc import *
import math
import os
import time

from LunaPre.simple_strategy import choose_attack_plan
from LunaPre.strategy_tools import BALL_HELD, BALL_NOT_FOUND, angle_error_deg, clamp, safe_stop

# 只保留跨 tick/运行方式有关的参数；具体策略阈值直接写在对应判断旁边。
OFFENCE_DT = float(os.environ.get("OFFENCE_DT", "0.03"))
ATTACK_DIR = 1 if float(os.environ.get("OFFENCE_ATTACK_SIGN", "1")) >= 0 else -1

state = "boot"
last_decision = {}
_last_free_ball = None
_last_free_ball_ts = 0.0


def _set_decision(**fields):
    """记录本 tick 的关键判断，供真实场景 trace 复盘。"""
    global last_decision
    last_decision = fields
    return fields


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
        if not 18 < forward_gap <= 70:
            continue
        if abs(lateral_gap) > 28:
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
    peripheral.Dribble(0.85)
    peripheral.ShootBall()
    peripheral.Dribble(False)
    chassis.stop()
    return "quick_shot"


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


def _lock_or_chase_free_ball(ball) -> str:
    """Free balls stay active: close balls still get forward/lateral pressure."""
    _remember_free_ball(ball)
    bx = float(ball[0])
    by = float(ball[1])
    try:
        pose = UnitedPosition.FieldRobotPos()
        field_ball = UnitedPosition.FieldBallPos()
        robot_progress = ATTACK_DIR * float(pose[0])
        ball_progress = ATTACK_DIR * float(field_ball[0])
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
            robot_progress < -42.0
            and ball_progress < -35.0
            and abs(float(field_ball[1])) >= 42.0
            and bx <= 24.0
            and abs(by) >= 9.0
        ):
            angle = math.degrees(math.atan2(by, bx)) if abs(bx) + abs(by) > 1e-6 else 0.0
            speed_x = 22.0 if abs(by) <= 18.0 else 14.0
            speed_y = clamp(by * 2.3, -38.0, 38.0)
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
    except Exception:
        pass
    if math.hypot(bx, by) >= 30 or bx < -5:
        return _direct_chase_ball(ball)

    angle = math.degrees(math.atan2(by, bx)) if abs(bx) + abs(by) > 1e-6 else 0.0
    if -2.0 <= bx <= 12.0 and abs(by) <= 10.0:
        peripheral.Dribble(0.95)
        speed_x = 8.0 if bx <= 4.0 else 12.0
        speed_y = clamp(by * 0.75, -10.0, 10.0)
        speed_w = clamp(angle * 0.35, -24.0, 24.0)
        chassis.RobotMoveVector(speed_x, speed_y, speed_w)
        _set_decision(
            action="near_ball_capture",
            ball=[round(bx, 3), round(by, 3)],
            speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(speed_w), 3)],
        )
        return "near_ball_capture"

    if bx <= 22.0 and abs(by) > 9.0:
        peripheral.Dribble(0.95)
        speed_x = clamp((bx - 8.5) * 0.65, 0.0, 12.0)
        speed_y = clamp(by * 1.9, -30.0, 30.0)
        speed_w = clamp(angle * 0.25, -18.0, 18.0)
        chassis.RobotMoveVector(speed_x, speed_y, speed_w)
        _set_decision(
            action="near_ball_align",
            ball=[round(bx, 3), round(by, 3)],
            distance=round(float(math.hypot(bx, by)), 3),
            angle=round(float(angle), 3),
            speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(speed_w), 3)],
        )
        return "near_ball_align"

    peripheral.Dribble(0.9)
    speed_x = clamp((bx - 8.5) * 1.6, 10.0, 34.0)
    speed_y = clamp(by * 1.2, -24.0, 24.0)
    speed_w = clamp(angle * 0.65, -55.0, 55.0)
    chassis.RobotMoveVector(speed_x, speed_y, speed_w)
    _set_decision(
        action="near_ball_acquire",
        ball=[round(bx, 3), round(by, 3)],
        distance=round(float(math.hypot(bx, by)), 3),
        angle=round(float(angle), 3),
        speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(speed_w), 3)],
    )
    return "near_ball_acquire"


def _attack_with_ball(goal_angle, pose, enemies, lane_score) -> str:
    """持球后持续推进；遇到单个拦截者时横向绕开，不停在原地等。"""
    x, y, _ = pose
    progress = ATTACK_DIR * float(x)
    blocker = _blocking_enemy(pose, enemies)
    yaw_error = angle_error_deg(chassis.GetYaw(), goal_angle)
    abs_y = abs(float(y))
    abs_yaw_error = abs(float(yaw_error))
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
    if blocker:
        lateral_gap = float(blocker["lateral_gap"])
        if abs(lateral_gap) < 6:
            side = -1.0 if float(y) > 0 else 1.0
        else:
            side = -1.0 if lateral_gap > 0 else 1.0

        dodge_limit = max(8.0, 45 - 6.0)
        target_y = clamp(float(y) + side * 34, -dodge_limit, dodge_limit)
        speed_y = clamp((target_y - float(y)) * 1.1, -34.0, 34.0)
        speed_x = ATTACK_DIR * (34 if float(blocker["forward_gap"]) < 35 else 44)
        chassis.FieldMoveVector(speed_x, speed_y, goal_angle, KpW=0.95)
        _set_decision(
            action="attack_dodge",
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
    action = "fast_attack" if progress >= 35 else "feint_drive"
    _set_decision(
        action=action,
        force_gate=force_reason,
        progress=round(float(progress), 3),
        y=round(float(y), 3),
        yaw_error=round(float(yaw_error), 3),
        lane_score=round(float(lane_score), 3),
        speed=[round(float(speed_x * ATTACK_DIR), 3), round(float(-float(y)), 3), round(float(goal_angle), 3)],
    )
    return action


def OffenceStep():
    """执行进攻策略的一个非阻塞 tick，并返回当前状态名。"""
    global state

    ball = UnitedPosition.RobotBallPos()
    home = list(Settings.CheckPoints.Home)
    while len(home) < 3:
        home.append(0)
    search_home = [home[0] + 20, home[1], 0]

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
        x, y, _ = pose
    except Exception:
        pose = [0, 0, 0]
        x, y = 0, 0

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
        state = "corner_recover"
        return state

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
        state = "penalty_box_escape"
        return state

    enemies = _relevant_enemies(pose, UnitedPosition.FieldChassisPos())
    goal_target, shot_lane_score = _best_goal_target(pose, enemies)
    goal_angle = UnitedPosition.Local2Angle(goal_target) % 360
    yaw_error = angle_error_deg(chassis.GetYaw(), goal_angle)
    abs_yaw_error = abs(float(yaw_error))

    if progress < 0:
        state = _carry_own_half(goal_angle, pose, shot_lane_score)
        return state

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
        state = _fire_now()
        return state

    shot_allowed = False
    if abs_y > 45 * 1.55:
        shot_reason = "too_wide"
    elif abs_y > 45 and progress < 50:
        shot_reason = "wide_before_pressure_zone"
    elif progress >= 18 and abs_y <= 28 and shot_lane_score >= 0.88:
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
        state = _fire_now()
        return state

    plan_action, plan_x, plan_y, plan_speed_x, plan_speed_y = choose_attack_plan(
        pose,
        enemies,
        ATTACK_DIR,
        shot_lane_score,
    )
    if plan_action == "sideline_shot":
        sideline_angle = UnitedPosition.Local2Angle([plan_x, plan_y]) % 360
        sideline_error = angle_error_deg(chassis.GetYaw(), sideline_angle)
        if abs(float(sideline_error)) <= 72:
            _set_decision(
                action="sideline_shot",
                shot_gate=shot_reason,
                progress=round(float(progress), 3),
                y=round(float(y), 3),
                yaw_error=round(float(sideline_error), 3),
                lane_score=round(float(shot_lane_score), 3),
                target=[round(float(plan_x), 3), round(float(plan_y), 3)],
            )
            state = _fire_now()
            return state

    if plan_action == "sideline_drive":
        sideline_angle = UnitedPosition.Local2Angle([plan_x, plan_y]) % 360
        peripheral.Dribble(0.85)
        chassis.FieldMoveVector(plan_speed_x, plan_speed_y, sideline_angle, KpW=0.9)
        _set_decision(
            action="sideline_drive",
            shot_gate=shot_reason,
            progress=round(float(progress), 3),
            y=round(float(y), 3),
            lane_score=round(float(shot_lane_score), 3),
            target=[round(float(plan_x), 3), round(float(plan_y), 3)],
            speed=[round(float(plan_speed_x), 3), round(float(plan_speed_y), 3), round(float(sideline_angle), 3)],
        )
        state = "sideline_drive"
        return state

    if abs(float(y)) >= 45 and progress < 50:
        if OhMyBackStep(HomePos=search_home, attack_dir=ATTACK_DIR):
            _set_decision(action="oh_my_back_shot", progress=round(float(progress), 3), y=round(float(y), 3))
            state = "oh_my_back_shot"
        else:
            _set_decision(action="oh_my_back", progress=round(float(progress), 3), y=round(float(y), 3))
            state = "oh_my_back"
        return state

    state = _attack_with_ball(goal_angle, pose, enemies, shot_lane_score)
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
