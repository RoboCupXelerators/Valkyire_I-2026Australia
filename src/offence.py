from BasicFuc import *
import math
import os
import time

from LunaPre.strategy_tools import BALL_HELD, BALL_NOT_FOUND, angle_error_deg, clamp, safe_stop

OFFENCE_DT = float(os.environ.get("OFFENCE_DT", "0.03"))
ATTACK_DIR = 1 if float(os.environ.get("OFFENCE_ATTACK_SIGN", "1")) >= 0 else -1
OHMYBACK_Y = float(os.environ.get("OFFENCE_OHMYBACK_Y", "45"))
QUICK_SHOT_X = float(os.environ.get("OFFENCE_QUICK_SHOT_X", "65"))
FAST_ATTACK_X = float(os.environ.get("OFFENCE_FAST_ATTACK_X", "35"))
LANE_BLOCK_WIDTH = float(os.environ.get("OFFENCE_LANE_BLOCK_WIDTH", "28"))
SHOT_BLOCK_WIDTH = float(os.environ.get("OFFENCE_SHOT_BLOCK_WIDTH", "12"))
FRONT_CLEAR_X = float(os.environ.get("OFFENCE_FRONT_CLEAR_X", "18"))
GOAL_X = float(os.environ.get("OFFENCE_GOAL_X", "110"))
GOAL_HALF_WIDTH = float(os.environ.get("OFFENCE_GOAL_HALF_WIDTH", "42"))
OPEN_GOAL_SHOT_X = float(os.environ.get("OFFENCE_OPEN_GOAL_SHOT_X", "18"))
OPEN_GOAL_Y_TOL = float(os.environ.get("OFFENCE_OPEN_GOAL_Y_TOL", "28"))
OPEN_GOAL_LANE_SCORE = float(os.environ.get("OFFENCE_OPEN_GOAL_LANE_SCORE", "0.88"))
OPEN_GOAL_YAW_TOL = float(os.environ.get("OFFENCE_OPEN_GOAL_YAW_TOL", "46"))
OPPORTUNITY_SHOT_X = float(os.environ.get("OFFENCE_OPPORTUNITY_SHOT_X", "38"))
PRESSURE_SHOT_X = float(os.environ.get("OFFENCE_PRESSURE_SHOT_X", "50"))
CONTESTED_SHOT_X = float(os.environ.get("OFFENCE_CONTESTED_SHOT_X", "30"))
CONTESTED_BLOCKER_X = float(os.environ.get("OFFENCE_CONTESTED_BLOCKER_X", "28"))
SHOOT_YAW_TOL = float(os.environ.get("OFFENCE_SHOOT_YAW_TOL", "36"))
FORCED_SHOT_YAW_TOL = float(os.environ.get("OFFENCE_FORCED_SHOT_YAW_TOL", "70"))
MIN_SHOT_LANE_SCORE = float(os.environ.get("OFFENCE_MIN_SHOT_LANE_SCORE", "0.15"))
FORCED_SHOT_LANE_SCORE = float(os.environ.get("OFFENCE_FORCED_SHOT_LANE_SCORE", "-0.05"))
CONTESTED_SHOT_YAW_TOL = float(os.environ.get("OFFENCE_CONTESTED_SHOT_YAW_TOL", "60"))
CONTESTED_SHOT_LANE_SCORE = float(os.environ.get("OFFENCE_CONTESTED_SHOT_LANE_SCORE", "0.0"))
BLOCKING_ENEMY_X = float(os.environ.get("OFFENCE_BLOCKING_ENEMY_X", "70"))
DODGE_Y = float(os.environ.get("OFFENCE_DODGE_Y", "34"))
DODGE_SPEED_Y = float(os.environ.get("OFFENCE_DODGE_SPEED_Y", "42"))
RELEVANT_ENEMY_BEHIND_X = float(os.environ.get("OFFENCE_RELEVANT_ENEMY_BEHIND_X", "10"))
RELEVANT_ENEMY_AHEAD_X = float(os.environ.get("OFFENCE_RELEVANT_ENEMY_AHEAD_X", "130"))
RELEVANT_ENEMY_Y = float(os.environ.get("OFFENCE_RELEVANT_ENEMY_Y", "70"))
CHASE_DIRECT_DISTANCE = float(os.environ.get("OFFENCE_CHASE_DIRECT_DISTANCE", "30"))
CHASE_KEEP_SECONDS = float(os.environ.get("OFFENCE_CHASE_KEEP_SECONDS", "0.35"))
CHASE_BALL_CATCH_X = float(os.environ.get("OFFENCE_CHASE_BALL_CATCH_X", "8.5"))
CHASE_MAX_X = float(os.environ.get("OFFENCE_CHASE_MAX_X", "95"))
CHASE_MAX_Y = float(os.environ.get("OFFENCE_CHASE_MAX_Y", "80"))
CHASE_MAX_W = float(os.environ.get("OFFENCE_CHASE_MAX_W", "140"))
CHASE_KP_X = float(os.environ.get("OFFENCE_CHASE_KP_X", "2.2"))
CHASE_KP_Y = float(os.environ.get("OFFENCE_CHASE_KP_Y", "2.0"))
CHASE_KP_W = float(os.environ.get("OFFENCE_CHASE_KP_W", "1.25"))
LOCK_BALL_MAX_SECONDS = float(os.environ.get("OFFENCE_LOCK_BALL_MAX_SECONDS", "0.55"))
LOCK_BALL_RESET_DISTANCE = float(os.environ.get("OFFENCE_LOCK_BALL_RESET_DISTANCE", "8"))
CLOSE_PRESS_MIN_X = float(os.environ.get("OFFENCE_CLOSE_PRESS_MIN_X", "38"))
CLOSE_PRESS_MAX_X = float(os.environ.get("OFFENCE_CLOSE_PRESS_MAX_X", "72"))

state = "boot"
last_decision = {}
_last_free_ball = None
_last_free_ball_ts = 0.0
_close_lock_since = None
_close_lock_anchor = None


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
        if forward_gap < -RELEVANT_ENEMY_BEHIND_X:
            continue
        if forward_gap > RELEVANT_ENEMY_AHEAD_X:
            continue
        if abs(lateral_gap) > RELEVANT_ENEMY_Y:
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
        if not FRONT_CLEAR_X < forward_gap <= BLOCKING_ENEMY_X:
            continue
        if abs(lateral_gap) > LANE_BLOCK_WIDTH:
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


def _lane_score(pose, target, enemies, block_width=SHOT_BLOCK_WIDTH) -> float:
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
    goal_x = ATTACK_DIR * GOAL_X
    aim_ys = [-GOAL_HALF_WIDTH * 0.9, 0.0, GOAL_HALF_WIDTH * 0.9]
    scored = []
    for aim_y in aim_ys:
        target = [goal_x, aim_y]
        score = _lane_score(pose, target, enemies)
        scored.append((score - abs(aim_y) * 0.003, score, target))
    _, raw_score, target = max(scored, key=lambda item: item[0])
    return target, raw_score


def _shot_decision(progress, y, yaw_error, lane_score):
    """判断是否应该主动射门，并给出可复盘的原因。"""
    abs_yaw_error = abs(float(yaw_error))
    abs_y = abs(float(y))
    if abs_y > OHMYBACK_Y * 1.55:
        return False, "too_wide"
    if abs_y > OHMYBACK_Y and progress < PRESSURE_SHOT_X:
        return False, "wide_before_pressure_zone"

    # 空门/开阔线路可以提前出脚，但必须同时满足位置、横向偏差、朝向和线路质量。
    if progress >= OPEN_GOAL_SHOT_X and abs_y <= OPEN_GOAL_Y_TOL and lane_score >= OPEN_GOAL_LANE_SCORE:
        if abs_yaw_error <= OPEN_GOAL_YAW_TOL:
            return True, "open_goal"
        return False, "open_goal_yaw_not_ready"
    if progress >= QUICK_SHOT_X:
        if abs_yaw_error <= FORCED_SHOT_YAW_TOL and lane_score >= FORCED_SHOT_LANE_SCORE:
            return True, "quick_zone"
        return False, "quick_zone_not_ready"
    if progress >= PRESSURE_SHOT_X:
        if abs_yaw_error <= FORCED_SHOT_YAW_TOL and lane_score >= FORCED_SHOT_LANE_SCORE:
            return True, "pressure_zone"
        return False, "pressure_zone_not_ready"
    if progress >= OPPORTUNITY_SHOT_X:
        if abs_yaw_error <= SHOOT_YAW_TOL and lane_score >= MIN_SHOT_LANE_SCORE:
            return True, "opportunity"
        return False, "opportunity_not_ready"
    if progress >= FAST_ATTACK_X:
        if abs_yaw_error <= 25 and lane_score >= 0.65:
            return True, "fast_attack_window"
        return False, "fast_attack_not_ready"
    return False, "advance_more"


def _should_shoot(progress, y, yaw_error, lane_score) -> bool:
    """兼容旧测试的布尔射门判断。"""
    allowed, _reason = _shot_decision(progress, y, yaw_error, lane_score)
    return allowed


def _force_shot_decision(progress, y, blocker, yaw_error, lane_score):
    """判断受压时是否应该强行射门，并给出原因。"""
    abs_y = abs(float(y))
    abs_yaw_error = abs(float(yaw_error))
    if abs_y > OHMYBACK_Y * 1.35:
        return False, "too_wide_for_forced_shot"
    if blocker is None:
        return False, "no_close_blocker"

    if (
        blocker is not None
        and progress >= CONTESTED_SHOT_X
        and float(blocker["forward_gap"]) <= CONTESTED_BLOCKER_X
        and abs_y <= OHMYBACK_Y * 1.1
    ):
        if abs_yaw_error <= CONTESTED_SHOT_YAW_TOL and lane_score >= CONTESTED_SHOT_LANE_SCORE:
            return True, "contested_window"
        return False, "contested_window_not_ready"
    if progress < PRESSURE_SHOT_X:
        return False, "forced_shot_before_threat_zone"
    if float(blocker["forward_gap"]) > max(24.0, FRONT_CLEAR_X + 10.0):
        return False, "blocker_not_urgent"
    if abs_yaw_error <= FORCED_SHOT_YAW_TOL and lane_score >= FORCED_SHOT_LANE_SCORE:
        return True, "pressure_blocker"
    return False, "pressure_blocker_not_ready"


def _should_force_shot(progress, y, blocker, yaw_error, lane_score) -> bool:
    """兼容旧测试的布尔受压射门判断。"""
    allowed, _reason = _force_shot_decision(progress, y, blocker, yaw_error, lane_score)
    return allowed


def _fire_now() -> str:
    """执行一次果断射门，并关闭吸球避免射门后继续拖住球。"""
    peripheral.Dribble(0.85)
    peripheral.ShootBall()
    peripheral.Dribble(False)
    chassis.stop()
    return "quick_shot"


def _try_shoot(goal_angle, pose, lane_score):
    """机会足够时直接射门，避免持球过久错过窗口。"""
    x, y, _ = pose
    progress = ATTACK_DIR * float(x)
    yaw_error = angle_error_deg(chassis.GetYaw(), goal_angle)
    allowed, reason = _shot_decision(progress, y, yaw_error, lane_score)
    _set_decision(
        action="hold_ball",
        shot_gate=reason,
        progress=round(float(progress), 3),
        y=round(float(y), 3),
        yaw_error=round(float(yaw_error), 3),
        lane_score=round(float(lane_score), 3),
        goal_angle=round(float(goal_angle), 3),
    )
    if not allowed:
        return None

    _set_decision(
        action="quick_shot",
        shot_gate=reason,
        progress=round(float(progress), 3),
        y=round(float(y), 3),
        yaw_error=round(float(yaw_error), 3),
        lane_score=round(float(lane_score), 3),
        goal_angle=round(float(goal_angle), 3),
    )
    return _fire_now()


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
    if time.monotonic() - _last_free_ball_ts > CHASE_KEEP_SECONDS:
        return None
    return _last_free_ball


def _reset_close_lock() -> None:
    """清空近球锁定计时，避免远球/持球状态继承旧超时。"""
    global _close_lock_since, _close_lock_anchor
    _close_lock_since = None
    _close_lock_anchor = None


def _close_lock_elapsed(ball) -> float:
    """统计同一个近球观测连续锁定了多久。"""
    global _close_lock_since, _close_lock_anchor
    now = time.monotonic()
    bx = float(ball[0])
    by = float(ball[1])
    if (
        _close_lock_since is None
        or _close_lock_anchor is None
        or math.hypot(bx - _close_lock_anchor[0], by - _close_lock_anchor[1]) > LOCK_BALL_RESET_DISTANCE
    ):
        _close_lock_since = now
        _close_lock_anchor = [bx, by]
        return 0.0
    return now - _close_lock_since


def _direct_chase_ball(ball, *, stale=False) -> str:
    """用当前球坐标直接追球，避免可追球被搜索/锁球分支拖慢。"""
    bx = float(ball[0])
    by = float(ball[1])
    dist = math.hypot(bx, by)
    angle = math.degrees(math.atan2(by, bx)) if dist > 1e-6 else 0.0

    peripheral.Dribble(0.75)
    speed_x = clamp((bx - CHASE_BALL_CATCH_X) * CHASE_KP_X, -22.0, CHASE_MAX_X)
    if 0 <= bx <= CHASE_BALL_CATCH_X + 5 and abs(by) <= 10:
        speed_x = max(speed_x, 24.0)
    speed_y = clamp(by * CHASE_KP_Y, -CHASE_MAX_Y, CHASE_MAX_Y)
    speed_w = clamp(angle * CHASE_KP_W, -CHASE_MAX_W, CHASE_MAX_W)
    chassis.RobotMoveVector(speed_x, speed_y, speed_w)
    _set_decision(
        action="chase_last_ball" if stale else "chase_ball",
        ball=[round(bx, 3), round(by, 3)],
        distance=round(float(dist), 3),
        angle=round(float(angle), 3),
        speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(speed_w), 3)],
    )
    return "chase_last_ball" if stale else "chase_ball"


def _pressure_close_ball(ball, elapsed: float) -> str:
    """近球长时间未形成持球时主动压球，避免无限卡在锁球 PID。"""
    bx = float(ball[0])
    by = float(ball[1])
    dist = math.hypot(bx, by)
    angle = math.degrees(math.atan2(by, bx)) if dist > 1e-6 else 0.0

    peripheral.Dribble(0.85)
    if bx >= -5:
        speed_x = clamp((bx - CHASE_BALL_CATCH_X) * CHASE_KP_X + CLOSE_PRESS_MIN_X, CLOSE_PRESS_MIN_X, CLOSE_PRESS_MAX_X)
    else:
        speed_x = clamp((bx - CHASE_BALL_CATCH_X) * CHASE_KP_X, -22.0, CLOSE_PRESS_MAX_X)
    speed_y = clamp(by * CHASE_KP_Y, -CHASE_MAX_Y, CHASE_MAX_Y)
    speed_w = clamp(angle * CHASE_KP_W, -CHASE_MAX_W, CHASE_MAX_W)
    chassis.RobotMoveVector(speed_x, speed_y, speed_w)
    _set_decision(
        action="press_close_ball",
        ball=[round(bx, 3), round(by, 3)],
        lock_elapsed=round(float(elapsed), 3),
        distance=round(float(dist), 3),
        angle=round(float(angle), 3),
        speed=[round(float(speed_x), 3), round(float(speed_y), 3), round(float(speed_w), 3)],
    )
    return "press_close_ball"


def _lock_or_chase_free_ball(ball) -> str:
    """远球直接追，近球交给 LockBallSlip 做精细吸球。"""
    _remember_free_ball(ball)
    bx = float(ball[0])
    by = float(ball[1])
    if math.hypot(bx, by) >= CHASE_DIRECT_DISTANCE or bx < -5:
        _reset_close_lock()
        return _direct_chase_ball(ball)

    lock_elapsed = _close_lock_elapsed(ball)
    if lock_elapsed >= LOCK_BALL_MAX_SECONDS:
        return _pressure_close_ball(ball, lock_elapsed)

    logger.debug("看到近球但未持球，进入锁球流程。")
    peripheral.Dribble(0.7)
    if lockball.LockBallSlip():
        _reset_close_lock()
    _set_decision(action="lock_ball", ball=[round(bx, 3), round(by, 3)], lock_elapsed=round(float(lock_elapsed), 3))
    return "lock_ball"


def _attack_with_ball(goal_angle, pose, enemies, lane_score) -> str:
    """持球后持续推进；遇到单个拦截者时横向绕开，不停在原地等。"""
    x, y, _ = pose
    progress = ATTACK_DIR * float(x)
    blocker = _blocking_enemy(pose, enemies)
    yaw_error = angle_error_deg(chassis.GetYaw(), goal_angle)
    force_allowed, force_reason = _force_shot_decision(progress, y, blocker, yaw_error, lane_score)

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

    peripheral.Dribble(0.85)
    if blocker:
        lateral_gap = float(blocker["lateral_gap"])
        if abs(lateral_gap) < 6:
            side = -1.0 if float(y) > 0 else 1.0
        else:
            side = -1.0 if lateral_gap > 0 else 1.0

        dodge_limit = max(8.0, OHMYBACK_Y - 6.0)
        target_y = clamp(float(y) + side * DODGE_Y, -dodge_limit, dodge_limit)
        speed_y = clamp((target_y - float(y)) * 1.25, -DODGE_SPEED_Y, DODGE_SPEED_Y)
        speed_x = ATTACK_DIR * (42 if float(blocker["forward_gap"]) < 35 else 52)
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

    speed_x = 65 if progress >= FAST_ATTACK_X else 52
    chassis.FieldMoveVector(speed_x * ATTACK_DIR, -float(y) * 1.0, goal_angle, KpW=0.9)
    action = "fast_attack" if progress >= FAST_ATTACK_X else "feint_drive"
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
    enemies = _relevant_enemies(pose, UnitedPosition.FieldChassisPos())
    goal_target, shot_lane_score = _best_goal_target(pose, enemies)
    goal_angle = UnitedPosition.Local2Angle(goal_target) % 360

    shot_state = _try_shoot(goal_angle, pose, shot_lane_score)
    if shot_state is not None:
        state = shot_state
        return state

    if abs(float(y)) >= OHMYBACK_Y and progress < PRESSURE_SHOT_X:
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
