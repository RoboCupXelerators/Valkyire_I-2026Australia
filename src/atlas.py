"""Atlas：Hyperion 的连续评分版本，每次 logic_step 只执行一帧。

沿用 +X 进攻方向及带球绕球转向技能。权重是待真机/模拟赛调优的初值，
不是进球概率。Bow/Jump 尚无完整动作，不参与选择。
"""

import math
import time

from BasicFuc import Circle, TurnTo_with_ball, lockball
from LunaPre import Settings, UnitedPosition, chassis, logger, peripheral
from LunaPre.strategy_tools import BALL_NOT_FOUND, angle_error_deg, clamp, safe_stop, valid_pose, timer


# 各项先归一化到 [0, 1]，每组权重之和为 1。越接近对方半场，推进项越高。
HUCK_ALIGNMENT_WEIGHT = 0.60
HUCK_PROGRESS_WEIGHT = 0.25
HUCK_CENTER_WEIGHT = 0.15
UPLANE_ALIGNMENT_WEIGHT = 0.45
UPLANE_LANE_WEIGHT = 0.35
UPLANE_PROGRESS_WEIGHT = 0.20
SWITCH_MARGIN = 0.08  # 新策略必须至少高出此分差才切换；可直接射门时不受此限制。
# 最终只会从这里列出的方案里选择；删掉名称即可禁用对应方案。
ENABLED_SHOTS = ("huck", "uplane", "u_shoot")
U_SHOOT_BACKWARD_MIN_DEG = 135.0

GOAL_POS = (100.0, 0.0)  # 沿用 Hyperion 的几何瞄准点，窗口不可用时仅用于转向。
UPLANE_REGION = ((75.0, 30.0), (100.0, 90.0))  # X 到 75cm 后进入瞄准；负半场逐帧镜像。
LANE_Y = 60.0
LANE_ENTRY_Y = 50.0
CARRY_SPEED_MPS = 0.60
CARRY_POSITION_KP = 0.02  # 距离单位 cm，输出 m/s；接近目标时减速。
LANE_ARRIVAL_CM = 5.0
UPLANE_LOST_GRACE_S = 0.4
UPLANE_TIMEOUT_S = 12.0
UPLANE_RETRY_S = 3.0
CONTROL_PERIOD_S = 0.03
SHOT_GOAL_MARGIN_CM = 5.0  # 直射轨迹必须穿过对方门口，且离门柱留出余量。
# field_fixed.fbx 原始球门网格：门口 X=103.7，内后壁 X=111.15，净宽 58.94cm。
# 视觉估宽与裁判历史默认门线不能替代实际碰撞网格；球须通过门内通道。
SHOT_GOAL_LINE_X_CM = 103.7
SHOT_GOAL_WIDTH_CM = 58.94
SHOT_GOAL_DEPTH_CM = 7.45
SHOT_CENTER_TOLERANCE_DEG = 3.0  # 瞄准可用通道中间，避免刚擦进窗口就射门。
SHOT_AIM_TOLERANCE_DEG = 1.0
SHOT_TURN_MIN_GAIN = 0.10  # 最后几度仍需克服阻力；沿用 Circle 的绕球轮速比例。
SHOT_CONFIRM_S = 0.09  # 连续控制周期都满足条件才射；任何一次失效都重计。
SHOT_COOLDOWN_S = 0.80  # 防止持球状态延迟更新时连续触发电磁铁。

_last_shot = None
_shoot_ready_since = None
_last_kick_at = -math.inf
_last_shot_log_at = -math.inf
_uplane_phase = None
_uplane_side = 1
_uplane_deadline = 0.0
_uplane_retry_after = -math.inf
_last_possession_at = -math.inf


def _alignment(yaw, target_yaw):
    """最短角差：同向 1，侧向 0.5，背向 0，跨 0/360 度连续。"""
    return 1.0 - abs(angle_error_deg(yaw, target_yaw)) / 180.0


def _progress(x):
    return clamp(0.5 + x / float(Settings.Bounds.Long), 0.0, 1.0)


def _goal_yaw(pose):
    return math.degrees(math.atan2(GOAL_POS[1] - pose[1], GOAL_POS[0] - pose[0])) % 360


def HuckScore(pose, aim_yaw=None):
    """直射：优先朝向，其次进攻纵深、靠近中路程度。"""
    x, y, yaw = pose
    target = _goal_yaw(pose) if aim_yaw is None else aim_yaw
    center = 1.0 - clamp(abs(y) / (float(Settings.Bounds.Short) / 2.0), 0.0, 1.0)
    return (
        HUCK_ALIGNMENT_WEIGHT * _alignment(yaw, target)
        + HUCK_PROGRESS_WEIGHT * _progress(x)
        + HUCK_CENTER_WEIGHT * center
    )


def UplaneScore(pose):
    """边路：朝向侧边、接近边路、进攻纵深；中线附近也能平滑进入。"""
    x, y, yaw = pose
    side_yaw = 90.0 if y >= 0 else -90.0
    lane = clamp(abs(y) / LANE_Y, 0.0, 1.0)
    return (
        UPLANE_ALIGNMENT_WEIGHT * _alignment(yaw, side_yaw)
        + UPLANE_LANE_WEIGHT * lane
        + UPLANE_PROGRESS_WEIGHT * _progress(x)
    )


def UshootScore(pose, yaw):
    """背对目标球门时才提供弹墙射门候选。"""
    return 1.01 if abs(angle_error_deg(yaw, _goal_yaw(pose))) >= U_SHOOT_BACKWARD_MIN_DEG else 0.0


def select_shot(scores, previous=None):
    """在启用的动作中选择；初次同分保持插入顺序，之后小分差保持。"""
    best = max(scores, key=scores.__getitem__)
    if previous in scores and scores[best] - scores[previous] <= SWITCH_MARGIN:
        return previous
    return best


def can_direct_shoot(pose, yaw, window):
    """直射许可不等于评分；接受视觉或位姿计算的窗口，拒绝假定正前方有门。

    沿用 Atlas 的 +X 进攻方向。将当前朝向射线与对方门线求交，只有交点
    落在门柱内侧才认可视觉窗口；此检查不代表弹墙路径验证。
    """
    if not (
        window.get("can_shoot_now")
        and window.get("window_open")
        and (
            (window.get("reason") == "ok" and window.get("goal_key") in ("blue_goal", "yellow_goal"))
            or (window.get("reason") == "geometry_ok" and window.get("goal_key") == "field_opponent_goal")
        )
    ):
        return False
    if yaw is None or not valid_pose(pose) or not all(math.isfinite(v) for v in (*pose, yaw)):
        return False
    target = window.get("target_yaw_deg")
    if target is None or not math.isfinite(target) or abs(angle_error_deg(yaw, target)) > SHOT_CENTER_TOLERANCE_DEG:
        return False
    x, y, _ = pose
    goal_x = SHOT_GOAL_LINE_X_CM
    half_field_length = float(Settings.Bounds.Long) / 2.0
    half_field_width = float(Settings.Bounds.Short) / 2.0
    half_goal_width = SHOT_GOAL_WIDTH_CM / 2.0 - SHOT_GOAL_MARGIN_CM
    if not (-half_field_length < x < goal_x and abs(y) < half_field_width):
        return False
    forward = math.cos(math.radians(yaw))
    if forward <= 1e-6:
        return False
    slope = math.sin(math.radians(yaw)) / forward
    # 只穿过门线仍可能斜撞远侧门框；整条直线需落在门内通道。
    return half_goal_width > 0 and all(
        abs(y + (plane_x - x) * slope) < half_goal_width
        for plane_x in (goal_x, goal_x + SHOT_GOAL_DEPTH_CM)
    )


def shooting_window(pose, yaw):
    """没有球门检测时，由场地位姿投影门柱；仍用现有算法扣除已知遮挡。

    XV 模拟器只有球/机器人数据，没有球门检测。这里的门位于场地 +X 门线，
    不使用 ShootWindow 原先的“机器人正前方假门”。视觉报告遮挡时不回退。
    """
    window = UnitedPosition.ShootWindow(current_yaw_deg=yaw, use_estimated_goal=False)
    if window.get("reason") != "no_goal_visible":
        # 视觉窗口也瞄准通道中间，而不是默认的“离当前朝向最近的边缘”。
        opened = window.get("open_windows_deg")
        if opened and window.get("window_open"):
            best = max(opened, key=lambda span: (span[1] - span[0], -abs((span[0] + span[1]) / 2.0)))
            window = {**window, "target_yaw_deg": (yaw + (best[0] + best[1]) / 2.0) % 360}
        return window
    if not valid_pose(pose) or not all(math.isfinite(v) for v in (*pose, yaw)):
        return window
    x, y, _ = pose
    dx = SHOT_GOAL_LINE_X_CM - x
    half_width = SHOT_GOAL_WIDTH_CM / 2.0 - SHOT_GOAL_MARGIN_CM
    if dx <= 0 or half_width <= 0:
        return window
    center_yaw = math.degrees(math.atan2(-y, dx))
    center = angle_error_deg(yaw, center_yaw)
    # 以球门中心展开角度，防止门口横跨 +/-180 时被错误扩成整个圆。
    planes = [
        [center + angle_error_deg(center_yaw, math.degrees(math.atan2(post_y - y, plane_dx)))
         for post_y in (-half_width, half_width)]
        for plane_dx in (dx, dx + SHOT_GOAL_DEPTH_CM)
    ]
    raw = [max(span[0] for span in planes), min(span[1] for span in planes)]
    if raw[0] >= raw[1]:
        return {"reason": "geometry_no_corridor", "window_open": False, "can_shoot_now": False}
    distance = math.hypot(dx, y)
    opened, blockers = UnitedPosition._shoot_blocker_windows(raw, center, distance, 3.0)
    best = max(opened, key=lambda span: (span[1] - span[0], -abs((span[0] + span[1]) / 2.0))) if opened else None
    can_shoot = any(lo < 0 < hi for lo, hi in opened)
    reason = "geometry_ok" if can_shoot else "geometry_need_turn" if opened else "geometry_blocked"
    target_yaw = (yaw + (best[0] + best[1]) / 2.0) % 360 if best is not None else None
    return {
        "goal_key": "field_opponent_goal", "reason": reason,
        "window_open": bool(opened), "can_shoot_now": can_shoot,
        "raw_window_deg": raw, "open_windows_deg": opened,
        "target_yaw_deg": target_yaw, "goal_distance_cm": distance, "blockers": blockers,
    }


def _turn_to_shot(pose, window):
    target = window.get("target_yaw_deg")
    if target is None or not math.isfinite(target):
        target = _goal_yaw(pose)
    # ShootWindow 已传入底盘 yaw，返回绝对航向；不再把相对转角取负后当绝对角。
    # 保留真机绕球转向技能的原始轮速语义。
    if TurnTo_with_ball(target % 360.0):
        yaw = chassis.GetYaw()
        if (window.get("window_open") or window.get("reason") in ("need_turn", "geometry_need_turn")) and abs(angle_error_deg(yaw, target)) > SHOT_AIM_TOLERANCE_DEG:
            # 老技能在 15 度内就返回成功，远处窄门口仍可能尚未对准。
            # 延续该技能同一套绕球轮速混合，只在此处补足最后的小角度。
            error = angle_error_deg(target, yaw)
            gain = math.copysign(max(abs(error) / 180.0, SHOT_TURN_MIN_GAIN), error)
            Circle(0.6, gain, -1)
            return
        # 对准但窗口仍关闭时，清掉上一帧沿边移动/绕球转向的速度。
        chassis.stop()


def _reset_uplane(reason):
    global _uplane_phase
    if _uplane_phase is not None:
        logger.info("Atlas uplane exit=%s phase=%s" % (reason, _uplane_phase))
    _uplane_phase = None


def _set_uplane_phase(phase):
    global _uplane_phase
    if _uplane_phase != phase:
        logger.info("Atlas uplane phase=%s -> %s side=%s" % (_uplane_phase, phase, _uplane_side))
        _uplane_phase = phase
        chassis.stop()  # 清掉上一阶段的速度，尤其是推进转瞄准的交接。


def UplaneStep(pose, window):
    """一次持球动作内阶段只向前走；绕球位移越过入区线不能撤销瞄准。"""
    global _uplane_side, _uplane_deadline, _uplane_retry_after, _last_shot
    x, y, _ = pose
    now = time.monotonic()
    if _uplane_phase is None:
        _uplane_side = -1 if y < 0 else 1
        _uplane_deadline = now + UPLANE_TIMEOUT_S
        region = [(px, _uplane_side * py) for px, py in UPLANE_REGION]
        if UnitedPosition.PointInShape(pose, region):
            _set_uplane_phase("aim")
        elif LANE_ENTRY_Y < abs(y) <= UPLANE_REGION[1][1]:
            _set_uplane_phase("advance")
        else:
            _set_uplane_phase("lane")

    if now >= _uplane_deadline:
        _reset_uplane("timeout")
        _uplane_retry_after = now + UPLANE_RETRY_S
        _last_shot = "huck"
        chassis.stop()
        return

    if _uplane_phase == "lane" and abs(y - _uplane_side * LANE_Y) <= LANE_ARRIVAL_CM:
        _set_uplane_phase("advance")
    if _uplane_phase == "advance":
        region = [(px, _uplane_side * py) for px, py in UPLANE_REGION]
        if UnitedPosition.PointInShape(pose, region):
            _set_uplane_phase("aim")

    if _uplane_phase == "aim":
        _turn_to_shot(pose, window)
        return

    facing = (90.0 * _uplane_side) % 360.0
    if not TurnTo_with_ball(facing):
        return

    speed_y = clamp((_uplane_side * LANE_Y - y) * CARRY_POSITION_KP, -CARRY_SPEED_MPS, CARRY_SPEED_MPS)/5
    if _uplane_phase == "lane":
        chassis.FieldMoveVector(0.0, speed_y, facing)
    else:
        target_x = (UPLANE_REGION[0][0] + UPLANE_REGION[1][0]) / 2.0
        speed_x = clamp((target_x - x) * CARRY_POSITION_KP, -CARRY_SPEED_MPS, CARRY_SPEED_MPS)/8
        # 推进时持续修正横向误差，而不是一越过 y=50 就重新转入边路阶段。
        chassis.FieldMoveVector(speed_x, speed_y, facing)


def U_Shoot(goal=(100, 0), hold_s=3.0, shake_speed=1.5, shake_s=0.20):
    """背对球门时保持带球，再横摆让球借边墙反弹。"""
    pose = UnitedPosition.FieldRobotPose()
    if not lockball.LockBallSlip():
        return False
    back_to_goal = (UnitedPosition.Pos2Angle(pose, goal) + 180) % 360
    if not TurnTo_with_ball(back_to_goal):
        return False
    else:
        if not lockball.LockBallSlip():
            return False
        else:
            dribble_timer = timer()
            if not dribble_timer.countdown(hold_s):
                peripheral.Dribble(0.99)
            else:
                dribble_timer.reset()
                dribble_timer.reset() if dribble_timer.countdown(shake_s*2) else chassis.RobotMoveVector(0, shake_speed, 0) if dribble_timer.countdown(shake_s) else chassis.RobotMoveVector(0, -shake_speed, 0)
                return True


def logic_step():
    """看球 → 持球 → 确认直射许可 → 连续评分及小分差保持 → 执行动作。"""
    global _last_shot, _shoot_ready_since, _last_kick_at, _last_shot_log_at
    global _last_possession_at
    now = time.monotonic()

    if UnitedPosition.RobotBallPos() == BALL_NOT_FOUND:
        _shoot_ready_since = None
        if _uplane_phase is not None and now - _last_possession_at <= UPLANE_LOST_GRACE_S:
            chassis.stop()
            return
        _reset_uplane("ball_lost")
        _last_shot = None
        peripheral.Dribble(False)
        UnitedPosition.Pos2Pos(Settings.CheckPoints.Home)
        return

    if not lockball.LockBallSlip():
        _shoot_ready_since = None
        if now - _last_possession_at > UPLANE_LOST_GRACE_S:
            _reset_uplane("possession_lost")
            _last_shot = None
        return
    _last_possession_at = now

    pose = UnitedPosition.FieldRobotPose()
    if not valid_pose(pose) or not all(math.isfinite(value) for value in pose):
        _last_shot = None
        _shoot_ready_since = None
        _reset_uplane("pose_lost")
        chassis.stop()
        return

    # 一帧只取一次窗口，射门许可和瞄准目标来自同一份结果。
    yaw = None
    try:
        yaw = chassis.GetYaw()
        window = shooting_window(pose, yaw)
    except Exception as exc:
        logger.warning("Atlas shooting window unavailable: %s" % exc)
        window = {}

    approved = can_direct_shoot(pose, yaw, window)
    now = time.monotonic()
    if now - _last_shot_log_at >= 0.5:
        logger.debug("Atlas shot_allowed=%s source=%s pose=%s yaw=%s target=%s" %
                     (approved, window.get("reason", "window_unavailable"), pose, yaw, window.get("target_yaw_deg")))
        _last_shot_log_at = now
    if approved and "huck" in ENABLED_SHOTS:
        if _shoot_ready_since is None:
            _shoot_ready_since = now
        chassis.stop()
        if now - _shoot_ready_since >= SHOT_CONFIRM_S and now - _last_kick_at >= SHOT_COOLDOWN_S:
            peripheral.ShootBall()
            logger.info("Atlas shot fired source=%s pose=%s yaw=%s" % (window.get("reason"), pose, yaw))
            _last_kick_at = time.monotonic()
            _shoot_ready_since = None
            _last_shot = None
            _reset_uplane("shot")
        return
    _shoot_ready_since = None

    aim_yaw = window.get("target_yaw_deg")
    if aim_yaw is not None and not math.isfinite(aim_yaw):
        aim_yaw = None
    scores = {}
    if "huck" in ENABLED_SHOTS:
        scores["huck"] = HuckScore(pose, aim_yaw)
    if "uplane" in ENABLED_SHOTS:
        scores["uplane"] = UplaneScore(pose)
    if "u_shoot" in ENABLED_SHOTS and UshootScore(pose, yaw):
        scores["u_shoot"] = UshootScore(pose, yaw)
    if not scores:
        chassis.stop()
        return
    if _uplane_phase is not None:
        _last_shot = "uplane"  # 当前动作完成前，朝向变化不触发重新竞选。
    elif now < _uplane_retry_after:
        _last_shot = "huck"
    else:
        _last_shot = select_shot(scores, _last_shot)
    logger.debug("Atlas scores=%s selected=%s" % (scores, _last_shot))
    if _last_shot == "u_shoot":
        U_Shoot(GOAL_POS)
    elif _last_shot == "huck":
        _turn_to_shot(pose, window)
    else:
        UplaneStep(pose, window)


def main():
    try:
        while True:
            logic_step()
            time.sleep(CONTROL_PERIOD_S)
    except KeyboardInterrupt:
        logger.info("Atlas interrupted by user.")
    finally:
        safe_stop(chassis, peripheral, UnitedPosition)


if __name__ == "__main__":
    main()
