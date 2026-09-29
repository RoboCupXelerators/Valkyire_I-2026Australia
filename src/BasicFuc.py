"""BasicFuc

历史策略常用函数集合。

注意：本文件存在大量历史代码。此次改动目标：
- 修复明显的语法/运行时错误（重复定义、错误调用）
- 避免把 `from LunaPre import *` 继续扩散到外部：本文件内部仍可使用，但对外建议显式导入
"""

from __future__ import annotations

from LunaPre import Settings, UnitedPosition, chassis, compass, logger, peripheral
from LunaPre import runtime
from LunaPre.strategy_tools import (
    BALL_HELD,
    BALL_NOT_FOUND,
    angle_error_deg,
    clamp,
)
import math
import simple_pid
import time

# Historical helpers refer to ``cfg``; keep it as an explicit alias rather
# than relying on a wildcard import to leak the name into this module.
cfg = Settings

# 持球检测：优先用归一化受载指数 index=(I-baseline)/(grip-baseline)（跨机器统一，
# 由 calibrate_dribble.py 标定）；没有 grip 标定时退回绝对 margin=current-baseline。
HOLD_INDEX_RATIO = 0.40
HOLD_MARGIN_A = 0.15


def LinearMap(value, input_range, output_range):
    """把 ``value`` 从一个数值区间线性映射到另一个区间。"""
    input_min, input_max = input_range
    output_min, output_max = output_range
    if input_min == input_max:
        raise ValueError("input_range 的两个端点不能相同")
    input_span = input_max - input_min
    output_span = output_max - output_min
    scaled_value = (value - input_min) / input_span
    return output_min + (scaled_value * output_span)


class LockSeries:
    """机器人坐标系下的抢球/锁球控制器（7 月最简版：抓到停、丢球停、纯比例追）。"""

    def __init__(self):
        catch_val = getattr(Settings.Motion, "CatchVal", [8.5, 0])
        self.catch_x = catch_val[0] if len(catch_val) > 0 else 8.5
        self.catch_y = catch_val[1] if len(catch_val) > 1 else 0
        self.lost_hold_seconds = 0.12
        self._lockball_last_seen = None
        self._lockball_last_seen_ts = 0.0
        self.pan_x = simple_pid.PID(0.057, 0, 0, setpoint=self.catch_x, output_limits=(-1.09, 0), auto_mode=True)
        self.pan_y = simple_pid.PID(0.022, 0, 0, setpoint=self.catch_y, output_limits=(-0.73, 0.73), auto_mode=True)
        self.pan_w = simple_pid.PID(2.3, 0, 0, setpoint=0, output_limits=(-155, 155), auto_mode=True)

    def _reset_lockball_pid(self):
        self.pan_x.reset()
        self.pan_y.reset()
        self.pan_w.reset()

    def _current_or_recent_ball(self):
        """返回当前球坐标；刚丢球很短时间就沿用上一帧坐标，避免追球抖动。"""
        ball = UnitedPosition.RobotBallPos()
        now = time.time()

        # 持球检测：先让 BallStatus 刷新受载电流/基线，再看受载程度。
        # 注意：只有“球在视野里”时才采信电流——球不在视野（哨兵值）一律不判
        # held，避免“没球也判定有”。视野内优先用归一化 index，退回绝对 margin。
        status = UnitedPosition.BallStatus(ball)
        gripped = False
        if ball != BALL_NOT_FOUND:
            try:
                info = UnitedPosition.BallLoadInfo()
            except Exception:
                info = {}
            index = info.get("index")
            margin = info.get("margin")
            if index is not None:
                gripped = index >= HOLD_INDEX_RATIO
            else:
                gripped = margin is not None and margin >= HOLD_MARGIN_A
        if gripped:
            self._lockball_last_seen = None
            self._lockball_last_seen_ts = 0.0
            return ball, True

        if status == "lost":
            if (
                self._lockball_last_seen is not None
                and now - self._lockball_last_seen_ts <= self.lost_hold_seconds
            ):
                return self._lockball_last_seen, False
            return ball, False
        if status == "held":
            # 球进接球窗口，但电流还没确认咬住 —— 不算抓到。返回坐标让控制器
            # 往前压，等电流确认（上面的 gripped）后才停车。
            self._lockball_last_seen = None
            self._lockball_last_seen_ts = 0.0
            return ball, False
        self._lockball_last_seen = list(ball)
        self._lockball_last_seen_ts = now
        return ball, False

    def LockBallSlip(self):
        """追到 ``CatchVal`` 指定的持球位置。"""
        ball, caught = self._current_or_recent_ball()
        if caught:
            peripheral.Dribble(0.3)
            chassis.stop()
            self._reset_lockball_pid()
            return True
        if ball == BALL_NOT_FOUND:
            peripheral.Dribble(0.3)
            chassis.stop()
            self._reset_lockball_pid()
            return False

        ball_x, ball_y = ball[0], ball[1]
        if ball_x >= 0 and ball_y == 0:
            ball_angle = 0
        else:
            ball_angle = int(math.degrees(math.atan2(ball_y, ball_x)))
        speed_x = max(0, -self.pan_x(ball_x))
        speed_y = -self.pan_y(ball_y)
        speed_w = -self.pan_w(ball_angle)

        # 没咬住就缓慢往前压，直到电流确认咬住（“只有真的抓到球再停”）。
        # 旧逻辑在这里把 speed_x 归零，会在球前停住、永远咬不住。
        if ball_x > 0 and abs(ball_y - self.catch_y) < 3 and ball_x <= self.catch_x + 3:
            speed_x = max(speed_x, 0.08)
        if ball_x <= 0:
            speed_x = 0

        peripheral.Dribble(0.3)
        if ball_x < 0:
            chassis.RobotMoveVector(speed_x, speed_y, ball_angle)
        if ball_x > 0:
            chassis.RobotMoveVector(speed_x, speed_y, speed_w)
        return False

    def slip(self):
        return self.LockBallSlip()

lockball = LockSeries()

# Historical orbit wheel mix: forward = 0.1*r, lateral = -0.9*r, yaw = 3/r.
# The lateral term is large; during big turns it flings the ball sideways out
# of a low-power gripper. TURN_ORBIT_LATERAL scales that term for the carry
# skill (Circle keeps the historical 0.9 for other callers).
def _orbit_twist(r, k=1, d=1, lateral=0.9):
    """Raw wheel mix for :func:`Circle`; returned so callers can add a trim."""
    return (0.1 * r * d * k, -lateral * r * d * k, 3 / r * d * k)


def Circle(r, k = 1,d = 1):
    chassis.RobotMoveTwist(*_orbit_twist(r, k, d))

# TurnTo_with_ball tuning. The orbit radius matches the historical wheel mix;
# the align tolerance is the heading band inside which the skill reports done.
TURN_ORBIT_RADIUS = 0.6
TURN_ALIGN_TOLERANCE_DEG = 15.0
# Carry orbit lateral scaling. 0.9 is the historical wheel mix and the value
# validated on hardware; lower values were tested to cut the sideways fling on
# big turns but measured worse (slower, and one ball loss), so keep 0.9.
TURN_ORBIT_LATERAL = 0.9
# Ball-hold envelope on the raw robot-frame ball coordinate. RobotBallPos()
# now returns that coordinate while carrying, and UnitedPosition.ballPos keeps
# the same raw vision reading, which is stable to ~0.1cm on hardware. Inside
# the envelope the carry skill runs its own closed loop on that coordinate;
# outside it hands back to LockBallSlip to re-seat the ball.
TURN_HOLD_X_MIN_CM = 3.0
TURN_HOLD_X_MAX_CM = 12.0
TURN_HOLD_Y_ABS_CM = 4.0
# Closed-loop hold gains: robot-frame velocity (m/s) per cm of ball offset from
# CatchVal, layered on top of the orbit. +x drives forward, +y drives left, so a
# ball ahead/left gets a positive correction (same sign as LockBallSlip's PIDs).
TURN_HOLD_X_MPS_PER_CM = 0.0
TURN_HOLD_Y_MPS_PER_CM = 0.0
TURN_HOLD_MAX_MPS = 0.20


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _raw_held_ball_xy():
    """Return the raw robot-frame ball coordinate while held, or None.

    ``RobotBallPos()`` returns the real coordinate while carrying, and also
    stores it on ``UnitedPosition.ballPos``. Returns None when the reading is
    missing or still a legacy sentinel.
    """
    raw = getattr(UnitedPosition, "ballPos", None)
    if raw is None:
        return None
    try:
        if len(raw) < 2:
            return None
        bx = float(raw[0])
        by = float(raw[1])
    except (TypeError, ValueError, IndexError):
        return None
    sentinel = {float(BALL_NOT_FOUND[0]), float(BALL_HELD[0]), 0.0}
    if bx in sentinel or by in sentinel:
        return None
    return bx, by


def _ball_seated(bx: float, by: float) -> bool:
    """Whether a raw ball reading is still inside the carrying envelope."""
    if bx < TURN_HOLD_X_MIN_CM or bx > TURN_HOLD_X_MAX_CM:
        return False
    return abs(by - lockball.catch_y) <= TURN_HOLD_Y_ABS_CM

# True once TurnTo_with_ball has commanded a stop for the current alignment;
# the stop latches on the motion backend, so it must only be sent once.
_turnto_stop_latched = False


def TurnTo_with_ball(Aim_Angle, SpeedK: float | None = 1):
    """带球绕球转向到目标角度。

    这个函数不是普通原地转向。它故意用四轮差速让机器人围着已控住的球转，
    真实机器上比直接 FieldTurn/RobotTurn 更稳；模拟器物理不准时可能表现很差。

    语义：未持球时只做追球并返回 False（运球器由 LockBallSlip 管理，这里不再
    重复下发 Dribble，避免 0.3/0.2 交替打穿 chassis.Dribble 的状态缓存）。
    持球后绕球转向，直到航向进入 ``TURN_ALIGN_TOLERANCE_DEG`` 才返回 True；
    此时只在对准跳变的那一 tick 下发一次停车指令（后端会保持该指令），后续
    tick 直接返回 True，不再每 tick 重复 stop 刷总线。``SpeedK`` 缩放绕球
    速度（None 等价 1.0），增益会被限制在 [-1, 1] 之内。

    持球闭环：用 ``UnitedPosition.ballPos``（原始机体系坐标）作为持球反馈。
    只要坐标还在持球包络内就持续绕球，不会因为 ``RobotBallPos`` 的持球语义
    抖动而中断（旧实现每拍先跑 LockBallSlip，语义闪一下就会打断绕球）；按
    ``TURN_HOLD_X/Y_MPS_PER_CM`` 叠加位置修正保持球贴合 ``CatchVal``。坐标一旦
    离开包络就交回 ``LockBallSlip`` 重新吃球，避免把球甩丢。

    TODO: 后续把轮速常量、角度误差和速度倍率整理成可读参数，并单独记录
    真机/模拟器差异。不要在未真机验证前改成普通旋转。
    """
    global _turnto_stop_latched
    yaw = chassis.GetYaw()
    Aim_Angle = Aim_Angle % 360

    # Refresh the raw reading (side effect of RobotBallPos) and decide whether
    # we already hold the ball. This is deliberately checked BEFORE LockBallSlip
    # so a momentary semantic dropout cannot interrupt an in-progress carry.
    # (merged from feat 23f2bd5; the per-tick Dribble call was dropped with the
    # upstream edge-trigger refactor — Peripherals only sends on status change.)
    UnitedPosition.RobotBallPos()
    ball = _raw_held_ball_xy()
    if ball is None or not _ball_seated(ball[0], ball[1]):
        # Not measurably seated: let LockBallSlip chase/capture and report
        # "not done". Do NOT forward its return value: a semantic "caught" with
        # an out-of-envelope raw reading must not be reported as an aligned turn.
        lockball.LockBallSlip()
        # An interrupted carry must re-arm the aligned-stop latch (upstream
        # edge-trigger semantics): the next aligned tick stops again.
        _turnto_stop_latched = False
        return False

    # Shortest signed heading error to the aim, normalized to [-180, 180).
    error = angle_error_deg(Aim_Angle, yaw)
    if abs(error) < TURN_ALIGN_TOLERANCE_DEG:
        # Aligned: stop once rather than let the previous orbit command keep
        # turning. The command latches, so repeating it every tick would only
        # reset the backend command filter and load the serial bus.
        if not _turnto_stop_latched:
            chassis.stop()
            _turnto_stop_latched = True
        return True

    # Intentional raw wheel mix: orbit around the held ball while turning.
    # LinearMap(error) gives the signed gain in [-1, 1]; the fixed d=-1 keeps
    # the historical sign convention (positive error turns counter-clockwise).
    speed_k = 1.0 if SpeedK is None else float(SpeedK)
    gain = LinearMap(error, [-180, 180], [-1, 1]) * speed_k
    gain = max(-1.0, min(1.0, gain))
    vx, vy, wz = _orbit_twist(TURN_ORBIT_RADIUS, gain, -1, TURN_ORBIT_LATERAL)

    # Position hold on top of the orbit (merged from feat 23f2bd5). A ball
    # ahead (ex>0) or left (ey>0) of CatchVal gets a positive (forward/left)
    # correction, matching the sign of LockBallSlip's XY proportional gains.
    # TURN_HOLD_*_MPS_PER_CM default to 0.0, so this stays a no-op until tuned.
    ex = ball[0] - lockball.catch_x
    ey = ball[1] - lockball.catch_y
    vx += _clamp(TURN_HOLD_X_MPS_PER_CM * ex, TURN_HOLD_MAX_MPS)
    vy += _clamp(TURN_HOLD_Y_MPS_PER_CM * ey, TURN_HOLD_MAX_MPS)

    chassis.RobotMoveTwist(vx, vy, wz)
    # An orbit command must re-arm the aligned stop (upstream edge-trigger).
    _turnto_stop_latched = False
    return False


def Slipsideshot(attack_dir=1): #溜边
    """One tick of sideline carry and shot."""
    robot_ball = UnitedPosition.RobotBallPos()
    ball_status = UnitedPosition.BallStatus(robot_ball)
    close_ball = False
    try:
        close_ball = 0.0 <= float(robot_ball[0]) <= 24.0 and abs(float(robot_ball[1])) <= 16.0
    except Exception:
        close_ball = False
    if not close_ball and ball_status != "held":
        return lockball.LockBallSlip()
    try:
        local_x, local_y, _ = UnitedPosition.FieldRobotPose()
    except Exception:
        return False
    peripheral.Dribble(1.0)
    attack_dir = 1 if attack_dir >= 0 else -1
    side = 1.0 if local_y >= 0.0 else -1.0
    half_long = float(getattr(cfg.Bounds, "Long", 260)) * 0.5
    half_short = float(getattr(cfg.Bounds, "Short", 200)) * 0.5
    edge_y = side * min(68.0, half_short - 24.0)
    corner_progress = half_long - 30.0
    progress = attack_dir * local_x
    edge_error = edge_y - local_y
    forward_yaw = 0.0 if attack_dir > 0 else 180.0
    hide_yaw = (forward_yaw + side * attack_dir * 20.0) % 360.0
    near_corner = progress >= corner_progress - 16.0
    edge_tolerance = 24.0 if near_corner else 9.0
    if abs(edge_error) > edge_tolerance and progress < corner_progress - 14.0:
        carry_yaw = hide_yaw
        yaw_error = angle_error_deg(compass(), carry_yaw)
        speed_x = attack_dir * (0.16 if progress < corner_progress - 12.0 else 0.07)
        speed_y = max(-0.13, min(0.13, edge_error * 0.0027))
        if abs(yaw_error) > 25.0:
            speed_x *= 0.55
            speed_y *= 0.35
        chassis.FieldMoveVector(speed_x, speed_y, carry_yaw, KpW=max(0.18, min(0.55, abs(yaw_error) / 80.0)))
        return False
    if progress < corner_progress:
        carry_yaw = hide_yaw
        yaw_error = angle_error_deg(compass(), carry_yaw)
        speed_x = attack_dir * (0.25 if progress < corner_progress - 28.0 else 0.16)
        speed_y = max(-0.05, min(0.05, edge_error * 0.0016))
        if abs(yaw_error) > 25.0:
            speed_x *= 0.55
            speed_y *= 0.35
        chassis.FieldMoveVector(speed_x, speed_y, carry_yaw, KpW=max(0.18, min(0.50, abs(yaw_error) / 85.0)))
        return False
    aim = UnitedPosition.Pos2Angle([local_x, local_y], [attack_dir * half_long, -side * 18.0]) % 360
    yaw_error = angle_error_deg(compass(), aim)
    if abs(yaw_error) <= 18.0 and abs(edge_error) <= 34.0:
        peripheral.ShootBall()
        peripheral.Dribble(False)
        logger.success("Slipsideshot shot.")
        return True
    speed_x = -attack_dir * 0.03 if progress > corner_progress + 8.0 else 0.0
    speed_y = max(-0.02, min(0.02, edge_error * 0.0005))
    chassis.FieldMoveVector(speed_x, speed_y, aim, KpW=max(0.22, min(0.60, abs(yaw_error) / 80.0)))
    return False
