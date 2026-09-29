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
import time

# Historical helpers refer to ``cfg``; keep it as an explicit alias rather
# than relying on a wildcard import to leak the name into this module.
cfg = Settings

# LockBallSlip dribble profile.  Chasing and possession use one fixed roller
# power; a lost ball falls back to a weaker command.
LOCKBALL_DRIBBLE_FAR_POWER = 0.30  # 常态追球功率（抓到后升到 HOLD_POWER=0.5）
# 注：0.2 在电机死区内（起不来、标定基线=怠速），会污染受载判断，弃用。
LOCKBALL_DRIBBLE_LOST_POWER = 0.30

# “只有真的抓到球再停”：球进接球窗口后，还要盘球电流确认滚轮确实咬住（受载），
# 才判定捕获并停车；否则以低速继续往里压，直到真咬住。读不到电流信息时退回旧行为
# （直接算捕获），避免传感器异常导致永远不停车。
LOCKBALL_SEAT_REQUIRE_LOAD = True
LOCKBALL_SEAT_PUSH_MPS = 0.15  # 未咬合时的按压前进速度（m/s）；0.06 低于死区会“贴住不动”

# 抓到球后的 transition：从追球功率在 HOLD_RAMP_S 内平滑提升到保持功率；
# 期间一旦丢球就中断（回到追球/丢球逻辑）。功率渐变回落到追球功率。
LOCKBALL_HOLD_POWER = 0.50
LOCKBALL_HOLD_RAMP_S = 0.5

# LockBallSlip 回滚到 PID 之前的人类手调版：按“最远轴距离”选一档比例增益，
# 直接对球坐标做比例控制。距离单位 cm，平移输出 m/s，转向输出 deg/s。
# 调参只改这张表：阈值从远到近排列，(触发距离cm, 平移增益 m/s/cm, 转向增益 deg/s/deg)。
LOCKBALL_GAIN_TABLE = (
    (75.0, 0.020, 4.0),   # 远：距离 > 75cm
    (25.0, 0.017, 1.5),   # 中：25cm < 距离 <= 75cm
    (0.0,  0.060, 1.0),   # 近：距离 <= 25cm（平移增益要够大，否则转弯半径 > 球距会绕圈）
)
LOCKBALL_FORWARD_BOOST = 1.5   # 旧版对前进分量的额外放大
LOCKBALL_MAX_FORWARD_MPS = 1.80
LOCKBALL_MAX_LATERAL_MPS = 0.85
# 近球防死区：球在正前方这段距离内时，前进速度不低于该值，否则会出现
# “球就在眼前却贴住不动/拿不到”的现象（低于电机启动死区）。
LOCKBALL_MIN_APPROACH_MPS = 0.22
LOCKBALL_MIN_APPROACH_CM = 18.0
# 姿态（转向）响应：旧人类版 KpZ=4 且不封顶，这里给一个远离物理上限的值
# （底盘 yaw 物理上限约 967 deg/s）。转向越大，底盘 turn_translation_scale
# 会自动收前进油，所以放开上限不会导致明显画弧。
LOCKBALL_MAX_YAW_DEGPS = 480.0
# 转向微分阻尼：只有 P 时大角度会过冲/绕球，D 项按球方位角变化率减速。
# 变化率先低通再参与 D，避免视觉跳变把 D 打成尖峰。
LOCKBALL_YAW_KD = 0.0
LOCKBALL_YAW_RATE_ALPHA = 0.30
# 近球铲球：进入该距离后限制前进速度，避免高速把球顶飞；未对准（方位角大）
# 时先转不冲；近球不再横移（横移会把球滚丢），只保留少量。
LOCKBALL_DOCK_CM = 22.0
LOCKBALL_DOCK_MAX_MPS = 0.45
# 贴到嘴边再降速（1~2cm 级），避免最后一下把球顶飞。
LOCKBALL_CLOSE_CM = 5.0
LOCKBALL_CLOSE_MAX_MPS = 0.12
LOCKBALL_DOCK_LATERAL_SCALE = 0.20
LOCKBALL_ALIGN_FULL_DEG = 12.0

# 丢球后短暂沿用上一帧坐标的窗口，以及跳变的确认帧数。
LOCKBALL_LOST_HOLD_SECONDS = 0.18
LOCKBALL_BALL_JUMP_REJECT_CM = 45.0
LOCKBALL_BALL_JUMP_CONFIRM_FRAMES = 2
# 近球丢视觉（球钻进车前/被遮住）时的“座位”宽限：这段时间内继续往前压去铲球，
# 而不是直接停车。半径内的最后已知球位才触发。
LOCKBALL_SEAT_RADIUS_CM = 15.0
LOCKBALL_SEAT_GRACE_S = 1.2
LOCKBALL_SEARCH_YAW_DEGPS = 60.0  # 丢球后慢速自转搜索速度
# 提速条件 = “盘到球”：直接用标定受载判据 loaded（BallLoadInfo），不再单独
# 设一个更低的“接触阈值”——空转/起转的电流起伏会淹没它，导致球没碰到就提速。

def LinearMap(value, input_range, output_range):
    """把 ``value`` 从一个数值区间线性映射到另一个区间。
    
    Args:
        value: 要映射的值
        input_range: 输入范围 (min, max)
        output_range: 输出范围 (min, max)
    
    Returns:
        映射后的值
    """
    input_min, input_max = input_range
    output_min, output_max = output_range
    if input_min == input_max:
        raise ValueError("input_range 的两个端点不能相同")
    
    # 计算映射
    input_span = input_max - input_min
    output_span = output_max - output_min
    
    scaled_value = (value - input_min) / input_span
    mapped_value = output_min + (scaled_value * output_span)
    
    return mapped_value


class LockSeries:
    """机器人坐标系下的抢球/锁球控制器。"""

    def __init__(self):
        catch_val = getattr(Settings.Motion, "CatchVal", [8.5, 0])
        self.catch_x = catch_val[0] if len(catch_val) > 0 else 8.5
        self.catch_y = catch_val[1] if len(catch_val) > 1 else 0
        self.lost_hold_seconds = LOCKBALL_LOST_HOLD_SECONDS
        self.ball_jump_reject_cm = LOCKBALL_BALL_JUMP_REJECT_CM
        self.ball_jump_confirm_frames = LOCKBALL_BALL_JUMP_CONFIRM_FRAMES
        self._lockball_last_seen = None
        self._lockball_last_seen_ts = 0.0
        self._lockball_jump_candidate = None
        self._lockball_jump_candidate_frames = 0
        self._pose_revision = None
        self._lockball_was_caught = False
        self._lockball_in_window = False
        self._lockball_holding = False
        self._lockball_hold_ts = 0.0
        self._lockball_last_angle = None
        self._lockball_last_angle_ts = 0.0
        self._lockball_rate_filt = 0.0

    @staticmethod
    def _chassis_can_move() -> bool:
        can_move = getattr(chassis, "CanMove", None)
        if not callable(can_move):
            return True
        try:
            return bool(can_move())
        except Exception:
            return False

    def _reset_on_pose_change(self):
        bits = getattr(runtime, "Bits", None)
        getter = getattr(bits, "GetPoseRevision", None)
        if not callable(getter):
            return
        revision = int(getter())
        if self._pose_revision is not None and revision != self._pose_revision:
            self._lockball_last_seen = None
            self._lockball_last_seen_ts = 0.0
            self._clear_ball_jump_candidate()
            self._abort_hold()
        self._pose_revision = revision

    def _clear_ball_jump_candidate(self):
        self._lockball_jump_candidate = None
        self._lockball_jump_candidate_frames = 0

    @staticmethod
    def _grip_confirmed() -> bool:
        """盘球电流是否确认滚轮真的咬住球（“真的抓到球”的判据）。

        ``BallStatus()`` 只说明球进了接球窗口；这里再叠加电流受载状态
        （``BallLoadInfo().loaded``，由校准 + 在线细化得出）。读不到电流信息时
        保守放行、退回旧行为，避免传感器异常导致永远不停车。
        """

        if not LOCKBALL_SEAT_REQUIRE_LOAD:
            return True
        try:
            loaded = UnitedPosition.BallLoadInfo().get("loaded")
        except Exception:
            return True
        return True if loaded is None else bool(loaded)

    def _begin_hold(self, now: float) -> None:
        """进入“保持”transition：记录起点，随后在 HOLD_RAMP_S 内升功率。"""
        self._lockball_holding = True
        self._lockball_hold_ts = now

    def _abort_hold(self) -> None:
        """中断保持 transition（丢球 / 脱落时调用），下次抓到重新从追球功率升。"""
        self._lockball_holding = False
        self._lockball_hold_ts = 0.0

    def _hold_power(self, now: float) -> float:
        """保持阶段的下发功率：从追球功率线性升到保持功率。"""
        if LOCKBALL_HOLD_RAMP_S <= 0.0:
            return LOCKBALL_HOLD_POWER
        ratio = (now - self._lockball_hold_ts) / LOCKBALL_HOLD_RAMP_S
        ratio = max(0.0, min(1.0, ratio))
        return LOCKBALL_DRIBBLE_FAR_POWER + ratio * (
            LOCKBALL_HOLD_POWER - LOCKBALL_DRIBBLE_FAR_POWER
        )

    def _current_or_recent_ball(self):
        """返回当前球坐标；如果刚丢球很短时间，就临时沿用上一帧坐标。

        视觉偶尔会闪一下 [0xfff, 0xfff]，直接停车会让追球动作抖动。
        所以这里保留一个很短的缓存窗口。持球状态由 ``BallStatus()`` 判定，
        真正持球时清掉缓存。
        """
        ball = UnitedPosition.RobotBallPos()
        ball_status = UnitedPosition.BallStatus(ball)
        self._lockball_in_window = ball_status == "held"
        now = time.time()
        if ball_status == "lost":
            self._clear_ball_jump_candidate()
            if self._lockball_last_seen is not None:
                age = now - self._lockball_last_seen_ts
                last = self._lockball_last_seen
                close = math.hypot(float(last[0]), float(last[1])) <= LOCKBALL_SEAT_RADIUS_CM
                grace = LOCKBALL_SEAT_GRACE_S if close else self.lost_hold_seconds
                if age <= grace:
                    # 近球丢视觉：当作“在窗口”，继续往前压去铲球，不要停死。
                    self._lockball_in_window = close
                    return last, False
            return ball, False
        if ball_status == "held":
            if not self._grip_confirmed():
                # 球进了接球窗口，但盘球电流还没确认滚轮咬住——不算“抓到”。
                # 保留坐标，交给 LockBallSlip 低速往里压，直到真咬住才停车。
                self._clear_ball_jump_candidate()
                self._lockball_last_seen = list(ball)
                self._lockball_last_seen_ts = now
                return ball, False
            self._lockball_last_seen = None
            self._lockball_last_seen_ts = 0.0
            self._clear_ball_jump_candidate()
            return ball, True

        # 电流已确认滚轮咬住、且球确实在车前接球口附近 —— 不管球坐标在不在接球
        # 窗口，都算抓到球（实测咬合点常略超窗口，只靠 BallStatus 会漏判）。
        # 必须限制在“车前近处”：否则盘球电流的升功率瞬态/噪声会在球还在侧后方
        # 时误判捕获，机器人就原地不追了。
        if (self._grip_confirmed()
                and 0.0 < float(ball[0]) <= self.catch_x + 6.0
                and abs(float(ball[1])) <= 6.0):
            self._lockball_last_seen = None
            self._lockball_last_seen_ts = 0.0
            self._clear_ball_jump_candidate()
            return ball, True

        # A scene reset or delayed simulator packet can briefly report a stale
        # ball position hundreds of centimetres away from the previous frame.
        # Reject a single impossible jump, but do not keep rejecting forever:
        # a genuine target change must be accepted after a few consecutive
        # observations and the PID state must be restarted for the new target.
        candidate = list(ball)
        if self._lockball_last_seen is not None:
            jump_cm = math.hypot(
                float(candidate[0]) - float(self._lockball_last_seen[0]),
                float(candidate[1]) - float(self._lockball_last_seen[1]),
            )
            if jump_cm > self.ball_jump_reject_cm:
                # Count consecutive observations that remain far from the
                # previous accepted target.  The ball can move between those
                # observations, so comparing the candidate lists for exact
                # equality would reject a real moving target forever.
                self._lockball_jump_candidate = candidate
                self._lockball_jump_candidate_frames += 1

                if self._lockball_jump_candidate_frames < self.ball_jump_confirm_frames:
                    return self._lockball_last_seen, False

                self._lockball_last_seen = candidate
                self._lockball_last_seen_ts = now
                self._clear_ball_jump_candidate()
                return candidate, False

        self._clear_ball_jump_candidate()

        self._lockball_last_seen = candidate
        self._lockball_last_seen_ts = now
        return ball, False

    def LockBallSlip(self):
        """追到 ``CatchVal`` 指定的持球位置。"""
        self._reset_on_pose_change()
        if not self._chassis_can_move():
            # 权限被裁判锁住时不要下发任何轮速。
            chassis.stop()
            return False

        ball, caught = self._current_or_recent_ball()
        now = time.time()

        # 提速条件 = “盘到球”：用标定受载判据 loaded（电流相对空转基线升过标定
        # margin，且过了启动屏蔽/迟滞）。不用更低的“接触阈值”，因为空转/起转的
        # 电流起伏会淹没它，导致球还没碰到就误提速。
        staging = caught
        if staging and not self._lockball_holding:
            self._begin_hold(now)
        elif not staging and self._lockball_holding:
            self._abort_hold()  # 电流回落（丢了/没咬住）：中断提升
        hold_power = (
            self._hold_power(now) if self._lockball_holding
            else LOCKBALL_DRIBBLE_FAR_POWER
        )

        if caught:
            # 电流确认滚轮真咬住：捕获沿停车一次，并维持提升后的保持功率。
            # （不停车会让车一直往前蹭、把球推走；调用方如 TurnTo_with_ball 会在
            #  随后自己发运动指令，所以只在沿上停一次。）
            if not self._lockball_was_caught:
                chassis.stop()
            self._lockball_was_caught = True
            peripheral.Dribble(hold_power)
            return True

        if self._lockball_in_window:
            self._lockball_was_caught = False
            peripheral.Dribble(hold_power)
            chassis.RobotMoveVector(LOCKBALL_SEAT_PUSH_MPS, 0.0, 0.0)
            return False

        if ball == BALL_NOT_FOUND:
            self._lockball_was_caught = False
            peripheral.Dribble(LOCKBALL_DRIBBLE_LOST_POWER)
            # 丢球别停死：慢速自转扫一圈找球（否则球滚到侧后方就永远追不到）。
            chassis.RobotMoveVector(0.0, 0.0, LOCKBALL_SEARCH_YAW_DEGPS)
            return False

        self._lockball_was_caught = False
        ball_x, ball_y = float(ball[0]), float(ball[1])
        if ball_x == 0.0 and ball_y == 0.0:
            ball_angle = 0.0
        else:
            ball_angle = math.degrees(math.atan2(ball_y, ball_x))

        # 人类手调分段比例律：按“最远轴距离”选一档增益，直接对球坐标做比例控制。
        # 后向球得到负的前进分量，自然倒车追球；大角度误差由转向项纠回。
        # 调参改 LOCKBALL_GAIN_TABLE。
        reach_cm = max(abs(ball_x), abs(ball_y))
        gain = LOCKBALL_GAIN_TABLE[-1][1]
        yaw_gain = LOCKBALL_GAIN_TABLE[-1][2]
        for min_cm, xy_gain, wz_gain in LOCKBALL_GAIN_TABLE:
            if reach_cm > min_cm:
                gain, yaw_gain = xy_gain, wz_gain
                break

        # 未对准先转：方位角越大，前进/横移越小；球在身后只转不冲。
        if abs(ball_angle) >= 90.0:
            align = 0.0
        else:
            align = math.cos(math.radians(min(abs(ball_angle), 90.0)))
            full = math.cos(math.radians(LOCKBALL_ALIGN_FULL_DEG))
            if full > 1e-6:
                align = min(1.0, align / full)

        # 近球限速铲球，避免高速把球顶飞。
        dist_cm = math.hypot(ball_x, ball_y)
        in_dock = dist_cm <= LOCKBALL_DOCK_CM
        if dist_cm <= LOCKBALL_CLOSE_CM:
            # 贴到嘴边（1~2cm 级）再降速，避免把球顶飞。
            fwd_cap = LOCKBALL_CLOSE_MAX_MPS
        elif in_dock:
            fwd_cap = LOCKBALL_DOCK_MAX_MPS
        else:
            fwd_cap = LOCKBALL_MAX_FORWARD_MPS
        speed_x = _clamp(ball_x * gain * LOCKBALL_FORWARD_BOOST, fwd_cap) * align
        speed_y = _clamp(ball_y * gain, LOCKBALL_MAX_LATERAL_MPS) * align
        # 近球不再横移：横移会把球滚来滚去，靠近时只靠转向对准 + 前压。
        if in_dock:
            speed_y *= LOCKBALL_DOCK_LATERAL_SCALE

        # 转向 PD：P 用球方位角，D 用其变化率做阻尼（抑制过冲）。
        # 变化率做低通：视觉球位偶尔跳一格，直接微分会把 D 打成尖峰。
        angle_rate = 0.0
        if self._lockball_last_angle is not None:
            angle_dt = now - self._lockball_last_angle_ts
            if angle_dt > 1e-4:
                delta = (ball_angle - self._lockball_last_angle + 180.0) % 360.0 - 180.0
                angle_rate = delta / angle_dt
        self._lockball_last_angle = ball_angle
        self._lockball_last_angle_ts = now
        self._lockball_rate_filt += LOCKBALL_YAW_RATE_ALPHA * (angle_rate - self._lockball_rate_filt)
        speed_w = _clamp(
            ball_angle * yaw_gain - LOCKBALL_YAW_KD * self._lockball_rate_filt,
            LOCKBALL_MAX_YAW_DEGPS,
        )

        # 近球防死区：正前方近距离保证一个能启动的最小前进速度（不超过当前限速档）。
        if 0.0 < ball_x <= LOCKBALL_MIN_APPROACH_CM and abs(ball_y) < 6.0:
            speed_x = max(speed_x, min(LOCKBALL_MIN_APPROACH_MPS, fwd_cap) * align)

        peripheral.Dribble(hold_power)
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
