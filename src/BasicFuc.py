"""BasicFuc

历史策略常用函数集合。

注意：本文件存在大量历史代码。此次改动目标：
- 修复明显的语法/运行时错误（重复定义、错误调用）
- 避免把 `from LunaPre import *` 继续扩散到外部：本文件内部仍可使用，但对外建议显式导入
"""

from __future__ import annotations

from LunaPre import Settings, UnitedPosition, chassis, compass, logger, peripheral
from LunaPre import runtime
from LunaPre.strategy_tools import BALL_HELD, BALL_NOT_FOUND, angle_error_deg, clamp
import math
import simple_pid
import time

# Historical helpers refer to ``cfg``; keep it as an explicit alias rather
# than relying on a wildcard import to leak the name into this module.
cfg = Settings

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
        tuning = getattr(Settings.Motion, "LockBallSlip", None)
        self.forward_kp = float(getattr(tuning, "ForwardKp", 0.020))
        self.forward_ki = max(0.0, float(getattr(tuning, "ForwardKi", 0.006)))
        self.forward_kd = max(0.0, float(getattr(tuning, "ForwardKd", 0.005)))
        self.lateral_kp = float(getattr(tuning, "LateralKp", 0.016))
        self.lateral_ki = max(0.0, float(getattr(tuning, "LateralKi", 0.0005)))
        self.lateral_kd = max(0.0, float(getattr(tuning, "LateralKd", 0.005)))
        self.angular_kp = float(getattr(tuning, "AngularKp", 1.30))
        self.angular_ki = max(0.0, float(getattr(tuning, "AngularKi", 0.06)))
        self.angular_kd = max(0.0, float(getattr(tuning, "AngularKd", 0.15)))
        self.forward_max = max(0.0, float(getattr(tuning, "ForwardMaxMps", 0.80)))
        self.lateral_max = max(0.0, float(getattr(tuning, "LateralMaxMps", 0.65)))
        self.angular_max = max(0.0, float(getattr(tuning, "AngularMaxDegS", 85.0)))
        self.turn_only_angle = max(10.0, float(getattr(tuning, "TurnOnlyAngleDeg", 70.0)))
        self.turn_only_kp = float(getattr(tuning, "TurnOnlyKp", 0.75))
        self.turn_only_max = max(0.0, float(getattr(tuning, "TurnOnlyMaxDegS", 55.0)))
        self.rear_forward_kp = float(getattr(tuning, "RearForwardKp", 0.012))
        self.rear_forward_max = max(0.0, float(getattr(tuning, "RearForwardMaxMps", 0.55)))
        self.rear_lateral_kp = float(getattr(tuning, "RearLateralKp", 0.016))
        self.rear_lateral_max = max(0.0, float(getattr(tuning, "RearLateralMaxMps", 0.65)))
        self.rear_angular_kp = float(getattr(tuning, "RearAngularKp", 1.80))
        self.rear_angular_max = max(0.0, float(getattr(tuning, "RearAngularMaxDegS", 85.0)))
        self.lost_hold_seconds = max(0.0, float(getattr(tuning, "LostHoldSeconds", 0.18)))
        self.near_creep_max = max(0.0, float(getattr(tuning, "NearCreepMaxMps", 0.10)))
        # Docking: a fast approach carries too much momentum for the low-power
        # dribbler to hold, so the ball gets punched back out right before it
        # seats. The forward speed is capped on a continuous constant-deceleration
        # braking curve v = sqrt(2 a d) once the required stopping distance is
        # reached, so the far field still runs at its configured limit while the
        # last centimetres slow down smoothly instead of stepping at a fixed
        # range. A lower deceleration brakes earlier and docks more gently.
        self.near_dock_decel = max(0.0, float(getattr(tuning, "NearDockDecelMps2", 0.6)))
        self.ball_jump_reject_cm = max(0.0, float(getattr(tuning, "BallJumpRejectCm", 45.0)))
        self.ball_jump_confirm_frames = max(
            1, int(getattr(tuning, "BallJumpConfirmFrames", 2))
        )
        self._lockball_last_seen = None
        self._lockball_last_seen_ts = 0.0
        self._lockball_jump_candidate = None
        self._lockball_jump_candidate_frames = 0
        self._pose_revision = None
        self.pan_x = simple_pid.PID(
            self.forward_kp,
            self.forward_ki,
            self.forward_kd,
            setpoint=self.catch_x,
            output_limits=(-self.forward_max, 0),
            auto_mode=True,
        )
        self.pan_y = simple_pid.PID(
            self.lateral_kp,
            self.lateral_ki,
            self.lateral_kd,
            setpoint=self.catch_y,
            output_limits=(-self.lateral_max, self.lateral_max),
            auto_mode=True,
        )
        self.pan_w = simple_pid.PID(
            self.angular_kp,
            self.angular_ki,
            self.angular_kd,
            setpoint=0,
            output_limits=(-self.angular_max, self.angular_max),
            auto_mode=True,
        )

    def _reset_lockball_pid(self):
        self.pan_x.reset()
        self.pan_y.reset()
        self.pan_w.reset()

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
            self._reset_lockball_pid()
            self._lockball_last_seen = None
            self._lockball_last_seen_ts = 0.0
            self._clear_ball_jump_candidate()
        self._pose_revision = revision

    def _clear_ball_jump_candidate(self):
        self._lockball_jump_candidate = None
        self._lockball_jump_candidate_frames = 0

    def _current_or_recent_ball(self):
        """返回当前球坐标；如果刚丢球很短时间，就临时沿用上一帧坐标。

        视觉偶尔会闪一下 [0xfff, 0xfff]，直接停车会让追球动作抖动。
        所以这里保留一个很短的缓存窗口。真正持球 [0xddd, 0xddd] 时清掉缓存。
        """
        ball = UnitedPosition.RobotBallPos()
        now = time.time()
        if ball == BALL_NOT_FOUND:
            self._clear_ball_jump_candidate()
            if (
                self._lockball_last_seen is not None
                and now - self._lockball_last_seen_ts <= self.lost_hold_seconds
            ):
                return self._lockball_last_seen, False
            return ball, False
        if ball == BALL_HELD:
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
                self._reset_lockball_pid()
                return candidate, False

        self._clear_ball_jump_candidate()

        self._lockball_last_seen = candidate
        self._lockball_last_seen_ts = now
        return ball, False

    def LockBallSlip(self):
        """追到 ``CatchVal`` 指定的持球位置。"""
        self._reset_on_pose_change()
        if not self._chassis_can_move():
            # PID output must not accumulate while referee permission blocks
            # wheel commands, or it will surge when permission returns.
            self._reset_lockball_pid()
            chassis.stop()
            return False

        ball, caught = self._current_or_recent_ball()
        if caught:
            peripheral.Dribble(0.5)
            # chassis.stop()
            peripheral.Dribble(0.3)
            chassis.stop()
            self._reset_lockball_pid()
            return True
        if ball == BALL_NOT_FOUND:
            peripheral.Dribble(0.3)
        elif ball == BALL_NOT_FOUND:
            peripheral.Dribble(0.2)
            chassis.stop()
            self._reset_lockball_pid()
            return False
        else:
            peripheral.Dribble(0.3)

        ball_x, ball_y = ball[0], ball[1]
        if ball_x == 0 and ball_y == 0:
            ball_angle = 0
        else:
            ball_angle = int(math.degrees(math.atan2(ball_y, ball_x)))

        # Large heading errors and translation create an orbit: the ball moves
        # across the camera while the robot keeps turning, then the controller
        # reverses direction on the next frame. For a ball in front of the
        # robot, align in place first. The rear-ball controller below has its
        # own sign and speed limits, so it must keep priority for ball_x <= 0.
        if ball_x > 0 and abs(ball_angle) >= self.turn_only_angle:
            self._reset_lockball_pid()
            speed_w = max(
                -self.turn_only_max,
                min(self.turn_only_max, float(ball_angle) * self.turn_only_kp),
            )
            peripheral.Dribble(True)
            peripheral.Dribble(0.3)
            chassis.RobotMoveVector(0.0, 0.0, speed_w)
            return False

        # A rear ball cannot be reached by forward PID. Turn toward it quickly
        # while using a small lateral correction, then hand control back to the
        # normal capture loop once the ball is in front of the robot.
        if ball_x <= 0:
            self._reset_lockball_pid()
            speed_x = max(-self.rear_forward_max, min(0.0, float(ball_x) * self.rear_forward_kp))
            speed_y = max(-self.rear_lateral_max, min(self.rear_lateral_max, float(ball_y) * self.rear_lateral_kp))
            speed_w = max(-self.rear_angular_max, min(self.rear_angular_max, float(ball_angle) * self.rear_angular_kp))
            peripheral.Dribble(True)
            peripheral.Dribble(0.3)
            chassis.RobotMoveVector(speed_x, speed_y, speed_w)
            return False

        speed_x = -self.pan_x(ball_x)
        speed_y = -self.pan_y(ball_y)
        speed_w = -self.pan_w(ball_angle)

        # If the ball is already closer than CatchVal but still free, keep a
        # small forward creep so a ~1cm shortfall still gets eaten into the
        # dribbler instead of freezing with speed_x=0.
        if ball_x < self.catch_x and abs(ball_y - self.catch_y) < 2.5:
            creep = min(self.near_creep_max, max(0.04, (self.catch_x - float(ball_x)) * 0.05))
            speed_x = max(float(speed_x), creep)

        peripheral.Dribble(True)
        # Gentle final dock: cap the forward speed on a continuous v = sqrt(2 a d)
        # braking curve so the robot does not punch a low-power-held ball back out
        # on contact while still approaching at full speed from far away. The
        # cruise limit comes from the live pan_x output limit so tuning one knob
        # (ForwardMaxMps) automatically moves where braking starts.
        if self.near_dock_decel > 0.0:
            cruise = abs(float(self.pan_x.output_limits[0]))
            remaining_m = max(0.0, float(ball_x) - self.catch_x) / 100.0
            brake_start_m = (cruise * cruise) / (2.0 * self.near_dock_decel)
            if cruise > 0.0 and remaining_m < brake_start_m:
                speed_x = min(
                    float(speed_x),
                    math.sqrt(2.0 * self.near_dock_decel * remaining_m),
                )

        peripheral.Dribble(0.3)
        chassis.RobotMoveVector(speed_x, speed_y, speed_w)
        return False

    def slip(self):
        return self.LockBallSlip()


lockball = LockSeries()

def Circle(r, k = 1,d = 1):
    chassis.RobotMoveTwist(0.1*r*d*k,-0.9*r*d*k,3/r*d*k)

def TurnTo_with_ball(Aim_Angle,SpeedK:int | None = 1):
# TurnTo_with_ball tuning. The orbit radius matches the historical wheel mix;
# the align tolerance is the heading band inside which the skill reports done.
TURN_ORBIT_RADIUS = 0.6
TURN_ALIGN_TOLERANCE_DEG = 15.0


def TurnTo_with_ball(Aim_Angle, SpeedK: float | None = 1):
    """带球绕球转向到目标角度。

    这个函数不是普通原地转向。它故意用四轮差速让机器人围着已控住的球转，
    真实机器上比直接 FieldTurn/RobotTurn 更稳；模拟器物理不准时可能表现很差。

    语义：未持球时只做追球并返回 False；持球后绕球转向，直到航向进入
    ``TURN_ALIGN_TOLERANCE_DEG`` 才返回 True（此时会主动停车）。``SpeedK``
    缩放绕球速度（None 等价 1.0），增益会被限制在 [-1, 1] 之内。

    TODO: 后续把轮速常量、角度误差和速度倍率整理成可读参数，并单独记录
    真机/模拟器差异。不要在未真机验证前改成普通旋转。
    """
    yaw = chassis.GetYaw()
    ball = UnitedPosition.RobotBallPos()
    Aim_Angle = Aim_Angle%360
    if lockball.LockBallSlip() == True:
        peripheral.Dribble(True)
        # Robot-frame ball angle (0=forward)
        iDeltaAngle = int(math.degrees(math.atan2(ball[1], ball[0])))
        Error = angle_error_deg(Aim_Angle, yaw)
        if Error < 180:
            iDirectionFactor = -1
        elif Error >= 180:
            iDirectionFactor = 1
        Error = LinearMap(Error,[-180,180],[-1,1])
        if abs(Aim_Angle-yaw) >= 15:
            # Intentional raw wheel mix: orbit around the held ball while turning.
            Circle(0.6,Error,iDirectionFactor)
            return False
        else:
            return True
    else:
        return lockball.LockBallSlip()
    Aim_Angle = Aim_Angle % 360

    # Not holding the ball yet: keep chasing this tick and report "not done".
    # The old code called LockBallSlip() twice and returned the second result,
    # which could report success without ever aligning.
    if not lockball.LockBallSlip():
        return False
    peripheral.Dribble(0.2)

    # Shortest signed heading error to the aim, normalized to [-180, 180).
    error = angle_error_deg(Aim_Angle, yaw)
    if abs(error) < TURN_ALIGN_TOLERANCE_DEG:
        # Aligned: stop rather than let the previous orbit command keep turning.
        chassis.stop()
        return True

    # Intentional raw wheel mix: orbit around the held ball while turning.
    # LinearMap(error) gives the signed gain in [-1, 1]; the fixed d=-1 keeps
    # the historical sign convention (positive error turns counter-clockwise).
    speed_k = 1.0 if SpeedK is None else float(SpeedK)
    gain = LinearMap(error, [-180, 180], [-1, 1]) * speed_k
    gain = max(-1.0, min(1.0, gain))
    Circle(TURN_ORBIT_RADIUS, gain, -1)
    return False


def Slipsideshot(attack_dir=1): #溜边
    """One tick of sideline carry and shot."""
    robot_ball = UnitedPosition.RobotBallPos()
    close_ball = False
    try:
        close_ball = 0.0 <= float(robot_ball[0]) <= 24.0 and abs(float(robot_ball[1])) <= 16.0
    except Exception:
        close_ball = False
    if not close_ball and robot_ball != BALL_HELD:
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