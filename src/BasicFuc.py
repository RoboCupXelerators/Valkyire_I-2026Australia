"""BasicFuc

历史策略常用函数集合。

注意：本文件存在大量历史代码。此次改动目标：
- 修复明显的语法/运行时错误（重复定义、错误调用）
- 避免把 `from LunaPre import *` 继续扩散到外部：本文件内部仍可使用，但对外建议显式导入
"""

from __future__ import annotations

# TODO: Replace this legacy wildcard import with explicit LunaPre facade names
# used by this module: Settings, UnitedPosition, chassis, peripheral, logger.
from LunaPre import *
from LunaPre import runtime
from LunaPre.strategy_tools import BALL_HELD, BALL_NOT_FOUND, angle_error_deg, clamp
import math
import simple_pid
import time

def RoundThresholdJudger(iValue, iRound, iMiddleValue, iOffset):
    """判断周期值是否落在允许跨零点的阈值窗口内。

    参数:
        iValue: 待判断的值。
        iRound: 周期长度，例如角度场景下通常是 360。
        iMiddleValue: 阈值窗口中心，与 ``iValue`` 使用同一单位。
        iOffset: 阈值窗口半宽。
    """
    if iRound == 0:
        raise ValueError("iRound 不能为 0")
    iValue = iValue % iRound
    
    iLowerThreshold = (iMiddleValue - iOffset) % iRound
    iUpperThreshold = (iMiddleValue + iOffset) % iRound

    if iLowerThreshold <= iUpperThreshold:
        return iLowerThreshold <= iValue <= iUpperThreshold

    else:
        return iValue >= iLowerThreshold or iValue <= iUpperThreshold

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
        self.lost_hold_seconds = 0.12
        self._lockball_last_seen = None
        self._lockball_last_seen_ts = 0.0
        self._pose_revision = None
        self.pan_x = simple_pid.PID(0.023, 0.018, 0.01, setpoint=self.catch_x, output_limits=(-1.0, 0), auto_mode=True)
        self.pan_y = simple_pid.PID(0.018, 0.002, 0.01, setpoint=self.catch_y, output_limits=(-0.8, 0.8), auto_mode=True)
        self.pan_w = simple_pid.PID(1.5, 0.2, 0.3, setpoint=0, output_limits=(-200, 200), auto_mode=True)

    def _reset_lockball_pid(self):
        self.pan_x.reset()
        self.pan_y.reset()
        self.pan_w.reset()

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
        self._pose_revision = revision

    def _current_or_recent_ball(self):
        """返回当前球坐标；如果刚丢球很短时间，就临时沿用上一帧坐标。

        视觉偶尔会闪一下 [0xfff, 0xfff]，直接停车会让追球动作抖动。
        所以这里保留一个很短的缓存窗口。真正持球 [0xddd, 0xddd] 时清掉缓存。
        """
        ball = UnitedPosition.RobotBallPos()
        now = time.time()
        if ball == BALL_NOT_FOUND:
            if (
                self._lockball_last_seen is not None
                and now - self._lockball_last_seen_ts <= self.lost_hold_seconds
            ):
                return self._lockball_last_seen, False
            return ball, False
        if ball == BALL_HELD:
            self._lockball_last_seen = None
            self._lockball_last_seen_ts = 0.0
            return ball, True
        self._lockball_last_seen = list(ball)
        self._lockball_last_seen_ts = now
        return ball, False

    def LockBallSlip(self):
        """追到 ``CatchVal`` 指定的持球位置。"""
        self._reset_on_pose_change()
        ball, caught = self._current_or_recent_ball()
        if caught:
            peripheral.Dribble(True)
            chassis.stop()
            self._reset_lockball_pid()
            return True
        if ball == BALL_NOT_FOUND:
            peripheral.Dribble(True)
            chassis.stop()
            self._reset_lockball_pid()
            return False

        ball_x, ball_y = ball[0], ball[1]
        if ball_x == 0 and ball_y == 0:
            ball_angle = 0
        else:
            ball_angle = int(math.degrees(math.atan2(ball_y, ball_x)))
        speed_x = -self.pan_x(ball_x)
        speed_y = -self.pan_y(ball_y)
        speed_w = -self.pan_w(ball_angle)

        # If the ball is already closer than CatchVal but still free, keep a
        # small forward creep so a ~1cm shortfall still gets eaten into the
        # dribbler instead of freezing with speed_x=0.
        if ball_x < self.catch_x and abs(ball_y - self.catch_y) < 2.5:
            creep = min(0.12, max(0.04, (self.catch_x - float(ball_x)) * 0.05))
            speed_x = max(float(speed_x), creep)
        if ball_x <= 0:
            speed_x = 0

        peripheral.Dribble(True)
        chassis.RobotMoveVector(speed_x, speed_y, speed_w)
        return False

    def slip(self):
        return self.LockBallSlip()


lockball = LockSeries()

def NormalShoot(x=1, HomePos=[0, -20, 0]):
    """执行旧版抢球/射门逻辑的一个 tick。

    参数:
        x: 为 ``0`` 时，只确认球已经进入前方捕获区域，不执行射门。
        HomePos: 丢球后回退使用的场地位姿。

    返回:
        持球或射门流程完成时返回 ``True``，否则返回锁球流程的结果。
    """

    bx, by = UnitedPosition.RobotBallPos()
    logger.debug("Ball Position: %s" % [bx, by])

    # 没看到球。
    if [bx, by] == BALL_NOT_FOUND or (bx == 0 and by == 0):
        logger.info("Ball not found, go home.")
        UnitedPosition.Pos2Pos(HomePos)
        peripheral.Dribble(False)
        return False

    # 已经持球。
    if [bx, by] == BALL_HELD:
        # 已经持球，后续动作由调用方决定。
        return True

    # 球在前方捕获区域内，沿用历史启发式判断。
    if bx == 0 and 5 < by <= 9:
        time.sleep(0.03)
        logger.info("Ball is in front")
        if x == 0:
            return True

        peripheral.Dribble(True)
        for _ in range(10):
            bx, by = UnitedPosition.RobotBallPos()
            if not (bx == 0 and by <= 9):
                break

            x0, y0, _ = UnitedPosition.FieldRobotPose()
            # 兜底：用场地中心和边界估算瞄准角。
            delta_y = cfg.Bounds.Long / 2 - y0
            delta_x = x0
            theta = math.degrees(math.atan2(delta_y, delta_x))
            aim = theta - 90
            current = compass()
            delta = abs(aim - current)

            if delta > 60:
                chassis.FieldTurn(aim)
                break
            else:
                chassis.FieldMoveAngle(0, aim, 50)
            time.sleep(0.03)

        peripheral.ShootBall()
        return True

    peripheral.Dribble(True)
    return lockball.LockBallSlip()

def CircleAround(iAimAngle, max_ticks: int | None = None):
    """围绕球调整姿态，直到机器人朝向 ``iAimAngle``。

    这是一个阻塞式历史辅助函数。``max_ticks`` 主要用于测试或受控运行；
    不传时保留历史上的无限等待行为。
    """
    iCompass = int(compass())
    if RoundThresholdJudger(iCompass,360,iAimAngle+90,90):
        iDirectionFactor = 1
    else:
        iDirectionFactor = -1
    lBallPos = UnitedPosition.RobotBallPos()

    TempBX = -lBallPos[1]
    TempBY = lBallPos[0]

    lBallPos = [TempBX,TempBY]


    tick_count = 0
    while True:
        lBallPos = UnitedPosition.RobotBallPos()

        TempBX = -lBallPos[1]
        TempBY = lBallPos[0]

        lBallPos = [TempBX,TempBY]


        iX, iY = lBallPos[0], lBallPos[1]
        iDeltaAngle = 0
        if iX != 0 or iY != 0:
            iDeltaAngle = -int(math.degrees(math.atan2(iY, iX)) - 90)     
        # print(iX,iY,iDeltaAngle,compass())
        if RoundThresholdJudger(compass(), 360, iAimAngle, 3):
            break
        else:
            chassis.SetMotor([iDirectionFactor * 15,-iDirectionFactor * (150 - iDeltaAngle),-iDirectionFactor * 15,iDirectionFactor * (150 - iDeltaAngle)])
        time.sleep(.03)
        tick_count += 1
        if max_ticks is not None and tick_count >= max_ticks:
            chassis.stop()
            return False
    chassis.TurnTo(iAimAngle)
    tick_count = 0
    while True:
        lBallPos = UnitedPosition.RobotBallPos()

        TempBX = -lBallPos[1]
        TempBY = lBallPos[0]

        lBallPos = [TempBX,TempBY]

        iBX = lBallPos[0]
        if lBallPos[0] + lBallPos[1] > 20:
            break
        if iBX <= -Settings.Motion.ErrorRange/3:
            logger.debug('atleft')
            chassis.FieldMoveAngle(iAimAngle,iAimAngle + 90,40)
        elif iBX >= Settings.Motion.ErrorRange/3:
            logger.debug('atright')
            chassis.FieldMoveAngle(iAimAngle,iAimAngle + 90,-40)
        else:
            for _ in range(3):
                chassis.stop()
            break
        time.sleep(.03)
        tick_count += 1
        if max_ticks is not None and tick_count >= max_ticks:
            chassis.stop()
            return False
    print('stopped')
    return True

def TurnTo_with_ball(Aim_Angle,SpeedK:int | None = 1):
    """带球绕球转向到目标角度。

    这个函数不是普通原地转向。它故意用四轮差速让机器人围着已控住的球转，
    真实机器上比直接 FieldTurn/RobotTurn 更稳；模拟器物理不准时可能表现很差。

    TODO: 后续把轮速常量、角度误差和速度倍率整理成可读参数，并单独记录
    真机/模拟器差异。不要在未真机验证前改成普通旋转。
    """
    yaw = chassis.GetYaw()
    ball = UnitedPosition.RobotBallPos()
    Aim_Angle = Aim_Angle%360
    logger.success(Aim_Angle)
    if lockball.LockBallSlip() == True:
        # Robot-frame ball angle (0=forward)
        iDeltaAngle = int(math.degrees(math.atan2(ball[1], ball[0])))
        Error = angle_error_deg(Aim_Angle, yaw)
        if Error < 180:
            iDirectionFactor = -1
        elif Error >= 180:
            iDirectionFactor = 1
        if abs(Aim_Angle-yaw) >= 10:
            logger.error(Error)
            # Intentional raw wheel mix: orbit around the held ball while turning.
            chassis.SetMotor([
                iDirectionFactor * 15 * SpeedK *Error/75,
                -iDirectionFactor * (150 - iDeltaAngle) * SpeedK *Error/75,
                -iDirectionFactor * 15 * SpeedK *Error/75,
                iDirectionFactor * (150 - iDeltaAngle) * SpeedK *Error/75
                ])
            return False
        else:
            return True
    else:
        return lockball.LockBallSlip()


def TurnTo_with_ball_orbit(
    Aim_Angle,
    SpeedK: float | None = 1.0,
    *,
    angle_tolerance: float = 8.0,
    turn_kp: float = 20,
    max_wz_deg_s: float | None = None,
    max_linear_mps: float = 0.38,
    orbit_radius_cm: float | None = None,
    center_gain: float = 0.010,
    dribble_power: float = 0.5,
    reacquire: bool = True,
):
    """绕球转向到目标角度，一次调用只执行一个控制 tick。

    这是 ``TurnTo_with_ball`` 的模拟器友好替代版，但不覆盖旧函数。
    它不直接写 raw wheel mix，而是把"绕球转"的几何关系转换成底盘速度：
    若球在机器人坐标 r=(x_forward, y_left)，要让球近似成为瞬时旋转中心，
    车体速度取 ``vx = omega*y``、``vy = -omega*x``。

    逻辑：先抓住球（LockBallSlip 返回 True），再执行绕球转向。
    """
    # 先尝试抓住球
    if not lockball.LockBallSlip():
        # 还没抓住球，继续抢球
        return False

    # 已持球，执行绕球转向逻辑
    try:
        target = float(Aim_Angle) % 360.0
    except Exception:
        chassis.stop()
        return False

    yaw = float(chassis.GetYaw())
    error = angle_error_deg(yaw, target)
    if abs(error) <= max(0.0, float(angle_tolerance)):
        peripheral.Dribble(dribble_power)
        chassis.stop()
        return True

    ball = UnitedPosition.RobotBallPos()
    if ball == BALL_NOT_FOUND:
        peripheral.Dribble(dribble_power)
        chassis.stop()
        return False

    if orbit_radius_cm is None:
        catch_val = getattr(Settings.Motion, "CatchVal", [7.0, 0.0])
        orbit_radius_cm = float(catch_val[0] if len(catch_val) > 0 else 8.5)
    orbit_radius_m = max(0.02, float(orbit_radius_cm) / 100.0)

    if ball == BALL_HELD:
        ball_x_m = orbit_radius_m
        ball_y_m = 0.0
        center_x_m = 0.0
        center_y_m = 0.0
    else:
        try:
            ball_x_cm = float(ball[0])
            ball_y_cm = float(ball[1])
        except Exception:
            chassis.stop()
            return False
        ball_x_m = ball_x_cm / 100.0
        ball_y_m = ball_y_cm / 100.0
        center_x_m = (ball_x_cm - float(orbit_radius_cm)) * float(center_gain)
        center_y_m = ball_y_cm * float(center_gain)

    speed_k = 1.0 if SpeedK is None else max(0.0, float(SpeedK))
    if max_wz_deg_s is None:
        max_wz_deg_s = 110.0 * max(0.35, speed_k)
    max_wz_deg_s = max(0.0, float(max_wz_deg_s))
    wz_deg_s = clamp(error * float(turn_kp) * max(0.35, speed_k), -max_wz_deg_s, max_wz_deg_s)
    omega = math.radians(wz_deg_s)

    speed_x = omega * ball_y_m + center_x_m
    speed_y = -omega * ball_x_m + center_y_m
    linear_mag = math.hypot(speed_x, speed_y)
    max_linear = max(0.0, float(max_linear_mps))
    if linear_mag > max_linear > 0.0:
        scale = max_linear / linear_mag
        speed_x *= scale
        speed_y *= scale

    peripheral.Dribble(dribble_power)
    chassis.RobotMoveVector(speed_x, speed_y, wz_deg_s)
    return False

def TurnTo_With_ball_Strong(angle):
    if UnitedPosition.RobotBallPos() == BALL_HELD:
        error = abs(UnitedPosition.FieldRobotPose()[2]- angle)
        if abs(UnitedPosition.FieldRobotPose()[2]- angle) == cfg.ExpectedVals.ErrorRange:
            return True
        else:
            if error < 0 and error < -100:
                error = -100
            elif error > 100:
                error = 100
            chassis.RobotTurn(error)
            return False
    else:
        return lockball.slip()

def GoBack():
    """调用 Pos2Pos 回到配置里的 Home 位姿。"""
    # 当前 config.json 把 Home 存在 CheckPoints 下。
    try:
        home = [cfg.CheckPoints.Home[0], cfg.CheckPoints.Home[1], cfg.CheckPoints.Home[2]]
    except Exception:
        home = [0, -20, 0]
    return UnitedPosition.Pos2Pos(home)

def Slipsideshot(attack_dir=1): #溜边
    """One tick of sideline carry and shot; loop-call like OhMyBackStep."""
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

def OhMyBackStep(
    HomePos=[0, -20, 0],
    edge_y=65,
    corner_x=85,
    arrive_cm=5,
    speed_gain=5,
    attack_dir=1,
    dribble_power=0.8,
):
    """老式非阻塞边路角落射门组件。

    每次调用只执行一个 tick：
    - 没持球：交给 NormalShoot/LockBallSlip 抢球
    - 持球但没到边路角落：沿边带球
    - 到角落：转向球门，角度够就射门
    """

    if UnitedPosition.RobotBallPos() != BALL_HELD:
        return NormalShoot(x=0, HomePos=HomePos)
    elif UnitedPosition.RobotBallPos() == BALL_NOT_FOUND:
        return UnitedPosition.Pos2Pos(HomePos)
    else:
        lockball.LockBallSlip()

    peripheral.Dribble(dribble_power)

    try:
        local_x, local_y, _ = UnitedPosition.FieldRobotPose()
    except Exception:
        lockball.LockBallSlip()
        return False

    local_x = float(local_x)
    local_y = float(local_y)
    attack_dir = 1 if attack_dir >= 0 else -1
    shoot_y = edge_y if local_y >= 0 else -edge_y
    target_x = attack_dir * abs(corner_x)
    target_y = shoot_y

    carry_angle = UnitedPosition.Pos2Angle([local_x, local_y], [target_x, target_y]) % 360
    at_corner = abs(target_x - local_x) < arrive_cm and abs(target_y - local_y) < arrive_cm
    if not at_corner:
        UnitedPosition.Pos2Pos([target_x, target_y, carry_angle], speed_gain)
        return False

    goal_x = attack_dir * (float(getattr(cfg.Bounds, "Long", 260)) / 2.0)
    aim = UnitedPosition.Pos2Angle([local_x, local_y], [goal_x, 0.0]) % 360
    yaw = compass()
    err = angle_error_deg(yaw, aim)

    if abs(err) < 8:
        peripheral.ShootBall()
        peripheral.Dribble(False)
        logger.success("OhMyBack shot.")
        return True

    chassis.FieldMoveVector(0, 0, aim, KpW=clamp(abs(err) / 45.0, 0.35, 1.0))
    return False
