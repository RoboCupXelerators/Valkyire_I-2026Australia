"""BasicFuc

历史策略常用函数集合。

注意：本文件存在大量历史代码。此次改动目标：
- 修复明显的语法/运行时错误（重复定义、错误调用）
- 按 README 命名规范补齐 PascalCase 新名称，同时保留旧名称别名
- 避免把 `from LunaPre import *` 继续扩散到外部：本文件内部仍可使用，但对外建议显式导入
"""

from __future__ import annotations

from LunaPre import *
import math
import os
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

def FindNearestAngle(arr, target):
    """在 360 度圆环上返回最接近 ``target`` 的候选角。"""
    return min(arr, key=lambda x: abs(_AngleErrorDeg(x, target)))

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
    """历史攻防辅助函数使用的抢球/锁球控制器。

    这里的公开方法有意保留状态：PID 积分、最近一次看到的球位置、
    吸球状态都会保存在 ``lockball`` 单例上。
    """

    def __init__(self):
        """从 ``Settings.Motion`` 读取锁球参数并初始化 PID 状态。"""
        self.D_time = 0
        import simple_pid
        catch_val = getattr(Settings.Motion, "CatchVal", [8.5, 0])
        catch_x = catch_val[0] if len(catch_val) > 0 else 8.5
        catch_y = catch_val[1] if len(catch_val) > 1 else 0
        self.catch_x = catch_x
        self.max_lock_angle = getattr(Settings.Motion, "LockBallSlipMaxAngle", 85)
        self.acquire_kp_x = getattr(Settings.Motion, "LockBallSlipAcquireKpX", 3.0)
        self.acquire_kp_y = getattr(Settings.Motion, "LockBallSlipAcquireKpY", 2.5)
        self.acquire_kp_w = getattr(Settings.Motion, "LockBallSlipAcquireKpW", 1.5)
        self.acquire_max_x = getattr(Settings.Motion, "LockBallSlipAcquireMaxX", 90)
        self.acquire_max_y = getattr(Settings.Motion, "LockBallSlipAcquireMaxY", 80)
        self.acquire_max_w = getattr(Settings.Motion, "LockBallSlipAcquireMaxW", 150)
        self.pid_kp_x = getattr(Settings.Motion, "LockBallSlipPidKpX", 3.0)
        self.pid_kp_y = getattr(Settings.Motion, "LockBallSlipPidKpY", 2.4)
        self.pid_kp_w = getattr(Settings.Motion, "LockBallSlipPidKpW", 1.2)
        self.pid_max_x = getattr(Settings.Motion, "LockBallSlipPidMaxX", 120)
        self.pid_max_y = getattr(Settings.Motion, "LockBallSlipPidMaxY", 80)
        self.pid_max_w = getattr(Settings.Motion, "LockBallSlipPidMaxW", 80)
        self.rear_acquire_kp_y = getattr(Settings.Motion, "LockBallSlipRearAcquireKpY", 0.0)
        self.rear_acquire_max_y = getattr(Settings.Motion, "LockBallSlipRearAcquireMaxY", 0)
        self.lost_hold_seconds = getattr(Settings.Motion, "LockBallSlipLostHoldSeconds", 0.12)
        self.dribble_power = getattr(Settings.Motion, "LockBallSlipDribblePower", 0.35)
        self.capture_slow_bbox = getattr(Settings.Motion, "LockBallSlipCaptureSlowBBox", 60)
        self.capture_stop_bbox = getattr(Settings.Motion, "LockBallSlipCaptureStopBBox", 100)
        self.capture_min_x_scale = getattr(Settings.Motion, "LockBallSlipCaptureMinXScale", 0.55)
        self.capture_min_w_scale = getattr(Settings.Motion, "LockBallSlipCaptureMinWScale", 0.6)
        self.capture_push_bbox = getattr(Settings.Motion, "LockBallSlipCapturePushBBox", 55)
        self.capture_push_y = getattr(Settings.Motion, "LockBallSlipCapturePushY", 2.5)
        self.capture_min_speed_x = getattr(Settings.Motion, "LockBallSlipCaptureMinSpeedX", 24)
        self.precise_y_tol = getattr(Settings.Motion, "LockBallSlipPreciseY", 2.0)
        self.align_slow_y = getattr(Settings.Motion, "LockBallSlipAlignSlowY", 2.5)
        self.align_stop_y = getattr(Settings.Motion, "LockBallSlipAlignStopY", 12)
        self.align_min_x_scale = getattr(Settings.Motion, "LockBallSlipAlignMinXScale", 0.45)
        rear_turn_sign = getattr(Settings.Motion, "LockBallSlipRearAcquireTurnSign", 1)
        self.rear_acquire_turn_sign = 1 if rear_turn_sign >= 0 else -1
        self._lockball_acquire_turn_sign = self.rear_acquire_turn_sign
        self._lockball_rear_acquire_active = False
        self._lockball_last_seen = None
        self._lockball_last_seen_ts = 0.0
        self._lockball_dribble_state = False
        self._lockball_last_capture_control = (1.0, 1.0, 0.0)
        self.pan_x = simple_pid.PID(
            self.pid_kp_x,0,0,
            setpoint=catch_x,
            output_limits=(-self.pid_max_x,0),
            auto_mode=True)
        self.pan_y = simple_pid.PID(
            self.pid_kp_y,0,0,
            setpoint=catch_y,
            output_limits=(-self.pid_max_y,self.pid_max_y),
            auto_mode=True)
        self.pan_w = simple_pid.PID(
            self.pid_kp_w,0,0,
            setpoint=0,
            output_limits=(-self.pid_max_w,self.pid_max_w),
            auto_mode=True)

    def Lockballangle(self):#贝尔巴托夫转身
        """按当前机器人参考系下的球角度转向。"""
        # iBX,iBY = UnitedPosition.AbsBallPos()#获取球的位置
        #  = lBallPos[0],lBallPos[1]#将球的位置赋值给iBX和iBY
        
        # Compass = chassis.GetYaw()#获取机器人的航向
        # Angle = (math.degrees(math.atan2(iBX, iBY))) % 360
        Angle = UnitedPosition.RobotBallAngle()
        logger.debug("Ball Angle: %f" % Angle)
        # 这里只做机器人参考系转向，不叠加场地航向修正。
        chassis.RobotTurn((Angle - 180) / 2)

    def Lockballmove(self):
        """按机器人参考系追向当前可见球。"""
        # 转成机器人参考系追球，避免混用场地参考系和机器人参考系。
        rBX, rBY = UnitedPosition.RobotBallPos()
        if [rBX, rBY] in ([0xfff, 0xfff], [0xddd, 0xddd]):
            return
        chassis.RobotMoveAngle((math.degrees(math.atan2(rBY, rBX)) + 360) % 360, min(300, int(math.hypot(rBX, rBY) * 4)))

    def _write_lockball_pid_log(self, state, iBX, iBY, angle=None, pid_x=None, pid_y=None, pid_w=None, speed_x=0, speed_y=0, speed_w=0):
        """在调试记录开启时追加一条锁球控制器采样。"""
        try:
            log_path = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', 'log', 'lockball_pid.csv'))
            os.makedirs(os.path.dirname(log_path), exist_ok=True)
            need_header = not os.path.exists(log_path)
            with open(log_path, 'a', encoding='utf-8') as f:
                if need_header:
                    f.write('ts,state,iBX,iBY,angle,pid_x,pid_y,pid_w,SpeedX,SpeedY,SpeedW,Beta\n')
                beta = int(bool(getattr(Settings.Debug, 'Beta', False)))
                f.write(f"{time.time()},{state},{iBX},{iBY},{angle},{pid_x},{pid_y},{pid_w},{speed_x},{speed_y},{speed_w},{beta}\n")
        except Exception:
            pass

    def _reset_lockball_pid(self):
        """重置锁球 PID 积分状态和帧计数。"""
        self.D_time = 0
        self.pan_x.reset()
        self.pan_y.reset()
        self.pan_w.reset()

    def _robot_ball_angle(self, iBX, iBY):
        """返回机器人参考系下的球角度，单位为度，0 表示正前方。"""
        if iBX == 0 and iBY == 0:
            return 0
        return int(math.degrees(math.atan2(iBY, iBX)))

    def _clamp(self, value, lower, upper):
        """把 ``value`` 限制在 ``[lower, upper]`` 范围内。"""
        return max(lower, min(upper, value))

    def _set_lockball_dribble(self, enabled):
        """仅在目标状态变化时更新吸球输出。"""
        target = self.dribble_power if enabled and self.dribble_power else False
        try:
            peripheral.Dribble(target)
            self._lockball_dribble_state = target
        except Exception:
            pass

    def _capture_control(self):
        """根据视觉框大小缩放前进和转向速度，避免近距离过冲。"""
        try:
            vision_data = getattr(UnitedPosition, "vision_data", {})
            debug = vision_data.get("ball_debug") if isinstance(vision_data, dict) else None
            bbox = debug.get("bbox") if isinstance(debug, dict) else None
            if not bbox or len(bbox) < 2:
                return self._lockball_last_capture_control
            bbox_size = max(float(bbox[0]), float(bbox[1]))
        except Exception:
            return self._lockball_last_capture_control

        slow_bbox = max(0.0, float(self.capture_slow_bbox))
        stop_bbox = max(slow_bbox + 1.0, float(self.capture_stop_bbox))
        min_x_scale = self._clamp(float(self.capture_min_x_scale), 0.0, 1.0)
        min_w_scale = self._clamp(float(self.capture_min_w_scale), 0.0, 1.0)
        if bbox_size <= slow_bbox:
            self._lockball_last_capture_control = (1.0, 1.0, bbox_size)
            return self._lockball_last_capture_control
        close_ratio = self._clamp((bbox_size - slow_bbox) / (stop_bbox - slow_bbox), 0.0, 1.0)
        x_scale = self._clamp(1.0 - close_ratio * (1.0 - min_x_scale), min_x_scale, 1.0)
        w_scale = self._clamp(1.0 - close_ratio * (1.0 - min_w_scale), min_w_scale, 1.0)
        self._lockball_last_capture_control = (x_scale, w_scale, bbox_size)
        return self._lockball_last_capture_control

    def _alignment_speed_x_scale(self, iBY):
        """横向球偏差较大时降低前进速度。"""
        slow_y = max(0.0, float(self.align_slow_y))
        stop_y = max(slow_y + 1.0, float(self.align_stop_y))
        min_scale = self._clamp(float(self.align_min_x_scale), 0.0, 1.0)
        abs_y = abs(float(iBY))
        if abs_y <= slow_y:
            return 1.0
        ratio = self._clamp((abs_y - slow_y) / (stop_y - slow_y), 0.0, 1.0)
        return self._clamp(1.0 - ratio * (1.0 - min_scale), min_scale, 1.0)

    def _acquire_ball_fast(self, iBX, iBY, angle, state_prefix=''):
        """处理后方球或大角度球的快速抢球分支。"""
        self._reset_lockball_pid()
        if iBX <= 0:
            if not self._lockball_rear_acquire_active:
                self._lockball_acquire_turn_sign = self.rear_acquire_turn_sign
                self._lockball_rear_acquire_active = True
            speed_x = 0
            speed_y = self._clamp(iBY * self.rear_acquire_kp_y, -self.rear_acquire_max_y, self.rear_acquire_max_y)
            state = 'acquire_rear'
        else:
            if not self._lockball_rear_acquire_active and angle != 0:
                self._lockball_acquire_turn_sign = 1 if angle > 0 else -1
            speed_x = self._clamp((iBX - self.catch_x) * self.acquire_kp_x, 0, self.acquire_max_x)
            speed_y = self._clamp(iBY * self.acquire_kp_y, -self.acquire_max_y, self.acquire_max_y)
            state = 'acquire_side'

        if self._lockball_rear_acquire_active:
            turn_angle = max(abs(angle), self.max_lock_angle) * self._lockball_acquire_turn_sign
        else:
            turn_angle = angle
        speed_w = self._clamp(turn_angle * self.acquire_kp_w, -self.acquire_max_w, self.acquire_max_w)
        chassis.RobotMoveVector(speed_x, speed_y, speed_w)
        self._write_lockball_pid_log(f'{state_prefix}{state}', iBX, iBY, angle, None, None, None, speed_x, speed_y, speed_w)

    def LockBallSlip(self):
        """执行标准锁球控制器的一个非阻塞 tick。

        返回:
            已经持球时返回 ``True``，否则返回 ``False``。
        """
        iBX,iBY = UnitedPosition.RobotBallPos()
        now = time.time()
        state_prefix = ''
        if [iBX,iBY] == [0xfff,0xfff]:
            if (
                self._lockball_last_seen is not None
                and now - self._lockball_last_seen_ts <= self.lost_hold_seconds
            ):
                iBX, iBY = self._lockball_last_seen
                state_prefix = 'hold_'
            else:
                self._set_lockball_dribble(False)
                chassis.stop()
                self._write_lockball_pid_log('no_ball', iBX, iBY)
                self._reset_lockball_pid()
                return False
        elif [iBX,iBY] == [0xddd,0xddd]:
            self._set_lockball_dribble(True)
            chassis.stop()
            self._write_lockball_pid_log('caught', iBX, iBY)
            self._lockball_rear_acquire_active = False
            self._lockball_last_seen = None
            self._lockball_last_seen_ts = 0.0
            self._reset_lockball_pid()
            return True
        else:
            self._lockball_last_seen = [iBX, iBY]
            self._lockball_last_seen_ts = now
        self._set_lockball_dribble(True)
        angle = self._robot_ball_angle(iBX, iBY)
        if iBX <= 0 or abs(angle) > self.max_lock_angle:
            self._acquire_ball_fast(iBX, iBY, angle, state_prefix)
            return False
        else:
            self.D_time += 1
            self._lockball_rear_acquire_active = False
            # if self.D_time > 5000:
            #     self.D_time = 0
            #     self.pan_x.reset()
            #     self.pan_y.reset()
            #     self.pan_w.reset()
            if Settings.Debug.Beta:
                pid_x = max(0, -self.pan_x(iBX))
                pid_y = -self.pan_y(iBY)
                pid_w = -self.pan_w(angle)
                capture_x_scale, capture_w_scale, capture_bbox = self._capture_control()
                SpeedX = pid_x * capture_x_scale * self._alignment_speed_x_scale(iBY)
                sim_bits = str(getattr(getattr(Settings, "RoboInfo", object()), "Bit", "")).upper() == "SIM"
                if (
                    (capture_bbox >= self.capture_push_bbox or sim_bits)
                    and iBX > self.catch_x
                    and abs(iBY) <= self.capture_push_y
                ):
                    SpeedX = max(SpeedX, self.capture_min_speed_x)
                SpeedY = pid_y
                SpeedW = pid_w * capture_w_scale
                if (
                    capture_bbox >= self.capture_push_bbox
                    and iBX <= self.catch_x + getattr(Settings.Motion, "CatchBBoxXMargin", 4)
                    and abs(iBY) > self.precise_y_tol
                ):
                    SpeedX = 0
                    SpeedW = 0
                chassis.RobotMoveVector(SpeedX,SpeedY,SpeedW)
                self._write_lockball_pid_log(f'{state_prefix}track_pid', iBX, iBY, angle, pid_x, pid_y, pid_w, SpeedX, SpeedY, SpeedW)
                return False
            else:
                abs_iBY = abs(iBY)
                if iBX > 75 or abs_iBY > 75:
                    Kp = 7
                    KpZ = 4
                elif 25 < iBX <= 75 or 25 < abs_iBY <= 75:
                    Kp = 6
                    KpZ = 1.5
                else:
                    Kp = 2
                    KpZ = 1
                # Kp = 3
                # KpZ = Fangle*self.D_time/500
                pid_x = None
                pid_y = None
                pid_w = None
                SpeedX = iBX * Kp
                SpeedY = iBY * Kp
                turn_speed = angle * KpZ
                if SpeedX > 300:
                    SpeedX = 300
                if SpeedY > 300:
                    SpeedY = 300
                if SpeedX < -300:
                    SpeedX = -300
                if SpeedY < -300:
                    SpeedY = -300
                # 整个滑步锁球逻辑保持在机器人参考系内。
                chassis.RobotMoveVector(SpeedX*1.5,SpeedY,turn_speed)
                self._write_lockball_pid_log(f'{state_prefix}track_legacy', iBX, iBY, angle, pid_x, pid_y, pid_w, SpeedX, SpeedY, turn_speed)
                return False

    def LockBallDash(self):
        """执行旧版冲球控制器的一个非阻塞 tick。"""
        iBX,iBY = UnitedPosition.RobotBallPos()
        if [iBX,iBY] == [0xfff,0xfff]:
            chassis.stop()
            return False
        elif [iBX,iBY] == [0xddd,0xddd]:
            chassis.RobotMoveVector(50,0,0)
            return True
        else:
            if iBY == 0:
                Fangle = 0
            else:
                Fangle = int(math.degrees(math.atan2(iBY,iBX)))
            abs_iBY = abs(iBY)
            if iBX > 75 or abs_iBY > 75:
                Kp = 7
                KpZ = 4
            elif 25 < iBX <= 75 or 25 < abs_iBY <= 75:
                Kp = 6
                KpZ = 1.5
            else:
                Kp = 2
                KpZ = 1
            if abs(Fangle) < Settings.Motion.ErrorRange:
                KpZ = 0
                Kp = 5
            # Kp = 3
            # KpZ = Fangle*self.D_time/500
            SpeedX = iBX * Kp
            SpeedY = iBY * Kp
            Fangle = Fangle * KpZ
            if SpeedX > 300:
                SpeedX = 300
            if SpeedY > 300:
                SpeedY = 300
            if SpeedX < -300:
                SpeedX = -300
            if SpeedY < -300:
                SpeedY = -300
            # 整个滑步锁球逻辑保持在机器人参考系内。
            chassis.RobotMoveVector(SpeedX*1.5,SpeedY,Fangle)
            return False
        
    def BallScore(self):
        """返回旧版逻辑中用于粗略衡量追球紧迫度的分数。"""
        distance = UnitedPosition.RobotBallDistance()
        speed = sum(chassis.speedCache)/4
        try:
            time = distance/speed
        except ZeroDivisionError:
            time = distance
        import random
        seed = random.random()
        score = round((abs(time) * 0.5 + abs(speed) * 0.2 + seed * 0.01)/100000,2)
        return score


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
    if [bx, by] == [0xFFF, 0xFFF] or (bx == 0 and by == 0):
        logger.info("Ball not found, go home.")
        UnitedPosition.Pos2Pos(HomePos)
        peripheral.Dribble(False)
        return False

    # 已经持球。
    if [bx, by] == [0xDDD, 0xDDD]:
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

            x0, y0, _ = UnitedPosition.FieldRobotPos()
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
    """在保持锁球的同时转向指定场地航向。

    返回:
        航向进入容差且球仍被锁住时返回 ``True``。
    """
    yaw = chassis.GetYaw()
    Aim_Angle = Aim_Angle%360
    logger.debug("TurnTo_with_ball target_yaw={}", Aim_Angle)
    locked = lockball.LockBallSlip()
    if locked == True:
        Error = _AngleErrorDeg(yaw, Aim_Angle)
        if not abs(Error) <= 10:
            logger.debug("TurnTo_with_ball yaw_error={}", Error)
            turn_gain = _Clamp(abs(Error) / 45.0, 0.35, 1.4)
            chassis.FieldMoveVector(0, 0, Aim_Angle, KpW=turn_gain * (SpeedK or 1))
            return False
        else:
            return True
    return locked


def GoBack():
    """调用 Pos2Pos 回到配置里的 Home 位姿。"""
    # 当前 config.json 把 Home 存在 CheckPoints 下。
    try:
        home = [cfg.CheckPoints.Home[0], cfg.CheckPoints.Home[1], cfg.CheckPoints.Home[2]]
    except Exception:
        home = [0, -20, 0]
    return UnitedPosition.Pos2Pos(home)

# ========= 向后兼容别名 =========

# 旧名称 -> 新名称
roundThresholdJudger = RoundThresholdJudger
FindNearstAngle = FindNearestAngle
linear_map = LinearMap



def Slipsideshot(EnemyPos=None): #溜边 10,-90
    """执行旧版溜边射门逻辑的一个 tick。

    参数:
        EnemyPos: 敌方在场地坐标系下的位置；缺失或无效时回退到场地中心。

    返回:
        进入射门分支后返回 ``True``，否则返回 ``False``。
    """
    peripheral.Dribble(True)
    LocalX,LocalY,_ = UnitedPosition.FieldRobotPos()
    try:
        has_enemy = EnemyPos is not None and len(EnemyPos) >= 2
    except TypeError:
        has_enemy = False
    if has_enemy:
        EnemyX,EnemyY = EnemyPos[:2]
    else:
        EnemyX = 0
        EnemyY = 0
    GoalX = 90
    AimX  = 80
    AimY  = 40
    if LocalX > 0:
        k = -45
    else:
        AimY = -AimY
        k = 45
    Angle = (math.degrees(math.atan2(LocalX - EnemyX, LocalY - EnemyY)) - 270) % 360
    UnitedPosition.Pos2Pos([AimX,AimY,Angle],10)
    if abs(AimX-LocalX)+abs(AimY-LocalY) < 5:
        DeltaX = LocalX - GoalX
        DeltaY = LocalY
        Theta =((math.degrees(math.atan2(DeltaX, DeltaY)) - 270) % 360) + 180 +k
        Compass = Theta
        for _ in range(20):
            # if not AbsBallPos() == [1207, 1207]:
            #     break
            chassis.FieldTurn(Compass)
            time.sleep(0.03)
        peripheral.ShootBall()
        logger.success("Ball is in possession, start shotting.")
        return True
    return False

def _AngleErrorDeg(current, target):
    """返回从 ``current`` 到 ``target`` 的有符号最短角度误差。"""
    return ((float(target) - float(current) + 180.0) % 360.0) - 180.0


def _Clamp(value, lo, hi):
    """把 ``value`` 限制在 ``[lo, hi]`` 范围内。"""
    return max(lo, min(hi, value))


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

    if UnitedPosition.RobotBallPos() != [0xDDD, 0xDDD]:
        return NormalShoot(x=0, HomePos=HomePos)
    elif UnitedPosition.RobotBallPos() == [0xFFF, 0xFFF]:
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
    err = _AngleErrorDeg(yaw, aim)

    if abs(err) < 8:
        peripheral.ShootBall()
        peripheral.Dribble(False)
        logger.success("OhMyBack shot.")
        return True

    chassis.FieldMoveVector(0, 0, aim, KpW=_Clamp(abs(err) / 45.0, 0.35, 1.0))
    return False


def OHMYBACK(HomePos=[0, -20, 0]):
    """兼容历史大写函数名的 ``OhMyBackStep`` 别名。"""
    return OhMyBackStep(HomePos=HomePos)


OhMyBack = OhMyBackStep
        
