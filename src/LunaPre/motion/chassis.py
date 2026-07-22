import math
from ReasonData.config import Settings
from ReasonData import logger
import time, threading

from .motion_backend import MotionBackend


class Car:
    def __init__(self, SetMotorFunc=None, GetYaw=None, permission=None, motor_encoder=None,
                 MotorEncoder=None, SetChassisTwistFunc=None,
                 GetChassisTelemetryFunc=None, ShutdownFunc=None,
                 StopFunc=None, DisableFunc=None, EmergencyStopFunc=None,
                 IsIMUHealthyFunc=None, GetIMUStateFunc=None):
        self.SetMotorFunc = SetMotorFunc
        self.SetChassisTwistFunc = SetChassisTwistFunc
        self.GetChassisTelemetryFunc = GetChassisTelemetryFunc
        self.ShutdownFunc = ShutdownFunc
        self.StopFunc = StopFunc
        self.DisableFunc = DisableFunc
        self.EmergencyStopFunc = EmergencyStopFunc
        self.IsIMUHealthyFunc = IsIMUHealthyFunc
        self.GetIMUStateFunc = GetIMUStateFunc
        self.GetYaw = GetYaw
        if permission is None:
            self.Permission = lambda: True
        else:
            self.Permission = permission
        self.MotorEncoder = motor_encoder if motor_encoder is not None else MotorEncoder
        self.Kp = 1.0
        self.KpW = 2.0
        self.cfg = Settings
        self.speedCache = [0,0,0,0]
        self._motion_backend = MotionBackend(self.cfg)
        import simple_pid
        max_wz = self._max_physical_wz_degps()
        self.Yaw_PID = simple_pid.PID(2.3, 0.08, 0.05, setpoint=0,output_limits=(-max_wz, max_wz),auto_mode=True)
        # Always use the unified project logger (ReasonData.logger)
        # to avoid AttributeError when FullLog=False.
        self.logger = logger

        self._last_vx = 0
        self._last_vy = 0
        self._last_wz = 0
        self._motor_output_hooks = []

    def add_motor_output_hook(self, hook):
        if hook not in self._motor_output_hooks:
            self._motor_output_hooks.append(hook)

    def remove_motor_output_hook(self, hook):
        if hook in self._motor_output_hooks:
            self._motor_output_hooks.remove(hook)

    def _notify_motor_output(self, speed):
        for hook in list(self._motor_output_hooks):
            try:
                hook(speed, self)
            except Exception as e:
                if self.cfg.Debug.FullLog:
                    self.logger.debug(f"[Car] motor output hook failed: {e}")

    def _motion_bool(self, name, default):
        return self._motion_backend._motion_bool(name, default)

    def _motion_float(self, name, default):
        return self._motion_backend._motion_float(name, default)

    def _motion_str(self, name, default):
        return self._motion_backend._motion_str(name, default)

    def _chassis_float(self, name, default):
        return self._motion_backend._chassis_float(name, default)

    @staticmethod
    def _clamp(value, lower, upper):
        return MotionBackend.clamp(value, lower, upper)

    @staticmethod
    def _angle_error_deg(current, target):
        return MotionBackend.angle_error_deg(current, target)

    @staticmethod
    def _max_abs(values):
        return MotionBackend.max_abs(values)

    def _clamp_motor_vector(self, values):
        return self._motion_backend.clamp_motor_vector(values)

    @staticmethod
    def _slew_value(current, target, max_delta):
        return MotionBackend.slew_value(current, target, max_delta)

    def _apply_wheel_slew_limit(self, values, immediate=False):
        return self._motion_backend.apply_wheel_slew_limit(values, immediate=immediate)

    def _filter_chassis_command(self, speed_x, speed_y, speed_z):
        return self._motion_backend.filter_chassis_command(speed_x, speed_y, speed_z)

    def _turn_translation_scale(self, speed_z):
        return self._motion_backend.turn_translation_scale(speed_z)

    def _max_scale_to_fit(self, base, scaled, max_speed):
        """Return the largest 0..1 scale for scaled so base + scaled*alpha fits."""
        return self._motion_backend.max_scale_to_fit(base, scaled, max_speed)

    def _allocate_wheel_mix(self, planar, turn, speed_z):
        return self._motion_backend.allocate_wheel_mix(planar, turn, speed_z)

    def _wheel_radius_m(self):
        return max(1e-6, self._chassis_float("WheelRadiusM", 0.029))

    def _robot_radius_m(self):
        return max(1e-6, self._chassis_float("RobotRadiusM", 0.09))

    def _rpm_per_mps(self):
        return 60.0 / (2.0 * math.pi * self._wheel_radius_m())

    def _rpm_per_radps(self):
        return self._robot_radius_m() * self._rpm_per_mps()

    def _rpm_per_degps(self):
        return self._rpm_per_radps() * math.pi / 180.0

    def _max_physical_wz_degps(self):
        max_rpm = abs(self._chassis_float("MaxWheelRPM", 500))
        return max_rpm / self._rpm_per_degps()

    def SetMotor(self,speedCache:list[int,int,int,int], immediate=False):
        if self.SetMotorFunc is None:
            raise RuntimeError("wheel-speed output is unavailable for this chassis backend")
        if self.Permission():
            self.speedCache = speedCache
            number = [self.cfg.Ports.LeftFront,self.cfg.Ports.LeftBack,self.cfg.Ports.RightFront,self.cfg.Ports.RightBack]
            ordered = [0,0,0,0]
            for n in range(4):
                ordered[n] = float(speedCache[number[n]-1])
            speed = self._clamp_motor_vector(ordered)
            speed = self._apply_wheel_slew_limit(speed, immediate=immediate)
            speed = [int(round(v)) for v in speed]
            if self.cfg.Debug.FullLog:
                if self.MotorEncoder:
                    EncoderCache = self.MotorEncoder()
                    Encoder =[0,0,0,0]
                    for n in number:
                        Encoder[n-1] = int(EncoderCache[n-1])
                    self.logger.debug("SetMotorVals: (%s, %s, %s, %s) | With feedbackVals %s" %
                                    (speed[0], speed[1], speed[2], speed[3],Encoder)
                                    )
                else:
                    self.logger.debug("SetMotorVals: %s, %s, %s, %s" % (speed[0], speed[1], -speed[2], -speed[3]))

            self._notify_motor_output(speed)

            self.SetMotorFunc(speed[0], speed[1], speed[2], speed[3])
        else:
            self.SetMotorFunc(0,0,0,0)
            self._motion_backend.reset_motor_output()
            self.logger.warning("Permission Denied: Unable to set motor speeds.")

    def SetKp(self,Kp):
        self.Kp = Kp

    def Compass(self):
        return self.GetYaw()

    def _field_imu_ready(self):
        if self.GetYaw is None:
            return False
        if self.IsIMUHealthyFunc is None:
            return True
        try:
            return bool(self.IsIMUHealthyFunc())
        except Exception as error:
            self.logger.warning(f"IMU health check failed: {error}")
            return False

    def IsFieldIMUReady(self):
        """Public health gate for non-chassis field-coordinate controllers."""
        return self._field_imu_ready()

    def _reject_field_motion(self):
        self.logger.warning("Field-frame motion rejected: IMU is stale, unhealthy, or axis-unverified.")
        self.stop()
        return False

    def RobotMoveVector(self, SpeedX, SpeedY, SpeedZ):
        """Move in the robot frame.

        Positive X is forward, positive Y is left, and positive Z is
        counterclockwise. Linear values use m/s and angular values use deg/s.
        """
        if self.SetChassisTwistFunc is not None:
            return self.RobotMoveTwist(
                float(SpeedX),
                float(SpeedY),
                math.radians(float(SpeedZ)),
            )
        # 缓存速度用于记录
        SpeedX, SpeedY, SpeedZ = self._filter_chassis_command(SpeedX, SpeedY, SpeedZ)
        self._last_vx = SpeedX
        self._last_vy = SpeedY
        self._last_wz = SpeedZ

        rpm_per_mps = self._rpm_per_mps()
        rpm_per_degps = self._rpm_per_degps()
        planar = [
            (SpeedX - SpeedY) * rpm_per_mps,
            (SpeedX + SpeedY) * rpm_per_mps,
            (SpeedX + SpeedY) * rpm_per_mps,
            (SpeedX - SpeedY) * rpm_per_mps,
        ]
        turn = [
            -SpeedZ * rpm_per_degps,
            -SpeedZ * rpm_per_degps,
            SpeedZ * rpm_per_degps,
            SpeedZ * rpm_per_degps,
        ]
        output = self._allocate_wheel_mix(planar, turn, SpeedZ)
        self.SetMotor(output)

    def RobotMoveTwist(self, vx_mps, vy_mps, wz_rad_s):
        """Move in the robot frame using m/s and rad/s."""
        values = (float(vx_mps), float(vy_mps), float(wz_rad_s))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("chassis twist must be finite")
        self._last_vx, self._last_vy, self._last_wz = values
        if not self.Permission():
            values = (0.0, 0.0, 0.0)
            self.logger.warning("Permission Denied: chassis twist forced to zero.")
        if self.SetChassisTwistFunc is not None:
            return self.SetChassisTwistFunc(*values)
        # Wheel-based backends consume the public degree-per-second API.
        return self.RobotMoveVector(values[0], values[1], math.degrees(values[2]))

    def RobotMoveAngle(self,Angle,Speed,Kp:float|None = 1):
        '''
        angle movement (Angle,Speed) without YawCorrect
        '''
        rad = math.radians(Angle)
        SpeedX = math.sin(rad) * Speed
        SpeedY = math.cos(rad) * Speed
        self.RobotMoveVector(SpeedY,SpeedX,0)

    def FieldMoveAngle(self,FacingAngle,MovingAngle,Speed,Kp:float|None = None):
        '''
        angle movement (FacingAngle,MovingAngle,speed) with YawCorrect
        '''
        if not self._field_imu_ready():
            return self._reject_field_motion()
        Yaw = self.GetYaw()
        rad = math.radians((MovingAngle-Yaw+FacingAngle))
        SpeedX = math.cos(rad) * Speed
        SpeedY = math.sin(rad) * Speed
        self.FieldMoveVector(SpeedX,SpeedY,FacingAngle,Kp)

    def FieldMoveVector(self,SpeedX,SpeedY,FacingAngle,Kp:float|None = None,KpW:float|None = None):
        """Field-frame command with yaw correction.

        SpeedX and SpeedY use m/s. FacingAngle is in degrees. Field motion is
        rejected while IMU axis mapping is unverified.
        """
        if not self._field_imu_ready():
            return self._reject_field_motion()
        try:
            Yaw = self.GetYaw()
        except (RuntimeError, ValueError):
            return self._reject_field_motion()
        # TODO change facing
        # FacingAngle = 360 - FacingAngle
        Error = self._angle_error_deg(Yaw, FacingAngle)
        Normaled_Yaw =  ( (Yaw + 180) % 360 - 180 )
        Normaled_Facing =  ( (FacingAngle + 180) % 360 - 180 )
        if not Kp:
            Kp = self.Kp
        if not KpW:
            KpW = self.KpW
        if Settings.Debug.Beta:
            # Feed shortest angular error to PID; raw angle PID breaks near +/-180.
            self.Yaw_PID.setpoint = 0
            SpeedZ = self.Yaw_PID(-Error)
            if getattr(Settings.Debug, "FullLog", False):
                logger.debug(f"Yaw PID output: {SpeedZ}, Yaw: {Normaled_Yaw}, FacingAngle: {Normaled_Facing}, Error: {Error}")
        else:
            SpeedZ = Error * KpW
        if abs(Error) < Settings.Motion.ErrorRange:
            self.Yaw_PID.reset()  # Reset PID when within acceptable error range
        WheelX = SpeedX * math.cos(math.radians(Yaw)) + SpeedY * math.sin(math.radians(Yaw))
        WheelY = - SpeedX * math.sin(math.radians(Yaw)) + SpeedY * math.cos(math.radians(Yaw))
        # Only print high-frequency motion logs when FullLog is enabled
        if getattr(self.cfg.Debug, "FullLog", False):
            self.logger.info(f"used FieldMoveVector: {WheelX, WheelY, SpeedZ}")
        self.RobotMoveVector(WheelX, WheelY, SpeedZ)
        return True

    def RobotXMove(self,Angle,Speed):
        self.RobotMoveAngle(Angle, Speed)

    def RobotYMove(self,Angle,Speed,Kp=None):
        self.RobotMoveAngle(Angle, Speed, Kp)

    def FieldTurn(self,Angle):
        if not self._field_imu_ready():
            return self._reject_field_motion()
        return self.FieldMoveVector(0,0,Angle)

    def RobotTurn(self,Speed): #自转
        self.RobotMoveVector(0,0,Speed)

    def TurnTo(self,Angle,Kp=None):
        while True:
            Error = self.GetYaw() - Angle
            if not abs(Error) < 5:
                self.FieldMoveVector(0,0,Angle)
            else:
                self.stop()
                break
            time.sleep(0.03)

    def stop(self):
        self._motion_backend.reset_command_filter()
        if self.StopFunc is not None:
            return self.StopFunc()
        if self.SetChassisTwistFunc is not None:
            return self.SetChassisTwistFunc(0.0, 0.0, 0.0)
        return self.SetMotor([0,0,0,0], immediate=True)

    def disable(self):
        self._motion_backend.reset_command_filter()
        if self.DisableFunc is not None:
            return self.DisableFunc()
        return self.stop()

    def emergency_stop(self):
        self._motion_backend.reset_command_filter()
        if self.EmergencyStopFunc is not None:
            return self.EmergencyStopFunc()
        return self.disable()

    def shutdown(self):
        self._motion_backend.reset_command_filter()
        if self.ShutdownFunc is not None:
            return self.ShutdownFunc()
        return self.disable()

    def GetChassisTelemetry(self):
        if self.GetChassisTelemetryFunc is None:
            return None
        return self.GetChassisTelemetryFunc()



class Peripherals:
    def __init__(self,IOFunc,permission=None):
        if permission is None:
            self.Permission = lambda: True
        else:
            self.Permission = permission
        self.SetIO = IOFunc
        from ReasonData import Settings
        self.cfg = Settings
        from ReasonData import logger
        self.logger = logger
        self.SetIO(self.cfg.Ports.ElecMagnet,1)
        self.DribbleStatus = False

    def ShootBall(self):
        if self.Permission():
            self.SetIO(self.cfg.Ports.ElecMagnet,0)
            self.logger.info("ShootBall Activated")
            # TODO 用更优雅的实现技能冷却
            time.sleep(0.3)
        else:
            self.logger.warning("Permission Denied: Unable to set dribble status.")

    def Dribble(self,Status):
        if self.Permission():
            if not self.DribbleStatus == Status:
                if Status == True:
                    self.SetIO(self.cfg.Ports.Dribble,0.5)
                elif Status == False:
                    self.SetIO(self.cfg.Ports.Dribble,0.0)
                else:
                    self.SetIO(self.cfg.Ports.Dribble,Status)
                self.DribbleStatus = Status
                self.logger.info("Dribble set to %s" % str(Status))
            # self.logger.info("Dribble set to %s" % ("ON" if Status else "OFF"))
        else:
            self.SetIO(self.cfg.Ports.Dribble,0.0)
            self.logger.warning("Permission Denied: Unable to set dribble status.")

class Key:
    def __init__(self,GetKeyFunc):
        self.KeyCount = 0
        self.LastPressed = False

        self.GetKey = GetKeyFunc

        self.KeyEventThread = threading.Thread(target=self.KeyEvent)
        self.KeyEventThread.daemon = True
        self.KeyEventThread.start()

    def KeyEvent(self):
        while True:
            if self.LastPressed == True:
                if self.GetKey():
                    self.KeyCount = self.KeyCount + 1
                else:
                    self.LastPressed = False
            else:
                if self.GetKey():
                    self.LastPressed = True
                    self.KeyCount = self.KeyCount + 1
            time.sleep(0.01)

    def LongPress(self):
        if self.KeyCount >= 500:
            return True
        else:
            return False

    def Press(self):
        return self.GetKey()
