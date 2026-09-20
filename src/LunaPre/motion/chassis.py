import math
import os
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
        self._permission_denied_logged = False
        self.peripherals = None
        self._last_permission = None

    def attach_peripherals(self, peripherals) -> None:
        """Register the peripheral IO owner so permission loss can zero it."""
        self.peripherals = peripherals

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

    def _wheel_radius_m(self):
        return max(1e-6, self._motion_backend._chassis_float("WheelRadiusM", 0.029))

    def _robot_radius_m(self):
        return max(1e-6, self._motion_backend._chassis_float("RobotRadiusM", 0.09))

    def _rpm_per_mps(self):
        return 60.0 / (2.0 * math.pi * self._wheel_radius_m())

    def _rpm_per_radps(self):
        return self._robot_radius_m() * self._rpm_per_mps()

    def _rpm_per_degps(self):
        return self._rpm_per_radps() * math.pi / 180.0

    def _max_physical_wz_degps(self):
        max_rpm = abs(self._motion_backend._chassis_float("MaxWheelRPM", 500))
        return max_rpm / self._rpm_per_degps()

    def CanMove(self) -> bool:
        """Return whether the referee/backend currently permits motion output."""

        try:
            permitted = bool(self.Permission())
        except Exception as error:
            self.logger.warning(f"Motion permission check failed: {error}")
            permitted = False
        if permitted:
            self._permission_denied_logged = False
        if self._last_permission is True and not permitted:
            self._on_permission_lost()
        self._last_permission = permitted
        return permitted

    def _on_permission_lost(self) -> None:
        """Referee gate closed: zero peripheral IO so nothing keeps running.

        The lower board latches the last IO state; without this edge the
        dribbler would keep spinning after the referee module shuts down,
        because strategy code stops calling Dribble() once motion is denied.
        """
        peripherals = self.peripherals
        if peripherals is not None:
            force_off = getattr(peripherals, "force_off", None)
            if callable(force_off):
                try:
                    force_off()
                except Exception as error:
                    self.logger.warning(f"Peripheral force-off failed: {error}")

    def _reset_closed_loop_state(self) -> None:
        """Discard controller state after a command cannot reach the motors."""

        self.Yaw_PID.reset()
        self._motion_backend.reset_command_filter()
        self._motion_backend.reset_motor_output()
        self.speedCache = [0, 0, 0, 0]
        self._last_vx = 0.0
        self._last_vy = 0.0
        self._last_wz = 0.0

    def _deny_motor_output(self) -> bool:
        """Force a zero command without retaining any stale controller state."""

        self._reset_closed_loop_state()
        self.SetMotorFunc(0, 0, 0, 0)
        if not self._permission_denied_logged:
            self.logger.warning("Permission Denied: Unable to set motor speeds.")
            self._permission_denied_logged = True
        return False

    def SetMotor(self,speedCache:list[int,int,int,int], immediate=False):
        if self.SetMotorFunc is None:
            raise RuntimeError("wheel-speed output is unavailable for this chassis backend")
        if self.CanMove():
            self.speedCache = speedCache
            number = [self.cfg.Ports.LeftFront,self.cfg.Ports.LeftBack,self.cfg.Ports.RightFront,self.cfg.Ports.RightBack]
            ordered = [0,0,0,0]
            for n in range(4):
                ordered[n] = float(speedCache[number[n]-1])
            speed = self._motion_backend.clamp_motor_vector(ordered)
            speed = self._motion_backend.apply_wheel_slew_limit(speed, immediate=immediate)
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
            return True
        else:
            return self._deny_motor_output()

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

        This is the direct velocity-command API: a requested speed must reach
        the chassis on this call, rather than becoming an acceleration target.
        Callers that need a ramp own that trajectory above this layer.
        """
        SpeedX, SpeedY, SpeedZ = (float(SpeedX), float(SpeedY), float(SpeedZ))
        if not all(math.isfinite(value) for value in (SpeedX, SpeedY, SpeedZ)):
            raise ValueError("chassis vector must be finite")
        if SpeedX == 0.0 and SpeedY == 0.0 and SpeedZ == 0.0:
            # A zero vector is an explicit brake request, not a target for the
            # traction-control deceleration ramp.
            return self.stop()
        if self.SetChassisTwistFunc is not None:
            return self.RobotMoveTwist(
                float(SpeedX),
                float(SpeedY),
                math.radians(float(SpeedZ)),
            )

        # Keep the filter state from a previous control mode from leaking into
        # this direct command. The configured mechanical RPM cap below still
        # protects the drivetrain; only artificial command ramping is skipped.
        self._motion_backend.reset_command_filter()
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
        output = self._motion_backend.allocate_wheel_mix(planar, turn, SpeedZ)
        return self.SetMotor(output, immediate=True)

    def RobotMoveTwist(self, vx_mps, vy_mps, wz_rad_s):
        """Move in the robot frame using m/s and rad/s."""
        values = (float(vx_mps), float(vy_mps), float(wz_rad_s))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("chassis twist must be finite")
        if values == (0.0, 0.0, 0.0):
            return self.stop()
        self._last_vx, self._last_vy, self._last_wz = values
        if not self.CanMove():
            self._reset_closed_loop_state()
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
        if not self.CanMove():
            # Do not run the yaw PID while the command is being rejected;
            # otherwise its integral term would be applied on re-enable.
            self._reset_closed_loop_state()
            self.stop()
            return False
        try:
            Yaw = self.GetYaw()
        except (RuntimeError, ValueError):
            return self._reject_field_motion()
        # TODO change facing
        # FacingAngle = 360 - FacingAngle
        Error = MotionBackend.angle_error_deg(Yaw, FacingAngle)
        Normaled_Yaw =  ( (Yaw + 180) % 360 - 180 )
        Normaled_Facing =  ( (FacingAngle + 180) % 360 - 180 )
        if not Kp:
            Kp = self.Kp
        explicit_kpw = KpW is not None
        if KpW is None:
            KpW = self.KpW
        if Settings.Debug.Beta:
            if explicit_kpw:
                # A caller that translates while turning can request a bounded
                # proportional heading rate. The old Beta path ignored KpW and
                # always used the aggressive chassis PID.
                SpeedZ = Error * float(KpW)
            else:
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
        return True

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
        # Watchdog: the lower board latches the last IO command, so the
        # dribbler keeps spinning after the referee gate closes unless
        # somebody pushes a dribble-off. Strategy loops only notice via
        # Car.CanMove(); IDLE/REPL or control-panel callers never do, so
        # poll the gate here independently of any caller.
        self._last_permission = None
        self._watchdog_stop = threading.Event()
        self._watchdog_thread = threading.Thread(
            target=self._permission_watchdog, daemon=True
        )
        # Record the baseline synchronously: a permission flip during the
        # first poll interval must still be seen as a True->False edge.
        try:
            self._last_permission = bool(self.Permission())
        except Exception:
            self._last_permission = False
        self._watchdog_thread.start()

    def _permission_watchdog(self):
        """Zero peripheral IO on the permission True->False edge."""
        while not self._watchdog_stop.wait(0.1):
            try:
                permitted = bool(self.Permission())
            except Exception:
                permitted = False
            last = self._last_permission
            self._last_permission = permitted
            if last is True and not permitted:
                try:
                    self.force_off()
                except Exception:
                    pass
            elif last is False and permitted:
                # Re-arm the cache so the next Dribble() call re-sends its
                # value instead of hitting a stale pre-shutdown entry.
                self.DribbleStatus = False

    def stop_watchdog(self):
        self._watchdog_stop.set()
        if self._watchdog_thread.is_alive():
            self._watchdog_thread.join(timeout=0.5)

    def ShootBall(self):
        if self.Permission():
            self.SetIO(self.cfg.Ports.ElecMagnet,0)
            self.logger.info("ShootBall Activated")
            # TODO 用更优雅的实现技能冷却
            # Physical kickers need the electromagnet pulse width. The SIM port
            # is an edge-triggered UDP event, so sleeping only hides its peak.
            if os.environ.get("ROBOCUPX_BITS", "").strip().upper() != "SIM":
                time.sleep(0.3)
        else:
            self.logger.warning("Permission Denied: Unable to set dribble status.")

    def force_off(self):
        """Referee gate lost: stop the dribbler and reset the IO cache.

        The lower board latches the last IO state, so the dribbler would keep
        spinning after the referee module shuts down if no new command is
        sent. Resetting DribbleStatus also makes the next Dribble() call
        re-send once permission returns.
        """
        try:
            self.SetIO(self.cfg.Ports.Dribble, 0.0)
        except Exception as error:
            self.logger.warning(f"Dribble force-off failed: {error}")
        try:
            self.SetIO(self.cfg.Ports.ElecMagnet, 1)
        except Exception as error:
            self.logger.warning(f"Electromagnet reset failed: {error}")
        self.DribbleStatus = False

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
                self.logger.debug("Dribble set to %s" % str(Status))
            # self.logger.info("Dribble set to %s" % ("ON" if Status else "OFF"))
        else:
            self.SetIO(self.cfg.Ports.Dribble,0.0)
            # The hardware actually received a dribble-off; record it so the
            # next Dribble() call after permission returns re-sends the value
            # instead of hitting a stale cache entry.
            self.DribbleStatus = False
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
