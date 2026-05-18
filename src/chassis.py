import math
from ReasonData.config import Settings
from ReasonData import logger
import time, threading


class MotionRecorder:
    """机器人动作记录器 - 植入到 chassis.py 中"""
    
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._initialized = False
        return cls._instance
    
    def __init__(self):
        if self._initialized:
            return
        self._initialized = True
        
        self.cfg = Settings
        self.logger = logger
        self._is_recording = False
        self._session_id = None
        self._record_counter = 0
        self._last_position = [0, 0]
        self._last_yaw = 0
        self._last_ball_pos = [0, 0]
        self._last_has_ball = False
        self._last_vx = 0
        self._last_vy = 0
        self._last_wz = 0
        
        # 延迟导入数据库模块
        self._db_initialized = False
        self._Positions = None
        self._Outputs = None
    
    def _init_db(self):
        """延迟初始化数据库"""
        if self._db_initialized:
            return
        try:
            from ReasonData.data import SetupDatabase, Positions, Outputs
            SetupDatabase()
            self._Positions = Positions
            self._Outputs = Outputs
            self._db_initialized = True
        except Exception as e:
            self.logger.warning(f"[MotionRecorder] Database init failed: {e}")
    
    @property
    def is_recording(self) -> bool:
        """检查是否正在记录"""
        return self._is_recording
    
    def start_recording(self, session_id: str = None):
        """开始记录"""
        self._init_db()
        if not self._db_initialized:
            self.logger.warning("[MotionRecorder] Database not available")
            return False
        
        if self._is_recording:
            self.logger.warning("[MotionRecorder] Already recording")
            return False
        
        self._session_id = session_id or time.strftime("%Y%m%d_%H%M%S", time.localtime())
        self._record_counter = 0
        self._is_recording = True
        
        # 创建会话
        try:
            from ReasonData.data import MatchSession
            MatchSession.create_session(self._session_id, description="Auto-recorded from chassis")
        except Exception as e:
            self.logger.warning(f"[MotionRecorder] Session create failed: {e}")
        
        self.logger.info(f"[MotionRecorder] Started recording session: {self._session_id}")
        return True
    
    def stop_recording(self):
        """停止记录"""
        if not self._is_recording:
            return
        
        self._is_recording = False
        
        # 结束会话
        try:
            from ReasonData.data import MatchSession
            session = MatchSession.get_or_none(MatchSession.SessionId == self._session_id)
            if session:
                session.end_session()
        except Exception:
            pass
        
        self.logger.info(f"[MotionRecorder] Stopped recording. Total frames: {self._record_counter}")
        self._session_id = None
    
    def record_position(self, x: float, y: float, yaw: float,
                        vx: float = 0, vy: float = 0, wz: float = 0,
                        ball_x: float = 0, ball_y: float = 0, has_ball: bool = False,
                        robot_id: int = 1):
        """记录位置数据"""
        if not self._is_recording:
            return
        
        # 检查记录间隔
        interval = getattr(self.cfg.Debug, 'RecordInterval', 1)
        self._record_counter += 1
        if self._record_counter % interval != 0:
            return
        
        # 缓存数据
        self._last_position = [x, y]
        self._last_yaw = yaw
        self._last_ball_pos = [ball_x, ball_y]
        self._last_has_ball = has_ball
        self._last_vx = vx
        self._last_vy = vy
        self._last_wz = wz
        
        # 写入数据库
        try:
            if self._Positions:
                self._Positions.setPos(
                    Position=[x, y],
                    Ball=[ball_x, ball_y],
                    yaw=yaw,
                    vx=vx, vy=vy, wz=wz,
                    has_ball=has_ball,
                    robot_id=robot_id,
                    session_id=self._session_id
                )
        except Exception as e:
            if self.cfg.Debug.FullLog:
                self.logger.debug(f"[MotionRecorder] Position record error: {e}")
    
    def record_output(self, motor1: float, motor2: float, motor3: float, motor4: float,
                      vx: float = 0, vy: float = 0, wz: float = 0,
                      kick: float = 0, dribble: float = 0, flags: int = 0,
                      robot_id: int = 1):
        """记录电机输出数据"""
        if not self._is_recording:
            return
        
        try:
            if self._Outputs:
                self._Outputs.SetOutput(
                    motor1, motor2, motor3, motor4,
                    vx=vx, vy=vy, wz=wz,
                    kick=kick, dribble=dribble,
                    flags=flags,
                    robot_id=robot_id,
                    session_id=self._session_id
                )
        except Exception as e:
            if self.cfg.Debug.FullLog:
                self.logger.debug(f"[MotionRecorder] Output record error: {e}")
    
    def update_ball_info(self, ball_x: float, ball_y: float, has_ball: bool):
        """更新球信息（供外部调用）"""
        self._last_ball_pos = [ball_x, ball_y]
        self._last_has_ball = has_ball


# 全局记录器实例
_motion_recorder = None

def get_motion_recorder() -> MotionRecorder:
    """获取全局动作记录器"""
    global _motion_recorder
    if _motion_recorder is None:
        _motion_recorder = MotionRecorder()
    return _motion_recorder


class Car:
    def __init__(self,SetMotorFunc,GetYaw=None,permission=None,MotorEncoder=None):
        self.SetMotorFunc = SetMotorFunc
        self.GetYaw = GetYaw
        if permission is None:
            self.Permission = lambda: True
        else:
            self.Permission = permission
        self.MotorEncoder = MotorEncoder
        self.Kp = 3
        self.KpW = 1
        self.cfg = Settings
        self.speedCache = [0,0,0,0]
        import simple_pid
        self.Yaw_PID = simple_pid.PID(2.3, 0.08, 0.05, setpoint=0,output_limits=(-100,100),auto_mode=True)
        # Always use the unified project logger (ReasonData.logger)
        # to avoid AttributeError when FullLog=False.
        self.logger = logger
        
        # 动作记录器
        self._recorder = get_motion_recorder()
        self._last_vx = 0
        self._last_vy = 0
        self._last_wz = 0
        
        # 检查是否启用记录
        self._check_recording_config()
        
        # 启动状态更新线程
        self._state_update_thread = None
        self._state_update_stop = threading.Event()
        self._start_state_update_thread()
    
    def _start_state_update_thread(self):
        """启动状态更新线程，定期更新全局状态"""
        def _update_loop():
            while not self._state_update_stop.is_set():
                try:
                    # 更新全局状态
                    from robot_state import set_robot_state
                    yaw = self.GetYaw() if self.GetYaw else 0.0
                    set_robot_state(
                        yaw_deg=yaw,
                        vx=self._last_vx,
                        vy=self._last_vy,
                        wz=self._last_wz,
                    )
                except Exception:
                    pass
                time.sleep(0.05)  # 20Hz 更新
        
        self._state_update_thread = threading.Thread(target=_update_loop, daemon=True)
        self._state_update_thread.start()
    
    def _check_recording_config(self):
        """检查配置是否启用记录"""
        try:
            if getattr(self.cfg.Debug, 'RecordMotion', False):
                session_id = getattr(self.cfg.Debug, 'RecordSessionId', None)
                if not self._recorder.is_recording:
                    self._recorder.start_recording(session_id)
                    self.logger.info("[Car] Motion recording enabled from config")
        except Exception as e:
            self.logger.debug(f"[Car] Recording config check failed: {e}")
    
    def SetMotor(self,speedCache:list[int,int,int,int]):
        if self.Permission():
            self.speedCache = speedCache
            number = [self.cfg.Ports.LeftFront,self.cfg.Ports.LeftBack,self.cfg.Ports.RightFront,self.cfg.Ports.RightBack]
            speed = [0,0,0,0]
            for n in range(4):
                if speedCache[number[n]-1] > self.cfg.Motion.MaxSpeed:
                    speed[n] = self.cfg.Motion.MaxSpeed
                else:
                    speed[n] = int(speedCache[number[n]-1])
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
            
            # 记录电机输出
            self._recorder.record_output(
                motor1=speed[0], motor2=speed[1], motor3=speed[2], motor4=speed[3],
                vx=self._last_vx, vy=self._last_vy, wz=self._last_wz,
                robot_id=getattr(self.cfg.RoboInfo, 'number', 1)
            )
            
            self.SetMotorFunc(speed[0], speed[1], speed[2], speed[3])
        else:
            self.SetMotorFunc(0,0,0,0)
            self.logger.warning("Permission Denied: Unable to set motor speeds.")
    
    def SetKp(self,Kp):
        self.Kp = Kp
    
    def Compass(self):
        return self.GetYaw()

    def RobotMoveVector(self,SpeedX,SpeedY,SpeedZ):
        '''
        a vector movement (SpeedX,SpeedY,SpeedZ) without YawCorrect
        '''
        # 缓存速度用于记录
        self._last_vx = SpeedX
        self._last_vy = SpeedY
        self._last_wz = SpeedZ
        
        Speed1 = SpeedX*self.Kp - SpeedY*self.Kp - SpeedZ*self.KpW
        Speed2 = SpeedX*self.Kp + SpeedY*self.Kp - SpeedZ*self.KpW
        Speed3 = SpeedX*self.Kp + SpeedY*self.Kp + SpeedZ*self.KpW
        Speed4 = SpeedX*self.Kp - SpeedY*self.Kp + SpeedZ*self.KpW
        output = [Speed1, Speed2, Speed3, Speed4]
        self.SetMotor(output)
    
    def RobotMoveAngle(self,Angle,Speed,Kp:float|None = 1):
        '''
        angle movement (Angle,Speed) without YawCorrect
        '''
        rad = math.radians(Angle)
        SpeedX = int(math.sin(rad) * Speed)
        SpeedY = int(math.cos(rad) * Speed)
        self.RobotMoveVector(SpeedY,SpeedX,0)

    def FieldMoveAngle(self,FacingAngle,MovingAngle,Speed,Kp:float|None = None):
        '''
        angle movement (FacingAngle,MovingAngle,speed) with YawCorrect
        '''
        Yaw = self.GetYaw()
        rad = math.radians((MovingAngle-Yaw+FacingAngle))
        SpeedX = int(math.cos(rad) * Speed)
        SpeedY = int(math.sin(rad) * Speed)
        self.FieldMoveVector(SpeedX,SpeedY,FacingAngle,Kp)
        
    def FieldMoveVector(self,SpeedX,SpeedY,FacingAngle,Kp:float|None = None,KpW:float|None = None):
        '''
        vector movement (SpeedX,SpeedY,AngleZ˚,Angle_Gain) with YawCorrect
        '''
        Yaw = self.GetYaw()
        # TODO change facing
        # FacingAngle = 360 - FacingAngle
        Delta = Yaw - FacingAngle
        Error = -( (Delta + 180) % 360 - 180 ) # Normalize to [-180, 180]
        Normaled_Yaw =  ( (Yaw + 180) % 360 - 180 )
        Normaled_Facing =  ( (FacingAngle + 180) % 360 - 180 )
        if not Kp:
            Kp = self.Kp
        if not KpW:
            KpW = self.KpW
        if Settings.Debug.Beta:
            self.Yaw_PID.setpoint = Normaled_Facing
            SpeedZ = self.Yaw_PID(Normaled_Yaw)
            logger.error(f"PID output: {SpeedZ}, Yaw: {Normaled_Yaw}, FacingAngle: {Normaled_Facing}")
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
    
    def RobotXMove(self,Angle,Speed):
        self.RobotMoveVector(90,Angle,Speed)
   
    def RobotYMove(self,Angle,Speed,Kp=None):
        self.RobotMoveVector(0,Angle,Speed,Kp)

    def FieldTurn(self,Angle):
        if self.GetYaw is None:
            return False
        self.FieldMoveVector(0,0,Angle)

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
        self.SetMotor([0,0,0,0])

        

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
        else:
            self.logger.warning("Permission Denied: Unable to set dribble status.")

    def Dribble(self,Status):
        if self.Permission():
            if Status == True:
                self.SetIO(self.cfg.Ports.Dribble,0.2)
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

