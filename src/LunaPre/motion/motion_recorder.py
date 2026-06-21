import threading
import time

from ReasonData.config import Settings
from ReasonData import logger


class MotionRecorder:
    """Motion recording side channel for match replay data."""

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

        try:
            from ReasonData.data import MatchSession

            session = MatchSession.get_or_none(MatchSession.SessionId == self._session_id)
            if session:
                session.end_session()
        except Exception:
            pass

        self.logger.info(f"[MotionRecorder] Stopped recording. Total frames: {self._record_counter}")
        self._session_id = None

    def record_position(
        self,
        x: float,
        y: float,
        yaw: float,
        vx: float = 0,
        vy: float = 0,
        wz: float = 0,
        ball_x: float = 0,
        ball_y: float = 0,
        has_ball: bool = False,
        robot_id: int = 1,
    ):
        """记录位置数据"""
        if not self._is_recording:
            return

        interval = getattr(self.cfg.Debug, "RecordInterval", 1)
        self._record_counter += 1
        if self._record_counter % interval != 0:
            return

        self._last_position = [x, y]
        self._last_yaw = yaw
        self._last_ball_pos = [ball_x, ball_y]
        self._last_has_ball = has_ball
        self._last_vx = vx
        self._last_vy = vy
        self._last_wz = wz

        try:
            if self._Positions:
                self._Positions.setPos(
                    Position=[x, y],
                    Ball=[ball_x, ball_y],
                    yaw=yaw,
                    vx=vx,
                    vy=vy,
                    wz=wz,
                    has_ball=has_ball,
                    robot_id=robot_id,
                    session_id=self._session_id,
                )
        except Exception as e:
            if self.cfg.Debug.FullLog:
                self.logger.debug(f"[MotionRecorder] Position record error: {e}")

    def record_output(
        self,
        motor1: float,
        motor2: float,
        motor3: float,
        motor4: float,
        vx: float = 0,
        vy: float = 0,
        wz: float = 0,
        kick: float = 0,
        dribble: float = 0,
        flags: int = 0,
        robot_id: int = 1,
    ):
        """记录电机输出数据"""
        if not self._is_recording:
            return

        try:
            if self._Outputs:
                self._Outputs.SetOutput(
                    motor1,
                    motor2,
                    motor3,
                    motor4,
                    vx=vx,
                    vy=vy,
                    wz=wz,
                    kick=kick,
                    dribble=dribble,
                    flags=flags,
                    robot_id=robot_id,
                    session_id=self._session_id,
                )
        except Exception as e:
            if self.cfg.Debug.FullLog:
                self.logger.debug(f"[MotionRecorder] Output record error: {e}")

    def update_ball_info(self, ball_x: float, ball_y: float, has_ball: bool):
        """更新球信息（供外部调用）"""
        self._last_ball_pos = [ball_x, ball_y]
        self._last_has_ball = has_ball


_motion_recorder = None


def get_motion_recorder() -> MotionRecorder:
    """获取全局动作记录器"""
    global _motion_recorder
    if _motion_recorder is None:
        _motion_recorder = MotionRecorder()
    return _motion_recorder
