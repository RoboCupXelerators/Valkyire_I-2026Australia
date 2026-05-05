# data.py | IntelliFusion Version 0.1.9(202308032000) Developer Alpha
# Enhanced for match replay functionality (2026-05)
from pathlib import Path
from peewee import *
import time
from . import config

# 基础类
Date = time.strftime("%Y%m", time.localtime())
RoBotName = config.Preference().read("model","number")
APP_DIR = Path(__file__).parent
DATA_DIR = APP_DIR / "data"
DATABASE_FILE = DATA_DIR / f"Xel-{RoBotName}.{Date}.sqlite"

db = SqliteDatabase(DATABASE_FILE)

class BaseModel(Model):
    class Meta:
        database = db

class Positions(BaseModel):
    """机器人位置记录表 - 用于比赛回放"""
    Tick = FloatField(null=False, index=True)  # 时间戳（秒）
    RobotId = IntegerField(null=False, default=1, index=True)  # 机器人ID
    SessionId = TextField(null=False, default="default")  # 比赛会话ID
    XPosition = FloatField(null=False, default=0)  # 场地X坐标 (cm, forward+)
    YPosition = FloatField(null=False, default=0)  # 场地Y坐标 (cm, left+)
    YawDeg = FloatField(null=False, default=0)  # 航向角 (度)
    Vx = FloatField(null=False, default=0)  # X方向速度
    Vy = FloatField(null=False, default=0)  # Y方向速度
    Wz = FloatField(null=False, default=0)  # 角速度
    BallXPosition = FloatField(null=False, default=0)  # 球相对X位置 (cm)
    BallYPosition = FloatField(null=False, default=0)  # 球相对Y位置 (cm)
    HasBall = BooleanField(null=False, default=False)  # 是否持球
    CreatedAt = FloatField(null=False, default=time.time)  # 记录创建时间

    class Meta:
        table_name = 'Positions'
        indexes = (
            (('SessionId', 'Tick', 'RobotId'), False),  # 复合索引便于查询
        )

    def setPos(self, Position: list, Ball: list, yaw: float = 0,
               vx: float = 0, vy: float = 0, wz: float = 0,
               has_ball: bool = False, robot_id: int = 1, session_id: str = "default"):
        """记录机器人位置数据"""
        Cache = Positions(
            Tick = time.time(),
            RobotId = robot_id,
            SessionId = session_id,
            XPosition = Position[0] if len(Position) > 0 else 0,
            YPosition = Position[1] if len(Position) > 1 else 0,
            YawDeg = yaw,
            Vx = vx,
            Vy = vy,
            Wz = wz,
            BallXPosition = Ball[0] if len(Ball) > 0 else 0,
            BallYPosition = Ball[1] if len(Ball) > 1 else 0,
            HasBall = has_ball,
            CreatedAt = time.time(),
        )
        Cache.save()
        return Cache

class Outputs(BaseModel):
    """电机输出记录表 - 用于比赛回放"""
    Tick = FloatField(null=False, index=True)  # 时间戳（秒）
    RobotId = IntegerField(null=False, default=1, index=True)  # 机器人ID
    SessionId = TextField(null=False, default="default")  # 比赛会话ID
    Motor1 = FloatField(null=False, default=0)  # 电机1速度
    Motor2 = FloatField(null=False, default=0)  # 电机2速度
    Motor3 = FloatField(null=False, default=0)  # 电机3速度
    Motor4 = FloatField(null=False, default=0)  # 电机4速度
    Vx = FloatField(null=False, default=0)  # X方向速度命令
    Vy = FloatField(null=False, default=0)  # Y方向速度命令
    Wz = FloatField(null=False, default=0)  # 角速度命令
    KickStrength = FloatField(null=False, default=0)  # 踢球力度
    DribbleStrength = FloatField(null=False, default=0)  # 盘球力度
    Flags = IntegerField(null=False, default=0)  # 命令标志位
    CreatedAt = FloatField(null=False, default=time.time)  # 记录创建时间

    class Meta:
        table_name = 'Outputs'
        indexes = (
            (('SessionId', 'Tick', 'RobotId'), False),
        )

    def SetOutput(self, input1, input2, input3, input4,
                  vx: float = 0, vy: float = 0, wz: float = 0,
                  kick: float = 0, dribble: float = 0, flags: int = 0,
                  robot_id: int = 1, session_id: str = "default"):
        """记录电机输出数据"""
        Datas = Outputs(
            Tick = time.time(),
            RobotId = robot_id,
            SessionId = session_id,
            Motor1 = input1,
            Motor2 = input2,
            Motor3 = input3,
            Motor4 = input4,
            Vx = vx,
            Vy = vy,
            Wz = wz,
            KickStrength = kick,
            DribbleStrength = dribble,
            Flags = flags,
            CreatedAt = time.time(),
        )
        Datas.save()
        return Datas

class MatchSession(BaseModel):
    """比赛会话表 - 记录每场比赛的基本信息"""
    SessionId = TextField(primary_key=True)  # 会话ID
    StartTime = FloatField(null=False)  # 开始时间戳
    EndTime = FloatField(null=True)  # 结束时间戳
    Description = TextField(null=True)  # 比赛描述
    RobotCount = IntegerField(null=False, default=1)  # 参赛机器人数

    class Meta:
        table_name = 'MatchSession'

    @classmethod
    def create_session(cls, session_id: str = None, description: str = "") -> 'MatchSession':
        """创建新的比赛会话"""
        if session_id is None:
            session_id = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        session = cls(
            SessionId=session_id,
            StartTime=time.time(),
            Description=description,
        )
        session.save()
        return session

    def end_session(self):
        """结束比赛会话"""
        self.EndTime = time.time()
        self.save()

def SetupDatabase():
    """初始化数据库表"""
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    db.create_tables([Positions, Outputs, MatchSession])

def GetSessionData(session_id: str, robot_id: int = None):
    """获取指定会话的所有位置数据"""
    query = Positions.select().where(Positions.SessionId == session_id)
    if robot_id is not None:
        query = query.where(Positions.RobotId == robot_id)
    return query.order_by(Positions.Tick)

def GetLatestSession() -> str:
    """获取最新的会话ID"""
    try:
        session = MatchSession.select().order_by(MatchSession.StartTime.desc()).first()
        return session.SessionId if session else None
    except:
        return None

def ExportSessionToDict(session_id: str) -> dict:
    """导出会话数据为字典格式（用于回放）"""
    positions = GetSessionData(session_id)
    return {
        "session_id": session_id,
        "positions": [
            {
                "tick": p.Tick,
                "robot_id": p.RobotId,
                "x_cm": p.XPosition,
                "y_cm": p.YPosition,
                "yaw_deg": p.YawDeg,
                "vx": p.Vx,
                "vy": p.Vy,
                "wz": p.Wz,
                "ball_x_cm": p.BallXPosition,
                "ball_y_cm": p.BallYPosition,
                "has_ball": p.HasBall,
            }
            for p in positions
        ]
    }

# Configuration management
CONFIG_FILE = APP_DIR / "config.json"

def GetConfig() -> dict:
    """读取配置文件"""
    try:
        if CONFIG_FILE.exists():
            with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
                return __import__('json').load(f)
    except Exception as e:
        print(f"[data] Error reading config: {e}")
    return {}

def SaveConfig(config: dict) -> None:
    """保存配置文件"""
    try:
        with open(CONFIG_FILE, 'w', encoding='utf-8') as f:
            __import__('json').dump(config, f, ensure_ascii=False, indent=4)
    except Exception as e:
        print(f"[data] Error saving config: {e}")
        raise

