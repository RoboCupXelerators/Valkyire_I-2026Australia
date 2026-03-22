# data.py | IntelliFusion Version 0.1.9(202308032000) Developer Alpha
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
    Tick = IntegerField(null=False)
    XPosition = IntegerField(null=False, default="not_required")#To request Model
    YPosition = IntegerField(null=False, default="not_required")#To request Model
    ZPosition = IntegerField(null=False, default="not_required")#To request Model
    BallXPosition = IntegerField(null=False, default="not_required")#To request Model
    BallYPosition = IntegerField(null=False, default="not_required")#To request Model

    class Meta:
        # 定义表名
        table_name = 'Positions'

    def setPos(self,Position:list[int,int,int],Ball:list[int,int]):
        Cache = Positions(
            Tick = time.time,
            XPosition = Position[0],
            YPosition = Position[1],
            ZPosition = Position[2],
            BallXPosition = Ball[0],
            BallYPosition = Ball[1],
        )
        Cache.save()

class Outputs(BaseModel):
    Tick = IntegerField()
    Motor1 = IntegerField(null=False, default="not_required")#To request Model
    Motor2 = IntegerField(null=False, default="not_required")#To request Model
    Motor3 = IntegerField(null=False, default="not_required")#To request Model
    Motor4 = IntegerField(null=False, default="not_required")#To request Model

    class Meta:
        # 定义表名
        table_name = 'Positions'

    def SetOutput(self,input1,input2,input3,input4):
        Datas = Outputs(
            Tick = time.time(),
            Motor1 = input1,
            Motor2 = input2,
            Motor3 = input3,
            Motor4 = input4,
        )
        Datas.save()

def SetupDatabase():
    db.create_tables([Positions,Outputs])

