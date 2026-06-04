"""ReasonData.config

精简版配置模块：
- 统一模型路径配置
- 移除冗余配置项
- 保持向后兼容
"""

try:
    import ujson as _json  # type: ignore
except ModuleNotFoundError:
    import json as _json
import os
from pathlib import Path
import random

APP_DIR = Path(__file__).parent
DATA_DIR = APP_DIR / "data"
CONFIG_FILE = DATA_DIR / "config.json"
CACHE_FILE = DATA_DIR / "cfg.cache.json"


class ConfigSection:
    """配置分段的通用容器"""
    def __init__(self, data):
        for key, value in data.items():
            if isinstance(value, dict):
                setattr(self, key, ConfigSection(value))
            else:
                setattr(self, key, value)


def ParseJsonToObj():
    parseData = Preference().cfg
    return ConfigSection(parseData)


class Preference:
    """配置管理类：负责读取、写入、迁移配置文件"""
    
    # 默认配置模板（唯一来源）
    DEFAULT_CONFIG = {
        "RoboInfo": {
            "number": 1,
            "type": "OP",           # OP(OffencePlayer) / DP(DefencePlayer)
            "Bit": "SIM",           # SIM / AB(ArisuBits) / IL(IceLoongBits)
            "Rec": "XV",            # AI(ArisuIntelligence) / TI(TunaIntelligence) / XV(XeleratorVision) / SY(SimYoloVision) / BB(BlobBased)
            "LidarType": "s1",      # s1 / s2
        },
        "Ports": {
            "LidarID": random.randrange(1, 99),
            "IceLoongPort": "/dev/iceloong",
            "ElecMagnet": 3,
            "Dribble": 4,
            "LeftFront": 1,
            "LeftBack": 2,
            "RightBack": 3,
            "RightFront": 4,
        },
        "Vision": {
            # 相机端口映射 (0:Back, 1:Left, 2:Front, 3:Right)
            "CamPorts": [0, 2, 4, 6],
            # 模型路径 (统一配置，相对于项目根目录)
            "ModelPath": "YOLO_Models/models/yolov8nForX5.pt",
            "EnginePath": "YOLO_Models/models/yolov8nForX5.engine",
            "HailoModelPath": "/yolov8s.hef",
            # 标定文件夹
            "CalibrationFolder": "Calibration",
            # 目标物理尺寸 (cm) - 用于bbox距离估计
            "BallDiameter": 4.2,
            "ChassisWidth": 18.0,
            "ChassisHeight": 18.0,
            "GoalWidth": 90.0,
            "GoalHeight": 18.0,
            "MinBallBBoxArea": 40,
            # 录制配置
            "Record": False,
            "RecordDir": "./Records",
        },
        "Field": {
            "Short": 200,           # 短边长度 (cm)
            "Long": 260,            # 长边长度 (cm)
            "Home": [-70, 0, 0],    # 默认回位点
        },
        "Motion": {
            "CatchVal": [6, 0],     # 抱球时的球位置
            "CatchBBoxSize": 90,    # 近距离视觉框大于该值时辅助判断持球
            "CatchBBoxXMargin": 4,  # bbox 持球判定允许的前向余量
            "CatchBBoxYTol": 2,     # bbox 持球判定允许的横向误差
            "ErrorRange": 10,       # 到达目标的误差范围 (cm)
            "MaxSpeed": 500,        # 最大速度值
            "NormalizeWheelOutput": True,
            "EnableTractionControl": True,
            "TractionPlanarAccelLimit": 600.0,
            "TractionAngularAccelLimit": 900.0,
            "TractionDtMax": 0.08,
            "EnableWheelSlewLimit": True,
            "TractionWheelSlewRate": 1800.0,
            "EnableCoupledWheelAllocator": True,
            "WheelAllocatorSaturationMode": "preserve_twist",
            "EnableTurnTranslationDerate": True,
            "TurnTranslationScaleWz": 120.0,
            "TurnTranslationMinScale": 0.6,
            "Move2PathPointTimeoutS": 12.0,
            "LockBallSlipMaxAngle": 82,
            "LockBallSlipAcquireKpX": 4.2,
            "LockBallSlipAcquireKpY": 3.1,
            "LockBallSlipAcquireKpW": 1.7,
            "LockBallSlipAcquireMaxX": 135,
            "LockBallSlipAcquireMaxY": 105,
            "LockBallSlipAcquireMaxW": 175,
            "LockBallSlipPidKpX": 3.6,
            "LockBallSlipPidKpY": 2.6,
            "LockBallSlipPidKpW": 1.1,
            "LockBallSlipPidMaxX": 135,
            "LockBallSlipPidMaxY": 95,
            "LockBallSlipPidMaxW": 85,
            "LockBallSlipLostHoldSeconds": 0.18,
            "LockBallSlipDribblePower": 0.75,
            "RestrictedArea": {
                # 坐标安全区配置，单位 cm。策略只依赖 FieldRobotPos，不依赖雷达原始距离。
                "XLimit": 90.0,
                "YLimit": 70.0,
                "GoalBoxWidth": 25.0,
                "GoalBoxYHalf": 50.0,
                "Clearance": 5.0,
                "RecoveryClearance": 16.0,
                "RecoveryExtra": 3.0
            },
            "Pos2PosPID": {
                "Enabled": True,
                "Kp": 1.1,
                "Ki": 0.0,
                "Kd": 0.08,
                "OutputLimit": 160,
                "SampleTime": 0.02,
                "ResetFrames": 300,
                "ArrivalScale": 0.8,
                "MinSpeedGain": 0.25,
                "MaxSpeedGain": 4.0
            },
        },
        "Network": {
            "Type": "Slave",        # Master / Slave
            "Method": "WIFI",       # WIFI / BLE
            "SelfIP": "192.168.1.x",
            "RemoteIP": "192.168.1.x",
            "Port": 20001,
            "SSID": "MisakaNetwork",
            "Password": "MisakaNetwork20001/",
        },
        "Sim": {
            "UnityIP": "127.0.0.1",
            "RobotId": 1,
            # 多机器人端口映射: CmdPort = PortBase + (RobotId-1) * PortStride
            "PortBase": 20001,
            "PortStride": 10,
            "MultiRobotMode": "per_port",  # per_port / hub
            "SceneControlPort": 20100,
            # 运动参数缩放
            "WheelToV": 0.008,
            "WheelToWz": 0.02,
            "YawOffsetDeg": 0.0,
            "KickStrength": 1.0,
            "DribbleScale": 3.0,
            "MoveScale": 1.0,
        },
        "Debug": {
            "FullLog": False,
            "Beta": True,
            "RecordMotion": False,
            "RecordInterval": 1,
            "RecordSessionId": None,
            "RecordPos2Pos": False,
            "Pos2PosRecordDir": "./Records/pos2pos",
        },
    }

    def __init__(self):
        # 确保配置文件存在
        try:
            os.stat(CONFIG_FILE)
        except FileNotFoundError:
            self._create_default_config()

        # 读取配置
        with open(CONFIG_FILE) as f:
            self.cfg = _json.load(f)

        # 检查是否需要迁移旧配置
        if self._needs_migration():
            self._migrate_config()

    def _create_default_config(self):
        try:
            with open(CONFIG_FILE, "w") as f:
                _json.dump(self.DEFAULT_CONFIG, f, indent=2)
        except Exception:
            raise FileNotFoundError("无法创建 config.json，请手动创建")

    def _needs_migration(self):
        """检查是否需要从旧配置迁移"""
        for section in self.DEFAULT_CONFIG:
            if section not in self.cfg:
                return True
            for key in self.DEFAULT_CONFIG[section]:
                if key not in self.cfg.get(section, {}):
                    return True
        return False

    def _migrate_config(self):
        """迁移旧配置到新格式"""
        # 备份旧配置
        if os.path.exists(CACHE_FILE):
            os.remove(CACHE_FILE)
        os.rename(CONFIG_FILE, CACHE_FILE)

        # 读取旧配置
        with open(CACHE_FILE, "r") as f:
            old_cfg = _json.load(f)

        # 创建新配置，先保留现有新格式值，再兼容旧字段映射。
        new_cfg = _json.loads(_json.dumps(self.DEFAULT_CONFIG))

        def merge_existing(dst, src):
            for key, value in src.items():
                if isinstance(value, dict) and isinstance(dst.get(key), dict):
                    merge_existing(dst[key], value)
                else:
                    dst[key] = value

        merge_existing(new_cfg, old_cfg)
        
        # 迁移映射表
        migrations = {
            # VisionVals -> Vision
            ("VisionVals", "Front"): ("Vision", "CamPorts", lambda v: None),  # 特殊处理
            ("VisionVals", "HailoModelName"): ("Vision", "HailoModelPath"),
            ("VisionVals", "TunaModelName"): ("Vision", "ModelPath"),
            ("VisionVals", "TunaEnginePath"): ("Vision", "EnginePath"),
            ("VisionVals", "CalibrationFolder"): ("Vision", "CalibrationFolder"),
            ("VisionVals", "BallDiameterCM"): ("Vision", "BallDiameter"),
            ("VisionVals", "ChassisWidthCM"): ("Vision", "ChassisWidth"),
            ("VisionVals", "ChassisHeightCM"): ("Vision", "ChassisHeight"),
            ("VisionVals", "BlueGoalWidthCM"): ("Vision", "GoalWidth"),
            ("VisionVals", "Record"): ("Vision", "Record"),
            ("VisionVals", "RecordDir"): ("Vision", "RecordDir"),
            # Bounds -> Field
            ("Bounds", "Short"): ("Field", "Short"),
            ("Bounds", "Long"): ("Field", "Long"),
            # CheckPoints -> Field
            ("CheckPoints", "Home"): ("Field", "Home"),
            # ExpectedVals -> Motion
            ("ExpectedVals", "CatchVal"): ("Motion", "CatchVal"),
            ("ExpectedVals", "ErrorRange"): ("Motion", "ErrorRange"),
            ("ExpectedVals", "MaxSpeedValue"): ("Motion", "MaxSpeed"),
            # WIFI -> Network
            ("WIFI", "SelfIP"): ("Network", "SelfIP"),
            ("WIFI", "RemoteIP"): ("Network", "RemoteIP"),
            ("WIFI", "Port"): ("Network", "Port"),
            ("WIFI", "SSID"): ("Network", "SSID"),
            ("WIFI", "PWD"): ("Network", "Password"),
            # Transimission -> Network
            ("Transimission", "Type"): ("Network", "Type"),
            ("Transimission", "Method"): ("Network", "Method"),
            # Sim (部分迁移)
            ("Sim", "CmdPortBase"): ("Sim", "PortBase"),
            ("Sim", "KickStrength"): ("Sim", "KickStrength"),
            ("Sim", "DribbleStrengthScale"): ("Sim", "DribbleScale"),
            ("Sim", "PlanarMoveSpeedScale"): ("Sim", "MoveScale"),
        }

        for (old_sec, old_key), new_path in migrations.items():
            try:
                old_val = old_cfg.get(old_sec, {}).get(old_key)
                if old_val is not None:
                    if len(new_path) == 2:
                        new_sec, new_key = new_path
                        if new_sec in new_cfg and new_key in new_cfg[new_sec]:
                            new_cfg[new_sec][new_key] = old_val
            except Exception:
                continue

        # 特殊处理: CamPorts 从 Front/Right/Back/Left 迁移
        try:
            vv = old_cfg.get("VisionVals", {})
            cam_ports = [
                vv.get("Back", 0),
                vv.get("Left", 2),
                vv.get("Front", 4),
                vv.get("Right", 6),
            ]
            # 去重保持顺序
            seen = set()
            unique_ports = []
            for p in cam_ports:
                if p not in seen:
                    seen.add(p)
                    unique_ports.append(p)
            new_cfg["Vision"]["CamPorts"] = unique_ports
        except Exception:
            pass

        # 写入新配置
        with open(CONFIG_FILE, "w") as f:
            _json.dump(new_cfg, f, indent=2)

        # 更新内存中的配置
        self.cfg = new_cfg
        os.remove(CACHE_FILE)

    def write(self, section: str, option: str, value) -> None:
        self.cfg[section][option] = value
        with open(CONFIG_FILE, "w") as f:
            _json.dump(self.cfg, f, indent=2)

    def read(self, section: str, option: str):
        try:
            return self.cfg[section][option]
        except KeyError:
            self.__init__()
            return self.cfg.get(section, {}).get(option)


# 向后兼容的属性访问类
class _CompatVisionVals:
    """向后兼容层：将旧属性名映射到新配置"""
    def __init__(self, settings):
        self._s = settings

    @property
    def Front(self): return self._s.Vision.CamPorts[2] if len(self._s.Vision.CamPorts) > 2 else 4
    @property
    def Right(self): return self._s.Vision.CamPorts[3] if len(self._s.Vision.CamPorts) > 3 else 6
    @property
    def Back(self): return self._s.Vision.CamPorts[0] if len(self._s.Vision.CamPorts) > 0 else 0
    @property
    def Left(self): return self._s.Vision.CamPorts[1] if len(self._s.Vision.CamPorts) > 1 else 2
    @property
    def HailoModelName(self): return self._s.Vision.HailoModelPath
    @property
    def TunaModelName(self): return self._s.Vision.ModelPath
    @property
    def TunaEnginePath(self): return self._s.Vision.EnginePath
    @property
    def CalibrationFolder(self): return self._s.Vision.CalibrationFolder
    @property
    def BallDiameterCM(self): return self._s.Vision.BallDiameter
    @property
    def ChassisWidthCM(self): return self._s.Vision.ChassisWidth
    @property
    def ChassisHeightCM(self): return self._s.Vision.ChassisHeight
    @property
    def BlueGoalWidthCM(self): return self._s.Vision.GoalWidth
    @property
    def BlueGoalHeightCM(self): return self._s.Vision.GoalHeight
    @property
    def YellowGoalWidthCM(self): return self._s.Vision.GoalWidth
    @property
    def YellowGoalHeightCM(self): return self._s.Vision.GoalHeight
    @property
    def EnableBBoxDistanceEstimate(self): return True
    @property
    def Record(self): return self._s.Vision.Record
    @property
    def RecordDir(self): return self._s.Vision.RecordDir
    @property
    def RecordCams(self): return [0, 1, 2, 3]
    @property
    def RecordWidth(self): return 320
    @property
    def RecordHeight(self): return 320
    @property
    def RecordSegmentSeconds(self): return 60
    @property
    def RecordFPS(self): return 30
    @property
    def RecordCodec(self): return "mp4v"
    @property
    def RecordExt(self): return "mp4"
    @property
    def ExposeVal(self): return 100
    @property
    def AutoExpose(self): return 0


class _CompatBounds:
    """向后兼容层：Bounds -> Field"""
    def __init__(self, settings):
        self._s = settings

    @property
    def Short(self): return self._s.Field.Short
    @property
    def Long(self): return self._s.Field.Long
    @property
    def ClsPos(self): return [85, 35, 0]
    @property
    def FarPos(self): return [95, 75, 0]
    @property
    def Distance2Bound(self): return 240
    @property
    def restricted_area_x(self): return 400
    @property
    def restricted_area_y(self): return 800


class _CompatExpectedVals:
    """向后兼容层：ExpectedVals -> Motion"""
    def __init__(self, settings):
        self._s = settings

    @property
    def CatchVal(self): return self._s.Motion.CatchVal
    @property
    def ErrorRange(self): return self._s.Motion.ErrorRange
    @property
    def MaxSpeedValue(self): return self._s.Motion.MaxSpeed
    @property
    def MaxWarnCount(self): return 3


class _CompatWIFI:
    """向后兼容层：WIFI -> Network"""
    def __init__(self, settings):
        self._s = settings

    @property
    def SelfIP(self): return self._s.Network.SelfIP
    @property
    def RemoteIP(self): return self._s.Network.RemoteIP
    @property
    def Port(self): return self._s.Network.Port
    @property
    def SSID(self): return self._s.Network.SSID
    @property
    def PWD(self): return self._s.Network.Password


class _CompatTransimission:
    """向后兼容层：Transimission -> Network"""
    def __init__(self, settings):
        self._s = settings

    @property
    def Type(self): return self._s.Network.Type
    @property
    def Method(self): return self._s.Network.Method


class _CompatSim:
    """向后兼容层：Sim新属性"""
    def __init__(self, settings):
        self._s = settings

    @property
    def UnityIP(self): return self._s.Sim.UnityIP
    @property
    def RobotId(self): return self._s.Sim.RobotId
    @property
    def CmdPortBase(self): return self._s.Sim.PortBase
    @property
    def StatusPortBase(self): return self._s.Sim.PortBase + 1
    @property
    def PortStride(self): return self._s.Sim.PortStride
    @property
    def MultiRobotMode(self): return self._s.Sim.MultiRobotMode
    @property
    def SceneControlPort(self): return self._s.Sim.SceneControlPort
    @property
    def CmdPort(self): return self._s.Sim.PortBase
    @property
    def StatusPort(self): return self._s.Sim.PortBase + 1
    @property
    def TwistFromWheels(self): return True
    @property
    def WheelToV(self): return self._s.Sim.WheelToV
    @property
    def WheelToWz(self): return self._s.Sim.WheelToWz
    @property
    def YawOffsetDeg(self): return self._s.Sim.YawOffsetDeg
    @property
    def YawZeroAtStart(self): return False
    @property
    def KickStrength(self): return self._s.Sim.KickStrength
    @property
    def DribbleStrengthScale(self): return self._s.Sim.DribbleScale
    @property
    def PlanarMoveSpeedScale(self): return self._s.Sim.MoveScale
    @property
    def UnityToCm(self): return 100.0
    @property
    def OriginXCM(self): return 0.0
    @property
    def OriginYCM(self): return 0.0
    @property
    def PoseScale(self): return 1.0
    @property
    def PoseOffsetXCM(self): return 0.0
    @property
    def PoseOffsetYCM(self): return 0.0
    @property
    def Debug(self): return False


class SettingsProxy:
    """设置代理：提供新配置API和向后兼容"""
    
    def __init__(self):
        self._raw = ParseJsonToObj()
        # 设置向后兼容层
        self.VisionVals = _CompatVisionVals(self)
        self.Bounds = _CompatBounds(self)
        self.ExpectedVals = _CompatExpectedVals(self)
        self.WIFI = _CompatWIFI(self)
        self.Transimission = _CompatTransimission(self)
        # Sim需要特殊处理，因为新旧配置混用
        self._sim_compat = _CompatSim(self)

    def __getattr__(self, name):
        # 优先返回原始配置
        if hasattr(self._raw, name):
            return getattr(self._raw, name)
        # Sim向后兼容
        if name == "Sim":
            return self._sim_compat
        raise AttributeError(f"'{type(self).__name__}' object has no attribute '{name}'")

    # 直接暴露常用配置段
    @property
    def RoboInfo(self): return self._raw.RoboInfo
    @property
    def Ports(self): return self._raw.Ports
    @property
    def Vision(self): return self._raw.Vision
    @property
    def Field(self): return self._raw.Field
    @property
    def Motion(self): return self._raw.Motion
    @property
    def Network(self): return self._raw.Network
    @property
    def Sim(self): return self._raw.Sim
    @property
    def Debug(self): return self._raw.Debug
    # 向后兼容别名
    @property
    def CheckPoints(self):
        class _CP:
            Home = self._raw.Field.Home
        return _CP()


Settings: SettingsProxy = SettingsProxy()
