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

APP_DIR = Path(__file__).parent
DATA_DIR = Path(os.environ.get("XEL_RUNTIME_DATA_DIR", APP_DIR / "data"))
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
            "Bit": "SIM",           # SIM / IL
            "Rec": "XV",            # AI(ArisuIntelligence) / TI(TunaIntelligence) / XV(XeleratorVision) / SY(SimYoloVision) / BB(BlobBased)
        },
        "Ports": {
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
            # 统一模型名称 (不含后缀，各推理库自动选择所需格式)
            # - ArisuIntelligence (AI/Hailo): 使用 .hef
            # - TunaIntelligence/HybridVision (TI/HV): 使用 .pt 和 .engine
            # - LunaIntelligence/SimYoloVision (SY/LI): 使用 .pt 和 .engine
            "ModelName": "ArisuModel_v1.2s",
            # 0=flexible automatic exposure, 1=auto-meter then lock exposure.
            "ExposureMode": 1,
            # -4..4 controls the camera's physical brightness control.
            "ExposureBias": 0,
            # Stable mode's fixed UVC exposure value (10..500). Lower is darker.
            "ManualExposure": 68,
            # 标定文件夹
            "CalibrationFolder": "Calibration",
            # Target dimensions (cm) for operator-facing object semantics.
            "BallDiameter": 4.2,
            "ChassisWidth": 18.0,
            "ChassisHeight": 5.0,
            "GoalWidth": 90.0,
            "GoalHeight": 18.0,
            "MinBallBBoxArea": 40,
            "BallOffsetXCM": 0.0,
            "BallOffsetYCM": 0.0,
            # HybridVision: low-rate search on every camera, high-rate focus on
            # the camera that recently saw the ball.
            "AdaptiveFrameDistribution": True,
            "CameraMinYoloHz": 4.0,
            "CameraFocusYoloHz": 20.0,
            "CameraFocusHoldMs": 250.0,
            "YoloMaxFramesPerTick": 4,
            # 录制配置
            "Record": False,
            "RecordDir": "./Records",
        },
        "Field": {
            "Short": 200,           # 短边长度 (cm)
            "Long": 260,            # 长边长度 (cm)
            "Home": [-70, 0, 0],    # 默认回位点
        },
        "Chassis": {
            "WheelRadiusM": 0.029,  # 轮子半径，单位 m
            "RobotRadiusM": 0.09,   # 机器人中心到轮子的半径，单位 m
            "MaxWheelRPM": 500,     # 电机命令单位 RPM，正方向为单轮顺时针
        },
        "Motion": {
            "CatchVal": [8.5, 0],   # 抱球时的球位置：机器人中心到球心 8.5cm
            "ErrorRange": 10,       # 到达目标的误差范围 (cm)
            # RobotMoveVector 是速度阶跃命令: 斜坡默认关闭 (2026-09-20).
            # 打开 EnableTractionControl/EnableWheelSlewLimit 可恢复机械保护斜坡.
            "EnableTractionControl": False,
            "EnableWheelSlewLimit": False,

            "LockBallSlip": {
                "ForwardKp": 0.025,
                "ForwardKi": 0.0,
                "ForwardKd": 0.0,
                "LateralKp": 0.012,
                "LateralKi": 0.0,
                "LateralKd": 0.0,
                "AngularKp": 0.9,
                "AngularKi": 0.0,
                "AngularKd": 0.0,
                "ForwardMaxMps": 0.35,
                "LateralMaxMps": 0.35,
                "AngularMaxDegS": 50.0,
                "TurnOnlyAngleDeg": 70.0,
                "TurnOnlyKp": 0.75,
                "TurnOnlyMaxDegS": 55.0,
                "RearForwardKp": 0.012,
                "RearForwardMaxMps": 0.55,
                "RearLateralKp": 0.016,
                "RearLateralMaxMps": 0.65,
                "RearAngularKp": 1.80,
                "RearAngularMaxDegS": 85.0,
                "LostHoldSeconds": 0.18,
                "NearCreepMaxMps": 0.10,
                "NearDockDecelMps2": 0.5,
                "BallJumpRejectCm": 45.0,
                "BallJumpConfirmFrames": 2
            },
            "RestrictedArea": {
                # 坐标安全区配置，单位 cm。策略只依赖 FieldRobotPose，不依赖雷达原始距离。
                "XLimit": 110.0,
                "YLimit": 80.0,
                "GoalBoxWidth": 25.0,
                "GoalBoxYHalf": 40.0,
                "GoalBoxCornerRadius": 20.0,
                # 1=严格整车不越线，2=澳洲赛规：允许局部越线但禁止整车进入
                "ProtectionLevel": 1,
                "Clearance": 5.0,
                "RecoveryClearance": 16.0,
                "RecoveryExtra": 3.0
            },
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

        self._validate_legacy_sensor(self.cfg)

        # 检查是否需要迁移旧配置
        if self._needs_migration():
            self._migrate_config()
        self._validate_active_config(self.cfg)

    @staticmethod
    def _active_bit(value) -> str:
        return str(value or "").strip().upper()

    def _validate_active_config(self, config: dict) -> None:
        bit = self._active_bit(config.get("RoboInfo", {}).get("Bit"))
        if bit not in {"SIM", "IL"}:
            raise ValueError(
                "Unsupported RoboInfo.Bit=%r. Active Bits are SIM and IL; "
                "DM2325 is supported on the legacy branch; ArisuBits is retired. "
                "Select SIM or IL before starting LunaPre." % bit
            )

    @staticmethod
    def _validate_legacy_sensor(config: dict) -> None:
        """Reject retired hardware selection before changing its configuration."""
        source = str(config.get("Sensor", {}).get("Source", "auto")).strip().lower()
        backend = config.get("RoboInfo", {}).get("LidarBackend")
        if source not in {"auto", "bits"} or backend is not None:
            raise ValueError(
                "Native lidar configuration requires the legacy branch "
                "(ROS2 / LD07 / LD06). Main uses SIM or IL Bits pose."
            )

    def _create_default_config(self):
        try:
            DATA_DIR.mkdir(parents=True, exist_ok=True)
            with open(CONFIG_FILE, "w") as f:
                _json.dump(self.DEFAULT_CONFIG, f, indent=2)
        except Exception:
            raise FileNotFoundError("无法创建 config.json，请手动创建")

    def _needs_migration(self):
        """检查是否需要从旧配置迁移"""
        def has_extra_key(default, current):
            if not isinstance(default, dict) or not isinstance(current, dict):
                return False
            for key, value in current.items():
                if key not in default:
                    return True
                if has_extra_key(default[key], value):
                    return True
            return False

        for section in self.DEFAULT_CONFIG:
            if section not in self.cfg:
                return True
            for key in self.DEFAULT_CONFIG[section]:
                if key not in self.cfg.get(section, {}):
                    return True
        return has_extra_key(self.DEFAULT_CONFIG, self.cfg)

    def _migrate_config(self):
        """迁移旧配置到新格式"""
        # Reject retired sensor selections before changing the source file.
        with open(CONFIG_FILE, "r") as f:
            old_cfg = _json.load(f)
        self._validate_legacy_sensor(old_cfg)

        # 创建新配置，先保留现有新格式值，再兼容旧字段映射。
        new_cfg = _json.loads(_json.dumps(self.DEFAULT_CONFIG))

        def merge_existing(dst, src):
            for key, value in src.items():
                if key not in dst:
                    continue
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
            ("ExpectedVals", "MaxSpeedValue"): ("Chassis", "MaxWheelRPM"),
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

        # Only legacy camera mappings override the current Vision.CamPorts.
        vv = old_cfg.get("VisionVals")
        if isinstance(vv, dict):
            cam_ports = [
                vv.get("Back", 0), vv.get("Left", 2),
                vv.get("Front", 4), vv.get("Right", 6),
            ]
            new_cfg["Vision"]["CamPorts"] = list(dict.fromkeys(cam_ports))

        self._validate_active_config(new_cfg)

        # Keep the original only after the complete migrated config validates.
        os.replace(CONFIG_FILE, CACHE_FILE)

        # 写入新配置
        with open(CONFIG_FILE, "w") as f:
            _json.dump(new_cfg, f, indent=2)

        # 更新内存中的配置
        self.cfg = new_cfg

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
    def HailoModelName(self): 
        # 向后兼容：返回 ModelName + .hef 或旧配置 HailoModelPath
        model_name = getattr(self._s.Vision, "ModelName", None)
        if model_name:
            return f"/{model_name}.hef"
        return getattr(self._s.Vision, "HailoModelPath", "/ArisuModel_v1.2s.hef")
    @property
    def TunaModelName(self):
        # 向后兼容：返回 ModelName + .pt 或旧配置 ModelPath
        model_name = getattr(self._s.Vision, "ModelName", None)
        if model_name:
            return f"/{model_name}.pt"
        return getattr(self._s.Vision, "ModelPath", "")
    @property
    def TunaEnginePath(self):
        # 向后兼容：返回 ModelName + .engine 或旧配置 EnginePath
        model_name = getattr(self._s.Vision, "ModelName", None)
        if model_name:
            return f"/{model_name}.engine"
        return getattr(self._s.Vision, "EnginePath", "")
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
    def EnableBBoxDistanceEstimate(self): return False
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
    def CamFps(self): return getattr(self._s.Vision, "CamFps", getattr(self._s.Vision, "CaptureFps", 30))
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
    def MaxSpeedValue(self): return self._s.Chassis.MaxWheelRPM
    @property
    def MaxWarnCount(self): return 3


class _CompatSim:
    """向后兼容层：旧代码仍可读 Settings.Sim，但默认值由 XeleratorSimBits 管理。"""
    def __init__(self, settings):
        self._s = settings

    def _get(self, key, default):
        raw_sim = getattr(self._s._raw, "Sim", None)
        return getattr(raw_sim, key, default)

    @property
    def UnityIP(self): return self._get("UnityIP", "127.0.0.1")
    @property
    def RobotId(self): return self._get("RobotId", getattr(self._s.RoboInfo, "number", 1))
    @property
    def CmdPortBase(self): return self._get("CmdPortBase", self._get("PortBase", 20001))
    @property
    def StatusPortBase(self): return self._get("StatusPortBase", self.CmdPortBase + 1)
    @property
    def PortStride(self): return self._get("PortStride", 10)
    @property
    def MultiRobotMode(self): return self._get("MultiRobotMode", "per_port")
    @property
    def SceneControlPort(self): return self._get("SceneControlPort", 20100)
    @property
    def CmdPort(self): return self._get("CmdPort", self.CmdPortBase)
    @property
    def StatusPort(self): return self._get("StatusPort", self.StatusPortBase)
    @property
    def TwistFromWheels(self): return self._get("TwistFromWheels", True)
    @property
    def WheelToV(self): return self._get("WheelToV", 0.0029321531433504737)
    @property
    def WheelToWz(self): return self._get("WheelToWz", 0.0337524905204023)
    @property
    def YawOffsetDeg(self): return self._get("YawOffsetDeg", 0.0)
    @property
    def YawZeroAtStart(self): return self._get("YawZeroAtStart", False)
    @property
    def KickStrength(self): return self._get("KickStrength", 4.0)
    @property
    def DribbleStrengthScale(self): return self._get("DribbleStrengthScale", self._get("DribbleScale", 3.0))
    @property
    def PlanarMoveSpeedScale(self): return self._get("PlanarMoveSpeedScale", self._get("MoveScale", 1.0))
    @property
    def VisionNoiseLevel(self): return self._get("VisionNoiseLevel", 0.0)
    @property
    def VisionNoiseSeed(self): return self._get("VisionNoiseSeed", 0)
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
    def Chassis(self): return self._raw.Chassis
    @property
    def Motion(self): return self._raw.Motion
    @property
    def Sim(self): return getattr(self._raw, "Sim", self._sim_compat)
    @property
    def Debug(self): return self._raw.Debug
    # 向后兼容别名
    @property
    def CheckPoints(self):
        class _CP:
            Home = self._raw.Field.Home
        return _CP()


Settings: SettingsProxy = SettingsProxy()
