"""ReasonData.config

本模块原本依赖 ujson。
在开发机/CI 环境中可能未安装 ujson，导致整个 SoccerSoftware 无法启动。
这里做一个向后兼容：优先用 ujson，没有则回退到标准库 json。
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
CACHE_FILE = DATA_DIR /  "cfg.cache.json"

# kits
class ConfigSection:
    """用于表示配置的每个部分"""
    def __init__(self, data):
        for key, value in data.items():
            if isinstance(value, dict):
                # 如果值是字典，创建新的ConfigSection实例
                setattr(self, key, ConfigSection(value))
            else:
                # 否则直接设置属性
                setattr(self, key, value)

def ParseJsonToObj():
    parseData = Preference().cfg
    result = ConfigSection(parseData)
    return result


class Preference:
    def __init__(self):
        data = {
                "RoboInfo": {
                    "number" : 1,
                    "type": "OP", # OffencePlayer/DefencePlayer
                    "Bit" : "IL", # AB(Arisu Bits)/IL (IceLoong Bits)/ 3Q (3Q Bits) .etc
                    "Rec" : "AI", # AI(Arisu Intelligence)/TV(Tuna Vision)/BB(Blob Based)
                    "LidarType": "s1", # "s1" or "s2" to select lidar launch file
                },
                "Ports": {
                    "LidarID": random.randrange(1, 99),  # Random ID for Lidar (if used)
                    "IceLoongPort": '/dev/iceloong',
                    "LowTigger" : 0, 
                    "ReferPort": 1,
                    "ElecMagnet" : 3,
                    "Dribble" : 4,
                    "LeftFront" : 1,
                    "LeftBack" : 2,
                    "RightBack" : 3,
                    "RightFront" : 4,
                },
                "VisionVals": {
                    'Front' : 4,
                    'Right' : 6,
                    'Back' : 0,
                    'Left' : 2,
                    "Record" : False, # False to Disable Record video
                    "RecordDir": "./Records", # where to save recordings (relative to run cwd)
                    "RecordCams": [0, 1, 2, 3], # which camera indices to record (0:Front,1:Right,2:Back,3:Left)
                    "RecordWidth": 320, # recording output width (writer size)
                    "RecordHeight": 320, # recording output height (writer size)
                    "RecordSegmentSeconds": 60, # split video into segments (seconds)
                    "RecordFPS": 30, # writer fps (not necessarily camera fps)
                    "RecordCodec": "mp4v", # mp4 codec: mp4v is widely supported; h264 may require extra deps
                    "RecordExt": "mp4", # file extension
                    "ExposeVal" : 100, # 0-255, 0: Auto, 1-255: Manual
                    "AutoExpose" : 0,  # 0: Manual, 1: Auto
                    "CalibrationFolder": '/Calibration/',
                    "HailoModelName": '/ArisuModel_v1.2s.hef',
                    # TunaIntelligence (Ultralytics/TensorRT) model paths
                    # - Use absolute path or path relative to repo root / working directory.
                    # - Engine is optional; if missing TunaIntelligence can export (FP16) on-device.
                    "TunaModelName": "/yolov8nForX5.pt",
                    "TunaEnginePath": "/yolov8nForX5.engine",

                    # Bounding-box based distance estimation (cm).
                    # These are fallback physical sizes used with camera intrinsics.
                    # Tune them for your robot/model if you want better range estimates.
                    "EnableBBoxDistanceEstimate": True,
                    "BallDiameterCM": 4.2,
                    "ChassisWidthCM": 18.0,
                    "ChassisHeightCM": 18.0,
                    "BlueGoalWidthCM": 90.0,
                    "BlueGoalHeightCM": 18.0,
                    "YellowGoalWidthCM": 90.0,
                    "YellowGoalHeightCM": 18.0,
                },
                "Bounds" : {
                    "ClsPos": [85,35,0], #Bound CheckPoint (Close to [0,0])
                    "FarPos": [95,75,0], #Bound CheckPoint (Far from [0,0])
                    "Short" : 200, # length of short side
                    "Long" : 260, # length of longer side
                    "Distance2Bound" : 240,
                    "restricted_area_x": 400,
                    "restricted_area_y": 800,
                },
                "CheckPoints" : {
                    "Home": [-70,0,0],
                },
                "ExpectedVals": {
                    "CatchVal" : [6,0], # Position when Robo Cathch the ball
                    "ErrorRange": 10,
                    "MaxWarnCount": 3,
                    "MaxSpeedValue": 500,
                },
                "Transimission": {
                    "Type" : "Slave", # master to start AP/ slave to connect
                    "Method" : "WIFI", # WIFI/BLE/Refer
                },
                "WIFI" : {
                    "SelfIP" : "192.168.1.x",
                    "RemoteIP" : "192.168.1.x",
                    "Port" : 20001,
                    "SSID" : "MisakaNetwork",
                    "PWD" : "MisakaNetwork20001/",
                },
                "Debug": {
                    "FullLog": False,
                    "Beta": True,
                },
                "Sim": {
                    "UnityIP": "127.0.0.1",
                    "CmdPort": 20001,
                    "StatusPort": 20002,
                    "RobotId": 1,
                    "TwistFromWheels": True,
                    "WheelToV": 0.008,
                    "WheelToWz": 0.02,
                    "YawOffsetDeg": 0.0,
                    "YawZeroAtStart": False,
                    "KickStrength": 1.0,
                    "DribbleStrengthScale": 1.0,
                    "PlanarMoveSpeedScale": 1.0,
                    "UnityToCm": 100.0,
                    "OriginXCM": 0.0,
                    "OriginYCM": 0.0,
                    "PoseScale": 1.0,
                    "PoseOffsetXCM": 0.0,
                    "PoseOffsetYCM": 0.0,
                    "Debug": False,
                    "CmdPortBase": 20001,
                    "StatusPortBase": 20002,
                    "PortStride": 10,
                    "MultiRobotMode": "per_port"
                }
            }
        try:
            os.stat(CONFIG_FILE)
        except:
            try:
                with open(CONFIG_FILE, "w") as f:
                    _json.dump(data, f)
            except:
                raise FileNotFoundError("Create config.json Failed, please delete config.json and try again.")
        with open(CONFIG_FILE) as f:
            self.cfg = _json.load(f)
            bUpdate = False
            for i in data:
                for c in data[i].keys():
                    try:
                        self.cfg[str(i)][str(c)]
                    except:
                        bUpdate = True
        if bUpdate:
            os.rename(CONFIG_FILE,CACHE_FILE)
            with open(CACHE_FILE, "r") as ca:
                self.cache = _json.load(ca)
            with open(CONFIG_FILE, "w") as d:
                _json.dump(data, d)
            with open(CONFIG_FILE, "r") as d:
                self.cfg = _json.load(d)
            for i in data:
                for c in data[i].keys():
                    try:
                        vCache = self.cache[str(i)][str(c)]
                        self.cfg[str(i)][str(c)] = vCache
                        with open(CONFIG_FILE, "w") as d:
                            _json.dump(self.cfg, d)
                    except:
                        pass
            os.remove(CACHE_FILE)

    def write(self, section: str, option: str, value: int) -> int:
        self.cfg[section][option] = value
        with open(CONFIG_FILE, "w") as f:
            _json.dump(self.cfg, f)

    def read(self, section: str, option: str) -> int:
        try:
            return self.cfg[section][option]
        except KeyError:
            self.__init__()

class Prompts(object):
    class _roboinfo(object):
        def __init__(self):
            self.number = 0
            self.type = 'OP'
            self.Bit = 'IL'
            self.Rec = 'AI'
            self.LidarType = 's1'  # default lidar type: 's1' or 's2'

    class _ports(object):
        def __init__(self):
            self.LowTigger = 0
            self.IceLoongPort = '/dev/ttyUSB0'
            self.ReferPort = 1
            self.ElecMagnet = 3
            self.Dribble = 4
            self.LidarID = 99
            self.LeftFront = 1
            self.LeftBack = 2
            self.RightFront = 3
            self.RightBack = 4

    class _visionvals(object):
        def __init__(self):
            self.Front = 4,
            self.Right = 6,
            self.Back = 0,
            self.Left = 2,
            self.Record = False
            self.RecordDir = "./Records"
            self.RecordCams = [0, 1, 2, 3]
            self.RecordWidth = 320
            self.RecordHeight = 320
            self.RecordSegmentSeconds = 60
            self.RecordFPS = 30
            self.RecordCodec = "mp4v"
            self.RecordExt = "mp4"
            self.ExposeVal = 100
            self.AutoExpose = 0
            self.CalibrationFolder = '/Calibration/'
            self.HailoModelName = '/yoloV8_Weighs/'
            # TunaIntelligence model paths
            self.TunaModelName = "yolov8nForX5.pt"
            self.TunaEngineName = "yolov8nForX5.engine"

    class _bounds(object):
        def __init__(self):
            self.ClsPos = [85,35,0]
            self.FarPos = [95,75,0]
            self.Short = 200
            self.Long = 250
            self.Distance2Bound = 240
            self.restricted_area_x = 400
            self.restricted_area_y = 800

    class _checkpoints(object):
        def __init__(self):
            self.Home = [-70,0,0]

    class _expectedvals(object):
        def __init__(self):
            self.CatchVal = [10,0]
            self.ErrorRange = 10
            self.MaxWarnCount = 3
            self.MaxSpeedValue = 500

    class _transimission(object):
        def __init__(self):
            self.Type = "Slave"
            self.Method = "WIFI"

    class _ble(object):
        def __init__(self):
            self.MAC = "NONE"
            self.REMOTE = "NONE"

    class _wifi(object):
        def __init__(self):
            self.SelfIP = "192.168.1.x"
            self.RemoteIP = "192.168.1.x"
            self.Port = 20001
            self.SSID = "MisakaNetwork"
            self.PWD = "MisakaNetwork20001/"

    class _debug(object):
        def __init__(self):
            self.FullLog = False
            self.Beta = False

    class _sim(object):
        def __init__(self):
            # Simulator (Unity) connection settings
            self.UnityIP = "127.0.0.1"
            # Single-robot explicit ports
            self.CmdPort = 20001
            self.StatusPort = 20002
            # If 0/None, will fallback to RoboInfo.number
            self.RobotId = 1
            # Kick strength is fixed per simulator config (triggered by any non-zero kick command)
            self.KickStrength = 1.0
            # Dribble strength = config scale * incoming 0..1 command
            self.DribbleStrengthScale = 1.0
            # Scale planar movement commands (vx, vy) in SIM to tame chasing overshoot
            self.PlanarMoveSpeedScale = 1.0
            # Multi-robot helpers (per-robot port mapping)
            # robotId -> cmdPort = CmdPortBase + (robotId-1)*PortStride
            # robotId -> statusPort = StatusPortBase + (robotId-1)*PortStride
            self.CmdPortBase = 20001
            self.StatusPortBase = 20002
            self.PortStride = 10
            # per_port: each robot listens on its own command port
            # hub: (future) all robots share one command port via a Unity hub component
            self.MultiRobotMode = "per_port"

    def __init__(self):
        self.RoboInfo = self._roboinfo()
        self.Ports = self._ports()
        self.VisionVals = self._visionvals()
        self.Bounds = self._bounds()
        self.CheckPoints = self._checkpoints()
        self.ExpectedVals = self._expectedvals()
        self.Transimission = self._transimission()
        self.BLE = self._ble()
        self.WIFI = self._wifi()
        self.Debug = self._debug()
        self.Sim = self._sim()


Settings: Prompts = ParseJsonToObj()