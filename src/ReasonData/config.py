import ujson
import os
from pathlib import Path

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
                    "LidarID": 88,
                    "IceLoongPort": '/dev/ttyUSB0',
                    "LowTigger" : 0, 
                    "ReferPort": 1,
                    "ElecMagnet" : 3,
                    "Dribble" : 4,
                    "LeftFront" : 1,
                    "LeftBack" : 2,
                    "RightFront" : 3,
                    "RightBack" : 4,
                },
                "VisionVals": {
                    'Front' : 4,
                    'Right' : 6,
                    'Back' : 0,
                    'Left' : 2,
                    "Record" : False, # False to Disable Record video
                    "ExposeVal" : 100, # 0-255, 0: Auto, 1-255: Manual
                    "AutoExpose" : 0,  # 0: Manual, 1: Auto
                    "CalibrationFolder": '/Calibration/',
                    "HailoModelPath": '/yoloV8_Weighs/',
                },
                "Bounds" : {
                    "ClsPos": [85,35,0], #Bound CheckPoint (Close to [0,0])
                    "FarPos": [95,75,0], #Bound CheckPoint (Far from [0,0])
                    "Short" : 200, # length of short side
                    "Long" : 260, # length of longer side
                },
                "CheckPoints" : {
                    "Home": [-70,0,0],
                },
                "ExpectedVals": {
                    "CatchVal" : [10,0], # Position when Robo Cathch the ball
                    "ErrorRange": 10,
                    "MaxWarnCount": 3,
                    "MaxSpeedValue": 500,
                },
                "Transimission": {
                    "Type" : "Slave", # master to start AP/ slave to connect
                    "Method" : "WIFI", # WIFI/BLE/Refer
                },
                "BLE" : {
                    "MAC" : "NONE",
                    "REMOTE" : "NONE",
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
                    "Database": False,
                }
            }
        try:
            os.stat(CONFIG_FILE)
        except:
            try:
                with open(CONFIG_FILE, "w") as f:
                    ujson.dump(data, f)
            except:
                raise FileNotFoundError("Create config.json Failed, please delete config.json and try again.")
        with open(CONFIG_FILE) as f:
            self.cfg = ujson.load(f)
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
                self.cache = ujson.load(ca)
            with open(CONFIG_FILE, "w") as d:
                ujson.dump(data, d)
            with open(CONFIG_FILE, "r") as d:
                self.cfg = ujson.load(d)
            for i in data:
                for c in data[i].keys():
                    try:
                        vCache = self.cache[str(i)][str(c)]
                        self.cfg[str(i)][str(c)] = vCache
                        with open(CONFIG_FILE, "w") as d:
                            ujson.dump(self.cfg, d)
                    except:
                        pass
            os.remove(CACHE_FILE)

    def write(self, section: str, option: str, value: int) -> int:
        self.cfg[section][option] = value
        with open(CONFIG_FILE, "w") as f:
            ujson.dump(self.cfg, f)

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
            self.ExposeVal = 100
            self.AutoExpose = 0
            self.CalibrationFolder = '/Calibration/'
            self.ModelPath = '/yoloV8_Weighs/'

    class _bounds(object):
        def __init__(self):
            self.ClsPos = [85,35,0]
            self.FarPos = [95,75,0]
            self.Short = 200
            self.Long = 250

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
            self.Database = False

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


Settings: Prompts = ParseJsonToObj()