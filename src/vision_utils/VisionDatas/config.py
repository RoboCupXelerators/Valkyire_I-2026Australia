import ujson
import os
from pathlib import Path

APP_DIR = Path(__file__).parent
DATA_DIR = APP_DIR / "data"
CONFIG_FILE = DATA_DIR / "vision.json"
CACHE_FILE = DATA_DIR /  "vision.cache.json"

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
                    "Bit" : "AB", # AB(Arisu Bits)/QB (3Q Bits RPI)/ 3Q (3Q Bits) .etc
                    "Rec" : "AI", # AI(Arisu Intelligence)/TI(Tuna Intelligence)/TV(Tuna Vision)/BB(Blob Based)
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
                "Debug": {
                    "DebugWifi": "RoboCup",
                    "DebugIP": "192.0.2.109",
                    "DebugPSWD": "12345",
                    "FullLog": True,
                    "Database": True,
                },
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
            self.Bit = 'AB'
            self.Rec = 'AI'

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

    class _debug(object):
        def __init__(self):
            self.DebugWifi = "RoboCup"
            self.DebugIP = "192.0.2.109"
            self.DebugPSWD = "RoboCup9"
            self.FullLog = True
            self.Database = True

    def __init__(self):
        self.RoboInfo = self._roboinfo()
        self.VisionVals = self._visionvals()
        self.Debug = self._debug()


Settings: Prompts = ParseJsonToObj()