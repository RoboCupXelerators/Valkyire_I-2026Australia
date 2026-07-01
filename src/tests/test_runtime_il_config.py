import importlib.util
import sys
import types
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_il_runtime_uses_configured_iceloong_port(monkeypatch):
    captured = {}

    reasondata_stub = types.ModuleType("ReasonData")

    class DummySettings:
        class RoboInfo:
            Bit = "IL"

        class Ports:
            IceLoongPort = "/dev/iceloong"

        class Debug:
            FullLog = False
            RecordMotion = False

    class DummyLogger:
        def info(self, *args, **kwargs):
            return None

        def error(self, *args, **kwargs):
            return None

        def debug(self, *args, **kwargs):
            return None

    reasondata_stub.Settings = DummySettings()
    reasondata_stub.logger = DummyLogger()
    monkeypatch.setitem(sys.modules, "ReasonData", reasondata_stub)
    monkeypatch.setitem(sys.modules, "simple_pid", types.ModuleType("simple_pid"))

    luna_pkg = types.ModuleType("LunaPre")
    luna_pkg.__path__ = [str(ROOT / "LunaPre")]
    monkeypatch.setitem(sys.modules, "LunaPre", luna_pkg)

    sensor_stub = types.ModuleType("LunaPre.sensor")

    class FakeLidar:
        def __init__(self, get_yaw, auto_start=True):
            self.get_yaw = get_yaw
            self.auto_start = auto_start

    sensor_stub.Lidar = FakeLidar
    monkeypatch.setitem(sys.modules, "LunaPre.sensor", sensor_stub)

    chassis_stub = types.ModuleType("LunaPre.motion.chassis")

    class FakePeripherals:
        def __init__(self, *args, **kwargs):
            return None

    class FakeKey:
        def __init__(self, *args, **kwargs):
            return None

    chassis_stub.Car = object
    chassis_stub.Peripherals = FakePeripherals
    chassis_stub.Key = FakeKey
    monkeypatch.setitem(sys.modules, "LunaPre.motion.chassis", chassis_stub)

    recording_stub = types.ModuleType("LunaPre.motion.recording_chassis")

    class FakeRecordingCar:
        def __init__(self, *args, **kwargs):
            return None

        def start_state_update(self):
            return None

    recording_stub.RecordingCar = FakeRecordingCar
    monkeypatch.setitem(sys.modules, "LunaPre.motion.recording_chassis", recording_stub)

    bits_stub = types.ModuleType("utils.IceLoongBits")

    class FakeIceLoongBits:
        def __init__(self, port="/dev/ttyUSB0", *args, **kwargs):
            captured["port"] = port

        def GetYaw(self):
            return 0.0

        def VirtualPorts(self, port, status):
            return None

        def SetWheelSpeed(self, *args):
            return None

        def GetRefereeUnlockPin(self):
            return True

        def GetKey(self):
            return False

        def MosePrint(self, msg):
            return None

    bits_stub.IceLoongBits = FakeIceLoongBits
    monkeypatch.setitem(sys.modules, "utils.IceLoongBits", bits_stub)

    spec = importlib.util.spec_from_file_location("LunaPre.runtime", ROOT / "LunaPre" / "runtime.py")
    runtime = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    monkeypatch.setitem(sys.modules, "LunaPre.runtime", runtime)
    spec.loader.exec_module(runtime)

    runtime._init_runtime(auto_start_lidar=False)

    assert captured["port"] == "/dev/iceloong"
