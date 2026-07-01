import importlib.util
import sys
import types
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_boolean_dribble_keeps_legacy_raw_output_after_mapping(monkeypatch):
    reasondata_stub = types.ModuleType("ReasonData")
    config_stub = types.ModuleType("ReasonData.config")

    class DummySettings:
        class Ports:
            ElecMagnet = 3
            Dribble = 4

    class DummyLogger:
        def info(self, *args, **kwargs):
            return None

        def warning(self, *args, **kwargs):
            return None

    reasondata_stub.Settings = DummySettings()
    reasondata_stub.logger = DummyLogger()
    config_stub.Settings = reasondata_stub.Settings

    monkeypatch.setitem(sys.modules, "ReasonData", reasondata_stub)
    monkeypatch.setitem(sys.modules, "ReasonData.config", config_stub)

    luna_pkg = types.ModuleType("LunaPre")
    luna_pkg.__path__ = [str(ROOT / "LunaPre")]
    motion_pkg = types.ModuleType("LunaPre.motion")
    motion_pkg.__path__ = [str(ROOT / "LunaPre" / "motion")]
    monkeypatch.setitem(sys.modules, "LunaPre", luna_pkg)
    monkeypatch.setitem(sys.modules, "LunaPre.motion", motion_pkg)

    spec = importlib.util.spec_from_file_location(
        "LunaPre.motion.chassis", ROOT / "LunaPre" / "motion" / "chassis.py"
    )
    chassis = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    monkeypatch.setitem(sys.modules, "LunaPre.motion.chassis", chassis)
    spec.loader.exec_module(chassis)

    calls = []
    peripheral = chassis.Peripherals(lambda port, status: calls.append((port, status)))

    peripheral.Dribble(True)

    assert calls[-1] == (DummySettings.Ports.Dribble, 0.5)
