import importlib.util
import json
from pathlib import Path

import pytest


CONFIG_MODULE_PATH = Path(__file__).parents[1] / "ReasonData" / "config.py"


def _load_config_module(tmp_path, monkeypatch):
    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir()
    monkeypatch.setenv("XEL_RUNTIME_DATA_DIR", str(runtime_dir))
    spec = importlib.util.spec_from_file_location("isolated_reason_data_config", CONFIG_MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.DATA_DIR = tmp_path
    module.CONFIG_FILE = tmp_path / "config.json"
    module.CACHE_FILE = tmp_path / "cfg.cache.json"
    return module


def test_legacy_ros2_config_migrates_to_sensor_section(tmp_path, monkeypatch):
    module = _load_config_module(tmp_path, monkeypatch)
    old_config = {
        "RoboInfo": {
            "Bit": "SIM",
            "LidarBackend": "ros2",
            "LidarType": "s1",
            "LidarInverted": True,
        }
    }
    module.CONFIG_FILE.write_text(json.dumps(old_config), encoding="utf-8")

    config = module.Preference().cfg

    assert config["Sensor"] == {
        "Source": "ros2",
        "Ros2Model": "s1",
        "Inverted": True,
    }
    persisted = json.loads(module.CONFIG_FILE.read_text(encoding="utf-8"))
    assert "LidarBackend" not in persisted["RoboInfo"]
    assert "DM2325Port" not in persisted["Ports"]


def test_direct_rplidar_config_fails_without_rewriting_the_old_config(tmp_path, monkeypatch):
    module = _load_config_module(tmp_path, monkeypatch)
    old_config = {
        "RoboInfo": {"Bit": "SIM", "LidarBackend": "rplidar", "LidarType": "s3"}
    }
    serialized = json.dumps(old_config)
    module.CONFIG_FILE.write_text(serialized, encoding="utf-8")

    with pytest.raises(ValueError, match="Direct RPLidar is no longer supported"):
        module.Preference()

    assert module.CONFIG_FILE.read_text(encoding="utf-8") == serialized
    assert not module.CACHE_FILE.exists()


@pytest.mark.parametrize("legacy_bit", ["AB", "DM2325"])
def test_archived_bit_modes_fail_explicitly(tmp_path, monkeypatch, legacy_bit):
    module = _load_config_module(tmp_path, monkeypatch)
    config = json.loads(json.dumps(module.Preference.DEFAULT_CONFIG))
    config["RoboInfo"]["Bit"] = legacy_bit
    module.CONFIG_FILE.write_text(json.dumps(config), encoding="utf-8")

    with pytest.raises(ValueError, match="Active Bits are SIM and IL"):
        module.Preference()
