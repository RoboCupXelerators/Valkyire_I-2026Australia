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


@pytest.mark.parametrize("source", ["ros2", "ld07", "ld06", "rplidar"])
def test_native_sensor_config_requires_legacy_without_rewriting(tmp_path, monkeypatch, source):
    module = _load_config_module(tmp_path, monkeypatch)
    serialized = json.dumps({"RoboInfo": {"Bit": "IL"}, "Sensor": {"Source": source}})
    module.CONFIG_FILE.write_text(serialized, encoding="utf-8")
    with pytest.raises(ValueError, match="legacy branch"):
        module.Preference()
    assert module.CONFIG_FILE.read_text(encoding="utf-8") == serialized
    assert not module.CACHE_FILE.exists()


def test_old_native_backend_requires_legacy_without_rewriting(tmp_path, monkeypatch):
    module = _load_config_module(tmp_path, monkeypatch)
    serialized = json.dumps({"RoboInfo": {"Bit": "SIM", "LidarBackend": "ros2"}})
    module.CONFIG_FILE.write_text(serialized, encoding="utf-8")
    with pytest.raises(ValueError, match="legacy branch"):
        module.Preference()
    assert module.CONFIG_FILE.read_text(encoding="utf-8") == serialized


@pytest.mark.parametrize("source", ["auto", "bits"])
def test_bits_config_removes_obsolete_scan_settings_preserving_tuning(tmp_path, monkeypatch, source):
    module = _load_config_module(tmp_path, monkeypatch)
    old = json.loads(json.dumps(module.Preference.DEFAULT_CONFIG))
    old["Sensor"] = {"Source": source, "Ros2Model": "s3", "Inverted": True}
    old["Ports"]["LidarPort"] = "/dev/sllidar"
    old["Vision"]["CamPorts"] = [6, 4, 2, 0]
    old["Motion"]["ErrorRange"] = 42
    module.CONFIG_FILE.write_text(json.dumps(old), encoding="utf-8")
    config = module.Preference().cfg
    assert config["Vision"]["CamPorts"] == [6, 4, 2, 0]
    assert "Sensor" not in config
    assert "LidarPort" not in config["Ports"]
    assert config["Motion"]["ErrorRange"] == 42
    assert json.loads(module.CACHE_FILE.read_text()) == old


@pytest.mark.parametrize("legacy_bit", ["AB", "DM2325"])
def test_archived_bit_modes_fail_explicitly(tmp_path, monkeypatch, legacy_bit):
    module = _load_config_module(tmp_path, monkeypatch)
    config = json.loads(json.dumps(module.Preference.DEFAULT_CONFIG))
    config["RoboInfo"]["Bit"] = legacy_bit
    module.CONFIG_FILE.write_text(json.dumps(config), encoding="utf-8")

    with pytest.raises(ValueError, match="Active Bits are SIM and IL"):
        module.Preference()
