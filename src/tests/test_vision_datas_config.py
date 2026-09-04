from vision_utils.VisionDatas import config as legacy_config
from vision_utils.merged_settings import _ensure_legacy_vision_vals


def test_exposure_settings_are_forwarded_to_legacy_camera_config():
    config = {"Vision": {"AutoExposure": 1, "Exposure": 78}}

    _ensure_legacy_vision_vals(config)

    assert config["VisionVals"]["AutoExposure"] == 1
    assert config["VisionVals"]["Exposure"] == 78


def test_legacy_vision_preference_does_not_create_overlay_on_import(monkeypatch, tmp_path):
    config_file = tmp_path / "config.json"
    monkeypatch.setattr(legacy_config, "DATA_DIR", tmp_path)
    monkeypatch.setattr(legacy_config, "CONFIG_FILE", config_file)
    monkeypatch.setattr(legacy_config, "LEGACY_CONFIG_FILE", tmp_path / "vision.json")
    monkeypatch.setattr(legacy_config, "CACHE_FILE", tmp_path / "vision.cache.json")

    legacy_config.Preference()

    assert not config_file.exists()
