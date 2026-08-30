"""vision_utils.merged_settings

This module provides a merged Settings object for the vision stack.

Goal
----
Keep *one* system config as the source of truth (ReasonData/data/config.json)
while allowing vision-only overrides without duplicating common fields.

Files
-----
- System base: ReasonData.Preference().cfg
- Vision overlay: vision_utils/VisionDatas/data/config.json
  (fallback legacy: vision_utils/VisionDatas/data/vision.json)

The returned `Settings` is a ConfigSection-like object (attribute access),
independent from `ReasonData.Settings` (does not mutate global state).
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict

# ujson is optional on dev machines; fall back to stdlib json.
try:
    import ujson as _json  # type: ignore
except ModuleNotFoundError:
    import json as _json


class ConfigSection:
    """Dict -> object converter (recursive)."""

    def __init__(self, data: Dict[str, Any]):
        for key, value in (data or {}).items():
            if isinstance(value, dict):
                setattr(self, key, ConfigSection(value))
            else:
                setattr(self, key, value)


def _deep_merge(base: Dict[str, Any], overlay: Dict[str, Any]) -> Dict[str, Any]:
    out: Dict[str, Any] = dict(base or {})
    for k, v in (overlay or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)  # type: ignore[arg-type]
        else:
            out[k] = v
    return out


def _load_json(path: Path) -> Dict[str, Any]:
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = _json.load(f)
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def _camera_port(vision: Dict[str, Any], index: int, fallback: int) -> int:
    ports = vision.get("CamPorts", [])
    try:
        return int(ports[index])
    except Exception:
        return int(fallback)


def _ensure_legacy_vision_vals(config: Dict[str, Any]) -> None:
    """Populate VisionVals from the newer Vision section for legacy vision code."""
    vision = config.get("Vision") if isinstance(config.get("Vision"), dict) else {}
    existing = config.get("VisionVals") if isinstance(config.get("VisionVals"), dict) else {}
    compat = {
        "Back": _camera_port(vision, 0, existing.get("Back", 0)),
        "Left": _camera_port(vision, 1, existing.get("Left", 2)),
        "Front": _camera_port(vision, 2, existing.get("Front", 4)),
        "Right": _camera_port(vision, 3, existing.get("Right", 6)),
        "Record": bool(vision.get("Record", existing.get("Record", False))),
        "RecordDir": vision.get("RecordDir", existing.get("RecordDir", "./Records")),
        "RecordCams": existing.get("RecordCams", [0, 1, 2, 3]),
        "RecordWidth": existing.get("RecordWidth", 320),
        "RecordHeight": existing.get("RecordHeight", 320),
        "RecordSegmentSeconds": existing.get("RecordSegmentSeconds", 60),
        "RecordFPS": existing.get("RecordFPS", 30),
        "RecordCodec": existing.get("RecordCodec", "mp4v"),
        "RecordExt": existing.get("RecordExt", "mp4"),
        "ExposeVal": existing.get("ExposeVal", 100),
        "AutoExpose": existing.get("AutoExpose", 0),
        "CalibrationFolder": vision.get("CalibrationFolder", existing.get("CalibrationFolder", "Calibration")),
        "HailoModelPath": vision.get("HailoModelPath", existing.get("HailoModelPath", "/yoloV8_Weighs/")),
        "TunaModelName": vision.get("ModelPath", existing.get("TunaModelName", "")),
        "TunaEnginePath": vision.get("EnginePath", existing.get("TunaEnginePath", "")),
        "BallDiameterCM": vision.get("BallDiameter", existing.get("BallDiameterCM", 4.2)),
        "BallOffsetXCM": vision.get("BallOffsetXCM", existing.get("BallOffsetXCM", 0.0)),
        "BallOffsetYCM": vision.get("BallOffsetYCM", existing.get("BallOffsetYCM", 0.0)),
        "ChassisWidthCM": vision.get("ChassisWidth", existing.get("ChassisWidthCM", 18.0)),
        "ChassisHeightCM": vision.get("ChassisHeight", existing.get("ChassisHeightCM", 5.0)),
        "BlueGoalWidthCM": vision.get("GoalWidth", existing.get("BlueGoalWidthCM", 90.0)),
        "BlueGoalHeightCM": vision.get("GoalHeight", existing.get("BlueGoalHeightCM", 18.0)),
        "YellowGoalWidthCM": vision.get("GoalWidth", existing.get("YellowGoalWidthCM", 90.0)),
        "YellowGoalHeightCM": vision.get("GoalHeight", existing.get("YellowGoalHeightCM", 18.0)),
        "CaptureBackend": existing.get("CaptureBackend", "auto"),
        "CaptureCodec": existing.get("CaptureCodec", "mjpg"),
        "CamWidth": existing.get("CamWidth", 640),
        "CamHeight": existing.get("CamHeight", 480),
        "CamFps": existing.get("CamFps", 30),
    }
    compat.update(existing)
    config["VisionVals"] = compat


def get_settings_dict() -> Dict[str, Any]:
    # Base from ReasonData when available
    base: Dict[str, Any] = {}
    try:
        # Installed package layout
        from ReasonData.config import Preference as SysPreference  # type: ignore

        base = SysPreference().cfg
    except Exception:
        # Repo layout (modules under ./src)
        try:
            from src.ReasonData.config import Preference as SysPreference  # type: ignore

            base = SysPreference().cfg
        except Exception:
            base = {}

    if not base:
        # Last-resort file fallback. Importing ReasonData may fail in minimal
        # vision-only environments when optional logging deps are absent, but
        # the config file is still the source of truth.
        reason_config = Path(__file__).resolve().parents[1] / "ReasonData" / "data" / "config.json"
        base = _load_json(reason_config)

    # Overlay from vision override config
    app_dir = Path(__file__).parent
    overlay_dir = app_dir / "VisionDatas" / "data"
    overlay_file = overlay_dir / "config.json"
    legacy_file = overlay_dir / "vision.json"

    overlay: Dict[str, Any] = {}
    if overlay_file.exists():
        overlay = _load_json(overlay_file)
    elif legacy_file.exists():
        overlay = _load_json(legacy_file)
    else:
        overlay = {}

    # If no base (standalone vision), ensure minimum defaults so callers don't crash.
    if not base:
        base = {
            "RoboInfo": {"number": 1, "type": "OP", "Bit": "AB", "Rec": "AI"},
            "Vision": {
                "CamPorts": [0, 2, 4, 6],
                "ModelPath": "",
                "EnginePath": "",
                "HailoModelPath": "/yoloV8_Weighs/",
                "CalibrationFolder": "Calibration",
                "BallDiameter": 4.2,
                "ChassisWidth": 18.0,
                "ChassisHeight": 5.0,
                "GoalWidth": 90.0,
                "GoalHeight": 18.0,
                "MinBallBBoxArea": 40,
                "BallOffsetXCM": 0.0,
                "BallOffsetYCM": 0.0,
                "AdaptiveFrameDistribution": True,
                "CameraMinYoloHz": 4.0,
                "CameraFocusYoloHz": 20.0,
                "CameraFocusHoldMs": 250.0,
                "YoloMaxFramesPerTick": 4,
                "Record": False,
                "RecordDir": "./Records",
            },
            "VisionVals": {
                "Front": 4,
                "Right": 6,
                "Back": 0,
                "Left": 2,
                "Record": False,
                "ExposeVal": 100,
                "AutoExpose": 0,
                "CalibrationFolder": "/Calibration/",
                "HailoModelPath": "/yoloV8_Weighs/",
            },
            "Debug": {"FullLog": False, "Beta": False},
        }

    merged = _deep_merge(base, overlay)
    _ensure_legacy_vision_vals(merged)
    return merged


def get_settings() -> ConfigSection:
    return ConfigSection(get_settings_dict())


# Public singleton
Settings = get_settings()
