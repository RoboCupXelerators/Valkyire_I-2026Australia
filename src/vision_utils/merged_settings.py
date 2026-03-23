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


def get_settings_dict() -> Dict[str, Any]:
    # Base from ReasonData when available
    base: Dict[str, Any] = {}
    try:
        from ReasonData.config import Preference as SysPreference  # type: ignore

        base = SysPreference().cfg
    except Exception:
        base = {}

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

    return _deep_merge(base, overlay)


def get_settings() -> ConfigSection:
    return ConfigSection(get_settings_dict())


# Public singleton
Settings = get_settings()
