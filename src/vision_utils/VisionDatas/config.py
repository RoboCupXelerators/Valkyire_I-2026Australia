"""vision_utils.VisionDatas.config

Historically vision_utils shipped its own independent config file
`vision_utils/VisionDatas/data/vision.json`.

To reduce duplicated config between the main system (ReasonData) and vision,
we now support a *two-layer* configuration model:

- Base: system config `src/ReasonData/data/config.json` (when available)
- Overlay: vision-only override `vision_utils/VisionDatas/data/config.json`

Overlay is merged into base recursively (dict deep-merge).

Backward compatibility:
- If `config.json` does not exist, but legacy `vision.json` exists,
  we still load it.
- Existing callers importing `vision_utils.VisionDatas.Settings` keep working.
"""

import os
from pathlib import Path
from typing import Any, Dict

try:
    import ujson  # type: ignore
except ModuleNotFoundError:  # pragma: no cover
    import json as ujson  # type: ignore

APP_DIR = Path(__file__).parent
DATA_DIR = APP_DIR / "data"
LEGACY_CONFIG_FILE = DATA_DIR / "vision.json"
CONFIG_FILE = DATA_DIR / "config.json"  # vision override
CACHE_FILE = DATA_DIR / "vision.cache.json"


def _deep_merge(base: Dict[str, Any], overlay: Dict[str, Any]) -> Dict[str, Any]:
    """Deep merge overlay into base (overlay wins)."""
    out: Dict[str, Any] = dict(base)
    for k, v in (overlay or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)  # type: ignore[arg-type]
        else:
            out[k] = v
    return out

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
        # default *overlay* (vision-only). Keep it minimal to avoid duplicating
        # the system config.
        default_overlay: Dict[str, Any] = {
            "Debug": {
                "Database": True,
            }
        }

        # 1) Load base config from ReasonData when available.
        base: Dict[str, Any] = {}
        try:
            from ReasonData.config import Preference as SysPreference  # type: ignore

            base = SysPreference().cfg
        except Exception:
            # When ReasonData isn't available (running vision_utils standalone),
            # keep base empty and rely on legacy vision.json / overlay defaults.
            base = {}

        # 2) Load overlay from vision config.json (preferred) or legacy vision.json.
        overlay: Dict[str, Any] = {}
        if CONFIG_FILE.exists():
            try:
                with open(CONFIG_FILE) as f:
                    overlay = ujson.load(f)
            except Exception:
                overlay = {}
        elif LEGACY_CONFIG_FILE.exists():
            try:
                with open(LEGACY_CONFIG_FILE) as f:
                    overlay = ujson.load(f)
            except Exception:
                overlay = {}
        else:
            # Importing a compatibility module must not create a local config.
            # Persist defaults only through Preference.write().
            overlay = dict(default_overlay)

        # 3) Ensure overlay has required keys (non-destructive update).
        need_write = False
        for sec, sec_data in default_overlay.items():
            if sec not in overlay or not isinstance(overlay.get(sec), dict):
                overlay[sec] = dict(sec_data)
                need_write = True
            else:
                for k, v in sec_data.items():
                    if k not in overlay[sec]:
                        overlay[sec][k] = v
                        need_write = True

        # 4) Merge and expose
        self.cfg = _deep_merge(base, overlay)

    def write(self, section: str, option: str, value: int) -> int:
        """Write a value into the *vision override* config.json.

        Note: We intentionally do NOT write back to the system config.
        """
        # Load current overlay
        overlay: Dict[str, Any] = {}
        if CONFIG_FILE.exists():
            try:
                with open(CONFIG_FILE) as f:
                    overlay = ujson.load(f)
            except Exception:
                overlay = {}

        overlay.setdefault(section, {})
        if not isinstance(overlay[section], dict):
            overlay[section] = {}
        overlay[section][option] = value

        with open(CONFIG_FILE, "w") as f:
            ujson.dump(overlay, f)

        # Update merged cfg in-memory
        self.cfg = _deep_merge(self.cfg, {section: {option: value}})

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
            self.FullLog = True
            self.Database = True

    def __init__(self):
        self.RoboInfo = self._roboinfo()
        self.VisionVals = self._visionvals()
        self.Debug = self._debug()


Settings: Prompts = ParseJsonToObj()
