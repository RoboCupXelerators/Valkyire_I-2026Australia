"""vision_utils package entry.

Detect_Method selection is controlled by Settings.RoboInfo.Rec.

Compatibility note:
- Prefer the *system* settings from `ReasonData` (src/ReasonData/data/config.json).
- Fall back to the legacy vision-only config `vision_utils/VisionDatas/data/vision.json`.
"""

from .merged_settings import Settings


def _safe_import(path: str, name: str):
    """Import helper to avoid hard-crash when optional accelerators are absent."""
    try:
        mod = __import__(path, fromlist=[name])
        return getattr(mod, name)
    except Exception as e:
        logger.warning(f"vision_utils: failed to import {path}.{name}: {e}")
        return None


# Select backend by config, but tolerate missing optional dependencies.
Detect_Method = None
_PKG = __name__  # could be 'vision_utils' or 'src.vision_utils'
if Settings.RoboInfo.Rec == 'AI':
    Detect_Method = _safe_import(f'{_PKG}.ArisuIntelligence', 'ArisuIntelligence')
elif Settings.RoboInfo.Rec == 'TI':
    Detect_Method = _safe_import(f'{_PKG}.TunaIntelligence', 'TunaIntelligence')
elif Settings.RoboInfo.Rec == 'TV':
    Detect_Method = _safe_import(f'{_PKG}.TunaVision', 'TunaVision')

if Detect_Method is None:
    # Fallback to pure OpenCV blob detection.
    from .Blob import BlobBased as Detect_Method

from .PreProcess import DATA_DIR

__version__ = '0.2.0'
# ReasonData 0.2.0(20250722A) Updated Preference instead QKjson


__all__ = ['ArisuIntelligence', 'TunaIntelligence', 'TunaVision', 'BlobBased', 'Detect_Method', 'DATA_DIR']
