"""vision_utils package entry.

Detect_Method selection is controlled by Settings.RoboInfo.Rec.

Compatibility note:
- Prefer the *system* settings from `ReasonData` (src/ReasonData/data/config.json).
- Fall back to the legacy vision-only config `vision_utils/VisionDatas/data/vision.json`.
"""

from .merged_settings import Settings

# logger is optional: prefer ReasonData logger if present, otherwise fallback to vision_utils logger
try:
    from ReasonData import logger as logger  # type: ignore
except Exception:  # pragma: no cover
    try:
        from .VisionDatas import logger as logger
    except Exception:  # pragma: no cover
        import logging

        logger = logging.getLogger(__name__)


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
"""vision_utils package entry.

Detect_Method selection is controlled by Settings.RoboInfo.Rec.

Compatibility note:
- Prefer the *system* settings from `ReasonData` (src/ReasonData/data/config.json).
- Fall back to the legacy vision-only config `vision_utils/VisionDatas/data/vision.json`.
"""

from .merged_settings import Settings

# logger is optional: prefer ReasonData logger if present, otherwise fallback to vision_utils logger
try:
    from ReasonData import logger as logger  # type: ignore
except Exception:  # pragma: no cover
    try:
        from .VisionDatas import logger as logger
    except Exception:  # pragma: no cover
        import logging

        logger = logging.getLogger(__name__)


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
    Detect_Method = _safe_import(f'{_PKG}.ArisuIntelligence', 'ArisuIntelligence')
elif Settings.RoboInfo.Rec == 'TI':
    Detect_Method = _safe_import(f'{_PKG}.TunaIntelligence', 'TunaIntelligence')
elif Settings.RoboInfo.Rec == 'TV':
    Detect_Method = _safe_import(f'{_PKG}.TunaVision', 'TunaVision')

if Detect_Method is None:
    # Fallback to pure OpenCV blob detection.
    from .Blob import BlobBased as Detect_Method
    Detect_Method = _safe_import(f'{_PKG}.TunaVision', 'TunaVision')
elif Settings.RoboInfo.Rec == 'XV':
    Detect_Method = _safe_import(f'{_PKG}.XeleratorVision', 'XeleratorVision')

if Detect_Method is None:
    # Fallback to pure OpenCV blob detection (optional).
    try:
        from .Blob import BlobBased as Detect_Method
    except Exception as e:
        logger.warning(f"vision_utils: BlobBased unavailable (likely missing cv2): {e}")
        # Last resort: try simulator vision adapter (no cv2 dependency)
        Detect_Method = _safe_import(f'{_PKG}.XeleratorVision', 'XeleratorVision')

# PreProcess depends on cv2; make it optional so SIM/dev machines can import vision_utils.
try:
    from .PreProcess import DATA_DIR
except Exception as e:  # pragma: no cover
    logger.warning(f"vision_utils: PreProcess unavailable (likely missing cv2): {e}")
    DATA_DIR = None

__version__ = '0.2.0'
# ReasonData 0.2.0(20250722A) Updated Preference instead QKjson


__all__ = ['ArisuIntelligence', 'TunaIntelligence', 'TunaVision', 'BlobBased', 'Detect_Method', 'DATA_DIR']
__all__ = ['ArisuIntelligence', 'TunaIntelligence', 'TunaVision', 'XeleratorVision', 'BlobBased', 'Detect_Method', 'DATA_DIR']
