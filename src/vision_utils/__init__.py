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
        if not logger.handlers:
            logger.addHandler(logging.NullHandler())


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
# Prefer package name over module name so direct imports like
# `import src.vision_utils.__init__` still resolve sibling modules correctly.
_PKG = __package__ or __name__.rpartition('.')[0] or __name__
_REC = str(getattr(Settings.RoboInfo, "Rec", "")).upper()

if _REC == 'AI':
    Detect_Method = _safe_import(f'{_PKG}.ArisuIntelligence', 'ArisuIntelligence')
elif _REC == 'TI':
    Detect_Method = _safe_import(f'{_PKG}.TunaIntelligence', 'TunaIntelligence')
elif _REC == 'XV':
    Detect_Method = _safe_import(f'{_PKG}.XeleratorVision', 'XeleratorVision')
elif _REC in ('SY', 'SIMYOLO'):
    Detect_Method = _safe_import(f'{_PKG}.LunaIntelligence', 'SimYoloVision')
elif _REC == 'TV':
    logger.warning("vision_utils: Rec='TV' has been removed. Use 'TI', 'XV', or 'SY'.")

if Detect_Method is None:
    # Raspberry Pi / missing-accelerator fallback: use the pure OpenCV blob detector.
    try:
        from .Blob import BlobBased as Detect_Method
    except Exception as e:
        logger.warning(f"vision_utils: BlobBased unavailable (likely missing cv2): {e}")
        Detect_Method = None

if Detect_Method is None and _REC == 'XV':
    # Simulator fallback when the requested backend is unavailable.
    Detect_Method = _safe_import(f'{_PKG}.XeleratorVision', 'XeleratorVision')

# PreProcess depends on cv2; make it optional so SIM/dev machines can import vision_utils.
try:
    from .PreProcess import DATA_DIR
except Exception as e:  # pragma: no cover
    logger.warning(f"vision_utils: PreProcess unavailable (likely missing cv2): {e}")
    DATA_DIR = None

__version__ = '0.2.0'
# ReasonData 0.2.0(20250722A) Updated Preference instead QKjson


__all__ = ['ArisuIntelligence', 'TunaIntelligence', 'XeleratorVision', 'SimYoloVision', 'BlobBased', 'Detect_Method', 'DATA_DIR']
