from .config import Preference, Settings
from pathlib import Path

__version__ = '0.2.0'
# ReasonData 0.2.0(20250722A) Updated Preference instead QKjson
try:
    from ReasonData import logger
except Exception:  # pragma: no cover
    import logging

    logger = logging.getLogger(__name__)
    if not logger.handlers:
        logger.addHandler(logging.NullHandler())

APP_DIR = Path(__file__).parent
DATA_DIR = APP_DIR / "data"
__all__ = ['Preference', 'Settings', 'logger', "DATA_DIR"]
