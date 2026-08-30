# config.py | IntelliFusion Version 0.2.2(2023092000) Developer Alpha
from pathlib import Path
from loguru import logger
import json
import time

Date = time.strftime("%Y%m", time.localtime())
APP_DIR = Path(__file__).parent
DATA_DIR = APP_DIR / "data"
LEGACY_CONFIG_FILE = DATA_DIR / "vision.json"
CONFIG_FILE = DATA_DIR / "config.json"  # vision override (new)
LOG_FILE = DATA_DIR / f"vision-{Date}.log"

def AutoSetup():
    """Keep legacy imports read-only; writes require Preference.write()."""
    if not CONFIG_FILE.exists() and LEGACY_CONFIG_FILE.exists():
        logger.info("Legacy vision.json detected; it remains a read-only fallback.")


AutoSetup()
