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
    if not DATA_DIR.exists():
        DATA_DIR.mkdir()
    if not LOG_FILE.exists():
        logger.add(LOG_FILE)
        logger.info('models.log is created successfully')
    # Migration: legacy vision.json -> new config.json (keep only vision-unique parts)
    if (not CONFIG_FILE.exists()) and LEGACY_CONFIG_FILE.exists():
        try:
            legacy = json.loads(LEGACY_CONFIG_FILE.read_text(encoding="utf-8"))
            # Keep only Debug overlay keys (avoid duplicating RoboInfo/VisionVals from system config)
            overlay = {}
            if isinstance(legacy, dict) and isinstance(legacy.get("Debug"), dict):
                overlay["Debug"] = {}
                for k in ["DebugWifi", "DebugIP", "DebugPSWD", "Database", "FullLog"]:
                    if k in legacy["Debug"]:
                        overlay["Debug"][k] = legacy["Debug"][k]
            if overlay:
                CONFIG_FILE.write_text(json.dumps(overlay, ensure_ascii=False), encoding="utf-8")
                logger.info("Migrated vision.json -> config.json (overlay only)")
        except Exception as e:
            logger.warning(f"Migration vision.json -> config.json failed: {e}")

    if not CONFIG_FILE.exists() and not LEGACY_CONFIG_FILE.exists():
        logger.info("vision config doesn't exist")
        logger.info("create vision config")

    from .config import Preference
    Preference().__init__()


AutoSetup()