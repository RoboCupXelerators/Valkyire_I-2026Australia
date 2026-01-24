# config.py | IntelliFusion Version 0.2.2(2023092000) Developer Alpha
from pathlib import Path
from loguru import logger
import json
import time

Date = time.strftime("%Y%m", time.localtime())
APP_DIR = Path(__file__).parent
DATA_DIR = APP_DIR / "data"
CONFIG_FILE = DATA_DIR / "vision.json"
LOG_FILE = DATA_DIR / f"vision-{Date}.log"

def AutoSetup():
    if not DATA_DIR.exists():
        DATA_DIR.mkdir()
    if not LOG_FILE.exists():
        logger.add(LOG_FILE)
        logger.info('models.log is created successfully')
    if not CONFIG_FILE.exists():
        logger.info("config.json doesn't exist")
        logger.info("create config.json")
    from .config import Preference
    Preference().__init__()


AutoSetup()