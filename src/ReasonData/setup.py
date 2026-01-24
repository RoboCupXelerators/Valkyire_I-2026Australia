# config.py | IntelliFusion Version 0.2.2(2023092000) Developer Alpha
from pathlib import Path
import json
import time
import os


from loguru import logger

Date = time.strftime("%Y%m", time.localtime())
APP_DIR = Path(__file__).parent
DATA_DIR = APP_DIR / "data"
CONFIG_FILE = DATA_DIR / "config.json"
LOG_FILE = DATA_DIR / f"{Date}.log"

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
    # from .data import SetupDatabase
    # RoBotName = Preference().read("model","number")
    # DATABASE_FILE = DATA_DIR / f"Xel-{RoBotName}.{Date}.sqlite"
    # if not DATABASE_FILE.exists():
    #     SetupDatabase()
    #     logger.info("Database is created successfully!")


AutoSetup()