from . import setup
from .config import Preference, Settings
# from .data import Positions, Date

import time
Date = time.strftime("%Y%m", time.localtime())
__version__ = '0.2.0'
# ReasonData 0.2.0(20250722A) Updated Preference instead QKjson
try:
    from loguru import logger
    from pathlib import Path
except ImportError:
    raise

APP_DIR = Path(__file__).parent
DATA_DIR = APP_DIR / "data"
LOG_FILE = DATA_DIR / f"{Date}.log"
logger.add(
    LOG_FILE,
    rotation="1 MB",
    retention="1 days",
    encoding="utf-8",
    backtrace=True,
    diagnose=True,
    enqueue=True,
    catch=True)
logger.info("ReasonData(%s) loaded."%__version__)

import os
base = "/opt/ros"
if not os.path.isdir(base):
    raise ImportError

setups = []
for distro in os.listdir(base):
    setup = os.path.join(base, distro, "setup.bash")
    if os.path.isfile(setup):
        setups.append((distro, setup))

ROS_ENV = setups[0][1]

__all__ = ['Preference', 'Positions', 'setup', 'Settings', 'logger', "DATA_DIR",'ROS_ENV']
