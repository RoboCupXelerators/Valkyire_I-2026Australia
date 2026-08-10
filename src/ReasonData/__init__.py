"""ReasonData

提供：
- logger（loguru）
- Settings / Preference（配置）
- DATA_DIR / LOG_FILE（运行数据目录）
- ROS_ENV（可选：ROS2 setup.bash 路径）

注意：开发机/无 ROS 环境下也应允许导入。
"""

from __future__ import annotations

from . import setup
from .config import Preference, Settings

import time
import os
import tempfile

Date = time.strftime("%Y%m", time.localtime())
__version__ = "0.2.1"

from loguru import logger
from pathlib import Path

APP_DIR = Path(__file__).parent
PROJECT_DIR = APP_DIR.parent.parent
DATA_DIR = APP_DIR / "data"
_runtime_log_dir = os.environ.get("XEL_RUNTIME_LOG_DIR")
LOG_DIR = Path(_runtime_log_dir) if _runtime_log_dir else PROJECT_DIR / "log"
try:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
except OSError:
    if _runtime_log_dir:
        raise
    LOG_DIR = Path(tempfile.mkdtemp(prefix="reasondata-log-"))
LOG_FILE = LOG_DIR / f"{Date}.log"
try:
    logger.add(
        LOG_FILE,
        rotation="5 MB",
        retention="1 days",
        encoding="utf-8",
        backtrace=True,
        diagnose=True,
        enqueue=True,
        catch=True)
except OSError:
    if _runtime_log_dir:
        raise
    LOG_DIR = Path(tempfile.mkdtemp(prefix="reasondata-log-"))
    LOG_FILE = LOG_DIR / f"{Date}.log"
    logger.add(
        LOG_FILE,
        rotation="5 MB",
        retention="1 days",
        encoding="utf-8",
        backtrace=True,
        diagnose=True,
        enqueue=True,
        catch=True)
logger.info("ReasonData(%s) loaded."%__version__)

# ROS2 环境（可选）。
# - 在比赛机（Linux + ROS2）通常存在 /opt/ros
# - 在开发机（macOS/Windows）不存在时不应阻断导入
ROS_ENV: str | None = None
base = "/opt/ros"
if os.path.isdir(base):
    setups: list[tuple[str, str]] = []
    for distro in os.listdir(base):
        setup_path = os.path.join(base, distro, "setup.bash")
        if os.path.isfile(setup_path):
            setups.append((distro, setup_path))
    if setups:
        ROS_ENV = setups[0][1]
    else:
        logger.warning("ReasonData: '/opt/ros' exists but no setup.bash found.")
else:
    logger.warning("ReasonData: ROS2 not found at /opt/ros (this is OK on dev machines).")

__all__ = [
    "Preference",
    "setup",
    "Settings",
    "logger",
    "DATA_DIR",
    "LOG_DIR",
    "ROS_ENV",
]
