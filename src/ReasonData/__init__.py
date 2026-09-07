"""ReasonData

提供：
- logger（loguru）
- Settings / Preference（配置）
- DATA_DIR / LOG_FILE（运行数据目录）

导入仅初始化配置和日志，不探测硬件环境。
"""

from __future__ import annotations

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
DATA_DIR = Path(os.environ.get("XEL_RUNTIME_DATA_DIR", APP_DIR / "data"))
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

__all__ = [
    "Preference",
    "Settings",
    "logger",
    "DATA_DIR",
    "LOG_DIR",
]
