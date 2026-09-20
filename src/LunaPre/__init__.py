"""LunaPre 兼容门面包。

外部策略代码仍然可以继续使用 ``from LunaPre import *``。本包在 import 时
就急切初始化 SIM/IL 运行时：``LunaPre.chassis`` 等顶层名字直接是真实对象，
不做任何惰性代理，方便在调试器里直接查看和打断点。
"""

from __future__ import annotations

import atexit
import csv
import errno
import math
import os
import pickle
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

import simple_pid

from ReasonData import Settings, logger

from . import runtime as _runtime
from .communication import Communication
from .motion.chassis import Car, Key, Peripherals
from .motion.recording_chassis import RecordingCar
from .position_recorder import RecordingPositions
from .positions import Positions
from .referee import RefereeSnapshot
from .sensor import Sensor

cfg = _runtime.cfg
Beta_Func = _runtime.Beta_Func
SendstatusThreadFuncStarted = _runtime.SendstatusThreadFuncStarted
PeerstatusThreadFuncStarted = _runtime.PeerstatusThreadFuncStarted


UnitedPosition = RecordingPositions()


def _bind_runtime_objects(*, ensure: bool = True) -> None:
    """把 runtime 的真实对象绑定为本模块的全局名字（无惰性代理）。"""
    global Bits, lidar, sensor, chassis, compass, peripheral, key
    global Odometer, Morse, referee, vision
    if ensure:
        _runtime.ensure_runtime()
    Bits = _runtime.Bits
    lidar = _runtime.lidar
    sensor = _runtime.sensor
    chassis = _runtime.chassis
    compass = _runtime.compass
    peripheral = _runtime.peripheral
    key = _runtime.key
    Odometer = _runtime.Odometer
    Morse = _runtime.Morse
    referee = _runtime.referee
    vision = _runtime.vision


# Eager initialization: importing LunaPre builds the selected SIM/IL runtime
# up front instead of hiding it behind lazy proxies.
_bind_runtime_objects()


def _maybe_autostart_vision() -> None:
    """``XEL_AUTO_VISION=1`` 时 import 即启动视觉，便于 IDLE/交互式调试。

    默认关闭：测试和无硬件环境不应在 import 时拉起相机子进程。
    """
    requested = os.environ.get("XEL_AUTO_VISION", "").strip().lower() in {
        "1",
        "true",
        "yes",
        "on",
    }
    if requested and UnitedPosition.StartVision():
        logger.info("Vision auto-started on import (XEL_AUTO_VISION).")


_maybe_autostart_vision()


def _register_exit_cleanup() -> None:
    """解释器退出时兜底停车并回收 vision 子进程（IDLE/REPL 防孤儿与溜车）。"""
    def _lunapre_exit_cleanup():
        # At interpreter shutdown the log sinks (e.g. pytest capture files)
        # may already be closed; drop them first so late log writes cannot
        # crash the cleanup itself.
        try:
            remove = getattr(logger, "remove", None)
            if callable(remove):
                remove()
        except Exception:
            pass
        try:
            chassis_obj = globals().get("chassis")
            if chassis_obj is not None:
                chassis_obj.stop()
        except Exception:
            pass
        try:
            UnitedPosition.Stop()
        except Exception:
            pass
        try:
            _runtime.shutdown_runtime()
        except Exception:
            pass

    atexit.register(_lunapre_exit_cleanup)


_register_exit_cleanup()


def ensure_runtime(*, auto_start_lidar: bool = True) -> None:
    """确保运行时已初始化，并把本模块的全名绑定刷新到最新对象。"""
    _runtime.ensure_runtime(auto_start_lidar=auto_start_lidar)
    _bind_runtime_objects(ensure=False)


_init_runtime = ensure_runtime


def shutdown_runtime() -> None:
    """释放运行时资源，并把本模块的绑定刷新为 shutdown 后的状态。"""
    _runtime.shutdown_runtime()
    _bind_runtime_objects(ensure=False)


__all__ = [
    "math",
    "errno",
    "os",
    "pickle",
    "csv",
    "socket",
    "subprocess",
    "sys",
    "threading",
    "time",
    "Path",
    "simple_pid",
    "logger",
    "Settings",
    "Sensor",
    "Car",
    "Peripherals",
    "Key",
    "RecordingCar",
    "cfg",
    "_init_runtime",
    "Beta_Func",
    "SendstatusThreadFuncStarted",
    "PeerstatusThreadFuncStarted",
    "Positions",
    "RecordingPositions",
    "RefereeSnapshot",
    "Communication",
    "Bits",
    "lidar",
    "sensor",
    "chassis",
    "compass",
    "peripheral",
    "key",
    "UnitedPosition",
    "ensure_runtime",
    "shutdown_runtime",
]
