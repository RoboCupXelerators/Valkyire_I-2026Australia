"""LunaPre 兼容门面包。

外部策略代码仍然可以继续使用 ``from LunaPre import *``。直接导入
``LunaPre.motion.chassis`` 这类子模块时保持轻量；硬件/SIM 运行时对象
会在从本包访问时再惰性初始化。
"""

from __future__ import annotations

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
from typing import Any, Callable, TYPE_CHECKING

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

_RUNTIME_NAMES = {
    "Bits",
    "lidar",
    "sensor",
    "chassis",
    "compass",
    "peripheral",
    "key",
    "Odometer",
    "Morse",
    "referee",
    "vision",
}


class _RuntimeObjectProxy:
    """历史 LunaPre 顶层运行时对象的惰性代理。"""

    def __init__(self, runtime_name: str):
        """记录需要代理的 runtime 对象名。"""
        object.__setattr__(self, "_runtime_name", runtime_name)

    def _resolve(self):
        """确保 runtime 已初始化，并返回真实对象。"""
        _runtime.ensure_runtime()
        return getattr(_runtime, self._runtime_name)

    def __getattr__(self, name: str):
        """把属性读取转发给真实 runtime 对象。"""
        return getattr(self._resolve(), name)

    def __setattr__(self, name: str, value) -> None:
        """把属性写入转发给真实 runtime 对象。"""
        if name == "_runtime_name":
            object.__setattr__(self, name, value)
            return
        setattr(self._resolve(), name, value)

    def __delattr__(self, name: str) -> None:
        """把属性删除转发给真实 runtime 对象。"""
        delattr(self._resolve(), name)

    def __call__(self, *args, **kwargs):
        """当真实对象可调用时，转发调用。"""
        return self._resolve()(*args, **kwargs)

    def __bool__(self) -> bool:
        """按真实对象的布尔值判断代理是否有效。"""
        return bool(self._resolve())

    def __repr__(self) -> str:
        """返回代理或真实对象的调试展示。"""
        runtime_name = object.__getattribute__(self, "_runtime_name")
        if getattr(_runtime, "_runtime_initialized", False):
            return repr(getattr(_runtime, runtime_name))
        return f"<lazy LunaPre.{runtime_name}>"


if TYPE_CHECKING:
    Bits: Any
    lidar: Sensor
    sensor: Sensor
    chassis: RecordingCar
    compass: Callable[[], float]
    peripheral: Peripherals
    key: Any
    Odometer: Any | None
    Morse: Any | None
    referee: Any | None
    vision: Any | None
else:
    Bits = _RuntimeObjectProxy("Bits")
    lidar = _RuntimeObjectProxy("lidar")
    sensor = _RuntimeObjectProxy("sensor")
    chassis = _RuntimeObjectProxy("chassis")
    compass = _RuntimeObjectProxy("compass")
    peripheral = _RuntimeObjectProxy("peripheral")
    key = _RuntimeObjectProxy("key")


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


# Stable proxies keep previously imported objects valid across runtime restarts.
ensure_runtime = _runtime.ensure_runtime
_init_runtime = ensure_runtime
shutdown_runtime = _runtime.shutdown_runtime


def __getattr__(name: str):
    """Resolve optional runtime objects only when explicitly requested."""
    if name in _RUNTIME_NAMES:
        _runtime.ensure_runtime()
        return getattr(_runtime, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
