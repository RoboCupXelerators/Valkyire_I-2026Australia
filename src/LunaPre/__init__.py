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
from .sensor import Lidar

cfg = _runtime.cfg
Beta_Func = _runtime.Beta_Func
SendstatusThreadFuncStarted = _runtime.SendstatusThreadFuncStarted
PeerstatusThreadFuncStarted = _runtime.PeerstatusThreadFuncStarted
chassis = _runtime.chassis
peripheral = _runtime.peripheral


UnitedPosition = RecordingPositions()

_RUNTIME_NAMES = {
    "Bits",
    "lidar",
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
        _ensure_runtime()
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
    lidar: Lidar
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
    "Lidar",
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
    "chassis",
    "compass",
    "peripheral",
    "key",
    "Odometer",
    "Morse",
    "referee",
    "vision",
    "UnitedPosition",
    "ensure_runtime",
]


def _sync_runtime_globals() -> None:
    """把 runtime 模块里的真实对象同步回 LunaPre 顶层命名空间。"""
    for name in _RUNTIME_NAMES:
        globals()[name] = getattr(_runtime, name)


def _ensure_runtime() -> None:
    """确保 runtime 初始化完成，并保证 UnitedPosition 存在。"""
    if not getattr(_runtime, "_runtime_initialized", False):
        _runtime._init_runtime(auto_start_lidar=True)
    _sync_runtime_globals()
    if "UnitedPosition" not in globals():
        globals()["UnitedPosition"] = RecordingPositions()


def _init_runtime(*, auto_start_lidar: bool = True):
    """显式初始化 runtime，并同步顶层兼容对象。"""
    result = _runtime._init_runtime(auto_start_lidar=auto_start_lidar)
    _sync_runtime_globals()
    return result


def ensure_runtime(*, auto_start_lidar: bool = True) -> None:
    """Initialize LunaPre providers before reading semantic observations."""

    if not getattr(_runtime, "_runtime_initialized", False):
        _runtime._init_runtime(auto_start_lidar=auto_start_lidar)
    _sync_runtime_globals()


def __getattr__(name: str):
    """按需解析历史顶层 runtime 名称。"""
    if name in _RUNTIME_NAMES or name == "UnitedPosition":
        _ensure_runtime()
        return globals()[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
