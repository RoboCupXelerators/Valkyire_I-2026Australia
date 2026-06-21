"""LunaPre compatibility package.

External strategy code can keep using ``from LunaPre import *``.  Importing
submodules such as ``LunaPre.motion.chassis`` stays lightweight; runtime
hardware/SIM objects are initialized lazily when requested from this package.
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

import simple_pid

from ReasonData import Settings, logger

from . import runtime as _runtime
from .communication import Communication
from .motion.chassis import Car, Key, Peripherals
from .motion.recording_chassis import RecordingCar
from .position_recorder import RecordingPositions
from .positions import Positions
from .sensor import Lidar

cfg = _runtime.cfg
Beta_Func = _runtime.Beta_Func
SendstatusThreadFuncStarted = _runtime.SendstatusThreadFuncStarted
PeerstatusThreadFuncStarted = _runtime.PeerstatusThreadFuncStarted

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
]


def _sync_runtime_globals() -> None:
    for name in _RUNTIME_NAMES:
        globals()[name] = getattr(_runtime, name)


def _ensure_runtime() -> None:
    if not getattr(_runtime, "_runtime_initialized", False):
        _runtime._init_runtime(auto_start_lidar=True)
    _sync_runtime_globals()
    if "UnitedPosition" not in globals():
        globals()["UnitedPosition"] = RecordingPositions()


def _init_runtime(*, auto_start_lidar: bool = True):
    result = _runtime._init_runtime(auto_start_lidar=auto_start_lidar)
    _sync_runtime_globals()
    return result


def __getattr__(name: str):
    if name in _RUNTIME_NAMES or name == "UnitedPosition":
        _ensure_runtime()
        return globals()[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
