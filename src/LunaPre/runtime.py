"""LunaPre

本文件提供上层策略常用的封装：
- 🤖 Positions：坐标/目标位置计算（机体坐标系 & 场地坐标系）
- 📡 Communication：队内通信（Wi-Fi / BLE 等）

命名规范（与 SoccerVisonWiki/ProgramingAdvanced.md 对齐）：
- ✅ 对外方法：PascalCase（大驼峰）
- ✅ 变量：snake_case（蛇形）

兼容性：为了不一次性炸掉历史代码，旧接口会保留为“别名/包装”。

TODO lower-controller compatibility:
- Positions should prefer Bits.GetPose() when the lower controller provides
  fused localization [x, y, yaw].
- If Bits.GetPose() is absent or invalid, keep falling back to Sensor.Lidar
  distance parsing through DirectDistance().
- Pos2Pos remains in the upper controller and should continue using the pose
  returned by FieldRobotPos(); chassis.py is not part of this compatibility pass.
- Do not expose this source selection to strategy code.
"""

from __future__ import annotations

import math
import errno
import os
import pickle
import csv
import socket
import subprocess
import sys
import threading
import time
import simple_pid
from pathlib import Path

from ReasonData import logger, Settings

cfg = Settings

Bits = None
lidar = None
chassis = None
compass = None
peripheral = None
key = None
Odometer = None
Morse = None
referee = None
vision = None
_runtime_ball_state_is_authoritative = False
_runtime_initialized = False

from .sensor import Lidar
from .motion.chassis import Car, Peripherals, Key  # Universal-Movement-Standard
from .motion.recording_chassis import RecordingCar


def _start_runtime_state_sync(car: Car) -> None:
    try:
        car.start_state_update()
    except Exception as e:
        logger.debug(f"Failed to start robot_state sync: {e}")


def get_runtime_ball_pos():
    """Return ball position from the active runtime provider when available."""

    global _runtime_ball_state_is_authoritative

    _runtime_ball_state_is_authoritative = False
    getter = getattr(Bits, "GetBallPos", None)
    if not callable(getter):
        return None

    ball_pos = getter()
    _runtime_ball_state_is_authoritative = _ball_source_preserves_possession_state()
    return ball_pos


def runtime_ball_state_is_authoritative() -> bool:
    return _runtime_ball_state_is_authoritative


def _ball_source_preserves_possession_state() -> bool:
    return str(getattr(cfg.RoboInfo, "Bit", "")).upper() == "SIM"


def default_vision_enabled() -> bool:
    enable_vision_env = os.environ.get("ROBOCUPX_ENABLE_VISION", "").lower() in {"1", "true", "yes"}
    if str(getattr(cfg.RoboInfo, "Bit", "")).upper() == "SIM":
        return enable_vision_env
    return True


def _init_runtime(*, auto_start_lidar: bool = True):
    """初始化 Bits / chassis / peripheral / lidar 等全局对象。

    支持的硬件类型：
    - SIM: Unity 仿真器 (XeleratorSimBits + XeleratorVision)
    - AB: Arisu Bit (比赛机)
    - IL: IceLoong Bit (比赛机)
    """

    global Bits, lidar, chassis, compass, peripheral, key, Odometer, Morse, referee, vision, _runtime_initialized

    if _runtime_initialized:
        return

    try:
        # Simulator runtime: UDP bridge to Unity (SoccerSimulatorForXels)
        # Enable via config: Settings.RoboInfo.Bit == "SIM"
        # Or env override: ROBOCUPX_BITS=SIM
        bits_override = os.environ.get("ROBOCUPX_BITS", "").strip().upper()
        if bits_override:
            try:
                cfg.RoboInfo.Bit = bits_override
            except Exception:
                pass

        if str(cfg.RoboInfo.Bit).upper() == "SIM":
            from utils.XeleratorSimBits import create_runtime

            Bits, lidar, vision = create_runtime(cfg)
            chassis = RecordingCar(Bits.SetWheelSpeed, Bits.GetYaw)
            _start_runtime_state_sync(chassis)
            compass = Bits.GetYaw
            peripheral = Peripherals(Bits.VirtualPorts)
            key = Key(Bits.GetKey)
            Odometer = None
            referee = None
            Morse = None
            logger.info(
                f"Simulator Bits loaded: unity={Bits.unity_ip}:{Bits.cmd_port}, "
                f"status_port={Bits.status_port}, robot_id={Bits.robot_id}"
            )
            _runtime_initialized = True
            return

        if cfg.RoboInfo.Bit == "AB":
            logger.error("Initializing Arisu Bit... Nice to See You Again, Arisu!")
            from utils.ArisuBits import ArisBit

            Bits = ArisBit()
            lidar = Lidar(Bits.GetYaw, auto_start=auto_start_lidar)
            chassis = RecordingCar(Bits.SetMotor, Bits.GetYaw, Bits.get_motor_encoder)
            _start_runtime_state_sync(chassis)
            compass = Bits.GetYaw
            peripheral = Peripherals(Bits.SetIO)
            key = Key(Bits.GetKey)
            Odometer = None
            referee = None
            Morse = None
            logger.info("Arisu Bit loaded.")
            _runtime_initialized = True
            return

        if cfg.RoboInfo.Bit == "IL":
            from utils.IceLoongBits import IceLoongBits

            Bits = IceLoongBits(port=getattr(cfg.Ports, "IceLoongPort", "/dev/iceloong"))
            lidar = Lidar(Bits.GetYaw, auto_start=auto_start_lidar)
            peripheral = Peripherals(Bits.VirtualPorts, Bits.GetRefereeUnlockPin)  # TODO
            chassis = RecordingCar(Bits.SetWheelSpeed, Bits.GetYaw, Bits.GetRefereeUnlockPin)
            _start_runtime_state_sync(chassis)
            compass = Bits.GetYaw
            key = Bits.GetKey
            referee = Bits.GetRefereeUnlockPin
            Morse = Bits.MosePrint
            Odometer = None  # TODO add odometer support
            logger.info("IceLoongBits Bit loaded.")
            _runtime_initialized = True
            return

        raise ImportError(f"Unknown Bits model: {cfg.RoboInfo.Bit}. Supported: SIM, AB, IL")

    except Exception as e:
        logger.error(f"Failed to init runtime: {e}")
        raise RuntimeError(
            f"Runtime initialization failed. Ensure config.RoboInfo.Bit is set correctly (SIM/AB/IL). Error: {e}"
        ) from e


Beta_Func = False

# Thread start flags (avoid NameError)
SendstatusThreadFuncStarted = False
PeerstatusThreadFuncStarted = False
