"""LunaPre

本文件提供上层策略常用的封装：
- 🤖 Positions：坐标/目标位置计算（机体坐标系 & 场地坐标系）
- 📡 Communication：队内通信（Wi-Fi / BLE 等）

命名规范（与 SoccerVisonWiki/ProgramingAdvanced.md 对齐）：
- ✅ 对外方法：PascalCase（大驼峰）
- ✅ 变量：snake_case（蛇形）

兼容性：为了不一次性炸掉历史代码，旧接口会保留为“别名/包装”。

Runtime owns hardware/provider selection. Positions consumes runtime-level
semantic providers such as field pose and vision; it should not reach into
concrete Bits implementations.
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
_field_pose_provider = None
_referee_status_provider = None
_runtime_initialized = False

from .sensor import Lidar
from .motion.chassis import Car, Peripherals, Key  # Universal-Movement-Standard
from .motion.recording_chassis import RecordingCar


def _make_chassis(set_motor, get_yaw, *, permission=None, motor_encoder=None):
    """Build the motion facade from plain callables, not concrete Bits classes."""

    return RecordingCar(
        set_motor,
        get_yaw,
        permission=permission,
        motor_encoder=motor_encoder,
    )


def get_field_pose():
    """Return runtime-provided field pose [x_cm, y_cm, yaw_deg], or None."""

    if _field_pose_provider is None:
        return None
    x, y, yaw = _field_pose_provider()
    return [round(float(x), 2), round(float(y), 2), float(yaw)]


def get_referee_status() -> bool:
    """Return referee/unlock state without exposing the concrete Bits object."""

    if _referee_status_provider is None:
        return False
    return bool(_referee_status_provider())


def default_vision_enabled() -> bool:
    enable_vision_env = os.environ.get("ROBOCUPX_ENABLE_VISION", "").lower() in {"1", "true", "yes"}
    if str(getattr(cfg.RoboInfo, "Bit", "")).upper() == "SIM":
        return enable_vision_env
    return True


# Bits 线程监控器相关变量
_bits_monitor_thread = None
_bits_monitor_stop = None


def _start_bits_thread_monitor(bits_obj):
    """启动后台线程监控器，确保 Bits 命令线程崩溃后能自动重启。
    
    Args:
        bits_obj: Bits 实例（需要有 IsThreadAlive 和 RestartThreadIfNeeded 方法）
    """
    global _bits_monitor_thread, _bits_monitor_stop
    
    # 检查 Bits 对象是否支持线程监控接口
    if not hasattr(bits_obj, 'IsThreadAlive') or not hasattr(bits_obj, 'RestartThreadIfNeeded'):
        logger.debug("Bits object does not support thread monitoring interface")
        return
    
    # 如果已经有监控线程在运行，先停止它
    if _bits_monitor_stop is not None:
        _bits_monitor_stop.set()
    
    _bits_monitor_stop = threading.Event()
    
    def _monitor_loop():
        """监控循环，定期检查线程状态并在需要时重启。"""
        while not _bits_monitor_stop.is_set():
            try:
                if not bits_obj.IsThreadAlive():
                    logger.warning("Bits background thread is dead, attempting restart...")
                    bits_obj.RestartThreadIfNeeded()
            except Exception as e:
                logger.exception("Bits thread monitor error: %s", e)
            # 每 1 秒检查一次
            _bits_monitor_stop.wait(1.0)
    
    _bits_monitor_thread = threading.Thread(
        target=_monitor_loop,
        daemon=True,
        name="Bits-ThreadMonitor"
    )
    _bits_monitor_thread.start()
    logger.info("Bits thread monitor started")


def _stop_bits_thread_monitor():
    """停止 Bits 线程监控器。"""
    global _bits_monitor_stop, _bits_monitor_thread
    
    if _bits_monitor_stop is not None:
        _bits_monitor_stop.set()
        _bits_monitor_stop = None
    
    if _bits_monitor_thread is not None:
        try:
            _bits_monitor_thread.join(timeout=2.0)
        except Exception:
            pass
        _bits_monitor_thread = None
    logger.info("Bits thread monitor stopped")


def _init_runtime(*, auto_start_lidar: bool = True):
    """初始化 Bits / chassis / peripheral / lidar 等全局对象。

    支持的硬件类型：
    - SIM: Unity 仿真器 (XeleratorSimBits + XeleratorVision)
    - AB: Arisu Bit (比赛机)
    - IL: IceLoong Bit (比赛机)
    """

    global Bits, lidar, chassis, compass, peripheral, key, Odometer, Morse, referee, vision
    global _field_pose_provider, _referee_status_provider, _runtime_initialized

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
            from utils.XeleratorSimBits import create_bits_and_lidar
            from vision_utils.XeleratorVision import XeleratorVision

            Bits, lidar = create_bits_and_lidar(cfg)
            vision = XeleratorVision(Bits)
            chassis = _make_chassis(Bits.SetWheelSpeed, Bits.GetYaw)
            compass = Bits.GetYaw
            peripheral = Peripherals(Bits.VirtualPorts)
            key = Key(Bits.GetKey)
            Odometer = None
            referee = None
            Morse = None
            _field_pose_provider = Bits.GetPose
            _referee_status_provider = None
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
            chassis = _make_chassis(
                Bits.SetMotor,
                Bits.GetYaw,
                motor_encoder=Bits.get_motor_encoder,
            )
            compass = Bits.GetYaw
            peripheral = Peripherals(Bits.SetIO)
            key = Key(Bits.GetKey)
            Odometer = None
            referee = None
            Morse = None
            _field_pose_provider = getattr(Bits, "GetPose", None)
            _referee_status_provider = lambda: bool(getattr(Bits, "referee_key", False))
            logger.info("Arisu Bit loaded.")
            _runtime_initialized = True
            return

        if cfg.RoboInfo.Bit == "IL":
            from utils.IceLoongBits import IceLoongBits

            Bits = IceLoongBits(port=getattr(cfg.Ports, "IceLoongPort", "/dev/iceloong"))
            lidar = Lidar(Bits.GetYaw, auto_start=auto_start_lidar)
            peripheral = Peripherals(Bits.VirtualPorts, Bits.GetRefereeUnlockPin)  # TODO
            chassis = _make_chassis(
                Bits.SetWheelSpeed,
                Bits.GetYaw,
                permission=Bits.GetRefereeUnlockPin,
            )
            compass = Bits.GetYaw
            key = Bits.GetKey
            referee = Bits.GetRefereeUnlockPin
            Morse = Bits.MosePrint
            Odometer = None  # TODO add odometer support
            _field_pose_provider = getattr(Bits, "GetPose", None)
            _referee_status_provider = Bits.GetRefereeUnlockPin
            
            # 启动后台线程监控器，确保 IceLoongBits 命令线程崩溃后能自动重启
            _start_bits_thread_monitor(Bits)
            
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
