"""Own SIM/IL providers, initialization and shutdown.

Strategies consume semantic providers through Positions and Sensor. Hardware
construction stays here; imports alone do not start a runtime.
"""

from __future__ import annotations

import os
import threading

from ReasonData import logger, Settings
from .referee import IoRefereeProvider, RefereeSnapshot, SerialRefereeProvider

cfg = Settings

Bits = None
lidar = None
sensor = None
chassis = None
compass = None
peripheral = None
key = None
Odometer = None
Morse = None
referee = None
vision = None
_field_pose_provider = None
_velocity_provider = None
_referee_status_provider = None
_referee_state_provider = None
_referee_serial_provider = None
_runtime_initialized = False
_runtime_lock = threading.RLock()

from .sensor import Sensor, coerce_pose
from .motion.chassis import Peripherals, Key
from .motion.recording_chassis import RecordingCar


def play_morse(message: str) -> None:
    """Play an optional status message when the active backend supports it."""

    callback = Morse
    if callable(callback):
        callback(message)


def get_field_pose():
    """Return the valid Bits pose as ``[x_cm, y_cm, yaw_deg]``, or ``None``."""

    if _field_pose_provider is None:
        return None
    try:
        pose = coerce_pose(_field_pose_provider())
    except Exception:
        return None
    if pose is None:
        return None
    x_cm, y_cm, yaw_deg = pose
    return [round(x_cm, 2), round(y_cm, 2), yaw_deg]


def get_referee_status() -> bool:
    """Return referee/unlock state without exposing the concrete Bits object."""

    return bool(get_referee_state().control_enabled)


def get_referee_state() -> RefereeSnapshot:
    """Return LunaPre's deployment-safe referee snapshot."""

    if _referee_state_provider is None:
        return RefereeSnapshot()
    try:
        return _referee_state_provider()
    except Exception:
        return RefereeSnapshot(source="provider-error")


def get_robot_velocity():
    """Return field velocity [vx_cm_s, vy_cm_s, wz_deg_s, valid]."""

    if _velocity_provider is None:
        return [0.0, 0.0, 0.0, False]
    try:
        vx, vy, wz = _velocity_provider()
        return [float(vx), float(vy), float(wz), True]
    except Exception:
        return [0.0, 0.0, 0.0, False]


def _configure_referee_provider() -> None:
    """Select serial when configured, otherwise retain the existing IO input."""

    global _referee_state_provider, _referee_serial_provider

    serial_port = os.environ.get("ROBOCUPX_REFEREE_PORT", "").strip()
    source = os.environ.get("ROBOCUPX_REFEREE_SOURCE", "auto").strip().lower()
    if source not in {"auto", "serial", "io"}:
        raise ValueError("ROBOCUPX_REFEREE_SOURCE must be auto, serial, or io")

    if source in {"auto", "serial"} and serial_port:
        _referee_serial_provider = SerialRefereeProvider(
            serial_port,
            baudrate=int(os.environ.get("ROBOCUPX_REFEREE_BAUD", "115200")),
        ).start()
        _referee_state_provider = _referee_serial_provider.snapshot
        logger.info(f"Referee input loaded from serial port {serial_port}")
        return

    if source in {"auto", "io"} and _referee_status_provider is not None:
        provider = IoRefereeProvider(_referee_status_provider)
        _referee_state_provider = provider.snapshot
        logger.info("Referee input loaded from Bits IO")
        return

    _referee_state_provider = None
    logger.warning("No referee input configured; control remains STOP")


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
    
    # 如果已经有监控线程在运行，先停止并等待它退出。
    _stop_bits_thread_monitor()

    stop_event = threading.Event()

    def _monitor_loop():
        """监控循环，定期检查线程状态并在需要时重启。"""
        while not stop_event.is_set():
            try:
                if not bits_obj.IsThreadAlive():
                    logger.warning("Bits background thread is dead, attempting restart...")
                    bits_obj.RestartThreadIfNeeded()
            except Exception as e:
                logger.exception(f"Bits thread monitor error: {e}")
            # 每 1 秒检查一次
            stop_event.wait(1.0)

    _bits_monitor_stop = stop_event
    
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
    
    stop_event = _bits_monitor_stop
    monitor_thread = _bits_monitor_thread
    _bits_monitor_stop = None
    _bits_monitor_thread = None

    if stop_event is not None:
        stop_event.set()

    if monitor_thread is not None:
        try:
            if monitor_thread is not threading.current_thread():
                monitor_thread.join(timeout=2.0)
        except Exception:
            pass
    logger.info("Bits thread monitor stopped")


def _shutdown_runtime_unlocked() -> None:
    """Release runtime-owned resources so a simulator worker can restart cleanly."""

    global Bits, lidar, sensor, chassis, compass, peripheral, key, Odometer, Morse, referee, vision
    global _field_pose_provider, _velocity_provider, _referee_status_provider
    global _referee_state_provider, _referee_serial_provider, _runtime_initialized

    _stop_bits_thread_monitor()
    stop_watchdog = getattr(peripheral, "stop_watchdog", None)
    if callable(stop_watchdog):
        try:
            stop_watchdog()
        except Exception:
            pass
    for resource in (vision, _referee_serial_provider):
        for method_name in ("Close", "close", "stop"):
            method = getattr(resource, method_name, None)
            if callable(method):
                try:
                    method()
                except Exception:
                    pass
                break
    for method_name in ("Close", "close", "Shutdown", "shutdown", "stop"):
        method = getattr(Bits, method_name, None)
        if callable(method):
            try:
                method()
            except Exception:
                pass
            break

    Bits = lidar = sensor = chassis = compass = peripheral = key = Odometer = Morse = referee = vision = None
    _field_pose_provider = _velocity_provider = _referee_status_provider = None
    _referee_state_provider = _referee_serial_provider = None
    _runtime_initialized = False


def shutdown_runtime() -> None:
    """Release runtime-owned resources without racing an initialization."""

    with _runtime_lock:
        _shutdown_runtime_unlocked()


def _build_sim_backend(*, auto_start_lidar: bool) -> dict:
    """Assemble the Unity adapter without mutating runtime globals."""

    from utils.XeleratorSimBits import create_bits
    from vision_utils.XeleratorVision import XeleratorVision

    bits = create_bits(cfg)
    return {
        "Bits": bits,
        "vision": XeleratorVision(bits),
        "chassis": RecordingCar(bits.SetWheelSpeed, bits.GetYaw),
        "compass": bits.GetYaw,
        "peripheral": Peripherals(
            bits.VirtualPorts,
            KickFunc=getattr(bits, "Kick", None),
        ),
        "key": Key(bits.GetKey),
        "Odometer": None,
        "Morse": None,
        "referee": None,
        "field_pose_provider": bits.GetPose,
        "velocity_provider": bits.GetVelocity,
        "referee_status_provider": bits.GetControlEnabled,
    }


def _debug_motion_enabled() -> bool:
    """Allow an explicit local debug run to bypass the referee motion gate."""

    return os.environ.get("ROBOCUPX_DEBUG", "").strip().lower() in {"1", "true", "yes", "on"}


def _motion_permission(bits) -> bool:
    """Use the hardware referee gate except during an explicitly enabled debug run."""

    return _debug_motion_enabled() or bool(bits.GetRefereeUnlockPin())


def _build_iceloong_backend(*, auto_start_lidar: bool) -> dict:
    """Assemble IceLoong hardware through the active Bits contract only."""

    from utils.IceLoongBits import IceLoongBits

    bits = IceLoongBits(port=getattr(cfg.Ports, "IceLoongPort", "/dev/iceloong"))
    return {
        "Bits": bits,
        "vision": None,
        "chassis": RecordingCar(
            bits.SetWheelSpeed,
            bits.GetYaw,
            permission=lambda: _motion_permission(bits),
        ),
        "compass": bits.GetYaw,
        "peripheral": Peripherals(
            bits.VirtualPorts,
            lambda: _motion_permission(bits),
            GetDribbleCurrentFunc=getattr(bits, "GetDribbleCurrent", None),
            KickFunc=getattr(bits, "Kick", None),
        ),
        "key": None,
        "Odometer": None,
        "Morse": bits.MorsePlay,
        "referee": bits.GetRefereeUnlockPin,
        "field_pose_provider": bits.GetPose,
        "velocity_provider": getattr(bits, "GetVelocity", None),
        "referee_status_provider": bits.GetRefereeUnlockPin,
    }


_BACKEND_FACTORIES = {
    "SIM": _build_sim_backend,
    "IL": _build_iceloong_backend,
}


def _selected_backend_name() -> str:
    """Return the requested active backend after honoring the env override."""

    override = os.environ.get("ROBOCUPX_BITS", "").strip().upper()
    if override:
        return override
    return str(getattr(cfg.RoboInfo, "Bit", "")).strip().upper()


def _init_runtime(*, auto_start_lidar: bool = True):
    """Initialize once. auto_start_lidar is retained for older callers only."""

    with _runtime_lock:
        return _init_runtime_unlocked(auto_start_lidar=auto_start_lidar)


def _init_runtime_unlocked(*, auto_start_lidar: bool = True):
    """Initialize the selected SIM or IceLoong runtime and semantic sensor."""

    global Bits, lidar, sensor, chassis, compass, peripheral, key, Odometer, Morse, referee, vision
    global _field_pose_provider, _velocity_provider, _referee_status_provider, _runtime_initialized

    if _runtime_initialized:
        return

    backend_name = _selected_backend_name()
    factory = _BACKEND_FACTORIES.get(backend_name)
    if factory is None:
        supported = ", ".join(_BACKEND_FACTORIES)
        raise RuntimeError(f"Unknown Bits model: {backend_name!r}. Supported: {supported}")

    try:
        assembly = factory(auto_start_lidar=auto_start_lidar)
        Bits = assembly["Bits"]
        chassis = assembly["chassis"]
        compass = assembly["compass"]
        peripheral = assembly["peripheral"]
        key = assembly["key"]
        Odometer = assembly["Odometer"]
        Morse = assembly["Morse"]
        referee = assembly["referee"]
        vision = assembly["vision"]
        bits_pose_provider = assembly["field_pose_provider"]
        _velocity_provider = assembly["velocity_provider"]
        _referee_status_provider = assembly["referee_status_provider"]
        sensor = Sensor(
            pose_provider=bits_pose_provider,
            bounds_long_cm=float(getattr(cfg.Bounds, "Long", 260.0)),
            bounds_short_cm=float(getattr(cfg.Bounds, "Short", 200.0)),
        )
        _field_pose_provider = sensor.GetPose
        # ``lidar`` stays as a compatibility global, but is now the semantic
        # Sensor facade instead of a second localization owner.
        lidar = sensor
        # Let the chassis zero peripheral IO when the referee gate closes;
        # without this the dribbler keeps its latched state after shutdown.
        attach_peripherals = getattr(chassis, "attach_peripherals", None)
        if callable(attach_peripherals):
            attach_peripherals(peripheral)
        _configure_referee_provider()
        if backend_name == "IL":
            _start_bits_thread_monitor(Bits)
        _runtime_initialized = True
        logger.info(f"LunaPre runtime loaded: backend={backend_name}")
    except Exception as e:
        shutdown_runtime()
        logger.error(f"Failed to init runtime: {e}")
        raise RuntimeError(
            f"Runtime initialization failed for Bits={backend_name!r}. Supported: SIM, IL. Error: {e}"
        ) from e


def ensure_runtime(*, auto_start_lidar: bool = True) -> None:
    """Initialize the selected runtime before a command path uses its objects."""

    if not _runtime_initialized:
        _init_runtime(auto_start_lidar=auto_start_lidar)


Beta_Func = False

# Thread start flags (avoid NameError)
SendstatusThreadFuncStarted = False
PeerstatusThreadFuncStarted = False
