"""Direct RPLidar scan driver using the ``rplidar-roboticia`` package."""

from __future__ import annotations

import glob
import importlib
import os
import threading
import time

from ReasonData import Preference, Settings, logger


DEFAULT_RPLIDAR_PORT = "/dev/sllidar"
DEFAULT_RPLIDAR_TIMEOUT = 1.0
DEFAULT_RPLIDAR_RECONNECT_DELAY = 0.5
DEFAULT_RPLIDAR_SCAN_MIN_POINTS = 5
DEFAULT_RPLIDAR_ANGLE_OFFSET_DEG = 0.0
DEFAULT_RPLIDAR_MAX_BUF_MEAS = 65536
DEFAULT_RPLIDAR_USE_SCAN_ITERATOR = True
RPLIDAR_MODEL_CONFIGS = {
    "s1": {
        "baudrate": 256000,
        "scan_type": "normal",
        "max_buf_meas": 65536,
        "legacy_supported": True,
    },
    "s2": {
        "baudrate": 1000000,
        "scan_type": "normal",
        "max_buf_meas": 131072,
        "legacy_supported": True,
    },
    "s3": {
        "baudrate": 1000000,
        "scan_type": "normal",
        "max_buf_meas": 131072,
        "legacy_supported": False,
    },
}


def _env_float(name: str, default: float) -> float:
    value = os.environ.get(name)
    if value is None:
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        logger.warning(f"Invalid {name}={value!r}; using {default}.")
        return default


def _env_int(name: str, default: int) -> int:
    value = os.environ.get(name)
    if value is None:
        return default
    try:
        return int(value)
    except (TypeError, ValueError):
        logger.warning(f"Invalid {name}={value!r}; using {default}.")
        return default


def _env_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    value_key = str(value).strip().lower()
    if value_key in {"1", "true", "yes", "on"}:
        return True
    if value_key in {"0", "false", "no", "off"}:
        return False
    logger.warning(f"Invalid {name}={value!r}; using {default}.")
    return default


def _env_max_buf_meas(name: str, default: int) -> int:
    value = os.environ.get(name)
    if value is None:
        return default
    value_key = str(value).strip().lower()
    if value_key in {"0", "false", "none", "off", "unlimited"}:
        return 0
    try:
        return int(value)
    except (TypeError, ValueError):
        logger.warning(f"Invalid {name}={value!r}; using {default}.")
        return default


def _setting_bool(section: str, key: str, default: bool = False) -> bool:
    value = _setting_value(section, key, default)
    if isinstance(value, bool):
        return value
    value_key = str(value).strip().lower()
    if value_key in {"1", "true", "yes", "on"}:
        return True
    if value_key in {"0", "false", "no", "off"}:
        return False
    return default


def _setting_value(section: str, key: str, default=None):
    try:
        value = Preference().read(section, key)
        if value not in (None, "", "None"):
            return value
    except Exception:
        pass

    try:
        value = getattr(getattr(Settings, section), key)
        if value not in (None, "", "None"):
            return value
    except Exception:
        pass
    return default


def _resolve_serial_port(configured_port: str) -> str:
    """Use a stable serial alias, falling back only when exactly one device exists."""
    port = str(configured_port or DEFAULT_RPLIDAR_PORT).strip()
    if os.path.exists(port):
        return port

    candidates = []
    seen_devices = set()
    for pattern in (
        "/dev/serial/by-id/*",
        "/dev/serial/by-path/*",
        "/dev/ttyCH343USB*",
        "/dev/ttyUSB*",
        "/dev/ttyACM*",
    ):
        for candidate in sorted(glob.glob(pattern)):
            if not os.path.exists(candidate):
                continue
            real_device = os.path.realpath(candidate)
            if real_device in seen_devices:
                continue
            seen_devices.add(real_device)
            candidates.append(candidate)

    if len(candidates) == 1:
        resolved = candidates[0]
        logger.warning(
            f"Configured lidar port {port} is missing; using the only serial device {resolved}"
        )
        return resolved
    return port


def _configured_lidar_type() -> str:
    lidar_type = str(_setting_value("RoboInfo", "LidarType", "s2")).strip().lower()
    if lidar_type not in RPLIDAR_MODEL_CONFIGS:
        logger.warning(f"Unknown LidarType={lidar_type!r}; using s2 defaults.")
        return "s2"
    return lidar_type


def _load_rplidar_module():
    try:
        return importlib.import_module("rplidar")
    except Exception as error:
        logger.error(f"Unable to import rplidar package: {error}")
        return None


class RPLidarDriver:
    """Acquire raw scan frames as ``(angle_deg, distance_m)`` tuples."""

    def __init__(
        self,
        *,
        lidar_type: str | None = None,
        port: str | None = None,
        baudrate: int | None = None,
        timeout: float | None = None,
        angle_offset_deg: float | None = None,
        scan_type: str | None = None,
        reconnect_delay: float | None = None,
        scan_min_points: int | None = None,
        max_buf_meas: int | None = None,
        use_scan_iterator: bool | None = None,
        inverted: bool | None = None,
    ):
        self.lidar_type = self._resolve_lidar_type(lidar_type)
        model_config = RPLIDAR_MODEL_CONFIGS[self.lidar_type]

        self.configured_port = port or os.environ.get("LIDAR_PORT") or str(
            _setting_value("Ports", "LidarPort", DEFAULT_RPLIDAR_PORT)
        )
        self.port = _resolve_serial_port(self.configured_port)
        self.baudrate = int(
            baudrate
            if baudrate is not None
            else _env_int("LIDAR_BAUDRATE", int(model_config["baudrate"]))
        )
        self.timeout = max(
            0.1,
            float(timeout if timeout is not None else _env_float("LIDAR_TIMEOUT", DEFAULT_RPLIDAR_TIMEOUT)),
        )
        self.angle_offset_deg = float(
            angle_offset_deg
            if angle_offset_deg is not None
            else _env_float("LIDAR_ANGLE_OFFSET_DEG", DEFAULT_RPLIDAR_ANGLE_OFFSET_DEG)
        )
        self.scan_type = scan_type or os.environ.get(
            "LIDAR_SCAN_TYPE", str(model_config.get("scan_type", "normal"))
        )
        if self.scan_type not in {"normal", "force", "express"}:
            logger.warning(f"Unknown LIDAR_SCAN_TYPE={self.scan_type!r}; using normal.")
            self.scan_type = "normal"
        self.reconnect_delay = max(
            0.05,
            float(
                reconnect_delay
                if reconnect_delay is not None
                else _env_float("LIDAR_RECONNECT_DELAY", DEFAULT_RPLIDAR_RECONNECT_DELAY)
            ),
        )
        self.scan_min_points = max(
            1,
            int(
                scan_min_points
                if scan_min_points is not None
                else _env_int("LIDAR_SCAN_MIN_POINTS", DEFAULT_RPLIDAR_SCAN_MIN_POINTS)
            ),
        )
        self.max_buf_meas = max(
            0,
            int(
                max_buf_meas
                if max_buf_meas is not None
                else _env_max_buf_meas("LIDAR_MAX_BUF_MEAS", int(model_config.get("max_buf_meas", DEFAULT_RPLIDAR_MAX_BUF_MEAS)))
            ),
        )
        self.use_scan_iterator = bool(
            use_scan_iterator
            if use_scan_iterator is not None
            else _env_bool("LIDAR_USE_SCAN_ITERATOR", DEFAULT_RPLIDAR_USE_SCAN_ITERATOR)
        )
        self.inverted = bool(
            inverted
            if inverted is not None
            else _env_bool(
                "LIDAR_INVERTED",
                _setting_bool("RoboInfo", "LidarInverted", False),
            )
        )

        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._state_lock = threading.RLock()
        self._lidar = None
        self.last_scan: tuple[tuple[float, float], ...] = ()
        self.last_scan_point_count = 0
        self.last_error = None

    def _resolve_lidar_type(self, lidar_type: str | None = None) -> str:
        lidar_type_key = str(lidar_type or _configured_lidar_type()).strip().lower()
        if lidar_type_key not in RPLIDAR_MODEL_CONFIGS:
            logger.warning(f"Unknown LidarType={lidar_type_key!r}; using s2 defaults.")
            return "s2"
        return lidar_type_key

    def start(self, scan_queue):
        if not RPLIDAR_MODEL_CONFIGS[self.lidar_type].get("legacy_supported", True):
            self.last_error = (
                f"RPLIDAR {self.lidar_type.upper()} is not supported by the legacy Python driver; "
                "use the official ROS2 backend"
            )
            logger.error(self.last_error)
            return False

        rplidar_module = _load_rplidar_module()
        if rplidar_module is None:
            self.last_error = "rplidar package is not installed"
            logger.error("rplidar module is not installed. Run: pip install rplidar-roboticia")
            return False

        with self._state_lock:
            if self._thread is not None and self._thread.is_alive():
                return True
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self.run_forever,
                args=(scan_queue,),
                daemon=True,
                name="RPLidarReader",
            )
            self._thread.start()
        return True

    def stop(self):
        self._stop_event.set()
        with self._state_lock:
            lidar = self._lidar
            thread = self._thread
        if lidar is not None:
            self._close_lidar(lidar)
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=max(1.0, self.timeout + 0.5))
        with self._state_lock:
            if self._lidar is lidar:
                self._lidar = None
            if self._thread is thread and (thread is None or not thread.is_alive()):
                self._thread = None

    def _is_running(self) -> bool:
        return not self._stop_event.is_set()

    def run_forever(self, scan_queue):
        rplidar_module = _load_rplidar_module()
        if rplidar_module is None:
            logger.error("rplidar module is not installed. Run: pip install rplidar-roboticia")
            return

        RPLidar = getattr(rplidar_module, "RPLidar")
        RPLidarException = getattr(rplidar_module, "RPLidarException", Exception)

        while self._is_running():
            lidar = None
            current_scan = []
            try:
                resolved_port = _resolve_serial_port(self.configured_port)
                if resolved_port != self.port:
                    logger.info(f"RPLidar serial path changed: {self.port} -> {resolved_port}")
                    self.port = resolved_port
                logger.info(
                    "Starting rplidar driver: type=%s port=%s baudrate=%s timeout=%s max_buf=%s scan_iterator=%s inverted=%s"
                    % (
                        self.lidar_type,
                        self.port,
                        self.baudrate,
                        self.timeout,
                        self.max_buf_meas or "unlimited",
                        self.use_scan_iterator,
                        self.inverted,
                    )
                )
                lidar = RPLidar(self.port, baudrate=self.baudrate, timeout=self.timeout)
                with self._state_lock:
                    self._lidar = lidar
                self._install_read_timeout(lidar, RPLidarException)
                self._prepare_lidar(lidar)
                if not self._is_running():
                    break
                iterator = self._iter_measurements(lidar)

                for measurement in self._normalize_measurements(iterator):
                    if not self._is_running():
                        break
                    new_scan, quality, angle, distance = measurement

                    if new_scan and current_scan:
                        if len(current_scan) >= self.scan_min_points:
                            self._publish_scan(scan_queue, current_scan)
                        current_scan = []

                    try:
                        distance = float(distance)
                        angle = self._normalize_angle(angle)
                    except (TypeError, ValueError):
                        continue

                    if distance > 0:
                        current_scan.append((angle, distance / 1000.0))

                if current_scan and self._is_running() and len(current_scan) >= self.scan_min_points:
                    self._publish_scan(scan_queue, current_scan)

                if self._is_running():
                    self.last_error = "RPLidar measurement iterator stopped"
                    logger.warning(f"{self.last_error}; reconnecting...")
            except RPLidarException as e:
                if not self._is_running():
                    break
                self.last_error = str(e)
                logger.warning(f"rplidar read error: {e}; reconnecting...")
                if lidar is not None:
                    self._clear_input(lidar)
            except Exception as e:
                if not self._is_running():
                    break
                self.last_error = str(e)
                logger.error(f"rplidar driver exception: {e}; reconnecting...")
            finally:
                if lidar is not None:
                    self._close_lidar(lidar)
                with self._state_lock:
                    if self._lidar is lidar:
                        self._lidar = None

            if self._is_running():
                self._stop_event.wait(self.reconnect_delay)

    def _iter_measurements(self, lidar):
        if self.use_scan_iterator and hasattr(lidar, "iter_scans"):
            return self._iter_scan_measurements(lidar)
        if hasattr(lidar, "iter_measurments"):
            try:
                return lidar.iter_measurments(max_buf_meas=self.max_buf_meas)
            except TypeError:
                return lidar.iter_measurments()
        if hasattr(lidar, "iter_measurements"):
            try:
                return lidar.iter_measurements(max_buf_meas=self.max_buf_meas)
            except TypeError:
                return lidar.iter_measurements()
        if hasattr(lidar, "iter_measures"):
            try:
                return lidar.iter_measures(scan_type=self.scan_type, max_buf_meas=self.max_buf_meas)
            except TypeError:
                return lidar.iter_measures()
        raise RuntimeError("rplidar module does not expose a measurement iterator")

    def _iter_scan_measurements(self, lidar):
        scan_iterator = self._iter_scans(lidar)
        for scan in scan_iterator:
            first_measurement = True
            for item in scan:
                measurement = self._normalize_measurement(item, default_new_scan=first_measurement)
                if measurement is None:
                    continue
                new_scan, quality, angle, distance = measurement
                yield bool(first_measurement or new_scan), quality, angle, distance
                first_measurement = False

    def _iter_scans(self, lidar):
        try:
            return lidar.iter_scans(
                scan_type=self.scan_type,
                max_buf_meas=self.max_buf_meas,
                min_len=self.scan_min_points,
            )
        except TypeError:
            try:
                return lidar.iter_scans(max_buf_meas=self.max_buf_meas, min_len=self.scan_min_points)
            except TypeError:
                try:
                    return lidar.iter_scans(max_buf_meas=self.max_buf_meas)
                except TypeError:
                    return lidar.iter_scans()

    def _normalize_measurements(self, iterator):
        """Yield measurements as (new_scan, quality, angle, distance)."""
        for item in iterator:
            measurement = self._normalize_measurement(item)
            if measurement is not None:
                yield measurement
                continue

            try:
                sub_iterator = iter(item)
            except TypeError:
                continue

            for sub_item in sub_iterator:
                measurement = self._normalize_measurement(sub_item, default_new_scan=False)
                if measurement is not None:
                    yield measurement

    def _normalize_angle(self, angle) -> float:
        angle = float(angle)
        if self.inverted:
            angle = -angle
        return (angle + self.angle_offset_deg) % 360.0

    def _normalize_measurement(self, item, default_new_scan=True):
        try:
            values = tuple(item)
        except TypeError:
            return None

        if len(values) == 4:
            new_scan, quality, angle, distance = values
            return bool(new_scan), quality, angle, distance
        if len(values) == 3:
            quality, angle, distance = values
            return bool(default_new_scan), quality, angle, distance
        return None

    def _prepare_lidar(self, lidar):
        try:
            stop = getattr(lidar, "stop", None)
            if callable(stop):
                stop()
        except Exception:
            pass
        self._clear_input(lidar)

    def _publish_scan(self, scan_queue, scan):
        if not scan:
            return
        data = tuple(scan)
        self.last_scan = data
        self.last_scan_point_count = len(data)
        self.last_error = None
        try:
            while scan_queue.full():
                scan_queue.get_nowait()
            scan_queue.put_nowait(data)
        except Exception:
            pass

    def _install_read_timeout(self, lidar, exception_type):
        """Bound the old package's busy-wait when a sensor emits no scan data."""
        timeout = max(0.1, float(self.timeout))

        def read_response(dsize):
            serial_port = getattr(lidar, "_serial", None)
            if serial_port is None:
                raise exception_type(f"Serial port for {self.port} is closed")

            def bytes_waiting():
                waiting = getattr(serial_port, "in_waiting", None)
                if waiting is not None:
                    return int(waiting)
                waiting_method = getattr(serial_port, "inWaiting", None)
                if callable(waiting_method):
                    return int(waiting_method())
                raise exception_type("Serial object has no input-buffer query method")

            deadline = time.monotonic() + timeout
            while bytes_waiting() < dsize:
                if time.monotonic() >= deadline:
                    raise exception_type(
                        f"Timed out waiting for {dsize} bytes from {self.port}"
                    )
                time.sleep(0.001)
            data = serial_port.read(dsize)
            if len(data) != dsize:
                raise exception_type(
                    f"Short response from {self.port}: {len(data)}/{dsize} bytes"
                )
            return data

        lidar._read_response = read_response

    def _clear_input(self, lidar):
        for name in ("clear_input", "clean_input", "reset_input_buffer"):
            clearer = getattr(lidar, name, None)
            if callable(clearer):
                try:
                    clearer()
                except Exception:
                    pass
                return

    def _close_lidar(self, lidar):
        for name in ("stop", "stop_motor", "disconnect"):
            closer = getattr(lidar, name, None)
            if callable(closer):
                try:
                    closer()
                except Exception:
                    pass
