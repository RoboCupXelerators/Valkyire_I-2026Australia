"""Direct RPLidar scan driver using the ``rplidar-roboticia`` package."""

from __future__ import annotations

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
DEFAULT_RPLIDAR_MAX_BUF_MEAS = 3000
RPLIDAR_MODEL_CONFIGS = {
    "s1": {
        "baudrate": 256000,
        "scan_type": "normal",
        "max_buf_meas": 3000,
    },
    "s2": {
        "baudrate": 1000000,
        "scan_type": "normal",
        "max_buf_meas": 3000,
    },
    "s3": {
        "baudrate": 1000000,
        "scan_type": "normal",
        "max_buf_meas": 3000,
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


def _configured_lidar_type() -> str:
    lidar_type = str(_setting_value("RoboInfo", "LidarType", "s2")).strip().lower()
    if lidar_type not in RPLIDAR_MODEL_CONFIGS:
        logger.warning(f"Unknown LidarType={lidar_type!r}; using s2 defaults.")
        return "s2"
    return lidar_type


def _load_rplidar_module():
    try:
        return importlib.import_module("rplidar")
    except Exception:
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
    ):
        self.lidar_type = self._resolve_lidar_type(lidar_type)
        model_config = RPLIDAR_MODEL_CONFIGS[self.lidar_type]

        self.port = port or os.environ.get("LIDAR_PORT") or str(
            _setting_value("Ports", "LidarPort", DEFAULT_RPLIDAR_PORT)
        )
        self.baudrate = int(
            baudrate
            if baudrate is not None
            else _env_int("LIDAR_BAUDRATE", int(model_config["baudrate"]))
        )
        self.timeout = float(timeout if timeout is not None else _env_float("LIDAR_TIMEOUT", DEFAULT_RPLIDAR_TIMEOUT))
        self.angle_offset_deg = float(
            angle_offset_deg
            if angle_offset_deg is not None
            else _env_float("LIDAR_ANGLE_OFFSET_DEG", DEFAULT_RPLIDAR_ANGLE_OFFSET_DEG)
        )
        self.scan_type = scan_type or os.environ.get("LIDAR_SCAN_TYPE", str(model_config.get("scan_type", "normal")))
        self.reconnect_delay = float(
            reconnect_delay
            if reconnect_delay is not None
            else _env_float("LIDAR_RECONNECT_DELAY", DEFAULT_RPLIDAR_RECONNECT_DELAY)
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
                else _env_int("LIDAR_MAX_BUF_MEAS", int(model_config.get("max_buf_meas", DEFAULT_RPLIDAR_MAX_BUF_MEAS)))
            ),
        )

        self._stop_event = threading.Event()
        self._thread = None
        self._lidar = None

    def _resolve_lidar_type(self, lidar_type: str | None = None) -> str:
        lidar_type_key = str(lidar_type or _configured_lidar_type()).strip().lower()
        if lidar_type_key not in RPLIDAR_MODEL_CONFIGS:
            logger.warning(f"Unknown LidarType={lidar_type_key!r}; using s2 defaults.")
            return "s2"
        return lidar_type_key

    def start(self, scan_queue):
        rplidar_module = _load_rplidar_module()
        if rplidar_module is None:
            logger.error("rplidar module is not installed. Run: pip install rplidar-roboticia")
            return False

        self._stop_event.clear()
        self._thread = threading.Thread(target=self.run_forever, args=(scan_queue,))
        self._thread.daemon = True
        self._thread.start()
        return True

    def stop(self):
        self._stop_event.set()
        lidar = self._lidar
        if lidar is not None:
            self._close_lidar(lidar)
            self._lidar = None

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
                logger.info(
                    "Starting rplidar driver: type=%s port=%s baudrate=%s timeout=%s"
                    % (self.lidar_type, self.port, self.baudrate, self.timeout)
                )
                lidar = RPLidar(self.port, baudrate=self.baudrate, timeout=self.timeout)
                self._lidar = lidar
                self._prepare_lidar(lidar)
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
                        angle = (float(angle) + self.angle_offset_deg) % 360.0
                    except (TypeError, ValueError):
                        continue

                    if distance > 0:
                        current_scan.append((angle, distance / 1000.0))

                if current_scan and self._is_running() and len(current_scan) >= self.scan_min_points:
                    self._publish_scan(scan_queue, current_scan)

                break
            except RPLidarException as e:
                if not self._is_running():
                    break
                logger.warning(f"rplidar read error: {e}; reconnecting...")
                if lidar is not None:
                    self._clear_input(lidar)
            except Exception as e:
                if not self._is_running():
                    break
                logger.error(f"rplidar driver exception: {e}; reconnecting...")
            finally:
                if lidar is not None:
                    self._close_lidar(lidar)
                if self._lidar is lidar:
                    self._lidar = None

            if self._is_running():
                self._stop_event.wait(self.reconnect_delay)

    def _iter_measurements(self, lidar):
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
        try:
            while scan_queue.full():
                scan_queue.get_nowait()
            scan_queue.put_nowait(data)
        except Exception:
            pass

    def _clear_input(self, lidar):
        for name in ("clear_input", "clean_input"):
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
