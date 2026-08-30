"""Direct serial driver for the LDROBOT LD07 55AA protocol."""

from __future__ import annotations

import logging
import math
import os
import struct
import threading

import serial

from ReasonData import Preference, Settings


LOGGER = logging.getLogger(__name__)

SYNC = b"\xaa\xaa\xaa\xaa"
SYNC_VALUE = 0xAAAAAAAA
FRAME_HEADER_LENGTH = 10
FRAME_PAYLOAD_PREFIX_LENGTH = 4
POINT_COUNT = 160
POINT_BYTES = 2
EXPECTED_DATA_LENGTH = FRAME_PAYLOAD_PREFIX_LENGTH + POINT_COUNT * POINT_BYTES

DEVICE_ADDRESS = 0x01
GET_DISTANCE_COMMAND = 0x02
STOP_MEASUREMENT_COMMAND = 0x0F
START_MEASUREMENT_COMMAND = struct.pack(
    "<LBBHHB", SYNC_VALUE, DEVICE_ADDRESS, GET_DISTANCE_COMMAND, 0, 0, 0x03
)
STOP_COMMAND = struct.pack(
    "<LBBHHB", SYNC_VALUE, DEVICE_ADDRESS, STOP_MEASUREMENT_COMMAND, 0, 0, 0x10
)

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUDRATE = 921600
DEFAULT_SERIAL_TIMEOUT = 0.2
DEFAULT_RECONNECT_DELAY = 0.5
DEFAULT_SCAN_MIN_POINTS = 5
DEFAULT_FOV_DEG = 90.0
DEFAULT_ANGLE_START_DEG = -45.0
DISTANCE_MASK = 0x01FF


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


def _env_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


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


class LD07Driver:
    """Read LD07 frames and publish ``(angle_deg, distance_m)`` scans."""

    def __init__(
        self,
        *,
        port: str | None = None,
        baudrate: int | None = None,
        timeout: float | None = None,
        angle_start_deg: float | None = None,
        fov_deg: float | None = None,
        angle_offset_deg: float | None = None,
        reconnect_delay: float | None = None,
        scan_min_points: int | None = None,
        verify_checksum: bool | None = None,
    ):
        configured_port = str(_setting_value("Ports", "LidarPort", DEFAULT_PORT))
        self.port = port or os.environ.get("LDLIDAR_PORT") or os.environ.get(
            "LD07_PORT", configured_port
        )
        self.baudrate = int(
            baudrate
            if baudrate is not None
            else _env_int("LDLIDAR_BAUDRATE", DEFAULT_BAUDRATE)
        )
        self.timeout = float(
            timeout
            if timeout is not None
            else _env_float("LDLIDAR_TIMEOUT", DEFAULT_SERIAL_TIMEOUT)
        )
        self.angle_start_deg = float(
            angle_start_deg
            if angle_start_deg is not None
            else _env_float("LD07_ANGLE_START_DEG", DEFAULT_ANGLE_START_DEG)
        )
        self.fov_deg = float(
            fov_deg
            if fov_deg is not None
            else _env_float("LD07_FOV_DEG", DEFAULT_FOV_DEG)
        )
        if self.fov_deg <= 0 or self.fov_deg > 360:
            raise ValueError("fov_deg must be in the range (0, 360]")
        self.angle_offset_deg = float(
            angle_offset_deg
            if angle_offset_deg is not None
            else _env_float("LD07_ANGLE_OFFSET_DEG", 0.0)
        )
        self.reconnect_delay = float(
            reconnect_delay
            if reconnect_delay is not None
            else _env_float("LDLIDAR_RECONNECT_DELAY", DEFAULT_RECONNECT_DELAY)
        )
        self.scan_min_points = max(
            1,
            int(
                scan_min_points
                if scan_min_points is not None
                else _env_int("LDLIDAR_SCAN_MIN_POINTS", DEFAULT_SCAN_MIN_POINTS)
            ),
        )
        self.verify_checksum = bool(
            verify_checksum
            if verify_checksum is not None
            else _env_bool("LDLIDAR_VERIFY_CHECKSUM", True)
        )
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._serial: serial.Serial | None = None
        self.last_packet: dict | None = None
        self.invalid_packet_count = 0
        self.last_frame_timestamp = 0

    def start(self, scan_queue) -> bool:
        if self._thread and self._thread.is_alive():
            return True
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self.run_forever,
            args=(scan_queue,),
            daemon=True,
            name="LD07Reader",
        )
        self._thread.start()
        return True

    def stop(self) -> None:
        self._stop_event.set()
        lidar_serial = self._serial
        if lidar_serial is not None:
            try:
                lidar_serial.write(STOP_COMMAND)
                lidar_serial.flush()
            except Exception:
                pass
            try:
                lidar_serial.close()
            except Exception:
                pass
        self._serial = None

    def run_forever(self, scan_queue) -> None:
        while not self._stop_event.is_set():
            lidar_serial = None
            try:
                LOGGER.info(f"Opening LD07 port={self.port} baudrate={self.baudrate}")
                lidar_serial = serial.Serial(
                    self.port,
                    baudrate=self.baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=self.timeout,
                )
                self._serial = lidar_serial
                lidar_serial.reset_input_buffer()
                lidar_serial.write(START_MEASUREMENT_COMMAND)
                lidar_serial.flush()
                buffer = bytearray()
                while not self._stop_event.is_set():
                    chunk = lidar_serial.read(4096)
                    if chunk:
                        buffer.extend(chunk)
                    for frame in self._take_frames(buffer):
                        decoded = self.decode_packet(frame)
                        if decoded is None:
                            self.invalid_packet_count += 1
                            continue
                        self.last_packet = decoded
                        self.last_frame_timestamp = decoded["timestamp"]
                        scan = tuple(
                            (sample["angle_deg"], sample["distance_m"])
                            for sample in decoded["samples"]
                            if sample["distance_m"] > 0
                        )
                        if len(scan) >= self.scan_min_points:
                            self._publish_scan(scan_queue, scan)
            except (serial.SerialException, OSError) as error:
                if not self._stop_event.is_set():
                    LOGGER.warning(f"LD07 serial error: {error}; reconnecting")
            except Exception:
                if not self._stop_event.is_set():
                    LOGGER.exception("LD07 driver error; reconnecting")
            finally:
                if lidar_serial is not None:
                    try:
                        lidar_serial.write(STOP_COMMAND)
                    except Exception:
                        pass
                    try:
                        lidar_serial.close()
                    except Exception:
                        pass
                if self._serial is lidar_serial:
                    self._serial = None
            self._stop_event.wait(self.reconnect_delay)

    @staticmethod
    def _take_frames(buffer: bytearray):
        while True:
            header_index = buffer.find(SYNC)
            if header_index < 0:
                if len(buffer) >= len(SYNC):
                    del buffer[:-len(SYNC) + 1]
                return
            if header_index:
                del buffer[:header_index]
            if len(buffer) < FRAME_HEADER_LENGTH:
                return
            data_length = int.from_bytes(buffer[8:10], "little")
            if data_length < FRAME_PAYLOAD_PREFIX_LENGTH or data_length > 800:
                del buffer[:len(SYNC)]
                continue
            frame_length = FRAME_HEADER_LENGTH + data_length + 1
            if len(buffer) < frame_length:
                return
            frame = bytes(buffer[:frame_length])
            del buffer[:frame_length]
            yield frame

    def decode_packet(self, packet: bytes | bytearray) -> dict | None:
        packet = bytes(packet)
        if len(packet) < FRAME_HEADER_LENGTH + 1 or packet[:4] != SYNC:
            return None
        device, command, offset, data_length = struct.unpack_from("<BBHH", packet, 4)
        frame_length = FRAME_HEADER_LENGTH + data_length + 1
        if len(packet) != frame_length or data_length != EXPECTED_DATA_LENGTH:
            return None
        if device != DEVICE_ADDRESS or command != GET_DISTANCE_COMMAND:
            return None
        payload = packet[FRAME_HEADER_LENGTH:FRAME_HEADER_LENGTH + data_length]
        received_checksum = packet[-1]
        if self.verify_checksum and received_checksum != self._checksum(packet[4:10], payload):
            return None
        timestamp = int.from_bytes(payload[:FRAME_PAYLOAD_PREFIX_LENGTH], "little")
        samples = []
        angle_step = self.fov_deg / POINT_COUNT
        for index in range(POINT_COUNT):
            raw = struct.unpack_from(
                "<H", payload, FRAME_PAYLOAD_PREFIX_LENGTH + index * POINT_BYTES
            )[0]
            distance_mm = raw & DISTANCE_MASK
            confidence = raw >> 9
            samples.append({
                "angle_deg": self.angle_start_deg + angle_step * index + self.angle_offset_deg,
                "distance_m": distance_mm / 1000.0,
                "distance_mm": distance_mm,
                "confidence": confidence,
                "raw": raw,
            })
        return {
            "device": device,
            "command": command,
            "offset": offset,
            "data_length": data_length,
            "timestamp": timestamp,
            "sample_count": POINT_COUNT,
            "samples": samples,
            "checksum": received_checksum,
        }

    def detect_objects(
        self,
        *,
        min_distance_mm: int = 30,
        max_distance_mm: int = 300,
        max_angle_gap_deg: float = 2.0,
        min_points: int = 3,
    ) -> list[dict]:
        packet = self.last_packet
        if not packet:
            return []
        clusters = []
        current = []
        previous_index = None
        previous_angle = None

        def flush_cluster() -> None:
            if len(current) < max(1, int(min_points)):
                current.clear()
                return
            points = list(current)
            forward_m = sum(
                sample["distance_m"] * math.cos(math.radians(sample["angle_deg"]))
                for sample in points
            ) / len(points)
            lateral_m = sum(
                sample["distance_m"] * math.sin(math.radians(sample["angle_deg"]))
                for sample in points
            ) / len(points)
            distance_m = math.hypot(forward_m, lateral_m)
            clusters.append({
                "x_forward_cm": round(forward_m * 100.0, 2),
                "y_left_cm": round(lateral_m * 100.0, 2),
                "distance_cm": round(distance_m * 100.0, 2),
                "angle_deg": round(math.degrees(math.atan2(lateral_m, forward_m)), 2),
                "point_count": len(points),
                "confidence": round(
                    sum(sample["confidence"] for sample in points) / len(points), 2
                ),
                "min_distance_mm": min(sample["distance_mm"] for sample in points),
                "max_distance_mm": max(sample["distance_mm"] for sample in points),
            })
            current.clear()

        for index, sample in enumerate(packet["samples"]):
            distance_mm = sample["distance_mm"]
            angle_deg = sample["angle_deg"]
            valid = min_distance_mm <= distance_mm <= max_distance_mm
            contiguous = (
                previous_index is not None
                and index == previous_index + 1
                and previous_angle is not None
                and abs(angle_deg - previous_angle) <= max_angle_gap_deg
            )
            if not valid:
                flush_cluster()
                previous_index = None
                previous_angle = None
                continue
            if current and not contiguous:
                flush_cluster()
            current.append(sample)
            previous_index = index
            previous_angle = angle_deg
        flush_cluster()
        return clusters

    @staticmethod
    def _checksum(header: bytes, payload: bytes) -> int:
        return (sum(header) + sum(payload)) & 0xFF

    @staticmethod
    def _publish_scan(scan_queue, scan) -> None:
        if not scan:
            return
        try:
            while scan_queue.full():
                scan_queue.get_nowait()
            scan_queue.put_nowait(tuple(scan))
        except Exception:
            pass
