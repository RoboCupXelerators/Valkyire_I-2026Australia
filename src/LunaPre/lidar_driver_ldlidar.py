"""Direct serial driver for the LDROBOT LD06/LD19 packet protocol.

LD06 emits a one-way stream at 230400 baud. Each packet is 47 bytes long and
contains twelve distance/intensity samples. The driver decodes packets and
aggregates them into one scan per revolution for the LunaPre facade.
"""

from __future__ import annotations

import logging
import math
import os
import struct
import threading
import time

import serial

from ReasonData import Preference, Settings
from .lidar_driver_ld07 import LD07Driver


LOGGER = logging.getLogger(__name__)

PACKET_HEADER = 0x54
PACKET_HEADER_BYTES = b"\x54\x2c"
PACKET_VERSION_LENGTH = 0x2C
PACKET_LENGTH = 47
PACKET_FIXED_LENGTH = 6  # header + ver_len + speed + start angle
POINT_COUNT = 12
POINT_BYTES = 3
POINTS_OFFSET = 6
END_ANGLE_OFFSET = POINTS_OFFSET + POINT_COUNT * POINT_BYTES
TIMESTAMP_OFFSET = END_ANGLE_OFFSET + 2
CRC_OFFSET = TIMESTAMP_OFFSET + 2

DEFAULT_PORT = "/dev/ttyCH343USB3"
DEFAULT_BAUDRATE = 230400
DEFAULT_SERIAL_TIMEOUT = 0.2
DEFAULT_RECONNECT_DELAY = 0.5
DEFAULT_SCAN_MIN_POINTS = 5
DEFAULT_ANGLE_OFFSET_DEG = 0.0


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
    value = value.strip().lower()
    if value in {"1", "true", "yes", "on"}:
        return True
    if value in {"0", "false", "no", "off"}:
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


class LDLidarDriver:
    """Read LD06 packets and publish complete scans as angle/distance pairs."""

    def __init__(
        self,
        *,
        port: str | None = None,
        baudrate: int | None = None,
        timeout: float | None = None,
        angle_offset_deg: float | None = None,
        reconnect_delay: float | None = None,
        scan_min_points: int | None = None,
        verify_checksum: bool | None = None,
    ):
        configured_port = str(_setting_value("Ports", "LidarPort", DEFAULT_PORT))
        self.port = (
            port
            or os.environ.get("LDLIDAR_PORT")
            or os.environ.get("LIDAR_PORT")
            or configured_port
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
        self.angle_offset_deg = float(
            angle_offset_deg
            if angle_offset_deg is not None
            else _env_float("LDLIDAR_ANGLE_OFFSET_DEG", DEFAULT_ANGLE_OFFSET_DEG)
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
        self.last_scan: tuple[tuple[float, float], ...] = ()
        self.last_packet_point_count = 0
        self.last_scan_point_count = 0
        self.last_scan_frequency_hz = 0.0
        self.invalid_packet_count = 0
        self.last_rotation_timestamp = 0.0

    def start(self, scan_queue) -> bool:
        """Start the background serial reader."""
        if self._thread and self._thread.is_alive():
            return True
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self.run_forever,
            args=(scan_queue,),
            daemon=True,
            name="LDLidarReader",
        )
        self._thread.start()
        return True

    def stop(self) -> None:
        """Stop reading and close the serial port.

        LD06 is a one-way device, so stopping only closes the host-side port;
        no command is written to the sensor.
        """
        self._stop_event.set()
        lidar_serial = self._serial
        if lidar_serial is not None:
            try:
                lidar_serial.close()
            except Exception:
                pass
        self._serial = None

    def run_forever(self, scan_queue) -> None:
        """Reconnect as needed and publish one scan whenever the angle wraps."""
        while not self._stop_event.is_set():
            lidar_serial = None
            try:
                LOGGER.info(f"Opening LD06 port={self.port} baudrate={self.baudrate} (8N1)")
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

                buffer = bytearray()
                scan = []
                previous_angle = None
                while not self._stop_event.is_set():
                    chunk = lidar_serial.read(4096)
                    if chunk:
                        buffer.extend(chunk)
                    for packet in self._take_frames(buffer):
                        decoded = self.decode_packet(packet)
                        if decoded is None:
                            self.invalid_packet_count += 1
                            continue
                        self.last_packet = decoded
                        self.last_packet_point_count = decoded["sample_count"]
                        for sample in decoded["samples"]:
                            angle = sample["angle_deg"]
                            if (
                                previous_angle is not None
                                and previous_angle - angle > 180.0
                            ):
                                self._publish_rotation(scan_queue, scan)
                                scan = []
                            if sample["distance_m"] > 0:
                                scan.append((angle, sample["distance_m"]))
                            previous_angle = angle
            except (serial.SerialException, OSError) as error:
                if not self._stop_event.is_set():
                    LOGGER.warning(f"LD06 serial error: {error}; reconnecting")
            except Exception:
                if not self._stop_event.is_set():
                    LOGGER.exception("LD06 driver error; reconnecting")
            finally:
                if lidar_serial is not None:
                    try:
                        lidar_serial.close()
                    except Exception:
                        pass
                if self._serial is lidar_serial:
                    self._serial = None

            self._stop_event.wait(self.reconnect_delay)

    @staticmethod
    def _take_frames(buffer: bytearray):
        """Yield complete 47-byte packets and retain an incomplete tail."""
        while True:
            header_index = buffer.find(PACKET_HEADER_BYTES)
            if header_index < 0:
                if len(buffer) >= len(PACKET_HEADER_BYTES):
                    del buffer[:-1]
                return
            if header_index:
                del buffer[:header_index]
            if len(buffer) < PACKET_LENGTH:
                return
            frame = bytes(buffer[:PACKET_LENGTH])
            del buffer[:PACKET_LENGTH]
            yield frame

    def decode_packet(self, packet: bytes | bytearray) -> dict | None:
        """Decode one LD06 packet, returning ``None`` for invalid input."""
        packet = bytes(packet)
        if len(packet) != PACKET_LENGTH:
            return None
        if packet[0] != PACKET_HEADER or packet[1] != PACKET_VERSION_LENGTH:
            return None
        if self.verify_checksum and packet[CRC_OFFSET] != self.crc8(packet[:-1]):
            return None

        speed_deg_s = struct.unpack_from("<H", packet, 2)[0]
        start_angle_raw = struct.unpack_from("<H", packet, 4)[0]
        end_angle_raw = struct.unpack_from("<H", packet, END_ANGLE_OFFSET)[0]
        start_angle_deg = start_angle_raw / 100.0
        end_angle_deg = end_angle_raw / 100.0
        angle_span = (end_angle_raw - start_angle_raw) % 36000

        samples = []
        for index in range(POINT_COUNT):
            offset = POINTS_OFFSET + index * POINT_BYTES
            distance_mm = struct.unpack_from("<H", packet, offset)[0]
            intensity = packet[offset + 2]
            if POINT_COUNT > 1:
                raw_angle = start_angle_raw + angle_span * index / (POINT_COUNT - 1)
            else:
                raw_angle = start_angle_raw
            angle_deg = (raw_angle / 100.0 + self.angle_offset_deg) % 360.0
            distance_m = distance_mm / 1000.0
            samples.append(
                {
                    "angle_deg": angle_deg,
                    "distance_m": distance_m,
                    "distance_mm": distance_mm,
                    "intensity": intensity,
                    "confidence": intensity,
                    "forward_m": distance_m * math.cos(math.radians(angle_deg)),
                    "right_m": distance_m * math.sin(math.radians(angle_deg)),
                }
            )

        timestamp = struct.unpack_from("<H", packet, TIMESTAMP_OFFSET)[0]
        return {
            "header": packet[0],
            "version_length": packet[1],
            "speed_deg_s": speed_deg_s,
            "start_angle_deg": start_angle_deg,
            "end_angle_deg": end_angle_deg,
            "timestamp": timestamp,
            "sample_count": POINT_COUNT,
            "samples": samples,
            "checksum": packet[CRC_OFFSET],
        }

    def _publish_rotation(self, scan_queue, scan) -> None:
        if len(scan) < self.scan_min_points:
            return
        rotation = tuple(scan)
        self.last_scan = rotation
        self.last_scan_point_count = len(rotation)
        now = time.monotonic()
        if self.last_rotation_timestamp:
            elapsed = now - self.last_rotation_timestamp
            if elapsed > 0:
                self.last_scan_frequency_hz = 1.0 / elapsed
        self.last_rotation_timestamp = now
        self._publish_scan(scan_queue, rotation)

    def detect_objects(
        self,
        *,
        min_distance_mm: int = 30,
        max_distance_mm: int = 300,
        max_angle_gap_deg: float = 2.0,
        min_points: int = 3,
    ) -> list[dict]:
        """Group the latest near-field points into relative target clusters."""
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
            clusters.append(
                {
                    "x_forward_cm": round(forward_m * 100.0, 2),
                    "y_right_cm": round(lateral_m * 100.0, 2),
                    "y_left_cm": round(-lateral_m * 100.0, 2),
                    "distance_cm": round(distance_m * 100.0, 2),
                    "angle_deg": round(math.degrees(math.atan2(lateral_m, forward_m)), 2),
                    "point_count": len(points),
                    "confidence": round(
                        sum(sample["confidence"] for sample in points) / len(points),
                        2,
                    ),
                    "min_distance_mm": min(sample["distance_mm"] for sample in points),
                    "max_distance_mm": max(sample["distance_mm"] for sample in points),
                }
            )
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
    def crc8(data: bytes | bytearray) -> int:
        """Calculate the LDROBOT CRC8 used over packet bytes 0 through 45."""
        crc = 0
        for value in data:
            for _ in range(8):
                mix = (crc ^ value) & 0x80
                crc = (crc << 1) & 0xFF
                if mix:
                    crc ^= 0x4D
                value = (value << 1) & 0xFF
        return crc

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


LD06Driver = LDLidarDriver
