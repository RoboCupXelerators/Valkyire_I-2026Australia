"""Minimal YDLIDAR 0x55AA serial driver.

The driver uses only pyserial, which is already a project dependency.  It
publishes scans as ``(angle_deg, distance_m)`` tuples, matching the direct
RPLidar driver used by LunaPre.
"""

from __future__ import annotations

import logging
import math
import os
import threading
import time

import serial


LOGGER = logging.getLogger(__name__)

PACKET_HEADER = b"\xaa\x55"
PACKET_HEADER_VALUE = 0x55AA
PACKET_FIXED_LENGTH = 10  # PH + CT + LSN + FSA + LSA + CS
START_SCAN_COMMAND = b"\xa5\x60"
STOP_SCAN_COMMAND = b"\xa5\x65"

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUDRATE = 230400
DEFAULT_SERIAL_TIMEOUT = 0.2
DEFAULT_RECONNECT_DELAY = 0.5
DEFAULT_SCAN_MIN_POINTS = 5
DEFAULT_SAMPLE_BYTES = 3
DEFAULT_DISTANCE_MODE = "triangle"
DEFAULT_ANGLE_CORRECTION = True


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


class YDLidarDriver:
    """Read YDLIDAR ``0x55AA`` packets and publish complete scans.

    Each queue item is a tuple of ``(angle_deg, distance_m)``.  The public
    ``decode_packet`` method is also useful for offline protocol tests.
    """

    def __init__(
        self,
        *,
        port: str | None = None,
        baudrate: int | None = None,
        timeout: float | None = None,
        sample_bytes: int | None = None,
        distance_mode: str | None = None,
        angle_correction: bool | None = None,
        angle_offset_deg: float | None = None,
        reconnect_delay: float | None = None,
        scan_min_points: int | None = None,
    ):
        self.port = port or os.environ.get("YDLIDAR_PORT", DEFAULT_PORT)
        self.baudrate = int(baudrate or _env_int("YDLIDAR_BAUDRATE", DEFAULT_BAUDRATE))
        self.timeout = float(timeout if timeout is not None else _env_float("YDLIDAR_TIMEOUT", DEFAULT_SERIAL_TIMEOUT))
        self.sample_bytes = int(sample_bytes or _env_int("YDLIDAR_SAMPLE_BYTES", DEFAULT_SAMPLE_BYTES))
        if self.sample_bytes not in (2, 3):
            raise ValueError("sample_bytes must be 2 or 3")

        self.distance_mode = (distance_mode or os.environ.get("YDLIDAR_DISTANCE_MODE", DEFAULT_DISTANCE_MODE)).lower()
        if self.distance_mode not in {"triangle", "tof"}:
            raise ValueError("distance_mode must be 'triangle' or 'tof'")
        self.angle_correction = bool(
            angle_correction
            if angle_correction is not None
            else _env_bool("YDLIDAR_ANGLE_CORRECTION", DEFAULT_ANGLE_CORRECTION)
        )
        self.angle_offset_deg = float(
            angle_offset_deg
            if angle_offset_deg is not None
            else _env_float("YDLIDAR_ANGLE_OFFSET_DEG", 0.0)
        )
        self.reconnect_delay = float(
            reconnect_delay
            if reconnect_delay is not None
            else _env_float("YDLIDAR_RECONNECT_DELAY", DEFAULT_RECONNECT_DELAY)
        )
        self.scan_min_points = max(
            1,
            int(
                scan_min_points
                if scan_min_points is not None
                else _env_int("YDLIDAR_SCAN_MIN_POINTS", DEFAULT_SCAN_MIN_POINTS)
            ),
        )

        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._serial: serial.Serial | None = None
        self.last_packet: dict | None = None
        self.last_scan_frequency_hz = 0.0
        self.invalid_packet_count = 0

    def start(self, scan_queue) -> bool:
        """Start the background serial reader."""
        if self._thread and self._thread.is_alive():
            return True
        self._stop_event.clear()
        self._thread = threading.Thread(target=self.run_forever, args=(scan_queue,), daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        """Stop scanning and close the serial port."""
        self._stop_event.set()
        lidar_serial = self._serial
        if lidar_serial is not None:
            try:
                lidar_serial.write(STOP_SCAN_COMMAND)
            except Exception:
                pass
            try:
                lidar_serial.close()
            except Exception:
                pass
        self._serial = None

    def run_forever(self, scan_queue) -> None:
        """Reconnect as needed and publish one queue item per completed scan."""
        while not self._stop_event.is_set():
            lidar_serial = None
            try:
                LOGGER.info("Opening YDLIDAR port=%s baudrate=%s", self.port, self.baudrate)
                lidar_serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
                self._serial = lidar_serial
                lidar_serial.reset_input_buffer()
                lidar_serial.write(START_SCAN_COMMAND)
                lidar_serial.flush()

                buffer = bytearray()
                current_scan = []
                while not self._stop_event.is_set():
                    chunk = lidar_serial.read(4096)
                    if chunk:
                        buffer.extend(chunk)
                    for packet in self._take_packets(buffer):
                        decoded = self.decode_packet(packet)
                        if decoded is None:
                            self.invalid_packet_count += 1
                            continue
                        self.last_packet = decoded
                        self.last_scan_frequency_hz = decoded["scan_frequency_hz"]

                        if decoded["start_of_scan"] and current_scan:
                            if len(current_scan) >= self.scan_min_points:
                                self._publish_scan(scan_queue, current_scan)
                            current_scan = []

                        for sample in decoded["samples"]:
                            if sample["distance_m"] > 0:
                                current_scan.append((sample["angle_deg"], sample["distance_m"]))

                if current_scan and len(current_scan) >= self.scan_min_points:
                    self._publish_scan(scan_queue, current_scan)
            except (serial.SerialException, OSError) as error:
                if not self._stop_event.is_set():
                    LOGGER.warning("YDLIDAR serial error: %s; reconnecting", error)
            except Exception:
                if not self._stop_event.is_set():
                    LOGGER.exception("YDLIDAR driver error; reconnecting")
            finally:
                if lidar_serial is not None:
                    try:
                        lidar_serial.write(STOP_SCAN_COMMAND)
                    except Exception:
                        pass
                    try:
                        lidar_serial.close()
                    except Exception:
                        pass
                if self._serial is lidar_serial:
                    self._serial = None

            self._stop_event.wait(self.reconnect_delay)

    def _take_packets(self, buffer: bytearray):
        """Yield complete packets while retaining an incomplete tail."""
        while True:
            header_index = buffer.find(PACKET_HEADER)
            if header_index < 0:
                if len(buffer) > 1:
                    del buffer[:-1]
                return
            if header_index:
                del buffer[:header_index]
            if len(buffer) < PACKET_FIXED_LENGTH:
                return

            sample_count = buffer[3]
            if sample_count == 0:
                del buffer[0]
                continue
            sample_bytes = self.sample_bytes
            packet_length = PACKET_FIXED_LENGTH + sample_count * sample_bytes
            if len(buffer) < packet_length:
                return
            packet = bytes(buffer[:packet_length])
            del buffer[:packet_length]
            yield packet

    def decode_packet(self, packet: bytes | bytearray) -> dict | None:
        """Decode and checksum one complete YDLIDAR packet."""
        packet = bytes(packet)
        if len(packet) < PACKET_FIXED_LENGTH or packet[:2] != PACKET_HEADER:
            return None

        ct = packet[2]
        sample_count = packet[3]
        sample_bytes = (len(packet) - PACKET_FIXED_LENGTH) // sample_count if sample_count else 0
        if sample_bytes not in (2, 3) or len(packet) != PACKET_FIXED_LENGTH + sample_count * sample_bytes:
            return None

        fsa_raw = int.from_bytes(packet[4:6], "little")
        lsa_raw = int.from_bytes(packet[6:8], "little")
        checksum = int.from_bytes(packet[8:10], "little")
        if self._checksum(packet[2:8], packet[10:], sample_bytes) != checksum:
            return None

        fsa_deg = (fsa_raw >> 1) / 64.0
        lsa_deg = (lsa_raw >> 1) / 64.0
        angle_diff = (lsa_deg - fsa_deg) % 360.0
        samples = []
        for index in range(sample_count):
            offset = 10 + index * sample_bytes
            raw = packet[offset:offset + sample_bytes]
            if sample_bytes == 3:
                intensity = ((raw[1] & 0x03) << 8) | raw[0]
                distance_raw = (raw[2] << 8) | raw[1]
                distance_mm = distance_raw >> 2 if self.distance_mode == "triangle" else distance_raw
            else:
                intensity = None
                distance_raw = int.from_bytes(raw, "little")
                distance_mm = distance_raw / 4.0 if self.distance_mode == "triangle" else distance_raw

            distance_m = float(distance_mm) / 1000.0
            angle_deg = fsa_deg if sample_count == 1 else fsa_deg + angle_diff * index / (sample_count - 1)
            if self.angle_correction and self.distance_mode == "triangle" and distance_mm > 0:
                angle_deg += math.degrees(math.atan(21.8 * (155.3 - distance_mm) / (155.3 * distance_mm)))
            samples.append({
                "angle_deg": (angle_deg + self.angle_offset_deg) % 360.0,
                "distance_m": distance_m,
                "distance_mm": distance_mm,
                "intensity": intensity,
            })

        return {
            "ct": ct,
            "start_of_scan": bool(ct & 0x01),
            "scan_frequency_hz": ((ct >> 1) / 10.0) if (ct & 0x01) else 0.0,
            "sample_count": sample_count,
            "fsa_deg": fsa_deg,
            "lsa_deg": lsa_deg,
            "checksum": checksum,
            "samples": samples,
        }

    @staticmethod
    def _checksum(header: bytes, samples: bytes, sample_bytes: int) -> int:
        ct = header[0]
        sample_count = header[1]
        fsa = int.from_bytes(header[2:4], "little")
        lsa = int.from_bytes(header[4:6], "little")
        checksum = PACKET_HEADER_VALUE ^ fsa
        for offset in range(0, len(samples), sample_bytes):
            sample = samples[offset:offset + sample_bytes]
            if sample_bytes == 3:
                checksum ^= sample[0]
                checksum ^= (sample[2] << 8) | sample[1]
            else:
                checksum ^= int.from_bytes(sample, "little")
        checksum ^= (sample_count << 8) | ct
        checksum ^= lsa
        return checksum & 0xFFFF

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


YDLidar55AA = YDLidarDriver
