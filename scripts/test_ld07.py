#!/usr/bin/env python3
"""Probe an LD07 over a serial adapter.

Run from the Software directory:
    python3 scripts/test_ld07.py --port /dev/ttyACM0
"""

from __future__ import annotations

import argparse
import binascii
import os
import sys
import time
from pathlib import Path

import serial


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from LunaPre.lidar_driver_ydlidar import (  # noqa: E402
    DEVICE_ADDRESS,
    GET_DISTANCE_COMMAND,
    LD07Driver,
    STOP_COMMAND,
    START_MEASUREMENT_COMMAND,
    SYNC,
)


DEVICE_COUNT_COMMAND = bytes(
    [0xAA, 0xAA, 0xAA, 0xAA, 0x00, 0x16, 0x00, 0x00, 0x00, 0x00, 0x16]
)


def _read_for(port: serial.Serial, seconds: float) -> bytes:
    data = bytearray()
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        chunk = port.read(4096)
        if chunk:
            data.extend(chunk)
    return bytes(data)


def _send_and_read(port: serial.Serial, command: bytes, seconds: float) -> bytes:
    port.reset_input_buffer()
    port.write(command)
    port.flush()
    return _read_for(port, seconds)


def main() -> int:
    parser = argparse.ArgumentParser(description="Probe an LD07 serial connection")
    parser.add_argument("--port", default=os.environ.get("YDLIDAR_PORT", "/dev/ttyACM0"))
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--raw-bytes", type=int, default=256)
    args = parser.parse_args()

    print(f"port={args.port} baudrate={args.baudrate} (8N1)")
    print(f"start command: {START_MEASUREMENT_COMMAND.hex(' ')}")
    print(f"stop command:  {STOP_COMMAND.hex(' ')}")
    print(f"sync: {SYNC.hex(' ')} device={DEVICE_ADDRESS} command=0x{GET_DISTANCE_COMMAND:02x}")

    driver = LD07Driver(port=args.port, baudrate=args.baudrate)
    buffer = bytearray()
    decoded_count = 0
    invalid_count = 0

    try:
        with serial.Serial(
            args.port,
            baudrate=args.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.2,
        ) as port:
            response = _send_and_read(port, DEVICE_COUNT_COMMAND, 0.5)
            print(f"device query: {len(response)} bytes")
            if response:
                print(f"device query hex: {binascii.hexlify(response[:args.raw_bytes]).decode()}")

            port.reset_input_buffer()
            port.write(START_MEASUREMENT_COMMAND)
            port.flush()
            deadline = time.monotonic() + args.seconds
            while time.monotonic() < deadline:
                chunk = port.read(4096)
                if not chunk:
                    continue
                buffer.extend(chunk)
                for frame in driver._take_frames(buffer):
                    decoded = driver.decode_packet(frame)
                    if decoded is None:
                        invalid_count += 1
                        continue
                    decoded_count += 1
                    distances = [sample["distance_mm"] for sample in decoded["samples"]]
                    valid = [value for value in distances if value > 0]
                    print(
                        "frame=%d bytes=%d timestamp=%d points=%d valid=%d min_mm=%s max_mm=%s"
                        % (
                            decoded_count,
                            len(frame),
                            decoded["timestamp"],
                            decoded["sample_count"],
                            len(valid),
                            min(valid) if valid else "-",
                            max(valid) if valid else "-",
                        )
                    )
                    print(f"frame hex: {binascii.hexlify(frame[:args.raw_bytes]).decode()}")
            port.write(STOP_COMMAND)
            port.flush()
    except (serial.SerialException, OSError) as error:
        print(f"serial error: {error}", file=sys.stderr)
        return 1

    print(f"raw buffered bytes remaining: {len(buffer)}")
    print(f"decoded frames: {decoded_count}; invalid frames: {invalid_count}")
    if decoded_count == 0:
        print("No valid LD07 frame received. Check TX->adapter RX, GND, power, and baudrate.")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
