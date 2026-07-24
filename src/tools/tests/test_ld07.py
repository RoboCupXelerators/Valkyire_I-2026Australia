#!/usr/bin/env python3
"""Probe an LD07 over its 55AA serial protocol."""

from __future__ import annotations

import argparse
import binascii
import os
import sys
import time
from pathlib import Path

import serial


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src"))

from LunaPre.lidar_driver_ld07 import (  # noqa: E402
    GET_DISTANCE_COMMAND,
    LD07Driver,
    START_MEASUREMENT_COMMAND,
    STOP_COMMAND,
    SYNC,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="Probe an LD07 serial connection")
    parser.add_argument("--port", default=os.environ.get("LDLIDAR_PORT", "/dev/ttyACM0"))
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--raw-bytes", type=int, default=64)
    args = parser.parse_args()

    driver = LD07Driver(port=args.port, baudrate=args.baudrate)
    buffer = bytearray()
    decoded_count = 0
    invalid_count = 0
    print(f"port={args.port} baudrate={args.baudrate} (8N1)")
    print(
        "protocol: sync=%s device=1 command=0x%02x"
        % (SYNC.hex(" "), GET_DISTANCE_COMMAND)
    )

    try:
        with serial.Serial(
            args.port,
            baudrate=args.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.2,
        ) as port:
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
                        "frame=%d timestamp=%d points=%d valid=%d min_mm=%s max_mm=%s"
                        % (
                            decoded_count,
                            decoded["timestamp"],
                            decoded["sample_count"],
                            len(valid),
                            min(valid) if valid else "-",
                            max(valid) if valid else "-",
                        )
                    )
                    if decoded_count == 1:
                        print(
                            "frame hex: "
                            + binascii.hexlify(frame[: args.raw_bytes]).decode()
                        )
            try:
                port.write(STOP_COMMAND)
                port.flush()
            except serial.SerialException:
                pass
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
