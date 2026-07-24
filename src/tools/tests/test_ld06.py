#!/usr/bin/env python3
"""Probe an LD06 over a serial adapter.

Run from the Software directory:
    python3 src/tools/tests/test_ld06.py --port /dev/ttyCH343USB3
"""

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

from LunaPre.lidar_driver_ldlidar import LDLidarDriver  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="Probe an LD06 serial connection")
    parser.add_argument(
        "--port",
        default=os.environ.get("LDLIDAR_PORT", "/dev/ttyCH343USB3"),
    )
    parser.add_argument("--baudrate", type=int, default=230400)
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--raw-bytes", type=int, default=47)
    args = parser.parse_args()

    driver = LDLidarDriver(port=args.port, baudrate=args.baudrate)
    buffer = bytearray()
    decoded_count = 0
    invalid_count = 0
    print(f"port={args.port} baudrate={args.baudrate} (8N1)")
    print("protocol: header=54 2c, packet_length=47, points=12, crc8=0x4d")

    try:
        with serial.Serial(
            args.port,
            baudrate=args.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.2,
        ) as port:
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
                        "packet=%d speed_deg_s=%d start=%.2f end=%.2f "
                        "points=%d valid=%d min_mm=%s max_mm=%s"
                        % (
                            decoded_count,
                            decoded["speed_deg_s"],
                            decoded["start_angle_deg"],
                            decoded["end_angle_deg"],
                            decoded["sample_count"],
                            len(valid),
                            min(valid) if valid else "-",
                            max(valid) if valid else "-",
                        )
                    )
                    if decoded_count == 1:
                        print(
                            "packet hex: "
                            + binascii.hexlify(frame[: args.raw_bytes]).decode()
                        )
                        for sample in decoded["samples"][:5]:
                            print(
                                "  angle=%7.2f deg distance=%5d mm -> "
                                "forward=%7.2f cm right=%7.2f cm intensity=%d"
                                % (
                                    sample["angle_deg"],
                                    sample["distance_mm"],
                                    sample["forward_m"] * 100.0,
                                    sample["right_m"] * 100.0,
                                    sample["intensity"],
                                )
                            )
    except (serial.SerialException, OSError) as error:
        print(f"serial error: {error}", file=sys.stderr)
        return 1

    print(f"raw buffered bytes remaining: {len(buffer)}")
    print(f"valid packets: {decoded_count}; invalid packets: {invalid_count}")
    if decoded_count == 0:
        print("No valid LD06 packet received. Check TX->adapter RX, GND, power, and baudrate.")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
