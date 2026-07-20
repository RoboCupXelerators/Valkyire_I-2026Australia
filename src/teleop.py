"""Conservative keyboard teleop for the DM-S2325 chassis backend.

Each motion key is also a deadman heartbeat. If key input stops, the chassis
command is zeroed; process exit always follows the backend shutdown path.
"""

from __future__ import annotations

import argparse
import os
import select
import sys
import termios
import time
import tty


LINEAR_MPS = 0.02
ANGULAR_RAD_S = 0.10
DEADMAN_TIMEOUT_S = 0.15

TWISTS = {
    "w": (LINEAR_MPS, 0.0, 0.0),
    "s": (-LINEAR_MPS, 0.0, 0.0),
    "a": (0.0, LINEAR_MPS, 0.0),
    "d": (0.0, -LINEAR_MPS, 0.0),
    "q": (0.0, 0.0, ANGULAR_RAD_S),
    "e": (0.0, 0.0, -ANGULAR_RAD_S),
}


class TerminalKeys:
    def __enter__(self):
        if not sys.stdin.isatty():
            raise RuntimeError("teleop requires an interactive terminal")
        self._fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, *_):
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)

    def read(self, timeout_s):
        ready, _, _ = select.select([sys.stdin], [], [], timeout_s)
        return sys.stdin.read(1).lower() if ready else None


def run_teleop(bits, key_source, *, clock=time.monotonic):
    """Run until ESC; `bits` is injectable so safety behavior is unit-testable."""
    last_motion = None
    moving = False
    try:
        while True:
            key = key_source.read(0.02)
            now = clock()
            if key == "\x1b":
                break
            if key == "r":
                bits.Arm()
                continue
            if key == " ":
                bits.stop()
                moving = False
                last_motion = None
                continue
            if key in TWISTS:
                bits.SetChassisTwist(*TWISTS[key])
                moving = True
                last_motion = now
            elif moving and last_motion is not None and now - last_motion >= DEADMAN_TIMEOUT_S:
                bits.stop()
                moving = False
                last_motion = None
    finally:
        # No motion command is permitted after shutdown starts.
        bits.shutdown()


def main():
    parser = argparse.ArgumentParser(description="DM-S2325 low-speed chassis teleop")
    parser.add_argument("--port", default=os.environ.get("DM2325_PORT", "/dev/ttyACM0"))
    args = parser.parse_args()
    from utils.DM2325Bits import DM2325Bits

    bits = DM2325Bits(port=args.port)
    bits.HandShake()
    with TerminalKeys() as keys:
        run_teleop(bits, keys)


if __name__ == "__main__":
    main()
