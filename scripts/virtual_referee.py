#!/usr/bin/env python3
"""Interactive PLAY/STOP virtual serial for local strategy testing."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from LunaPre.referee import VirtualRefereeSerial


def main() -> int:
    parser = argparse.ArgumentParser(description="Create an RCJ-compatible virtual referee serial port")
    parser.add_argument("--initial", choices=("PLAY", "STOP"), default="STOP")
    args = parser.parse_args()

    with VirtualRefereeSerial(initial_state=args.initial) as referee:
        print(referee.port, flush=True)
        print("Commands: play, stop, penalty, quit", file=sys.stderr, flush=True)
        for line in sys.stdin:
            command = line.strip().lower()
            if command == "play":
                referee.play()
            elif command == "stop":
                referee.stop()
            elif command == "penalty":
                referee.penalty()
            elif command in {"quit", "exit"}:
                return 0
            elif command:
                print(f"unknown command: {command}", file=sys.stderr, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
