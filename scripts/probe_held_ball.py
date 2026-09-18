#!/usr/bin/env python3
"""Probe whether the raw ball coordinate stays usable while the ball is held.

Hold the ball (dribble on, chassis stopped), then sample ``UnitedPosition.ballPos``
(raw vision frame) and ``RobotBallPos()`` (semantic state, may be BALL_HELD) for a
few seconds so we can see if the raw coordinate is present and how noisy it is.
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))


NO_BALL = [0xFFF, 0xFFF]
CAUGHT_BALL = [0xDDD, 0xDDD]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--duration", type=float, default=5.0)
    parser.add_argument("--period", type=float, default=0.05)
    parser.add_argument("--vision-warmup", type=float, default=20.0)
    parser.add_argument("--dribble", type=float, default=0.3)
    args = parser.parse_args()

    import BasicFuc
    from BasicFuc import LockSeries, Settings
    from LunaPre import UnitedPosition, peripheral, chassis

    Settings.Debug.Beta = True

    print(f"Waiting up to {args.vision_warmup:.1f}s for ball data...")
    deadline = time.monotonic() + args.vision_warmup
    last = None
    while time.monotonic() < deadline:
        last = UnitedPosition.RobotBallPos()
        if last not in (None, NO_BALL, CAUGHT_BALL, [0, 0]):
            break
        time.sleep(0.1)
    print(f"ball seen: {last}")

    lock = LockSeries()
    # Capture the ball first, then hold still and sample.
    capture_deadline = time.monotonic() + 6.0
    caught = False
    while time.monotonic() < capture_deadline:
        if lock.LockBallSlip():
            caught = True
            break
        time.sleep(0.03)
    print(f"locked={caught}")

    peripheral.Dribble(args.dribble)
    chassis.stop()

    xs, ys, raw_kinds, semantic = [], [], {}, {}
    sentinels = (NO_BALL, CAUGHT_BALL, [0, 0])
    start = time.monotonic()
    next_tick = start
    print("t      ballPos(raw)      RobotBallPos      semantic")
    while time.monotonic() - start < args.duration:
        raw = list(getattr(UnitedPosition, "ballPos", [None, None]))
        semantic_pos = UnitedPosition.RobotBallPos()
        if semantic_pos == CAUGHT_BALL:
            sem = "HELD"
        elif semantic_pos == NO_BALL:
            sem = "NOT_FOUND"
        elif semantic_pos == [0, 0]:
            sem = "ZERO"
        else:
            sem = "FREE"
        semantic[sem] = semantic.get(sem, 0) + 1
        raw_kinds[tuple(raw)] = raw_kinds.get(tuple(raw), 0) + 1
        # Track raw coordinates whenever they are a real reading (not a sentinel).
        if raw not in sentinels and raw[0] is not None:
            xs.append(float(raw[0]))
            ys.append(float(raw[1]))

        t = time.monotonic() - start
        print(f"{t:5.2f}  {str(raw):>16}  {str(semantic_pos):>16}  {sem}")
        next_tick += args.period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)

    print("---- summary ----")
    print(f"semantic counts: {semantic}")
    print(f"unique raw coords: {len(raw_kinds)}")
    if xs:
        print(
            f"raw x: n={len(xs)} mean={statistics.mean(xs):.2f} sd={statistics.pstdev(xs):.2f} "
            f"min={min(xs):.2f} max={max(xs):.2f}"
        )
        print(
            f"raw y: n={len(ys)} mean={statistics.mean(ys):.2f} sd={statistics.pstdev(ys):.2f} "
            f"min={min(ys):.2f} max={max(ys):.2f}"
        )
    else:
        print("raw x/y: no real-coordinate samples (raw was always a sentinel)")

    chassis.stop()
    peripheral.Dribble(False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
