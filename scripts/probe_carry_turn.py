#!/usr/bin/env python3
"""Log a hardware TurnTo_with_ball run: yaw, raw ball coordinate, command, result.

Captures the ball first, then repeatedly calls TurnTo_with_ball so we can see the
closed-loop carry behaviour (does the ball stay in the envelope, does the heading
converge, does the guard abort the orbit).
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

NO_BALL = [0xFFF, 0xFFF]
CAUGHT_BALL = [0xDDD, 0xDDD]
SENTINELS = {float(NO_BALL[0]), float(CAUGHT_BALL[0]), 0.0}

HEADER = [
    "run_id", "t", "target_deg", "yaw_deg", "error_deg",
    "raw_bx", "raw_by", "seated", "cmd", "result",
]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", type=float, default=90.0)
    parser.add_argument("-d", "--duration", type=float, default=6.0)
    parser.add_argument("--period", type=float, default=0.03)
    parser.add_argument("--vision-warmup", type=float, default=20.0)
    parser.add_argument("--trim", type=float, default=None, help="Override TURN_HOLD_Y_TRIM_MPS_PER_CM.")
    parser.add_argument("--out", type=Path, default=ROOT_DIR / "log" / "carry_turn.csv")
    args = parser.parse_args()

    import BasicFuc
    from BasicFuc import Settings
    from LunaPre import UnitedPosition, chassis, peripheral

    Settings.Debug.Beta = True
    if args.trim is not None:
        BasicFuc.TURN_HOLD_Y_TRIM_MPS_PER_CM = args.trim
        print(f"trim gain = {args.trim}")

    print(f"Waiting up to {args.vision_warmup:.1f}s for ball data...")
    deadline = time.monotonic() + args.vision_warmup
    last = None
    while time.monotonic() < deadline:
        last = UnitedPosition.RobotBallPos()
        if last not in (None, NO_BALL, CAUGHT_BALL, [0, 0]):
            break
        time.sleep(0.1)
    print(f"ball seen: {last}")

    # Capture first.
    capture_deadline = time.monotonic() + 6.0
    while time.monotonic() < capture_deadline:
        if BasicFuc.lockball.LockBallSlip():
            break
        time.sleep(0.03)
    print("capture done, starting carry turn")

    # Wrap (not replace) the motion functions so the robot still moves while we
    # record the commanded twist.
    cmd = [0.0, 0.0, 0.0]
    real_twist = chassis.RobotMoveTwist
    real_vector = chassis.RobotMoveVector
    real_stop = chassis.stop

    def rec_twist(vx, vy, wz):
        cmd[:] = (float(vx), float(vy), math.degrees(float(wz)))
        return real_twist(vx, vy, wz)

    def rec_vector(vx, vy, wz):
        cmd[:] = (float(vx), float(vy), float(wz))
        return real_vector(vx, vy, wz)

    def rec_stop(*a, **k):
        cmd[:] = (0.0, 0.0, 0.0)
        return real_stop(*a, **k)

    chassis.RobotMoveTwist = rec_twist
    chassis.RobotMoveVector = rec_vector
    chassis.stop = rec_stop

    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    new_file = not args.out.exists()
    handle = args.out.open("a", newline="", encoding="utf-8")
    writer = csv.writer(handle)
    if new_file:
        writer.writerow(HEADER)

    aligned_frames = 0
    guard_frames = 0
    start = time.monotonic()
    next_tick = start
    try:
        while time.monotonic() - start < args.duration:
            result = BasicFuc.TurnTo_with_ball(args.target)
            raw = list(getattr(UnitedPosition, "ballPos", [None, None]))
            yaw = chassis.GetYaw()
            error = ((args.target - yaw + 180) % 360) - 180
            try:
                bx, by = float(raw[0]), float(raw[1])
                has_raw = bx not in SENTINELS and by not in SENTINELS
            except (TypeError, ValueError, IndexError):
                bx = by = None
                has_raw = False
            seated = has_raw and BasicFuc._ball_seated(bx, by)
            if result:
                aligned_frames += 1
            if has_raw and not seated:
                guard_frames += 1
            t = time.monotonic() - start
            writer.writerow([
                run_id, round(t, 3), args.target, round(yaw, 2), round(error, 2),
                bx, by, int(seated), str(cmd), int(bool(result)),
            ])
            print(
                f"{t:5.2f} yaw={yaw:7.2f} err={error:7.2f} raw=({bx},{by}) "
                f"seated={seated} cmd={cmd[0]:.2f},{cmd[1]:.2f},{cmd[2]:.1f} res={result}"
            )
            next_tick += args.period
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
    finally:
        handle.close()
        chassis.stop()
        peripheral.Dribble(False)
        try:
            UnitedPosition.Stop()
        except Exception as exc:
            print("UnitedPosition.Stop error:", exc)

    print("---- summary ----")
    print(f"aligned_frames={aligned_frames} guard_frames={guard_frames} log={args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
