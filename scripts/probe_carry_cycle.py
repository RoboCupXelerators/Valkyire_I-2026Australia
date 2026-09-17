#!/usr/bin/env python3
"""Cycle TurnTo_with_ball through several target angles on hardware.

Captures the ball once, then turns to each target in ``--targets`` in order,
logging per-segment time-to-align, guard (out-of-envelope) frames and ball
deviation so we can see stability across headings.
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
HEADER = [
    "run_id", "segment", "target_deg", "t", "yaw_deg", "error_deg",
    "raw_bx", "raw_by", "seated", "cmd_w", "result",
]


def parse_targets(text: str) -> list[float]:
    return [float(v) for v in text.replace(",", " ").split()]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--targets", default="90 -90 0 180 45 -135")
    parser.add_argument("--segment-timeout", type=float, default=8.0)
    parser.add_argument("--settle", type=float, default=0.4)
    parser.add_argument("--period", type=float, default=0.03)
    parser.add_argument("--vision-warmup", type=float, default=20.0)
    parser.add_argument("--lateral", type=float, default=None, help="Override TURN_ORBIT_LATERAL (carry orbit lateral scale).")
    parser.add_argument("--out", type=Path, default=ROOT_DIR / "log" / "carry_cycle.csv")
    args = parser.parse_args()
    targets = parse_targets(args.targets)

    import BasicFuc
    from BasicFuc import Settings
    from LunaPre import UnitedPosition, chassis, peripheral

    Settings.Debug.Beta = True
    if args.lateral is not None:
        BasicFuc.TURN_ORBIT_LATERAL = args.lateral
        print(f"orbit lateral = {args.lateral}")

    print(f"Waiting up to {args.vision_warmup:.1f}s for ball data...")
    deadline = time.monotonic() + args.vision_warmup
    last = None
    while time.monotonic() < deadline:
        last = UnitedPosition.RobotBallPos()
        if last not in (None, NO_BALL, CAUGHT_BALL, [0, 0]):
            break
        time.sleep(0.1)
    print(f"ball seen: {last}")

    # Capture once.
    cap_deadline = time.monotonic() + 6.0
    captured = False
    while time.monotonic() < cap_deadline:
        if BasicFuc.lockball.LockBallSlip():
            captured = True
            break
        time.sleep(0.03)
    if not captured:
        print("capture FAILED (ball not held); aborting before the cycle.")
        try:
            chassis.stop()
            peripheral.Dribble(False)
            UnitedPosition.Stop()
        except Exception as exc:
            print("cleanup error:", exc)
        return 2
    print("capture done; starting angle cycle")

    cmd_w = [0.0]
    real_twist = chassis.RobotMoveTwist
    real_vector = chassis.RobotMoveVector
    real_stop = chassis.stop

    def rec_twist(vx, vy, wz):
        cmd_w[0] = math.degrees(float(wz))
        return real_twist(vx, vy, wz)

    def rec_vector(vx, vy, wz):
        cmd_w[0] = float(wz)
        return real_vector(vx, vy, wz)

    def rec_stop(*a, **k):
        cmd_w[0] = 0.0
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

    summary = []
    try:
        for seg, target in enumerate(targets):
            seg_start = time.monotonic()
            guard = 0
            samples = 0
            max_dev = 0.0
            aligned_t = None
            printed = -1.0
            last_yaw = last_bx = last_by = None
            while time.monotonic() - seg_start < args.segment_timeout:
                result = BasicFuc.TurnTo_with_ball(target)
                raw = list(getattr(UnitedPosition, "ballPos", [None, None]))
                yaw = chassis.GetYaw()
                error = ((target - yaw + 180) % 360) - 180
                try:
                    bx, by = float(raw[0]), float(raw[1])
                    has_raw = bx not in (4095.0, 3549.0, 0.0) and by not in (4095.0, 3549.0, 0.0)
                except (TypeError, ValueError, IndexError):
                    bx = by = None
                    has_raw = False
                seated = has_raw and BasicFuc._ball_seated(bx, by)
                if has_raw and not seated:
                    guard += 1
                if has_raw:
                    dev = math.hypot(bx - BasicFuc.lockball.catch_x, by - BasicFuc.lockball.catch_y)
                    max_dev = max(max_dev, dev)
                samples += 1
                last_yaw, last_bx, last_by = yaw, bx, by
                t = time.monotonic() - seg_start
                writer.writerow([
                    run_id, seg, target, round(t, 3), round(yaw, 2), round(error, 2),
                    bx, by, int(seated), round(cmd_w[0], 1), int(bool(result)),
                ])
                if t - printed >= 0.2 or result:
                    print(
                        f"seg{seg} tgt={target:>6} t={t:4.2f} yaw={yaw:7.2f} err={error:7.2f} "
                        f"raw=({bx},{by}) guard={guard} cmdw={cmd_w[0]:6.1f} res={result}"
                    )
                    printed = t
                if result:
                    aligned_t = t
                    break
                next_tick = seg_start + samples * args.period
                sleep_s = next_tick - time.monotonic()
                if sleep_s > 0:
                    time.sleep(sleep_s)
            summary.append({
                "seg": seg, "target": target,
                "aligned_s": round(aligned_t, 2) if aligned_t is not None else None,
                "guard": guard, "max_dev_cm": round(max_dev, 2),
                "final_yaw": round(last_yaw, 1) if last_yaw is not None else None,
                "final_err": round(((target - (last_yaw or 0) + 180) % 360) - 180, 2),
                "raw": (last_bx, last_by),
            })
            chassis.stop()
            time.sleep(args.settle)
    finally:
        handle.close()
        chassis.stop()
        peripheral.Dribble(False)
        try:
            UnitedPosition.Stop()
        except Exception as exc:
            print("UnitedPosition.Stop error:", exc)

    print("==== cycle summary ====")
    print(f"{'seg':>3} {'target':>7} {'align_s':>8} {'guard':>6} {'maxdev':>7} {'final_err':>10} raw")
    for row in summary:
        print(
            f"{row['seg']:>3} {row['target']:>7} {str(row['aligned_s']):>8} "
            f"{row['guard']:>6} {row['max_dev_cm']:>7} {row['final_err']:>10} {row['raw']}"
        )
    n_aligned = sum(1 for r in summary if r["aligned_s"] is not None)
    print(f"aligned {n_aligned}/{len(summary)} segments; log={args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
