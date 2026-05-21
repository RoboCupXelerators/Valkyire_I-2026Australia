#!/usr/bin/env python3
"""Run LockBallSlip with the PID branch on the real robot and log CSV data."""
from __future__ import annotations

import argparse
import math
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))


LOG_PATH = ROOT_DIR / "log" / "lockball_pid.csv"
NO_BALL = [0xFFF, 0xFFF]
CAUGHT_BALL = [0xDDD, 0xDDD]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Tune BasicFuc.LockBallSlip PID on hardware.")
    parser.add_argument("duration_pos", nargs="?", type=float, help="Run duration in seconds, kept for old usage.")
    parser.add_argument("-d", "--duration", type=float, default=20.0, help="Run duration in seconds.")
    parser.add_argument("--period", type=float, default=0.03, help="Control loop period in seconds.")
    parser.add_argument("--kp-x", type=float, default=3.0)
    parser.add_argument("--ki-x", type=float, default=0.0)
    parser.add_argument("--kd-x", type=float, default=0.0)
    parser.add_argument("--kp-y", type=float, default=2.4)
    parser.add_argument("--ki-y", type=float, default=0.0)
    parser.add_argument("--kd-y", type=float, default=0.0)
    parser.add_argument("--kp-w", type=float, default=1.2)
    parser.add_argument("--ki-w", type=float, default=0.0)
    parser.add_argument("--kd-w", type=float, default=0.0)
    parser.add_argument("--limit-xy", type=float, default=120.0, help="PID output limit for vx/vy before chassis Kp.")
    parser.add_argument("--limit-w", type=float, default=80.0, help="PID output limit for angular speed.")
    parser.add_argument("--allow-reverse-x", action="store_true", help="Allow PID to command reverse x speed.")
    parser.add_argument("--acquire-kp-x", type=float, default=None, help="Fast acquire x gain before entering the lock cone.")
    parser.add_argument("--acquire-kp-y", type=float, default=None, help="Fast acquire y gain before entering the lock cone.")
    parser.add_argument("--acquire-kp-w", type=float, default=None, help="Fast acquire angular gain before entering the lock cone.")
    parser.add_argument("--acquire-max-x", type=float, default=None, help="Fast acquire x speed limit.")
    parser.add_argument("--acquire-max-y", type=float, default=None, help="Fast acquire y speed limit.")
    parser.add_argument("--acquire-max-w", type=float, default=None, help="Fast acquire angular speed limit.")
    parser.add_argument("--rear-acquire-kp-y", type=float, default=None, help="Rear-half-plane lateral gain during fast acquire.")
    parser.add_argument("--rear-acquire-max-y", type=float, default=None, help="Rear-half-plane lateral speed limit during fast acquire.")
    parser.add_argument("--sample-time", type=float, default=None, help="simple_pid sample_time override.")
    parser.add_argument("--vision-warmup", type=float, default=6.0, help="Wait this many seconds for vision to report a ball before starting control.")
    parser.add_argument("--warmup-period", type=float, default=0.1, help="Vision warmup polling period in seconds.")
    parser.add_argument("--allow-no-ball-start", action="store_true", help="Start the control loop even if warmup never sees a ball.")
    parser.add_argument("--front-start-only", action="store_true", help="Only start after a ball is inside the lock cone.")
    parser.add_argument("--rear-start-only", action="store_true", help="Only start after a ball is behind the robot.")
    parser.add_argument("--allow-back-ball-start", action="store_true", help="Allow warmup to start on a ball behind the robot.")
    parser.add_argument("--allow-side-ball-start", action="store_true", help="Allow warmup to start on a ball outside the lock cone.")
    parser.add_argument("--min-start-x", type=float, default=0.0, help="Minimum robot-frame x for warmup ball data.")
    parser.add_argument("--max-start-angle", type=float, default=85.0, help="Maximum absolute robot-frame ball angle for warmup.")
    parser.add_argument("--dry-run", action="store_true", help="Read sensors and print commands without moving motors.")
    parser.add_argument("--clear-log", action="store_true", help="Remove previous log/lockball_pid.csv before running.")
    return parser


def configure_pid(lock, args: argparse.Namespace) -> None:
    lock.pan_x.tunings = (args.kp_x, args.ki_x, args.kd_x)
    lock.pan_y.tunings = (args.kp_y, args.ki_y, args.kd_y)
    lock.pan_w.tunings = (args.kp_w, args.ki_w, args.kd_w)
    limit_xy = abs(args.limit_xy)
    limit_w = abs(args.limit_w)
    lock.pan_x.output_limits = (-limit_xy, limit_xy if args.allow_reverse_x else 0.0)
    lock.pan_y.output_limits = (-limit_xy, limit_xy)
    lock.pan_w.output_limits = (-limit_w, limit_w)
    for attr, value in (
        ("acquire_kp_x", args.acquire_kp_x),
        ("acquire_kp_y", args.acquire_kp_y),
        ("acquire_kp_w", args.acquire_kp_w),
        ("acquire_max_x", args.acquire_max_x),
        ("acquire_max_y", args.acquire_max_y),
        ("acquire_max_w", args.acquire_max_w),
        ("rear_acquire_kp_y", args.rear_acquire_kp_y),
        ("rear_acquire_max_y", args.rear_acquire_max_y),
    ):
        if value is not None:
            setattr(lock, attr, abs(value) if attr.endswith("_max_y") or attr.startswith("acquire_max_") else value)
    if args.sample_time is not None:
        lock.pan_x.sample_time = args.sample_time
        lock.pan_y.sample_time = args.sample_time
        lock.pan_w.sample_time = args.sample_time


def install_dry_run_hooks(basicfuc) -> None:
    def fake_move(speed_x, speed_y, speed_z):
        print(f"{time.time():.3f} dry_move vx={speed_x:.2f} vy={speed_y:.2f} wz={speed_z:.2f}")

    def fake_stop():
        print(f"{time.time():.3f} dry_stop")

    basicfuc.chassis.RobotMoveVector = fake_move
    basicfuc.chassis.stop = fake_stop


def has_usable_ball(
    pos,
    min_start_x: float = 0.0,
    max_start_angle: float = 70.0,
    allow_back_ball: bool = False,
    allow_side_ball: bool = False,
    rear_start_only: bool = False,
) -> bool:
    if pos in (None, NO_BALL, [0, 0]):
        return False
    if pos == CAUGHT_BALL:
        return not rear_start_only
    try:
        ball_x = float(pos[0])
        ball_y = float(pos[1])
    except (TypeError, ValueError, IndexError):
        return False
    if rear_start_only:
        return ball_x <= min_start_x
    if allow_back_ball:
        return True
    if ball_x <= min_start_x:
        return False
    if allow_side_ball:
        return True
    angle = math.degrees(math.atan2(ball_y, ball_x))
    return abs(angle) <= max_start_angle


def wait_for_vision_ball(
    united_position,
    timeout_s: float,
    period_s: float,
    min_start_x: float,
    max_start_angle: float,
    allow_back_ball: bool,
    allow_side_ball: bool,
    rear_start_only: bool,
):
    if timeout_s <= 0:
        return united_position.RobotBallPos(), True

    started = time.monotonic()
    deadline = started + timeout_s
    next_print = started
    last_pos = None
    last_exc = None
    last_debug = None

    print(f"Waiting up to {timeout_s:.1f}s for vision ball data...")
    while time.monotonic() < deadline:
        try:
            last_pos = united_position.RobotBallPos()
            last_debug = getattr(united_position, "vision_data", {}).get("ball_debug")
            last_exc = None
        except Exception as exc:
            last_exc = exc
            last_pos = None

        if has_usable_ball(last_pos, min_start_x, max_start_angle, allow_back_ball, allow_side_ball, rear_start_only):
            elapsed = time.monotonic() - started
            print(f"Vision ready after {elapsed:.2f}s: ball={last_pos} debug={last_debug}")
            return last_pos, True

        now = time.monotonic()
        if now >= next_print:
            if last_exc is None:
                print(f"{time.time():.3f} waiting vision ball={last_pos} debug={last_debug}")
            else:
                print(f"{time.time():.3f} waiting vision error={last_exc}")
            next_print = now + 1.0

        time.sleep(max(period_s, 0.01))

    print(f"Vision warmup timed out: last_ball={last_pos} debug={last_debug}")
    return last_pos, False


def print_vision_preflight(settings) -> None:
    rec = str(getattr(settings.RoboInfo, "Rec", "")).upper()
    if rec != "AI":
        return

    hailortcli = shutil.which("hailortcli")
    if hailortcli is None:
        print("Vision preflight: Rec=AI but hailortcli is not in PATH.")
        return

    try:
        result = subprocess.run(
            [hailortcli, "scan"],
            capture_output=True,
            text=True,
            timeout=3.0,
            check=False,
        )
    except Exception as exc:
        print(f"Vision preflight: hailortcli scan failed: {exc}")
        return

    output = (result.stdout + result.stderr).strip()
    if output:
        print(f"Vision preflight: hailortcli scan -> {output}")
    if "not found" in output.lower():
        print("Vision preflight: Hailo device is not available; vision may keep reporting no ball.")


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.front_start_only and args.rear_start_only:
        parser.error("--front-start-only and --rear-start-only cannot be used together.")
    duration_s = args.duration_pos if args.duration_pos is not None else args.duration

    if args.clear_log:
        try:
            LOG_PATH.unlink()
        except FileNotFoundError:
            pass

    import BasicFuc
    from BasicFuc import LockSeries, Settings
    from LunaPre import UnitedPosition

    Settings.Debug.Beta = True
    print_vision_preflight(Settings)

    if args.dry_run:
        install_dry_run_hooks(BasicFuc)

    lock = LockSeries()
    configure_pid(lock, args)

    try:
        allow_back_ball = args.allow_back_ball_start or not args.front_start_only
        allow_side_ball = args.allow_side_ball_start or not args.front_start_only
        ball, vision_ready = wait_for_vision_ball(
            UnitedPosition,
            args.vision_warmup,
            args.warmup_period,
            args.min_start_x,
            args.max_start_angle,
            allow_back_ball,
            allow_side_ball,
            args.rear_start_only,
        )
        if not vision_ready and not args.allow_no_ball_start:
            print("No usable ball data after warmup; exiting without starting PID control.")
            return 2

        print(f"Starting LockBallSlip for {duration_s:.2f}s with PID (Settings.Debug.Beta=True), initial_ball={ball}")
        print(f"PID x={lock.pan_x.tunings} setpoint={lock.pan_x.setpoint} limits={lock.pan_x.output_limits}")
        print(f"PID y={lock.pan_y.tunings} setpoint={lock.pan_y.setpoint} limits={lock.pan_y.output_limits}")
        print(f"PID w={lock.pan_w.tunings} setpoint={lock.pan_w.setpoint} limits={lock.pan_w.output_limits}")
        print(
            "Acquire "
            f"angle<={lock.max_lock_angle} "
            f"kp=({lock.acquire_kp_x}, {lock.acquire_kp_y}, {lock.acquire_kp_w}) "
            f"limits=({lock.acquire_max_x}, {lock.acquire_max_y}, {lock.acquire_max_w})"
        )
        print(
            "Rear acquire "
            f"kp_y={lock.rear_acquire_kp_y} "
            f"limit_y={lock.rear_acquire_max_y}"
        )
        print(f"Log: {LOG_PATH}")
        if args.dry_run:
            print("Dry-run enabled: motor commands are printed, not sent.")

        start = None
        next_tick = time.monotonic()
        while start is None or time.monotonic() - start < duration_s:
            try:
                res = lock.LockBallSlip()
                if start is None:
                    start = time.monotonic()
                    next_tick = start
                ball = getattr(BasicFuc.UnitedPosition, "ballPosOut", None)
                ball_debug = getattr(BasicFuc.UnitedPosition, "vision_data", {}).get("ball_debug")
                print(f"{time.time():.3f} ball={ball} debug={ball_debug} res={res}")
            except Exception as exc:
                if start is None:
                    start = time.monotonic()
                    next_tick = start
                print("LockBallSlip error:", exc)
            next_tick += args.period
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
    finally:
        try:
            UnitedPosition.Stop()
        except Exception as exc:
            print("UnitedPosition.Stop error:", exc)
        print("Finished run, robot stopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
