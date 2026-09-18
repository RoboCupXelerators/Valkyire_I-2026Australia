#!/usr/bin/env python3
"""Run LockBallSlip on the real robot and log PID data to CSV.

Only the normal-capture branch of ``LockSeries.LockBallSlip`` uses the
``pan_x / pan_y / pan_w`` PID controllers. The turn-only and rear-ball
branches are pure-P and are recorded with their own branch label so the PID
gains are never tuned from data produced by a different controller.
"""
from __future__ import annotations

import argparse
import csv
import math
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))


LOG_PATH = ROOT_DIR / "log" / "lockball_pid.csv"
NO_BALL = [0xFFF, 0xFFF]
CAUGHT_BALL = [0xDDD, 0xDDD]

CSV_HEADER = [
    "run_id",
    "t",
    "duration_t",
    "ball_x",
    "ball_y",
    "angle_deg",
    "branch",
    "cmd_x",
    "cmd_y",
    "cmd_w",
    "comp_px",
    "comp_ix",
    "comp_dx",
    "comp_py",
    "comp_iy",
    "comp_dy",
    "comp_pw",
    "comp_iw",
    "comp_dw",
    "sat_x",
    "sat_y",
    "sat_w",
    "res",
]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Tune BasicFuc.LockBallSlip PID on hardware.")
    parser.add_argument("duration_pos", nargs="?", type=float, help="Run duration in seconds, kept for old usage.")
    parser.add_argument("-d", "--duration", type=float, default=20.0, help="Run duration in seconds.")
    parser.add_argument("--period", type=float, default=0.03, help="Control loop period in seconds.")
    # Gains default to None: keep whatever Settings.Motion.LockBallSlip loaded.
    parser.add_argument("--kp-x", type=float, default=None, help="Override pan_x Kp (default: config).")
    parser.add_argument("--ki-x", type=float, default=None, help="Override pan_x Ki (default: config).")
    parser.add_argument("--kd-x", type=float, default=None, help="Override pan_x Kd (default: config).")
    parser.add_argument("--kp-y", type=float, default=None, help="Override pan_y Kp (default: config).")
    parser.add_argument("--ki-y", type=float, default=None, help="Override pan_y Ki (default: config).")
    parser.add_argument("--kd-y", type=float, default=None, help="Override pan_y Kd (default: config).")
    parser.add_argument("--kp-w", type=float, default=None, help="Override pan_w Kp (default: config).")
    parser.add_argument("--ki-w", type=float, default=None, help="Override pan_w Ki (default: config).")
    parser.add_argument("--kd-w", type=float, default=None, help="Override pan_w Kd (default: config).")
    parser.add_argument("--limit-x", type=float, default=None, help="pan_x forward speed limit in m/s (default: config).")
    parser.add_argument("--limit-y", type=float, default=None, help="pan_y lateral speed limit in m/s (default: config).")
    parser.add_argument("--limit-w", type=float, default=None, help="pan_w angular speed limit in deg/s (default: config).")
    parser.add_argument("--allow-reverse-x", action="store_true", help="Allow PID to command reverse x speed.")
    parser.add_argument("--near-dock-decel", type=float, default=None, help="Braking-curve deceleration in m/s^2 (default: config); lower docks more gently.")
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
    parser.add_argument("--out", type=Path, default=LOG_PATH, help="CSV output path.")
    parser.add_argument("--clear-log", action="store_true", help="Truncate the CSV before writing.")
    parser.add_argument("--quiet", action="store_true", help="Do not print one line per control tick.")
    return parser


def configure_pid(lock, args: argparse.Namespace) -> None:
    def apply(pid, kp, ki, kd):
        cur = pid.tunings
        new = (
            cur[0] if kp is None else kp,
            cur[1] if ki is None else ki,
            cur[2] if kd is None else kd,
        )
        if new != cur:
            pid.tunings = new

    apply(lock.pan_x, args.kp_x, args.ki_x, args.kd_x)
    apply(lock.pan_y, args.kp_y, args.ki_y, args.kd_y)
    apply(lock.pan_w, args.kp_w, args.ki_w, args.kd_w)

    if args.limit_x is not None:
        limit = abs(args.limit_x)
        lock.pan_x.output_limits = (-limit, limit if args.allow_reverse_x else 0.0)
    elif args.allow_reverse_x:
        lo, _hi = lock.pan_x.output_limits
        lock.pan_x.output_limits = (-abs(lo), abs(lo))
    if args.limit_y is not None:
        limit = abs(args.limit_y)
        lock.pan_y.output_limits = (-limit, limit)
    if args.limit_w is not None:
        limit = abs(args.limit_w)
        lock.pan_w.output_limits = (-limit, limit)

    if args.near_dock_decel is not None:
        lock.near_dock_decel = max(0.0, args.near_dock_decel)
    if args.sample_time is not None:
        lock.pan_x.sample_time = args.sample_time
        lock.pan_y.sample_time = args.sample_time
        lock.pan_w.sample_time = args.sample_time

    lock._reset_lockball_pid()


def install_dry_run_hooks(basicfuc) -> None:
    def fake_move(speed_x, speed_y, speed_z):
        print(f"{time.time():.3f} dry_move vx={speed_x:.2f} vy={speed_y:.2f} wz={speed_z:.2f}")

    def fake_stop():
        print(f"{time.time():.3f} dry_stop")

    def fake_dribble(status):
        print(f"{time.time():.3f} dry_dribble status={status}")

    def fake_shoot():
        print(f"{time.time():.3f} dry_shoot")

    def fake_set_io(port, status):
        print(f"{time.time():.3f} dry_set_io port={port} status={status}")

    basicfuc.chassis.RobotMoveVector = fake_move
    basicfuc.chassis.stop = fake_stop
    basicfuc.peripheral.Dribble = fake_dribble
    basicfuc.peripheral.ShootBall = fake_shoot
    basicfuc.peripheral.SetIO = fake_set_io


class CommandRecorder:
    """Records the last chassis command emitted by any LockBallSlip branch."""

    def __init__(self, chassis) -> None:
        self.cmd = (0.0, 0.0, 0.0)
        self._move = chassis.RobotMoveVector
        self._stop = chassis.stop

        def record_move(vx, vy, wz):
            self.cmd = (float(vx), float(vy), float(wz))
            return self._move(vx, vy, wz)

        def record_stop(*a, **k):
            self.cmd = (0.0, 0.0, 0.0)
            return self._stop(*a, **k)

        chassis.RobotMoveVector = record_move
        chassis.stop = record_stop


def infer_branch(ball, lock) -> str:
    if ball == CAUGHT_BALL:
        return "caught"
    if ball == NO_BALL:
        return "no_ball"
    try:
        ball_x = float(ball[0])
        ball_y = float(ball[1])
    except (TypeError, ValueError, IndexError):
        return "invalid"
    if ball_x == 0 and ball_y == 0:
        return "zero"
    angle = math.degrees(math.atan2(ball_y, ball_x))
    if ball_x > 0 and abs(angle) >= lock.turn_only_angle:
        return "turn_only"
    if ball_x <= 0:
        return "rear"
    return "pid"


def pid_row(pid) -> tuple[float, float, float, bool]:
    """Return (P, I, D, saturated) for the last computation of ``pid``."""
    try:
        px, ix, dx = (float(term) for term in pid.components)
    except Exception:
        return 0.0, 0.0, 0.0, False
    lo, hi = pid.output_limits
    total = px + ix + dx
    saturated = (hi is not None and total > hi) or (lo is not None and total < lo)
    return px, ix, dx, saturated


def ball_xy(ball):
    try:
        return float(ball[0]), float(ball[1])
    except (TypeError, ValueError, IndexError):
        return None, None


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


def open_log(path: Path, clear: bool):
    path.parent.mkdir(parents=True, exist_ok=True)
    write_header = clear or not path.exists() or path.stat().st_size == 0
    handle = path.open("w" if write_header else "a", newline="", encoding="utf-8")
    writer = csv.writer(handle)
    if write_header:
        writer.writerow(CSV_HEADER)
    return handle, writer


def summarize(rows: list[dict], start: float, end: float) -> None:
    if not rows:
        print("No samples logged.")
        return
    branches: dict[str, int] = {}
    for row in rows:
        branches[row["branch"]] = branches.get(row["branch"], 0) + 1
    catch = [r for r in rows if r["res"]]
    pid_rows = [r for r in rows if r["branch"] == "pid"]

    def sign_flips(values):
        flips = 0
        prev = 0
        dead = 0.01
        for v in values:
            if abs(v) < dead:
                continue
            s = 1 if v > 0 else -1
            if prev != 0 and s != prev:
                flips += 1
            prev = s
        return flips

    def peak(values):
        return max((abs(v) for v in values), default=0.0)

    print("---- run summary ----")
    print(f"samples={len(rows)} duration={end - start:.2f}s branches={branches}")
    if catch:
        print(f"caught=True time_to_catch={catch[0]['duration_t']:.2f}s")
    else:
        print("caught=False")
    if pid_rows:
        cmd_x = [r["cmd_x"] for r in pid_rows]
        cmd_y = [r["cmd_y"] for r in pid_rows]
        cmd_w = [r["cmd_w"] for r in pid_rows]
        print(
            "pid branch: "
            f"n={len(pid_rows)} "
            f"flips_x={sign_flips(cmd_x)} flips_y={sign_flips(cmd_y)} flips_w={sign_flips(cmd_w)} "
            f"peak_x={peak(cmd_x):.3f} peak_y={peak(cmd_y):.3f} peak_w={peak(cmd_w):.1f} "
            f"sat=({sum(r['sat_x'] for r in pid_rows)},{sum(r['sat_y'] for r in pid_rows)},{sum(r['sat_w'] for r in pid_rows)})"
        )
    print("---------------------")


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.front_start_only and args.rear_start_only:
        parser.error("--front-start-only and --rear-start-only cannot be used together.")
    duration_s = args.duration_pos if args.duration_pos is not None else args.duration

    import BasicFuc
    from BasicFuc import LockSeries, Settings
    from LunaPre import UnitedPosition

    Settings.Debug.Beta = True
    print_vision_preflight(Settings)

    if args.dry_run:
        install_dry_run_hooks(BasicFuc)

    recorder = CommandRecorder(BasicFuc.chassis)

    lock = LockSeries()
    configure_pid(lock, args)

    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    handle, writer = open_log(args.out, args.clear_log)

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

        print(f"Starting LockBallSlip for {duration_s:.2f}s (Settings.Debug.Beta=True), initial_ball={ball}")
        print(f"PID x={lock.pan_x.tunings} setpoint={lock.pan_x.setpoint} limits={lock.pan_x.output_limits}")
        print(f"PID y={lock.pan_y.tunings} setpoint={lock.pan_y.setpoint} limits={lock.pan_y.output_limits}")
        print(f"PID w={lock.pan_w.tunings} setpoint={lock.pan_w.setpoint} limits={lock.pan_w.output_limits}")
        print(f"Near dock: braking curve v<=sqrt(2*{lock.near_dock_decel}*d) m/s")

        print(f"Front turn-only: angle>={lock.turn_only_angle:.1f} kp={lock.turn_only_kp} max={lock.turn_only_max}")
        print(f"Rear: fwd_kp={lock.rear_forward_kp} lat_kp={lock.rear_lateral_kp} ang_kp={lock.rear_angular_kp}")
        print(f"Log: {args.out} (run_id={run_id})")
        if args.dry_run:
            print("Dry-run enabled: motor commands are recorded, not sent to motors.")

        rows: list[dict] = []
        start = None
        next_tick = time.monotonic()
        while start is None or time.monotonic() - start < duration_s:
            try:
                res = lock.LockBallSlip()
                if start is None:
                    start = time.monotonic()
                    next_tick = start
                now = time.monotonic()
                ball = getattr(UnitedPosition, "ballPosOut", None)
                branch = infer_branch(ball, lock)
                cmd_x, cmd_y, cmd_w = recorder.cmd
                px, ix, dx, sat_x = pid_row(lock.pan_x)
                py, iy, dy, sat_y = pid_row(lock.pan_y)
                pw, iw, dw, sat_w = pid_row(lock.pan_w)
                bx, by = ball_xy(ball)
                if branch in ("pid", "turn_only", "rear") and (bx not in (None, 0) or by not in (None, 0)):
                    angle = round(math.degrees(math.atan2(by, bx)), 2)
                else:
                    angle = None
                row = {
                    "run_id": run_id,
                    "t": f"{time.time():.3f}",
                    "duration_t": round(now - start, 4),
                    "ball_x": bx,
                    "ball_y": by,
                    "angle_deg": angle,
                    "branch": branch,
                    "cmd_x": round(cmd_x, 4),
                    "cmd_y": round(cmd_y, 4),
                    "cmd_w": round(cmd_w, 3),
                    "comp_px": round(px, 5),
                    "comp_ix": round(ix, 5),
                    "comp_dx": round(dx, 5),
                    "comp_py": round(py, 5),
                    "comp_iy": round(iy, 5),
                    "comp_dy": round(dy, 5),
                    "comp_pw": round(pw, 5),
                    "comp_iw": round(iw, 5),
                    "comp_dw": round(dw, 5),
                    "sat_x": int(sat_x),
                    "sat_y": int(sat_y),
                    "sat_w": int(sat_w),
                    "res": int(bool(res)),
                }
                rows.append(row)
                writer.writerow([row[col] for col in CSV_HEADER])
                if not args.quiet:
                    print(
                        f"{time.time():.3f} branch={branch} ball=({row['ball_x']},{row['ball_y']}) "
                        f"cmd=({cmd_x:.2f},{cmd_y:.2f},{cmd_w:.1f}) comp_x=({px:.2f},{ix:.3f},{dx:.2f}) res={res}"
                    )
            except Exception as exc:
                if start is None:
                    start = time.monotonic()
                    next_tick = start
                print("LockBallSlip error:", exc)
            next_tick += args.period
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)

        handle.flush()
        summarize(rows, start, time.monotonic())
    finally:
        handle.close()
        try:
            UnitedPosition.Stop()
        except Exception as exc:
            print("UnitedPosition.Stop error:", exc)
        print("Finished run, robot stopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
