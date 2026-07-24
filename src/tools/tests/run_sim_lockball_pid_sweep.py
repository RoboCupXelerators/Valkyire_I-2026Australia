#!/usr/bin/env python3
"""Run repeatable LockBallSlip PID sweeps in the Unity simulator."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any

ROOT_DIR = Path(__file__).resolve().parents[3]
SRC_DIR = ROOT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

os.environ.setdefault("ROBOCUPX_BITS", "SIM")

NO_BALL = [0xFFF, 0xFFF]
CAUGHT_BALL = [0xDDD, 0xDDD]


@dataclass(frozen=True)
class Scenario:
    name: str
    robot_x: float
    robot_y: float
    robot_yaw: float
    ball_x: float
    ball_y: float


@dataclass(frozen=True)
class PidProfile:
    name: str
    kp_x: float
    kp_y: float
    kp_w: float
    limit_x: float
    limit_y: float
    limit_w: float
    acquire_kp_x: float
    acquire_kp_y: float
    acquire_kp_w: float
    acquire_max_x: float
    acquire_max_y: float
    acquire_max_w: float
    max_lock_angle: float = 85.0
    lost_hold_s: float = 0.18


def default_scenarios() -> list[Scenario]:
    return [
        Scenario("front_center_55", -80.0, 0.0, 0.0, -25.0, 0.0),
        Scenario("front_left_55_18", -80.0, 0.0, 0.0, -25.0, 18.0),
        Scenario("front_right_55_18", -80.0, 0.0, 0.0, -25.0, -18.0),
        Scenario("far_left_70_30", -80.0, 0.0, 0.0, -10.0, 30.0),
    ]


def default_profiles() -> list[PidProfile]:
    return [
        PidProfile("current", 3.0, 2.4, 1.2, 120.0, 80.0, 80.0, 3.0, 2.5, 1.5, 90.0, 80.0, 150.0),
        PidProfile("balanced", 3.2, 2.3, 1.0, 120.0, 80.0, 75.0, 3.6, 2.8, 1.5, 115.0, 90.0, 160.0, 82.0, 0.18),
        PidProfile("fast", 3.6, 2.6, 1.1, 135.0, 95.0, 85.0, 4.2, 3.1, 1.7, 135.0, 105.0, 175.0, 82.0, 0.18),
    ]


def configure_lock(lock: Any, profile: PidProfile) -> None:
    lock.pan_x.tunings = (profile.kp_x, 0.0, 0.0)
    lock.pan_y.tunings = (profile.kp_y, 0.0, 0.0)
    lock.pan_w.tunings = (profile.kp_w, 0.0, 0.0)
    lock.pan_x.output_limits = (-abs(profile.limit_x), 0.0)
    lock.pan_y.output_limits = (-abs(profile.limit_y), abs(profile.limit_y))
    lock.pan_w.output_limits = (-abs(profile.limit_w), abs(profile.limit_w))
    lock.acquire_kp_x = profile.acquire_kp_x
    lock.acquire_kp_y = profile.acquire_kp_y
    lock.acquire_kp_w = profile.acquire_kp_w
    lock.acquire_max_x = abs(profile.acquire_max_x)
    lock.acquire_max_y = abs(profile.acquire_max_y)
    lock.acquire_max_w = abs(profile.acquire_max_w)
    lock.max_lock_angle = abs(profile.max_lock_angle)
    lock.lost_hold_seconds = max(0.0, profile.lost_hold_s)


def normal_ball_distance(pos: Any) -> float | None:
    if pos in (None, NO_BALL, CAUGHT_BALL):
        return None
    try:
        return math.hypot(float(pos[0]), float(pos[1]))
    except Exception:
        return None


def prepare_scene(scene_ctl: Any, scenario: Scenario, args: argparse.Namespace) -> None:
    scene_ctl.set_rules(penalties=False, lack=False, pushing=False)
    scene_ctl.set_robot_tuning(
        robot_id=0,
        linear_scale=args.linear_scale,
        lateral_scale=args.lateral_scale,
        angular_scale=args.angular_scale,
    )
    scene_ctl.set_robot(args.robot_id, scenario.robot_x, scenario.robot_y, scenario.robot_yaw, active=True)
    scene_ctl.set_ball(scenario.ball_x, scenario.ball_y)
    time.sleep(args.settle_s)


def run_case(profile: PidProfile, scenario: Scenario, args: argparse.Namespace, *, scene_ctl: Any) -> dict[str, Any]:
    import BasicFuc
    from BasicFuc import LockSeries
    from LunaPre import UnitedPosition, chassis, peripheral

    prepare_scene(scene_ctl, scenario, args)
    BasicFuc.Settings.Debug.Beta = True
    lock = LockSeries()
    configure_lock(lock, profile)

    started = time.monotonic()
    deadline = started + args.duration
    next_tick = started
    min_dist = None
    no_ball_frames = 0
    caught = False
    caught_t = None
    last_ball = None
    samples = 0

    try:
        while time.monotonic() < deadline:
            result = lock.LockBallSlip()
            now = time.monotonic()
            ball = UnitedPosition.RobotBallPos()
            last_ball = ball
            samples += 1

            dist = normal_ball_distance(ball)
            if dist is None:
                if ball == NO_BALL:
                    no_ball_frames += 1
            else:
                min_dist = dist if min_dist is None else min(min_dist, dist)

            if result or ball == CAUGHT_BALL:
                caught = True
                caught_t = now - started
                break

            next_tick += args.period
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
    finally:
        try:
            peripheral.Dribble(False)
        except Exception:
            pass
        chassis.stop()

    return {
        "profile": profile.name,
        "scenario": scenario.name,
        "caught": caught,
        "time_to_catch_s": round(caught_t, 4) if caught_t is not None else None,
        "min_dist_cm": round(min_dist, 4) if min_dist is not None else None,
        "no_ball_frames": no_ball_frames,
        "samples": samples,
        "last_ball": last_ball,
    }


def aggregate(results: list[dict[str, Any]]) -> list[dict[str, Any]]:
    by_profile: dict[str, list[dict[str, Any]]] = {}
    for row in results:
        by_profile.setdefault(str(row["profile"]), []).append(row)

    summary = []
    for name, rows in by_profile.items():
        caught_rows = [r for r in rows if r["caught"] and r["time_to_catch_s"] is not None]
        times = [float(r["time_to_catch_s"]) for r in caught_rows]
        misses = len(rows) - len(caught_rows)
        summary.append(
            {
                "profile": name,
                "cases": len(rows),
                "caught": len(caught_rows),
                "misses": misses,
                "median_time_to_catch_s": round(sorted(times)[len(times) // 2], 4) if times else None,
                "mean_time_to_catch_s": round(sum(times) / len(times), 4) if times else None,
            }
        )
    return sorted(summary, key=lambda r: (r["misses"], r["mean_time_to_catch_s"] or 999.0))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--period", type=float, default=0.03)
    parser.add_argument("--settle-s", type=float, default=0.20)
    parser.add_argument("--robot-id", type=int, default=1)
    parser.add_argument("--linear-scale", type=float, default=0.26)
    parser.add_argument("--lateral-scale", type=float, default=0.167)
    parser.add_argument("--angular-scale", type=float, default=2.20)
    parser.add_argument("--out", type=Path, default=Path("Records/sim_motion/lockball_pid_sweep_latest.json"))
    args = parser.parse_args()

    from utils.XeleratorSceneControl import XeleratorSceneControl

    scene_ctl = XeleratorSceneControl(timeout_s=1.0)
    scene_ctl.ping()

    results = []
    profiles = default_profiles()
    scenarios = default_scenarios()
    for profile in profiles:
        for scenario in scenarios:
            print(f"[LockBallSweep] profile={profile.name} scenario={scenario.name}")
            row = run_case(profile, scenario, args, scene_ctl=scene_ctl)
            print(json.dumps(row, ensure_ascii=False))
            results.append(row)

    payload = {
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime()),
        "robot_tuning": {
            "linear_scale": args.linear_scale,
            "lateral_scale": args.lateral_scale,
            "angular_scale": args.angular_scale,
        },
        "profiles": [asdict(p) for p in profiles],
        "scenarios": [asdict(s) for s in scenarios],
        "summary": aggregate(results),
        "results": results,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(payload["summary"], ensure_ascii=False, indent=2))
    print(f"Wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
