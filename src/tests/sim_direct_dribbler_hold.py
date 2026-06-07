"""Direct Unity dribbler hold smoke test.

This test uses the real XEL UDP command path rather than scene-control dribble
overrides. It places one robot and the ball, streams dribble + twist commands,
and records whether strict possession stays true while the robot drives.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from pathlib import Path
from typing import Any

SOFTWARE_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SOFTWARE_ROOT / "src"))

from utils.XeleratorSceneControl import XeleratorSceneControl
from utils.XeleratorSimBits import XeleratorSimBits


def _robot(status: dict[str, Any], robot_id: int) -> dict[str, Any]:
    for item in status.get("robots", []):
        if int(item.get("id", -1)) == int(robot_id):
            return item
    return {}


def _finite_float(value: Any) -> float | None:
    try:
        x = float(value)
    except Exception:
        return None
    return x if math.isfinite(x) else None


def _stats(samples: list[dict[str, Any]], key: str) -> dict[str, float] | None:
    values = [_finite_float(item.get(key)) for item in samples]
    values = [value for value in values if value is not None]
    if not values:
        return None
    return {"min": min(values), "max": max(values), "mean": statistics.fmean(values)}


def _first_strict_drop(samples: list[dict[str, Any]]) -> dict[str, Any] | None:
    for item in samples:
        if not (item.get("bits_has_ball") and item.get("scene_has_ball") and item.get("window")):
            return item
    return None


def _first_control_drop(samples: list[dict[str, Any]]) -> dict[str, Any] | None:
    for item in samples:
        if not (item.get("window") or item.get("recovery_window")):
            return item
    return None


def _write_jsonl(path: Path, payload: dict[str, Any]) -> None:
    with path.open("a", encoding="utf-8") as fh:
        fh.write(json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n")


def run(args: argparse.Namespace) -> dict[str, Any]:
    out_dir = Path(args.output) / time.strftime("run_%Y%m%d_%H%M%S")
    out_dir.mkdir(parents=True, exist_ok=True)
    trace_path = out_dir / "trace.jsonl"

    ctl = XeleratorSceneControl(timeout_s=float(args.scene_timeout))

    def sample(bits: XeleratorSimBits, phase: str, t0: float) -> dict[str, Any]:
        status = ctl.status()
        r = _robot(status, int(args.robot_id))
        ball = status.get("ball") or {}
        item = {
            "phase": phase,
            "t": round(time.time() - t0, 4),
            "bits_has_ball": bool(bits.HasBall()),
            "scene_has_ball": bool(r.get("has_ball")),
            "window": bool(r.get("ball_in_dribbler_window")),
            "recovery_window": bool(r.get("ball_in_dribbler_recovery_window")),
            "dribble_command": bool(r.get("dribble_command")),
            "robot_x_cm": r.get("x_cm"),
            "robot_y_cm": r.get("y_cm"),
            "robot_yaw_deg": r.get("yaw_deg"),
            "ball_x_cm": ball.get("x_cm"),
            "ball_y_cm": ball.get("y_cm"),
            "ball_speed_cm_s": ball.get("speed_cm_s"),
            "ball_forward_cm": r.get("ball_forward_cm"),
            "target_error_cm": r.get("ball_center_target_error_cm"),
            "surface_gap_cm": r.get("ball_surface_gap_cm"),
            "dribbler_center_forward_cm": r.get("dribbler_center_forward_cm"),
            "capture_depth_cm": r.get("dribble_capture_depth_cm"),
            "gap_tolerance_cm": r.get("dribble_surface_gap_tolerance_cm"),
            "recovery_depth_cm": r.get("dribble_recovery_depth_cm"),
            "recovery_gap_tolerance_cm": r.get("dribble_recovery_surface_gap_tolerance_cm"),
        }
        _write_jsonl(trace_path, item)
        return item

    def collect(
        bits: XeleratorSimBits,
        phase: str,
        duration_s: float,
        vx: float,
        t0: float,
        interval_s: float,
    ) -> list[dict[str, Any]]:
        samples: list[dict[str, Any]] = []
        start = time.time()
        next_sample = start
        while time.time() - start < duration_s:
            bits.SetTwist(vx, 0.0, 0.0)
            bits.VirtualPorts(4, float(args.dribble_strength))
            now = time.time()
            if now >= next_sample:
                samples.append(sample(bits, phase, t0))
                next_sample += interval_s
            time.sleep(0.01)
        return samples

    ctl.reset_score()
    ctl.set_rules(penalties=False, lack=False, pushing=False)

    # Toggle all robots once after a script reload so runtime dribbler defaults
    # are applied even to objects that were previously inactive.
    for rid in (1, 2, 3, 4):
        yaw = 0.0 if rid <= 2 else 180.0
        ctl.set_robot(rid, -100.0 + rid * 20.0, 80.0, yaw, active=True)
    time.sleep(0.2)

    for rid in (1, 2, 3, 4):
        if rid != int(args.robot_id):
            ctl.set_robot(rid, 120.0 + rid * 5.0, 80.0, 0.0, active=False)

    ctl.set_robot(int(args.robot_id), 0.0, 0.0, 0.0, active=True)
    ctl.set_ball(float(args.ball_x), float(args.ball_y), reset_velocity=True)
    time.sleep(0.25)

    bits = XeleratorSimBits(
        cmd_port=int(args.cmd_port),
        status_port=int(args.status_port),
        robot_id=int(args.robot_id),
        hz=float(args.hz),
    )

    try:
        bits.SetTwist(0.0, 0.0, 0.0)
        bits.VirtualPorts(4, float(args.dribble_strength))
        t0 = time.time()
        settle = collect(bits, "settle_dribble_on", float(args.settle_s), 0.0, t0, float(args.sample_interval_s))
        static = collect(bits, "static_hold", float(args.static_s), 0.0, t0, float(args.sample_interval_s))
        forward = collect(
            bits,
            "forward_drive",
            float(args.forward_s),
            float(args.forward_vx),
            t0,
            float(args.sample_interval_s),
        )
        brake = collect(bits, "brake_hold", float(args.brake_s), 0.0, t0, float(args.sample_interval_s))
        bits.SetTwist(0.0, 0.0, 0.0)
        bits.VirtualPorts(4, 0.0)
        time.sleep(0.2)
        final = sample(bits, "final_dribble_off", t0)
    finally:
        try:
            bits.SetTwist(0.0, 0.0, 0.0)
            bits.VirtualPorts(4, 0.0)
        except Exception:
            pass
        try:
            bits.Close()
        except Exception:
            pass

    samples = settle + static + forward + brake + [final]
    relevant = [item for item in samples if item["phase"] in {"static_hold", "forward_drive", "brake_hold"}]
    summary = {
        "out_dir": str(out_dir),
        "trace": str(trace_path),
        "sample_count": len(samples),
        "held_all_relevant_strict": _first_strict_drop(relevant) is None,
        "held_all_forward_strict": _first_strict_drop(forward) is None,
        "controlled_all_relevant": _first_control_drop(relevant) is None,
        "controlled_all_forward": _first_control_drop(forward) is None,
        "first_strict_drop_relevant": _first_strict_drop(relevant),
        "first_strict_drop_forward": _first_strict_drop(forward),
        "first_control_drop_relevant": _first_control_drop(relevant),
        "first_control_drop_forward": _first_control_drop(forward),
        "counts": {
            "relevant": len(relevant),
            "forward": len(forward),
            "has_ball": sum(1 for item in relevant if item.get("scene_has_ball")),
            "window": sum(1 for item in relevant if item.get("window")),
            "recovery": sum(1 for item in relevant if item.get("recovery_window")),
        },
        "forward_distance_cm": _stats(relevant, "ball_forward_cm"),
        "target_error_cm": _stats(relevant, "target_error_cm"),
        "surface_gap_cm": _stats(relevant, "surface_gap_cm"),
        "ball_speed_forward_cm_s": _stats(forward, "ball_speed_cm_s"),
        "robot_x_forward_cm": _stats(forward, "robot_x_cm"),
        "ball_x_forward_cm": _stats(forward, "ball_x_cm"),
        "first_sample": samples[0] if samples else None,
        "last_forward_sample": forward[-1] if forward else None,
        "final_sample": final,
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a direct Unity UDP dribbler hold test.")
    parser.add_argument("--robot-id", type=int, default=1)
    parser.add_argument("--cmd-port", type=int, default=20001)
    parser.add_argument("--status-port", type=int, default=20002)
    parser.add_argument("--hz", type=float, default=60.0)
    parser.add_argument("--ball-x", type=float, default=8.5)
    parser.add_argument("--ball-y", type=float, default=0.0)
    parser.add_argument("--dribble-strength", type=float, default=1.0)
    parser.add_argument("--forward-vx", type=float, default=0.75)
    parser.add_argument("--settle-s", type=float, default=0.8)
    parser.add_argument("--static-s", type=float, default=1.0)
    parser.add_argument("--forward-s", type=float, default=3.0)
    parser.add_argument("--brake-s", type=float, default=1.0)
    parser.add_argument("--sample-interval-s", type=float, default=0.1)
    parser.add_argument("--scene-timeout", type=float, default=1.0)
    parser.add_argument("--output", default="Records/alpha_debug/direct_dribbler_hold")
    args = parser.parse_args()

    summary = run(args)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if summary["held_all_relevant_strict"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
