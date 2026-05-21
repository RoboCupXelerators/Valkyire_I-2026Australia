"""Single-robot simulator acceptance smoke suite.

Start Unity, then run:

    python -m src.tests.sim_acceptance

This checks the software stack through the simulator path, not mock objects.
It is intentionally conservative: short moves, bounded timeouts, and a JSON
report under Records/sim_acceptance.
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, "src")
os.environ.setdefault("ROBOCUPX_BITS", "SIM")

from LunaPre import Bits, UnitedPosition, chassis, peripheral, logger  # noqa: E402


INVALID_POSE = [0xFFF, 0xFFF, 0xFFF]


def _angle_diff_deg(current: float, start: float) -> float:
    return (float(current) - float(start) + 180.0) % 360.0 - 180.0


def _pose() -> list[float]:
    pose = UnitedPosition.FieldRobotPos()
    if not isinstance(pose, list) or len(pose) < 3 or pose == INVALID_POSE:
        return list(INVALID_POSE)
    return [float(pose[0]), float(pose[1]), float(pose[2])]


def _wait_for_sim(timeout_s: float = 6.0) -> bool:
    start_rx = int(getattr(Bits, "_rx_count", 0))
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if _pose() != INVALID_POSE and int(getattr(Bits, "_rx_count", 0)) > start_rx:
            return True
        time.sleep(0.2)
    return False


def _short_motion(cmd: tuple[float, float, float], hold_s: float = 0.8) -> dict:
    start = _pose()
    t0 = time.time()
    while time.time() - t0 < hold_s:
        chassis.RobotMoveVector(*cmd)
        time.sleep(0.02)
    chassis.stop()
    time.sleep(0.25)
    end = _pose()
    return {
        "start": start,
        "end": end,
        "dx_cm": round(end[0] - start[0], 3) if end != INVALID_POSE and start != INVALID_POSE else None,
        "dy_cm": round(end[1] - start[1], 3) if end != INVALID_POSE and start != INVALID_POSE else None,
        "dyaw_deg": round(_angle_diff_deg(end[2], start[2]), 3) if end != INVALID_POSE and start != INVALID_POSE else None,
    }


def _path_smoke() -> dict:
    origin = _pose()
    if origin == INVALID_POSE:
        return {"ok": False, "reason": "invalid origin pose"}

    x, y, yaw = origin
    path = [
        [x + 14.0, y, yaw],
        [x + 14.0, y + 12.0, yaw],
        [x, y + 12.0, yaw],
        [x, y, yaw],
    ]
    ok = UnitedPosition.Move2Path(path, 0.05, timeout_s=5.0, tick_s=0.02)
    end = _pose()
    return {
        "ok": bool(ok),
        "origin": [round(v, 3) for v in origin],
        "end": [round(v, 3) for v in end],
        "path": [[round(float(v), 3) for v in p] for p in path],
    }


def _ball_io_smoke() -> dict:
    ball = UnitedPosition.RobotBallPos()
    try:
        peripheral.Dribble(0.25)
        time.sleep(0.15)
        peripheral.Dribble(False)
        peripheral.ShootBall()
        return {"ok": True, "ball_pos": ball}
    except Exception as exc:
        return {"ok": False, "ball_pos": ball, "error": str(exc)}


def main() -> int:
    out_dir = Path(os.environ.get("SIM_ACCEPTANCE_OUTPUT", "Records/sim_acceptance")).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    report_path = out_dir / f"sim_acceptance_{time.strftime('%Y%m%d_%H%M%S')}.json"

    report: dict = {
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "checks": {},
    }

    try:
        connected = _wait_for_sim()
        report["checks"]["connection"] = {"ok": connected, "pose": _pose(), "rx_count": int(getattr(Bits, "_rx_count", 0))}
        if not connected:
            report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
            logger.error("[SimAcceptance] simulator connection failed; wrote {}", report_path)
            return 2

        report["checks"]["move_x"] = _short_motion((18.0, 0.0, 0.0))
        report["checks"]["move_y"] = _short_motion((0.0, 18.0, 0.0))
        report["checks"]["turn_w"] = _short_motion((0.0, 0.0, 16.0))
        report["checks"]["path"] = _path_smoke()
        report["checks"]["ball_io"] = _ball_io_smoke()

        ok = (
            abs(report["checks"]["move_x"].get("dx_cm") or 0.0) > 1.0
            and abs(report["checks"]["move_y"].get("dy_cm") or 0.0) > 1.0
            and abs(report["checks"]["turn_w"].get("dyaw_deg") or 0.0) > 2.0
            and bool(report["checks"]["path"].get("ok"))
            and bool(report["checks"]["ball_io"].get("ok"))
        )
        report["ok"] = ok
        return 0 if ok else 1
    finally:
        try:
            chassis.stop()
            UnitedPosition.Stop()
        finally:
            report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
            logger.info("[SimAcceptance] wrote {}", report_path)


if __name__ == "__main__":
    raise SystemExit(main())
