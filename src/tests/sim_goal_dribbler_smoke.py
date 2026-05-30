"""Unity smoke test for goal scoring and dribbler possession semantics.

Start Unity Play mode first, then run from ``Software``:
    python src/tests/sim_goal_dribbler_smoke.py
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Callable

SOFTWARE_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SOFTWARE_ROOT / "src"))

from utils.XeleratorSceneControl import XeleratorSceneControl  # noqa: E402


def _write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def _wait_for(
    ctl: XeleratorSceneControl,
    pred: Callable[[dict[str, Any]], bool],
    *,
    timeout_s: float = 2.0,
    interval_s: float = 0.05,
) -> dict[str, Any]:
    deadline = time.time() + timeout_s
    last: dict[str, Any] = {}
    while time.time() < deadline:
        last = ctl.status()
        if pred(last):
            return last
        time.sleep(interval_s)
    return last


def _score(status: dict[str, Any]) -> dict[str, Any]:
    score = status.get("score")
    return score if isinstance(score, dict) else {}


def _robot(status: dict[str, Any], robot_id: int) -> dict[str, Any]:
    for robot in status.get("robots") or []:
        if int(robot.get("id", -1)) == int(robot_id):
            return robot
    return {}


def _ok(condition: bool, report: dict[str, Any], name: str, details: dict[str, Any]) -> None:
    row = {"name": name, "ok": bool(condition), "details": details}
    report.setdefault("checks", []).append(row)
    if not condition:
        report.setdefault("failures", []).append(row)


def _near_center(status: dict[str, Any], tol_cm: float = 5.0) -> bool:
    ball = status.get("ball") or {}
    return abs(float(ball.get("x_cm", 999))) <= tol_cm and abs(float(ball.get("y_cm", 999))) <= tol_cm


def _prepare_single_robot(ctl: XeleratorSceneControl) -> None:
    ctl.set_rules(penalties=False, lack=False, pushing=False)
    for robot_id in (1, 2, 3, 4):
        ctl.set_robot(robot_id, 0.0, 0.0, 0.0 if robot_id <= 2 else 180.0, active=robot_id == 1)
    ctl.set_dribble(1, False, strength=0.0)
    ctl.set_ball(0.0, 0.0, reset_velocity=True)
    time.sleep(0.2)


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke-test Unity goal and dribbler logic.")
    parser.add_argument("--output", default="Records/sim_goal_dribbler_smoke")
    parser.add_argument("--timeout", type=float, default=2.0)
    args = parser.parse_args()

    out_dir = SOFTWARE_ROOT.parent / args.output / time.strftime("run_%Y%m%d_%H%M%S")
    report: dict[str, Any] = {
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "ok": False,
        "checks": [],
        "failures": [],
    }

    try:
        ctl = XeleratorSceneControl(timeout_s=float(args.timeout))
        report["ping"] = ctl.ping()
        _prepare_single_robot(ctl)

        status = ctl.reset_score()
        _ok("score" in status, report, "scene status exposes score", {"status": status})
        if "score" not in status:
            report["stale_unity_protocol"] = True
            report["hint"] = "Exit Play mode and enter Play again so Unity reloads XelRuntimeControlUi/XeleratorRobotController."
            return 1

        ctl.set_ball(116.0, 0.0, reset_velocity=True)
        status = _wait_for(
            ctl,
            lambda st: int(_score(st).get("ally", 0)) == 1 and _near_center(st),
            timeout_s=float(args.timeout),
        )
        _ok(
            int(_score(status).get("ally", 0)) == 1,
            report,
            "direct +X goal scores immediately",
            {"score": _score(status), "ball": status.get("ball")},
        )

        ctl.reset_score()
        ctl.set_ball(-116.0, 0.0, reset_velocity=True)
        status = _wait_for(
            ctl,
            lambda st: int(_score(st).get("enemy", 0)) == 1 and _near_center(st),
            timeout_s=float(args.timeout),
        )
        _ok(
            int(_score(status).get("enemy", 0)) == 1,
            report,
            "direct -X goal scores immediately",
            {"score": _score(status), "ball": status.get("ball")},
        )

        ctl.reset_score()
        ctl.set_ball(105.0, 0.0, vx_cm_s=1600.0, vy_cm_s=0.0, reset_velocity=False)
        status = _wait_for(
            ctl,
            lambda st: int(_score(st).get("ally", 0)) == 1,
            timeout_s=float(args.timeout),
        )
        _ok(
            int(_score(status).get("ally", 0)) == 1,
            report,
            "swept +X goal crossing scores",
            {"score": _score(status), "ball": status.get("ball")},
        )

        _prepare_single_robot(ctl)
        ctl.set_ball(8.0, 0.0, reset_velocity=True)
        ctl.set_dribble(1, False, strength=0.0)
        status = _wait_for(
            ctl,
            lambda st: bool(_robot(st, 1).get("ball_in_dribbler_window")) and not bool(_robot(st, 1).get("has_ball")),
            timeout_s=float(args.timeout),
        )
        r1 = _robot(status, 1)
        _ok(
            bool(r1.get("ball_in_dribbler_window")) and not bool(r1.get("has_ball")),
            report,
            "ball in mouth is not possession while dribbler is off",
            {"robot": r1, "ball": status.get("ball")},
        )

        ctl.set_dribble(1, True, strength=0.7)
        status = _wait_for(
            ctl,
            lambda st: bool(_robot(st, 1).get("has_ball")),
            timeout_s=float(args.timeout),
        )
        r1 = _robot(status, 1)
        _ok(
            bool(r1.get("has_ball")) and bool(r1.get("dribble_command")),
            report,
            "dribbler on captures ball in mouth",
            {"robot": r1, "ball": status.get("ball")},
        )

        ctl.set_dribble(1, False, strength=0.0)
        status = _wait_for(
            ctl,
            lambda st: not bool(_robot(st, 1).get("has_ball")),
            timeout_s=float(args.timeout),
        )
        r1 = _robot(status, 1)
        _ok(
            not bool(r1.get("has_ball")) and not bool(r1.get("dribble_command")),
            report,
            "dribbler off releases possession state",
            {"robot": r1, "ball": status.get("ball")},
        )

        ctl.set_ball(-8.0, 0.0, reset_velocity=True)
        ctl.set_dribble(1, True, strength=0.7)
        status = _wait_for(
            ctl,
            lambda st: not bool(_robot(st, 1).get("ball_in_dribbler_window")),
            timeout_s=float(args.timeout),
        )
        r1 = _robot(status, 1)
        _ok(
            not bool(r1.get("ball_in_dribbler_window")) and not bool(r1.get("has_ball")),
            report,
            "ball behind robot is outside dribbler window",
            {"robot": r1, "ball": status.get("ball")},
        )

        report["ok"] = not report["failures"]
        return 0 if report["ok"] else 1
    except Exception as exc:
        report["exception"] = repr(exc)
        return 2
    finally:
        _write_json(out_dir / "summary.json", report)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        print(f"summary: {out_dir / 'summary.json'}")


if __name__ == "__main__":
    raise SystemExit(main())
