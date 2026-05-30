"""Unity regression runner for the rule teacher.

Start Unity Play mode first, then run from Software:
    python src/tests/sim_teacher_regression.py
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, "src")

os.environ.setdefault("ROBOCUPX_BITS", "SIM")
os.environ.setdefault("XEL_UNITY_IP", "127.0.0.1")
os.environ.setdefault("XEL_ROBOT_ID", "1")

from LunaPre import Bits, UnitedPosition, chassis, logger  # noqa: E402
from mlp.contracts import observation_from_lunapre  # noqa: E402
from mlp.skill_executor import SkillExecutor  # noqa: E402
from teacher.badcase import Badcase, TeacherBadcaseDetector  # noqa: E402
from teacher.rule_teacher import RuleTeacher  # noqa: E402
from teacher.scenarios import TeacherScenario, load_teacher_scenarios  # noqa: E402
from teacher.trace import TeacherTraceRecorder  # noqa: E402
from utils.XeleratorSceneControl import XeleratorSceneControl  # noqa: E402


INVALID_POSE = [0xFFF, 0xFFF, 0xFFF]


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


def _scene_control() -> XeleratorSceneControl:
    ctl = XeleratorSceneControl(timeout_s=float(os.environ.get("SIM_SCENE_TIMEOUT_S", "1.0")))
    ctl.ping()
    try:
        ctl.set_rules(penalties=False, lack=False, pushing=False)
    except Exception:
        pass
    return ctl


def _place_scenario(ctl: XeleratorSceneControl, scenario: TeacherScenario) -> None:
    robot = scenario.robot
    ball = scenario.ball
    ctl.set_robot(1, robot[0], robot[1], robot[2], active=True)
    ctl.set_ball(ball[0], ball[1], reset_velocity=True)
    for idx, enemy in enumerate(scenario.enemies, start=3):
        ctl.set_robot(idx, enemy[0], enemy[1], 180.0, active=True)
    time.sleep(0.3)


def _run_one(
    *,
    scenario: TeacherScenario,
    out_dir: Path,
    duration_scale: float,
    include_possession: bool,
) -> dict:
    trace_path = out_dir / f"{scenario.name}.jsonl"
    recorder = TeacherTraceRecorder(trace_path)
    detector = TeacherBadcaseDetector()
    teacher = RuleTeacher()
    executor = SkillExecutor()

    rows: list[dict] = []
    skipped = False
    if scenario.has_ball and not include_possession:
        skipped = True

    if skipped:
        return {
            "scenario": scenario.to_dict(),
            "skipped": True,
            "skip_reason": "has_ball scenario needs a future scene-control possession hook",
            "summary": {"frames": 0, "states": {}, "badcase_count": 0, "badcases": []},
            "ok": True,
        }

    t0 = time.time()
    duration_s = max(0.2, scenario.duration_s * duration_scale)
    while time.time() - t0 < duration_s:
        obs = observation_from_lunapre()
        decision = teacher.decide(obs)
        result = executor.execute(decision.packet)
        row = recorder.record(
            obs=obs,
            decision=decision,
            scenario=scenario.name,
            runtime={
                "t_s": round(time.time() - t0, 3),
                "execute_ok": result.executed,
                "execute_status": result.status,
                "pose": _pose(),
            },
        )
        rows.append(row)
        time.sleep(0.04)

    chassis.stop()
    issues = detector.analyze(rows)
    states_seen = {row.get("decision", {}).get("state", "") for row in rows}
    for expected in scenario.expected_states:
        if expected not in states_seen:
            issues.append(
                Badcase(
                    "missing_expected_state",
                    "high",
                    "teacher did not enter an expected state during the scenario",
                    len(rows) - 1,
                    {"expected": expected, "states_seen": sorted(states_seen)},
                )
            )

    summary = detector.summarize(rows, issues)
    ok = not any(issue.severity == "high" for issue in issues)
    return {
        "scenario": scenario.to_dict(),
        "skipped": False,
        "trace": str(trace_path),
        "start_pose": rows[0]["runtime"]["pose"] if rows else INVALID_POSE,
        "end_pose": _pose(),
        "summary": summary,
        "ok": ok,
    }


def _write_markdown(out_dir: Path, report: dict) -> None:
    lines = [
        "# Teacher Regression",
        "",
        f"- started_at: {report['started_at']}",
        f"- ok: {report['ok']}",
        f"- output: {out_dir}",
        "",
        "## Scenarios",
        "",
    ]
    for item in report["scenarios"]:
        summary = item.get("summary", {})
        states = summary.get("states", {})
        lines.append(f"- {item['scenario']['name']}: ok={item['ok']} skipped={item.get('skipped', False)}")
        lines.append(f"  states={states} badcases={summary.get('badcase_count', 0)}")
        if item.get("skipped"):
            lines.append(f"  skip_reason={item.get('skip_reason', '')}")
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Run Unity teacher regression scenarios.")
    parser.add_argument("--scenario-file", default=os.environ.get("SIM_TEACHER_SCENARIOS"))
    parser.add_argument("--output", default=os.environ.get("SIM_TEACHER_OUTPUT", "Records/teacher_debug"))
    parser.add_argument("--duration-scale", type=float, default=float(os.environ.get("SIM_TEACHER_DURATION_SCALE", "1.0")))
    parser.add_argument("--max-scenarios", type=int, default=int(os.environ.get("SIM_TEACHER_MAX_SCENARIOS", "0")))
    parser.add_argument("--include-possession", action="store_true", default=os.environ.get("SIM_TEACHER_INCLUDE_POSSESSION") == "1")
    args = parser.parse_args()

    out_dir = Path(args.output) / time.strftime("run_%Y%m%d_%H%M%S")
    out_dir.mkdir(parents=True, exist_ok=True)

    report: dict = {
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "ok": False,
        "scenarios": [],
    }

    try:
        try:
            ctl = _scene_control()
            report["scene_control"] = {"ok": True, "host": ctl.host, "port": ctl.port}
        except Exception as exc:
            report["scene_control"] = {"ok": False, "error": str(exc)}
            logger.error("[TeacherRegression] scene control unavailable: {}", exc)
            return 2

        connected = _wait_for_sim()
        report["connection"] = {"ok": connected, "pose": _pose(), "rx_count": int(getattr(Bits, "_rx_count", 0))}
        if not connected:
            logger.error("[TeacherRegression] simulator status unavailable")
            return 2

        scenarios = load_teacher_scenarios(args.scenario_file)
        if args.max_scenarios > 0:
            scenarios = scenarios[: args.max_scenarios]

        for scenario in scenarios:
            logger.info("[TeacherRegression] scenario {}", scenario.name)
            _place_scenario(ctl, scenario)
            item = _run_one(
                scenario=scenario,
                out_dir=out_dir,
                duration_scale=args.duration_scale,
                include_possession=args.include_possession,
            )
            report["scenarios"].append(item)

        report["ok"] = all(item.get("ok", False) for item in report["scenarios"])
        return 0 if report["ok"] else 1
    finally:
        try:
            chassis.stop()
        except Exception:
            pass
        try:
            UnitedPosition.Stop()
        except Exception:
            pass
        (out_dir / "summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        _write_markdown(out_dir, report)
        logger.info("[TeacherRegression] wrote {}", out_dir)


if __name__ == "__main__":
    raise SystemExit(main())
