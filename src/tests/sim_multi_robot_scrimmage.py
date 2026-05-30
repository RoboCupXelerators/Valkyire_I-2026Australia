"""Multi-robot Unity scrimmage runner.

Start Unity Play mode first, then run from Software:
    python src/tests/sim_multi_robot_scrimmage.py

The runner starts one Python subprocess per robot. That mirrors the real
deployment shape and avoids sharing LunaPre globals across robots.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

SOFTWARE_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SOFTWARE_ROOT / "src"))

from utils.XeleratorSceneControl import XeleratorSceneControl  # noqa: E402


DEFAULT_LAYOUT = {
    1: (-60.0, -30.0, 0.0),
    2: (-70.0, 60.0, 0.0),
    3: (60.0, 30.0, 180.0),
    4: (70.0, -60.0, 180.0),
}


def _parse_robots(value: str) -> list[int]:
    robots: list[int] = []
    for part in value.split(","):
        part = part.strip()
        if not part:
            continue
        robots.append(int(part))
    return robots or [1, 3]


def _parse_roles(value: str, robots: list[int]) -> dict[int, str]:
    if value.strip().lower() in {"", "auto"}:
        return {robot_id: ("offence" if robot_id in (1, 3) else "defence") for robot_id in robots}

    pieces = [part.strip() for part in value.split(",") if part.strip()]
    roles: dict[int, str] = {}
    for idx, robot_id in enumerate(robots):
        roles[robot_id] = pieces[idx] if idx < len(pieces) else ("offence" if robot_id in (1, 3) else "defence")
    return roles


def _write_jsonl(path: Path, row: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as fh:
        fh.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n")


def _place_robots(ctl: XeleratorSceneControl, robots: list[int], ball: tuple[float, float]) -> None:
    active = set(robots)
    for robot_id in (1, 2, 3, 4):
        x, y, yaw = DEFAULT_LAYOUT[robot_id]
        ctl.set_robot(robot_id, x, y, yaw, active=robot_id in active)
    ctl.set_ball(ball[0], ball[1], reset_velocity=True)


def _spawn_robot(
    *,
    robot_id: int,
    role: str,
    duration_s: float,
    out_dir: Path,
) -> subprocess.Popen:
    env = os.environ.copy()
    env.setdefault("ROBOCUPX_BITS", "SIM")
    env.setdefault("XEL_UNITY_IP", "127.0.0.1")
    env["XEL_ROBOT_ID"] = str(robot_id)
    attack_sign = "-1" if robot_id >= 3 else "1"
    env["OFFENCE_ATTACK_SIGN"] = attack_sign
    env["DEFENCE_ATTACK_SIGN"] = attack_sign
    env["XEL_SIM_BALL_STATUS_FALLBACK"] = "1"
    env.setdefault("OFFENCE_LOG_EVERY", "1.0")

    trace = out_dir / f"robot_{robot_id}_{role}.jsonl"
    log_path = out_dir / f"robot_{robot_id}_{role}.log"
    cmd = [
        sys.executable,
        "src/tests/sim_robot_role_runner.py",
        "--robot-id",
        str(robot_id),
        "--role",
        role,
        "--duration",
        str(duration_s),
        "--trace",
        str(trace),
    ]
    log_fh = log_path.open("w", encoding="utf-8")
    proc = subprocess.Popen(
        cmd,
        cwd=SOFTWARE_ROOT,
        env=env,
        stdout=log_fh,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    proc._codex_log_fh = log_fh  # type: ignore[attr-defined]
    return proc


def _read_tail(path: Path, max_lines: int = 40) -> list[str]:
    if not path.exists():
        return []
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    return lines[-max_lines:]


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.exists():
        return rows
    with path.open("r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def _trace_summary(path: Path) -> dict[str, Any]:
    rows = [row for row in _load_jsonl(path) if "pose" in row]
    states: dict[str, int] = {}
    for row in rows:
        state = str(row.get("state", ""))
        states[state] = states.get(state, 0) + 1

    start = rows[0]["pose"] if rows else None
    end = rows[-1]["pose"] if rows else None
    moved = None
    if start and end and len(start) >= 2 and len(end) >= 2:
        dx = float(end[0]) - float(start[0])
        dy = float(end[1]) - float(start[1])
        moved = round((dx * dx + dy * dy) ** 0.5, 3)

    return {
        "frames": len(rows),
        "states": states,
        "start_pose": start,
        "end_pose": end,
        "moved_cm": moved,
    }


def _write_report(out_dir: Path, report: dict[str, Any]) -> None:
    lines = [
        "# Multi-Robot Scrimmage",
        "",
        f"- started_at: {report['started_at']}",
        f"- ok: {report['ok']}",
        f"- output: {out_dir}",
        "",
        "## Robots",
        "",
    ]
    for robot in report["robots"]:
        lines.append(
            f"- R{robot['robot_id']} {robot['role']}: exit={robot['exit_code']} "
            f"frames={robot['trace_summary']['frames']} moved_cm={robot['trace_summary']['moved_cm']}"
        )
        lines.append(f"  states={robot['trace_summary']['states']}")
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a Unity multi-robot scrimmage.")
    parser.add_argument("--robots", default=os.environ.get("SIM_SCRIMMAGE_ROBOTS", "1,3"))
    parser.add_argument("--roles", default=os.environ.get("SIM_SCRIMMAGE_ROLES", "auto"))
    parser.add_argument("--duration", type=float, default=float(os.environ.get("SIM_SCRIMMAGE_DURATION", "8.0")))
    parser.add_argument("--ball-x", type=float, default=0.0)
    parser.add_argument("--ball-y", type=float, default=0.0)
    parser.add_argument("--output", default=os.environ.get("SIM_SCRIMMAGE_OUTPUT", "Records/multi_robot_scrimmage"))
    parser.add_argument("--rules-on", action="store_true", help="Leave penalties/lack/pushing enabled.")
    args = parser.parse_args()

    robots = _parse_robots(args.robots)
    roles = _parse_roles(args.roles, robots)
    out_dir = Path(args.output) / time.strftime("run_%Y%m%d_%H%M%S")
    out_dir.mkdir(parents=True, exist_ok=True)
    scene_trace = out_dir / "scene_status.jsonl"

    report: dict[str, Any] = {
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "ok": False,
        "robots": [],
    }

    processes: dict[int, subprocess.Popen] = {}
    try:
        ctl = XeleratorSceneControl(timeout_s=float(os.environ.get("SIM_SCENE_TIMEOUT_S", "1.0")))
        ctl.ping()
        if not args.rules_on:
            ctl.set_rules(penalties=False, lack=False, pushing=False)
        _place_robots(ctl, robots, (float(args.ball_x), float(args.ball_y)))
        time.sleep(0.3)

        for robot_id in robots:
            processes[robot_id] = _spawn_robot(
                robot_id=robot_id,
                role=roles[robot_id],
                duration_s=float(args.duration),
                out_dir=out_dir,
            )

        deadline = time.time() + max(0.2, float(args.duration)) + 8.0
        while time.time() < deadline and any(proc.poll() is None for proc in processes.values()):
            try:
                status = ctl.status()
                _write_jsonl(scene_trace, {"t_wall": time.time(), "status": status})
            except Exception as exc:
                _write_jsonl(scene_trace, {"t_wall": time.time(), "error": str(exc)})
            time.sleep(0.25)

        for proc in processes.values():
            if proc.poll() is None:
                proc.terminate()
        for proc in processes.values():
            try:
                proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                proc.kill()
            try:
                getattr(proc, "_codex_log_fh").close()
            except Exception:
                pass

        robot_reports = []
        for robot_id, proc in processes.items():
            role = roles[robot_id]
            trace = out_dir / f"robot_{robot_id}_{role}.jsonl"
            log_path = out_dir / f"robot_{robot_id}_{role}.log"
            robot_reports.append(
                {
                    "robot_id": robot_id,
                    "role": role,
                    "exit_code": proc.returncode,
                    "trace": str(trace),
                    "log": str(log_path),
                    "trace_summary": _trace_summary(trace),
                    "log_tail": _read_tail(log_path),
                }
            )
        report["robots"] = robot_reports
        report["scene_trace"] = str(scene_trace)
        report["ok"] = all(
            item["exit_code"] == 0 and item["trace_summary"]["frames"] > 5
            for item in robot_reports
        )
        return 0 if report["ok"] else 1
    finally:
        for proc in processes.values():
            if proc.poll() is None:
                try:
                    proc.terminate()
                except Exception:
                    pass
            try:
                getattr(proc, "_codex_log_fh").close()
            except Exception:
                pass
        (out_dir / "summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        _write_report(out_dir, report)


if __name__ == "__main__":
    raise SystemExit(main())
