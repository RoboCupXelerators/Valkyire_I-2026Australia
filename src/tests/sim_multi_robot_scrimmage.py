"""Multi-robot Unity scrimmage runner.

Start Unity Play mode first, then run from Software:
    python src/tests/sim_multi_robot_scrimmage.py

The runner starts one Python subprocess per robot. That mirrors the real
deployment shape and avoids sharing LunaPre globals across robots. Match rules
are enabled by default; use --rules-off only for isolated debug experiments.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import re
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

STAGGERED_LAYOUT = {
    1: (-60.0, -30.0, 0.0),
    2: (-85.0, 65.0, 0.0),
    3: (85.0, 45.0, 180.0),
    4: (95.0, -65.0, 180.0),
}

INACTIVE_PARKING = {
    1: (-110.0, 82.0, 0.0),
    2: (-98.0, 82.0, 0.0),
    3: (98.0, -82.0, 180.0),
    4: (110.0, -82.0, 180.0),
}

LAYOUTS = {
    "default": DEFAULT_LAYOUT,
    "staggered": STAGGERED_LAYOUT,
}

DEFAULT_BALL = (-12.0, 0.0)
SIM_WHEEL_DIAMETER_M = 0.056
SIM_WHEEL_MAX_RPM = 500.0
SIM_WHEEL_TO_V_56MM_500RPM = math.pi * SIM_WHEEL_DIAMETER_M / 60.0
SIM_LINEAR_SCALE = 1.0
SIM_LATERAL_SCALE = 0.64
SIM_ANGULAR_SCALE = 2.20
SIM_BALL_DRAG = 0.02
SIM_BALL_ANGULAR_DRAG = 0.08
SIM_BALL_ROLLING_RESISTANCE = 3.1


def _clip(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _avoid_goal_box_start(x: float, y: float) -> tuple[float, float]:
    """Keep randomized starts out of the simulator penalty box."""

    if abs(x) > 62.0 and abs(y) < 52.0:
        if abs(y) >= 36.0:
            y = 52.0 if y >= 0.0 else -52.0
        else:
            x = 62.0 if x >= 0.0 else -62.0
    return x, y


def _random_layout(
    robots: list[int],
    ball: tuple[float, float],
    seed: int,
) -> dict[int, tuple[float, float, float]]:
    rng = random.Random(seed)
    ball_x, ball_y = float(ball[0]), float(ball[1])
    layout = dict(DEFAULT_LAYOUT)

    for robot_id in robots:
        if robot_id in (1, 2):
            x = ball_x - rng.uniform(34.0, 92.0)
            y = ball_y + rng.uniform(-58.0, 58.0)
            yaw = 0.0
        else:
            x = ball_x + rng.uniform(30.0, 92.0)
            y = ball_y + rng.uniform(-58.0, 58.0)
            yaw = 180.0

        if robot_id == 2:
            y += 30.0 if y <= ball_y else -30.0
        elif robot_id == 4:
            y += 34.0 if y <= ball_y else -34.0

        x = _clip(x, -86.0, 86.0)
        y = _clip(y, -58.0, 58.0)
        x, y = _avoid_goal_box_start(x, y)
        layout[robot_id] = (x, y, yaw)

    return layout


def _layout_for_round(
    layout_name: str,
    robots: list[int],
    ball: tuple[float, float],
    seed: int,
    round_index: int,
) -> dict[int, tuple[float, float, float]]:
    if layout_name == "random":
        return _random_layout(robots, ball, seed + round_index * 7919)
    return LAYOUTS.get(layout_name, DEFAULT_LAYOUT)


def _inactive_parking_pose(robot_id: int) -> tuple[float, float, float]:
    return INACTIVE_PARKING.get(int(robot_id), (0.0, 88.0, 0.0))


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


def _recover_out_of_bounds_ball(
    ctl: XeleratorSceneControl,
    status: dict[str, Any],
) -> dict[str, Any] | None:
    ball_status = status.get("ball") if isinstance(status.get("ball"), dict) else {}
    try:
        ball_x = float(ball_status.get("x_cm", 0.0))
        ball_y = float(ball_status.get("y_cm", 0.0))
    except (TypeError, ValueError):
        return None

    if abs(ball_x) <= 128.0 and abs(ball_y) <= 98.0:
        return None

    target_x = _clip(ball_x, -67.0, 67.0)
    target_y = _clip(ball_y, -50.0, 50.0)
    if abs(target_x) > 60.0 and abs(target_y) < 34.0:
        target_y = 50.0 if target_y >= 0.0 else -50.0
    ctl.set_ball(target_x, target_y, reset_velocity=True)
    return {
        "from": [round(ball_x, 3), round(ball_y, 3)],
        "to": [round(target_x, 3), round(target_y, 3)],
    }


def _place_robots(
    ctl: XeleratorSceneControl,
    robots: list[int],
    ball: tuple[float, float],
    layout: dict[int, tuple[float, float, float]],
    *,
    reset_score: bool = True,
) -> None:
    if reset_score:
        try:
            ctl.reset_score()
        except Exception:
            pass
    active = set(robots)
    for robot_id in (1, 2, 3, 4):
        try:
            ctl.set_dribble(robot_id, False, strength=0.0)
        except Exception:
            pass
        if robot_id in active:
            x, y, yaw = layout[robot_id]
            ctl.set_robot(robot_id, x, y, yaw, active=True)
        else:
            x, y, yaw = _inactive_parking_pose(robot_id)
            ctl.set_robot(robot_id, x, y, yaw, active=False)
    ctl.set_ball(ball[0], ball[1], reset_velocity=True)


def _set_rules(
    ctl: XeleratorSceneControl,
    *,
    enabled: bool,
) -> dict[str, Any]:
    return ctl.set_rules(
        penalties=enabled,
        lack=enabled,
        pushing=enabled,
    )


def _refresh_rules_after_manual_place(
    ctl: XeleratorSceneControl,
    *,
    enabled: bool,
) -> dict[str, Any] | None:
    if not enabled:
        return None
    _set_rules(ctl, enabled=False)
    return _set_rules(ctl, enabled=True)


def _uses_controlled_goal_reset(args: argparse.Namespace) -> bool:
    return bool(getattr(args, "continue_after_goal", False) or getattr(args, "allow_no_goal", False))


def _lock_active_robots(
    ctl: XeleratorSceneControl,
    status: dict[str, Any],
    robots: list[int],
    suspended: set[int] | None = None,
) -> dict[str, Any] | None:
    expected = set(robots) - set(suspended or set())
    scene_robots = status.get("robots") if isinstance(status.get("robots"), list) else []
    unexpected = [robot for robot in scene_robots if robot.get("active") and int(robot.get("id", -1)) not in expected]
    missing = [robot for robot in scene_robots if not robot.get("active") and int(robot.get("id", -1)) in expected]
    if not unexpected and not missing:
        return None

    for robot in unexpected:
        robot_id = int(robot.get("id", -1))
        if robot_id < 0:
            continue
        try:
            ctl.set_dribble(robot_id, False, strength=0.0)
        except Exception:
            pass
        x, y, yaw = _inactive_parking_pose(robot_id)
        ctl.set_robot(
            robot_id,
            x,
            y,
            yaw,
            active=False,
        )

    for robot in missing:
        robot_id = int(robot.get("id", -1))
        if robot_id < 0:
            continue
        ctl.set_robot(
            robot_id,
            float(robot.get("x_cm", 0.0)),
            float(robot.get("y_cm", 0.0)),
            float(robot.get("yaw_deg", 0.0)),
            active=True,
        )

    return {
        "unexpected_active": [int(robot.get("id", -1)) for robot in unexpected],
        "missing_active": [int(robot.get("id", -1)) for robot in missing],
    }


def _penalized_robot(event: str, robots: list[int]) -> int | None:
    normalized = event.strip().lower()
    if "penalty" not in normalized:
        return None
    for robot_id in robots:
        if f"r{robot_id} penalty" in normalized:
            return robot_id
    return None


def _apply_physics_tuning(ctl: XeleratorSceneControl) -> dict[str, Any]:
    robot_response = ctl.set_robot_tuning(
        robot_id=0,
        linear_scale=SIM_LINEAR_SCALE,
        lateral_scale=SIM_LATERAL_SCALE,
        angular_scale=SIM_ANGULAR_SCALE,
    )
    ball_response = ctl.set_ball_physics(
        drag=SIM_BALL_DRAG,
        angular_drag=SIM_BALL_ANGULAR_DRAG,
        ball_rolling_resistance=SIM_BALL_ROLLING_RESISTANCE,
    )
    return {
        "robot": robot_response,
        "ball": ball_response,
        "wheel_diameter_m": SIM_WHEEL_DIAMETER_M,
        "wheel_max_rpm": SIM_WHEEL_MAX_RPM,
        "wheel_to_v": SIM_WHEEL_TO_V_56MM_500RPM,
        "linear_scale": SIM_LINEAR_SCALE,
        "lateral_scale": SIM_LATERAL_SCALE,
        "angular_scale": SIM_ANGULAR_SCALE,
        "ball_drag": SIM_BALL_DRAG,
        "ball_angular_drag": SIM_BALL_ANGULAR_DRAG,
        "ball_rolling_resistance": SIM_BALL_ROLLING_RESISTANCE,
    }


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
    env["XEL_WHEEL_TO_V"] = str(SIM_WHEEL_TO_V_56MM_500RPM)
    env["XEL_SIM_PLANAR_MOVE_SCALE"] = "1.0"
    env["XEL_SIM_BALL_STATUS_FALLBACK"] = "1"
    env["XEL_SIM_PORT_FALLBACK"] = "0"
    env.setdefault("OFFENCE_LOG_EVERY", "1.0")
    env.setdefault("LUNAPRE_FULL_LOG", "1")  # enable Pos2Pos PID debug logs

    # === Robot-specific strategy differentiation ===
    if robot_id == 1:
        # R1: Aggressive striker - quick shots, fast approach
        env["OFFENCE_APPROACH_STUCK_S"] = "2.5"  # faster stuck detection
        env["OFFENCE_SHOOT_YAW_TOL"] = "15"  # looser aim, quicker shots
        env["OFFENCE_CARRY_GAIN"] = "2.8"  # faster carry
        env["OFFENCE_POUNCE_DIST"] = "50"  # pounce from further
        env["OFFENCE_DROP_BACK_DIST"] = "25"  # shorter drop back
        env["OFFENCE_DEEP_SHOOT_X"] = "35"  # shoot from closer range
    elif robot_id == 2:
        # R2: Midfield interceptor - aggressive defence
        env["DEFENCE_CHASE_DISTANCE"] = "100"  # larger intercept range
        env["DEFENCE_GUARD_X"] = "40"  # start from near midfield
        env["DEFENCE_INTERCEPT_GAIN"] = "1.8"  # faster intercept
        env["DEFENCE_CHASE_PROGRESS"] = "20"  # intercept further up field
        env["DEFENCE_HOLD_GAIN"] = "1.3"  # faster home return
        if role == "offence":
            # In 2v2 videos R2 is the second attacker: occupy a lane and let
            # R1 be the ball carrier, otherwise both robots pinch the ball.
            env["OFFENCE_SUPPORT_MODE"] = "1"
            env["OFFENCE_SUPPORT_SIDE"] = "1"
    elif robot_id == 3:
        # R3: Deep striker - carry deeper before shooting
        env["OFFENCE_DEEP_SHOOT_X"] = "30"  # shoot from very close
        env["OFFENCE_CARRY_X"] = "100"  # carry deeper into enemy half
        env["OFFENCE_SHOOT_YAW_TOL"] = "5"  # precise aim
        env["OFFENCE_APPROACH_STUCK_S"] = "4.0"  # more patient
        env["OFFENCE_CARRY_GAIN"] = "2.5"  # fast carry
    elif robot_id == 4:
        # R4: Goalkeeper - stay deep, only challenge in box
        env["DEFENCE_GUARD_X"] = "75"  # deeper home position (near goal)
        env["DEFENCE_CHASE_RETREAT"] = "-45"  # only challenge very deep balls
        env["DEFENCE_CHASE_DISTANCE"] = "60"  # larger chase range for goalkeeper
        env["DEFENCE_HOLD_GAIN"] = "0.9"  # slower, steady positioning

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
    if rows:
        total = 0.0
        previous_xy: tuple[float, float] | None = None
        for row in rows:
            pose = row.get("pose")
            if not pose or len(pose) < 2:
                continue
            try:
                current_xy = (float(pose[0]), float(pose[1]))
            except (TypeError, ValueError):
                continue
            if previous_xy is not None:
                dx = current_xy[0] - previous_xy[0]
                dy = current_xy[1] - previous_xy[1]
                total += (dx * dx + dy * dy) ** 0.5
            previous_xy = current_xy
        moved = round(total, 3)

    displacement = None
    if start and end and len(start) >= 2 and len(end) >= 2:
        dx = float(end[0]) - float(start[0])
        dy = float(end[1]) - float(start[1])
        displacement = round((dx * dx + dy * dy) ** 0.5, 3)

    return {
        "frames": len(rows),
        "states": states,
        "start_pose": start,
        "end_pose": end,
        "moved_cm": moved,
        "displacement_cm": displacement,
    }


def _is_rule_event(event: str) -> bool:
    normalized = event.strip().lower()
    if not normalized:
        return False
    return (
        "lack of progress" in normalized
        or "pushing" in normalized
        or "penalty" in normalized
    )


def _is_fresh_event_time(value: Any) -> bool:
    try:
        return float(value) >= 0.0
    except (TypeError, ValueError):
        return True


_PUSHING_EVENT_RE = re.compile(r"pushing\s+r(\d+)\s*/\s*r(\d+)", re.IGNORECASE)


def _robot_xy(status: dict[str, Any], robot_id: int) -> tuple[float, float] | None:
    robots = status.get("robots") if isinstance(status.get("robots"), list) else []
    for robot in robots:
        try:
            if int(robot.get("id", -1)) != int(robot_id):
                continue
            return float(robot.get("x_cm", 0.0)), float(robot.get("y_cm", 0.0))
        except (TypeError, ValueError):
            return None
    return None


def _event_is_supported_by_status(event: str, status: dict[str, Any]) -> bool:
    match = _PUSHING_EVENT_RE.search(event)
    if not match:
        return True

    first = _robot_xy(status, int(match.group(1)))
    second = _robot_xy(status, int(match.group(2)))
    if first is None or second is None:
        return True

    return math.hypot(first[0] - second[0], first[1] - second[1]) <= 36.0


def _score_total(score: dict[str, Any] | None) -> int:
    if not isinstance(score, dict):
        return 0
    try:
        ally = int(score.get("ally", 0))
        enemy = int(score.get("enemy", 0))
    except (TypeError, ValueError):
        return 0
    return ally + enemy


def _scene_summary(path: Path) -> dict[str, Any]:
    statuses: list[dict[str, Any]] = []
    errors: list[str] = []
    if not path.exists():
        return {
            "frames": 0,
            "errors": ["scene trace missing"],
            "multi_possession_frames": 0,
            "max_holders": 0,
        }

    with path.open("r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError as exc:
                errors.append(f"bad jsonl row: {exc}")
                continue
            if "error" in row:
                errors.append(str(row["error"]))
                continue
            status = row.get("status")
            if isinstance(status, dict):
                statuses.append(status)

    holder_counts: dict[str, int] = {}
    multi_possession_frames = 0
    max_holders = 0
    first_multi_possession = None
    ball_xs: list[float] = []
    ball_ys: list[float] = []
    ball_speeds: list[float] = []
    rule_counts: dict[str, int] = {}
    event_counts: dict[str, int] = {}
    first_rule_event = None
    previous_event_key = None

    for idx, status in enumerate(statuses):
        rules = status.get("rules") if isinstance(status.get("rules"), dict) else {}
        if rules:
            key = ",".join(
                f"{name}={str(bool(rules.get(name))).lower()}"
                for name in ("penalties", "lack", "pushing")
            )
            rule_counts[key] = rule_counts.get(key, 0) + 1

        score = status.get("score") if isinstance(status.get("score"), dict) else {}
        event = str(score.get("last_event", "")).strip() if score else ""
        event_time = score.get("last_time") if score else None
        event_key = (event, event_time)
        if (
            event
            and _is_fresh_event_time(event_time)
            and _event_is_supported_by_status(event, status)
            and event_key != previous_event_key
        ):
            event_counts[event] = event_counts.get(event, 0) + 1
            if _is_rule_event(event) and first_rule_event is None:
                first_rule_event = {
                    "frame": idx,
                    "time": status.get("time"),
                    "event": event,
                    "event_time": event_time,
                    "ball": status.get("ball"),
                }
        previous_event_key = event_key

        ball = status.get("ball") if isinstance(status.get("ball"), dict) else {}
        if ball:
            ball_xs.append(float(ball.get("x_cm", 0.0)))
            ball_ys.append(float(ball.get("y_cm", 0.0)))
            ball_speeds.append(float(ball.get("speed_cm_s", 0.0)))

        robots = status.get("robots") if isinstance(status.get("robots"), list) else []
        holders = [int(robot.get("id", -1)) for robot in robots if robot.get("has_ball")]
        key = ",".join(str(holder) for holder in holders) if holders else "none"
        holder_counts[key] = holder_counts.get(key, 0) + 1
        max_holders = max(max_holders, len(holders))
        if len(holders) > 1:
            multi_possession_frames += 1
            if first_multi_possession is None:
                first_multi_possession = {
                    "frame": idx,
                    "time": status.get("time"),
                    "holders": holders,
                    "ball": ball,
                }

    start_ball = None
    end_ball = None
    final_score = None
    final_score_total = 0
    if statuses:
        start_ball = statuses[0].get("ball")
        end_ball = statuses[-1].get("ball")
        final_score = statuses[-1].get("score")
        final_score_total = _score_total(final_score)

    return {
        "frames": len(statuses),
        "errors": errors,
        "holder_counts": holder_counts,
        "multi_possession_frames": multi_possession_frames,
        "max_holders": max_holders,
        "first_multi_possession": first_multi_possession,
        "rule_counts": rule_counts,
        "event_counts": event_counts,
        "first_rule_event": first_rule_event,
        "ball_start": start_ball,
        "ball_end": end_ball,
        "ball_x_min": round(min(ball_xs), 3) if ball_xs else None,
        "ball_x_max": round(max(ball_xs), 3) if ball_xs else None,
        "ball_y_min": round(min(ball_ys), 3) if ball_ys else None,
        "ball_y_max": round(max(ball_ys), 3) if ball_ys else None,
        "ball_speed_max": round(max(ball_speeds), 3) if ball_speeds else None,
        "final_score": final_score,
        "final_score_total": final_score_total,
    }


def _write_report(out_dir: Path, report: dict[str, Any]) -> None:
    lines = [
        "# Multi-Robot Scrimmage",
        "",
        f"- started_at: {report['started_at']}",
        f"- ok: {report['ok']}",
        f"- output: {out_dir}",
        "",
        "## Scene",
        "",
    ]
    scene = report.get("scene_summary", {})
    if scene:
        score = scene.get("final_score") or {}
        lines.extend(
            [
                f"- frames={scene.get('frames')} multi_possession_frames={scene.get('multi_possession_frames')} "
                f"max_holders={scene.get('max_holders')}",
                f"- ball_x={scene.get('ball_x_min')}..{scene.get('ball_x_max')} "
                f"ball_y={scene.get('ball_y_min')}..{scene.get('ball_y_max')} "
                f"max_speed={scene.get('ball_speed_max')}",
                f"- score ally={score.get('ally')} enemy={score.get('enemy')} "
                f"last_event={score.get('last_event', '')}",
            ]
        )
        if report.get("ended_after_goal") is not None:
            lines.append(f"- ended_after_goal={report.get('ended_after_goal')}")
        if scene.get("rule_counts"):
            lines.append(f"- rules={scene.get('rule_counts')}")
        if scene.get("event_counts"):
            lines.append(f"- events={scene.get('event_counts')}")
        if scene.get("goal_required") is not None:
            lines.append(
                f"- goal_required={scene.get('goal_required')} "
                f"goal_ok={scene.get('goal_ok')} target_goals={scene.get('target_goals')} "
                f"total_goals={scene.get('final_score_total')}"
            )
        if scene.get("errors"):
            lines.append(f"- scene_errors={scene.get('errors')}")
        if scene.get("first_multi_possession"):
            lines.append(f"- first_multi_possession={scene.get('first_multi_possession')}")
        if scene.get("first_rule_event"):
            lines.append(f"- first_rule_event={scene.get('first_rule_event')}")
        lines.append("")

    physics = report.get("physics_tuning") or {}
    if physics:
        lines.extend(
            [
                "## Physics",
                "",
                f"- robot linear_scale={physics.get('linear_scale')} lateral_scale={physics.get('lateral_scale')} "
                f"angular_scale={physics.get('angular_scale')}",
                f"- ball drag={physics.get('ball_drag')} angular_drag={physics.get('ball_angular_drag')} "
                f"rolling_resistance={physics.get('ball_rolling_resistance')}",
                "",
            ]
        )

    layout_history = report.get("layout_history") or []
    if layout_history:
        lines.extend(["## Layout", ""])
        for item in layout_history:
            layout = item.get("layout") if isinstance(item, dict) else {}
            entries = []
            for robot_id in sorted(layout, key=lambda value: int(value)):
                pose = layout[robot_id]
                if not pose or len(pose) < 3:
                    continue
                entries.append(f"R{robot_id}=({float(pose[0]):.1f},{float(pose[1]):.1f},{float(pose[2]):.0f})")
            lines.append(f"- round {item.get('round')}: {' '.join(entries)}")
        lines.append("")

    lines.extend(
        [
            "## Robots",
            "",
        ]
    )
    for robot in report["robots"]:
        lines.append(
            f"- R{robot['robot_id']} {robot['role']}: exit={robot['exit_code']} "
            f"frames={robot['trace_summary']['frames']} moved_cm={robot['trace_summary']['moved_cm']} "
            f"displacement_cm={robot['trace_summary'].get('displacement_cm')}"
        )
        lines.append(f"  states={robot['trace_summary']['states']}")
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a Unity multi-robot scrimmage.")
    parser.add_argument("--robots", default=os.environ.get("SIM_SCRIMMAGE_ROBOTS", "1,3"))
    parser.add_argument("--roles", default=os.environ.get("SIM_SCRIMMAGE_ROLES", "auto"))
    parser.add_argument("--duration", type=float, default=float(os.environ.get("SIM_SCRIMMAGE_DURATION", "15.0")))
    parser.add_argument("--ball-x", type=float, default=float(os.environ.get("SIM_SCRIMMAGE_BALL_X", str(DEFAULT_BALL[0]))))
    parser.add_argument("--ball-y", type=float, default=float(os.environ.get("SIM_SCRIMMAGE_BALL_Y", str(DEFAULT_BALL[1]))))
    parser.add_argument(
        "--layout",
        choices=sorted([*LAYOUTS, "random"]),
        default=os.environ.get("SIM_SCRIMMAGE_LAYOUT", "default"),
        help="Initial robot layout. Use 'random' with --seed for reproducible randomized starts.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Seed for --layout random. Defaults to current time when omitted.",
    )
    parser.add_argument("--output", default=os.environ.get("SIM_SCRIMMAGE_OUTPUT", "Records/multi_robot_scrimmage"))
    parser.add_argument(
        "--rules-off",
        action="store_true",
        help="Disable penalties/lack/pushing for isolated debug experiments.",
    )
    parser.add_argument(
        "--rules-on",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--allow-no-possession",
        action="store_true",
        help="Do not fail when no robot ever gets strict Unity possession.",
    )
    parser.add_argument(
        "--allow-no-goal",
        action="store_true",
        help="Do not fail when the scrimmage ends 0-0.",
    )
    parser.add_argument(
        "--continue-after-goal",
        action="store_true",
        help="Keep running for the full duration after a goal instead of ending the smoke run early.",
    )
    parser.add_argument(
        "--target-goals",
        type=int,
        default=int(os.environ.get("SIM_SCRIMMAGE_TARGET_GOALS", "1")),
        help="Number of total goals required before ending early or passing the goal check.",
    )
    parser.add_argument(
        "--startup-grace",
        type=float,
        default=float(os.environ.get("SIM_SCRIMMAGE_STARTUP_GRACE", "15.0")),
        help="Extra seconds allowed for robot subprocess startup and Unity UDP handshake.",
    )
    parser.add_argument(
        "--status-interval",
        type=float,
        default=float(os.environ.get("SIM_SCRIMMAGE_STATUS_INTERVAL", "0.05")),
        help="Seconds between scene status samples used by reports and replay rendering.",
    )
    args = parser.parse_args()

    robots = _parse_robots(args.robots)
    roles = _parse_roles(args.roles, robots)
    ball = (float(args.ball_x), float(args.ball_y))
    layout_seed = int(args.seed) if args.seed is not None else int(time.time())
    layout_round = 0
    layout = _layout_for_round(args.layout, robots, ball, layout_seed, layout_round)
    out_dir = Path(args.output) / time.strftime("run_%Y%m%d_%H%M%S")
    out_dir.mkdir(parents=True, exist_ok=True)
    scene_trace = out_dir / "scene_status.jsonl"

    report: dict[str, Any] = {
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "ok": False,
        "robots": [],
        "roster_corrections": [],
        "layout": args.layout,
        "layout_seed": layout_seed,
        "layout_history": [],
    }

    processes: dict[int, subprocess.Popen] = {}
    try:
        ctl = XeleratorSceneControl(timeout_s=float(os.environ.get("SIM_SCENE_TIMEOUT_S", "1.0")))
        ctl.ping()
        rules_enabled = args.rules_on or not args.rules_off
        report["rules_requested"] = {
            "penalties": rules_enabled,
            "lack": rules_enabled,
            "pushing": rules_enabled,
        }
        report["rules_response"] = _set_rules(ctl, enabled=rules_enabled)
        report["physics_tuning"] = _apply_physics_tuning(ctl)
        report["layout_history"].append({"round": layout_round, "layout": layout})
        _place_robots(ctl, robots, ball, layout)
        report["initial_rule_refresh"] = _refresh_rules_after_manual_place(ctl, enabled=rules_enabled)
        time.sleep(0.3)

        for robot_id in robots:
            processes[robot_id] = _spawn_robot(
                robot_id=robot_id,
                role=roles[robot_id],
                duration_s=float(args.duration),
                out_dir=out_dir,
            )

        deadline = time.time() + max(0.2, float(args.duration)) + max(1.0, float(args.startup_grace))
        ended_after_goal = False
        stop_after_goal_at: float | None = None
        suspended_robots: set[int] = set()
        previous_goal_total = 0
        status_interval = max(0.02, float(args.status_interval))
        while time.time() < deadline and any(proc.poll() is None for proc in processes.values()):
            try:
                status = ctl.status()
                score = status.get("score") if isinstance(status.get("score"), dict) else {}
                event = str(score.get("last_event", "")).strip() if score else ""
                penalized = _penalized_robot(event, robots)
                if penalized is not None:
                    suspended_robots.add(penalized)

                current_goal_total = _score_total(score)
                if _uses_controlled_goal_reset(args) and current_goal_total > previous_goal_total:
                    layout_round += 1
                    layout = _layout_for_round(args.layout, robots, ball, layout_seed, layout_round)
                    report["layout_history"].append({"round": layout_round, "layout": layout})
                    if rules_enabled:
                        report.setdefault("round_rule_suspends", []).append(
                            {
                                "round": layout_round,
                                "response": _set_rules(ctl, enabled=False),
                            }
                        )
                    _place_robots(
                        ctl,
                        robots,
                        ball,
                        layout,
                        reset_score=False,
                    )
                    report.setdefault("round_rule_refreshes", []).append(
                        {
                            "round": layout_round,
                            "response": _refresh_rules_after_manual_place(ctl, enabled=rules_enabled),
                        }
                    )
                    time.sleep(0.05)
                    previous_goal_total = current_goal_total
                    status = ctl.status()
                    suspended_robots.clear()

                correction = _lock_active_robots(ctl, status, robots, suspended_robots)
                if correction is not None:
                    correction["time"] = status.get("time")
                    report["roster_corrections"].append(correction)
                    status = ctl.status()
                ball_recovery = _recover_out_of_bounds_ball(ctl, status)
                if ball_recovery is not None:
                    ball_recovery["time"] = status.get("time")
                    report.setdefault("ball_recoveries", []).append(ball_recovery)
                    status = ctl.status()
                _write_jsonl(scene_trace, {"t_wall": time.time(), "status": status})
                target_goals = max(1, int(args.target_goals))
                if not args.allow_no_goal and not args.continue_after_goal and _score_total(status.get("score")) >= target_goals:
                    if stop_after_goal_at is None:
                        stop_after_goal_at = time.time() + 0.5
            except Exception as exc:
                _write_jsonl(scene_trace, {"t_wall": time.time(), "error": str(exc)})
            if stop_after_goal_at is not None and time.time() >= stop_after_goal_at:
                ended_after_goal = True
                break
            time.sleep(status_interval)

        report["ended_after_goal"] = ended_after_goal
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
        report["scene_summary"] = _scene_summary(scene_trace)
        scene_summary = report["scene_summary"]
        possession_ok = args.allow_no_possession or scene_summary.get("max_holders", 0) > 0
        target_goals = max(1, int(args.target_goals))
        goal_ok = args.allow_no_goal or int(scene_summary.get("final_score_total", 0)) >= target_goals
        rule_event_ok = scene_summary.get("first_rule_event") is None
        scene_summary["goal_required"] = not args.allow_no_goal
        scene_summary["target_goals"] = 0 if args.allow_no_goal else target_goals
        scene_summary["goal_ok"] = goal_ok
        robot_process_ok = all(
            (
                item["exit_code"] == 0
                or (ended_after_goal and item["exit_code"] is not None and item["exit_code"] != 2)
            )
            and item["trace_summary"]["frames"] > 5
            for item in robot_reports
        )
        report["ok"] = (
            robot_process_ok
            and scene_summary.get("multi_possession_frames", 0) == 0
            and possession_ok
            and goal_ok
            and rule_event_ok
        )
        report["possession_required"] = not args.allow_no_possession
        report["goal_required"] = not args.allow_no_goal
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
