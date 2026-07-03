"""Render a multi-robot scrimmage scene trace at simulator-time speed."""

from __future__ import annotations

import argparse
import json
import math
import re
import subprocess
from bisect import bisect_right
from pathlib import Path
from typing import Any

import cv2
import numpy as np


FIELD_X_CM = 120.0
FIELD_Y_CM = 90.0
TEAM_COLORS = {
    1: (80, 190, 255),
    2: (80, 190, 255),
    3: (255, 140, 80),
    4: (255, 140, 80),
}

_PUSHING_EVENT_RE = re.compile(r"pushing\s+r(\d+)\s*/\s*r(\d+)", re.IGNORECASE)


def _read_rows(scene_trace: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for line in scene_trace.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        status = row.get("status") or {}
        if "time" in status:
            rows.append(row)
    if len(rows) < 2:
        raise ValueError(f"not enough scene rows in {scene_trace}")
    return rows


def _read_robot_traces(run_dir: Path) -> dict[int, dict[str, Any]]:
    traces: dict[int, dict[str, Any]] = {}
    for path in sorted(run_dir.glob("robot_*_*.jsonl")):
        rows: list[dict[str, Any]] = []
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            if not line.strip():
                continue
            row = json.loads(line)
            if "t_s" not in row:
                continue
            rows.append(row)
        if not rows:
            continue
        robot_id = int(rows[0].get("robot_id", 0))
        if robot_id <= 0:
            continue
        traces[robot_id] = {
            "path": path,
            "rows": rows,
            "times": [float(row.get("t_s", 0.0)) for row in rows],
        }
    return traces


def _read_run_timestamp(run_dir: Path) -> str:
    summary_path = run_dir / "summary.json"
    if summary_path.exists():
        try:
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            started_at = str(summary.get("started_at") or "").strip()
            if started_at:
                return started_at.replace("T", " ")
        except (OSError, json.JSONDecodeError):
            pass

    return run_dir.name


def _trace_at(trace: dict[str, Any] | None, t_rel: float) -> dict[str, Any] | None:
    if not trace:
        return None
    rows = trace.get("rows") or []
    times = trace.get("times") or []
    if not rows or not times:
        return None
    idx = bisect_right(times, t_rel) - 1
    if idx < 0:
        return rows[0]
    if idx >= len(rows):
        return rows[-1]
    return rows[idx]


def _point(width: int, height: int, scale: float, x_cm: float, y_cm: float) -> tuple[int, int]:
    return int(width / 2 + x_cm * scale), int(height / 2 - y_cm * scale)


def _draw_text(
    img: np.ndarray,
    text: str,
    xy: tuple[int, int],
    color: tuple[int, int, int] = (245, 245, 245),
    size: float = 0.6,
    thick: int = 1,
) -> None:
    x, y = xy
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, size, (0, 0, 0), thick + 3, cv2.LINE_AA)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, size, color, thick, cv2.LINE_AA)


def _short(text: str, max_chars: int = 44) -> str:
    if len(text) <= max_chars:
        return text
    return text[: max(0, max_chars - 3)] + "..."


def _trace_action(row: dict[str, Any] | None) -> str:
    if not row:
        return ""
    decision = row.get("offence_decision") or row.get("defence_decision") or {}
    return str(decision.get("action") or row.get("state") or "")


def _trace_field_label(row: dict[str, Any] | None) -> str:
    if not row:
        return ""
    decision = row.get("offence_decision") or row.get("defence_decision") or {}
    state = str(row.get("state") or "")
    action = str(decision.get("action") or "")
    if state and action and state != action:
        return f"{state}>{action}"
    return state or action


def _decision_detail_lines(decision: dict[str, Any]) -> list[str]:
    if not decision:
        return []

    lines: list[str] = []
    for key in ("mode", "shot_gate", "force_gate", "control_source", "distance", "angle", "behind_gap"):
        if key in decision:
            lines.append(f"{key}={decision.get(key)}")

    if "progress" in decision or "y" in decision:
        lines.append(f"progress={decision.get('progress', '')} y={decision.get('y', '')}")

    if "lane_score" in decision or "yaw_error" in decision:
        lines.append(f"lane={decision.get('lane_score', '')} yaw={decision.get('yaw_error', '')}")

    for key in ("target", "target_y", "speed", "field_ball", "ball", "blocker", "pressure"):
        if key in decision:
            lines.append(f"{key}={decision.get(key)}")

    return lines


def _robot_xy(status: dict[str, Any], robot_id: int) -> tuple[float, float] | None:
    for robot in status.get("robots") or []:
        try:
            if int(robot.get("id", -1)) == int(robot_id):
                return float(robot.get("x_cm", 0.0)), float(robot.get("y_cm", 0.0))
        except (TypeError, ValueError):
            return None
    return None


def _display_event(status: dict[str, Any]) -> str:
    score = status.get("score") if isinstance(status.get("score"), dict) else {}
    event = str(score.get("last_event", "")).strip() if score else ""
    match = _PUSHING_EVENT_RE.search(event)
    if not match:
        return event

    first = _robot_xy(status, int(match.group(1)))
    second = _robot_xy(status, int(match.group(2)))
    if first is None or second is None:
        return event
    if math.hypot(first[0] - second[0], first[1] - second[1]) > 36.0:
        return ""
    return event


def _draw_debug_panel(
    img: np.ndarray,
    *,
    width: int,
    height: int,
    trace_snapshots: dict[int, dict[str, Any] | None],
) -> None:
    panel_w = min(width - 36, max(760, int(width * 0.48)))
    x0 = 18
    y0 = 86
    y1 = min(height - 18, max(y0 + 500, height - 28))

    overlay = img.copy()
    cv2.rectangle(overlay, (x0, y0), (x0 + panel_w, y1), (18, 24, 29), -1, cv2.LINE_AA)
    cv2.addWeighted(overlay, 0.64, img, 0.36, 0, img)
    cv2.rectangle(img, (x0, y0), (x0 + panel_w, y1), (120, 135, 145), 1, cv2.LINE_AA)

    _draw_text(img, "strategy trace", (x0 + 14, y0 + 26), (255, 255, 255), 0.62, 2)
    y = y0 + 54
    for robot_id in sorted(trace_snapshots):
        row = trace_snapshots[robot_id]
        if not row:
            continue
        role = str(row.get("role", ""))
        state = str(row.get("state", ""))
        action = _trace_action(row)
        decision = row.get("offence_decision") or row.get("defence_decision") or {}
        cmd_vx = float(row.get("cmd_vx", 0.0))
        cmd_vy = float(row.get("cmd_vy", 0.0))
        cmd_wz = float(row.get("cmd_wz", 0.0))
        color = TEAM_COLORS.get(robot_id, (235, 235, 235))

        _draw_text(img, f"R{robot_id} {role}", (x0 + 14, y), color, 0.54, 2)
        y += 22
        phase = str(decision.get("phase") or "")
        headline = f"phase={phase} action={action}" if phase else f"action={action}"
        _draw_text(img, _short(headline, 92), (x0 + 26, y), (245, 245, 245), 0.46, 1)
        y += 20
        _draw_text(img, _short(f"fn={role}.{state} branch={action}", 92), (x0 + 26, y), (195, 205, 212), 0.40, 1)
        y += 20
        for detail in _decision_detail_lines(decision)[:4]:
            _draw_text(img, _short(detail, 94), (x0 + 26, y), (220, 230, 235), 0.40, 1)
            y += 18
        _draw_text(img, f"cmd vx={cmd_vx:.1f} vy={cmd_vy:.1f} wz={cmd_wz:.1f}", (x0 + 26, y), (220, 230, 235), 0.40, 1)
        y += 26
        if y > y1 - 20:
            break


def _lerp(a: float, b: float, ratio: float) -> float:
    return a + (b - a) * ratio


def _angle_lerp(a: float, b: float, ratio: float) -> float:
    delta = (b - a + 180.0) % 360.0 - 180.0
    return a + delta * ratio


def _interpolated_status(rows: list[dict[str, Any]], times: list[float], t_abs: float) -> dict[str, Any]:
    idx = bisect_right(times, t_abs) - 1
    if idx < 0:
        return rows[0].get("status") or {}
    if idx >= len(rows) - 1:
        return rows[-1].get("status") or {}

    first = rows[idx].get("status") or {}
    second = rows[idx + 1].get("status") or {}
    t0 = times[idx]
    t1 = times[idx + 1]
    ratio = 0.0 if t1 <= t0 else (t_abs - t0) / (t1 - t0)

    out = dict(first)
    ball_first = first.get("ball") or {}
    ball_second = second.get("ball") or {}
    ball = dict(ball_first)
    for key in ("x_cm", "y_cm", "vx_cm_s", "vy_cm_s", "speed_cm_s"):
        if key in ball_first and key in ball_second:
            ball[key] = _lerp(float(ball_first[key]), float(ball_second[key]), ratio)
    out["ball"] = ball

    second_robots = {int(robot.get("id", 0)): robot for robot in second.get("robots") or []}
    robots: list[dict[str, Any]] = []
    for robot_first in first.get("robots") or []:
        robot_id = int(robot_first.get("id", 0))
        robot_second = second_robots.get(robot_id, robot_first)
        robot = dict(robot_first)
        for key in ("x_cm", "y_cm", "vx_forward_mps", "vy_left_mps", "wz_rad_s"):
            if key in robot_first and key in robot_second:
                robot[key] = _lerp(float(robot_first[key]), float(robot_second[key]), ratio)
        if "yaw_deg" in robot_first and "yaw_deg" in robot_second:
            robot["yaw_deg"] = _angle_lerp(float(robot_first["yaw_deg"]), float(robot_second["yaw_deg"]), ratio)
        robots.append(robot)
    out["robots"] = robots
    return out


def _draw_pitch(img: np.ndarray, width: int, height: int, scale: float) -> None:
    def pt(x_cm: float, y_cm: float) -> tuple[int, int]:
        return _point(width, height, scale, x_cm, y_cm)

    x0, y0 = pt(-FIELD_X_CM, FIELD_Y_CM)
    x1, y1 = pt(FIELD_X_CM, -FIELD_Y_CM)
    cv2.rectangle(img, (x0, y0), (x1, y1), (235, 235, 235), 2)
    cv2.line(img, pt(0, FIELD_Y_CM), pt(0, -FIELD_Y_CM), (210, 210, 210), 1)
    cv2.circle(img, pt(0, 0), int(18 * scale), (210, 210, 210), 1)
    cv2.rectangle(img, pt(-FIELD_X_CM, 28), pt(-FIELD_X_CM + 18, -28), (210, 210, 210), 1)
    cv2.rectangle(img, pt(FIELD_X_CM - 18, 28), pt(FIELD_X_CM, -28), (210, 210, 210), 1)
    cv2.rectangle(img, pt(-FIELD_X_CM - 4, 16), pt(-FIELD_X_CM, -16), (180, 180, 180), 2)
    cv2.rectangle(img, pt(FIELD_X_CM, 16), pt(FIELD_X_CM + 4, -16), (180, 180, 180), 2)


def _render_raw(
    rows: list[dict[str, Any]],
    robot_traces: dict[int, dict[str, Any]],
    output: Path,
    cover: Path,
    *,
    fps: float,
    width: int,
    height: int,
    title: str,
    run_timestamp: str,
) -> float:
    margin = 62
    scale = min((width - margin * 2) / (FIELD_X_CM * 2), (height - margin * 2) / (FIELD_Y_CM * 2))
    times = [float((row.get("status") or {}).get("time", 0.0)) for row in rows]
    start_time = times[0]
    duration = max(0.1, times[-1] - start_time)
    frame_count = int(math.ceil(duration * fps)) + 1

    writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError(f"VideoWriter failed: {output}")

    last_frame: np.ndarray | None = None
    cover_frame: np.ndarray | None = None
    cover_frame_idx = min(frame_count - 1, max(0, int(round(min(max(duration * 0.16, 8.0), duration - 2.0) * fps))))

    def pt(x_cm: float, y_cm: float) -> tuple[int, int]:
        return _point(width, height, scale, x_cm, y_cm)

    for frame_idx in range(frame_count):
        t_rel = min(duration, frame_idx / fps)
        status = _interpolated_status(rows, times, start_time + t_rel)
        score = status.get("score") or {}
        ball = status.get("ball") or {}
        trace_snapshots = {
            robot_id: _trace_at(trace, t_rel)
            for robot_id, trace in robot_traces.items()
        }

        img = np.zeros((height, width, 3), dtype=np.uint8)
        img[:] = (31, 92, 61)
        _draw_pitch(img, width, height, scale)

        trail_start = max(start_time, start_time + t_rel - 6.0)
        prev: tuple[int, int] | None = None
        for idx, trace_time in enumerate(times):
            if trace_time < trail_start or trace_time > start_time + t_rel:
                continue
            trace_ball = (rows[idx].get("status") or {}).get("ball") or {}
            if "x_cm" not in trace_ball or "y_cm" not in trace_ball:
                continue
            cur = pt(float(trace_ball["x_cm"]), float(trace_ball["y_cm"]))
            if prev is not None:
                cv2.line(img, prev, cur, (90, 225, 255), 2)
            prev = cur

        if "x_cm" in ball and "y_cm" in ball:
            cv2.circle(img, pt(float(ball["x_cm"]), float(ball["y_cm"])), max(5, int(2.5 * scale)), (30, 230, 255), -1, cv2.LINE_AA)
            _draw_text(img, f"ball {float(ball.get('speed_cm_s', 0.0)) / 100:.2f} m/s", (24, height - 28), (240, 240, 240), 0.55, 1)

        for robot in status.get("robots") or []:
            if not robot.get("active", False):
                continue
            robot_id = int(robot.get("id", 0))
            x_cm = float(robot.get("x_cm", 0.0))
            y_cm = float(robot.get("y_cm", 0.0))
            yaw = math.radians(float(robot.get("yaw_deg", 0.0)))
            cx, cy = pt(x_cm, y_cm)
            color = TEAM_COLORS.get(robot_id, (220, 220, 220))
            radius = int(7.5 * scale)
            cv2.circle(img, (cx, cy), radius, color, -1, cv2.LINE_AA)
            cv2.circle(img, (cx, cy), radius, (20, 20, 20), 2, cv2.LINE_AA)
            nose = (int(cx + math.cos(yaw) * radius * 1.4), int(cy - math.sin(yaw) * radius * 1.4))
            cv2.line(img, (cx, cy), nose, (10, 10, 10), 3, cv2.LINE_AA)
            _draw_text(img, f"R{robot_id}", (cx - 15, cy - radius - 8), (255, 255, 255), 0.55, 1)
            if robot.get("has_ball"):
                cv2.circle(img, (cx, cy), radius + 5, (0, 255, 255), 2, cv2.LINE_AA)
            trace_row = trace_snapshots.get(robot_id)
            label_action = _trace_field_label(trace_row)
            if label_action:
                label = f"{robot_id}:{label_action}"
                _draw_text(img, _short(label, 26), (cx + radius + 8, cy + 5), color, 0.42, 1)

        _draw_text(img, title, (24, 34), (255, 255, 255), 0.72, 2)
        _draw_text(
            img,
            f"t={t_rel:05.1f}s / {duration:05.1f}s  ally={score.get('ally', 0)} enemy={score.get('enemy', 0)}  event={_display_event(status)}",
            (24, 64),
            (235, 235, 235),
            0.58,
            1,
        )
        _draw_text(img, "simulator time 1:1", (width - 250, 34), (235, 235, 235), 0.58, 1)
        _draw_text(img, f"run {run_timestamp}", (width - 360, 64), (235, 235, 235), 0.48, 1)
        _draw_debug_panel(img, width=width, height=height, trace_snapshots=trace_snapshots)
        writer.write(img)
        last_frame = img
        if frame_idx == cover_frame_idx:
            cover_frame = img.copy()

    writer.release()
    frame_for_cover = cover_frame if cover_frame is not None else last_frame
    if frame_for_cover is not None:
        cv2.imwrite(str(cover), frame_for_cover, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    return duration


def _transcode_h264(raw: Path, output: Path) -> None:
    subprocess.run(
        [
            "ffmpeg",
            "-y",
            "-i",
            str(raw),
            "-vf",
            "format=yuv420p",
            "-c:v",
            "libx264",
            "-profile:v",
            "baseline",
            "-level",
            "4.0",
            "-preset",
            "veryfast",
            "-crf",
            "18",
            "-movflags",
            "+faststart",
            "-an",
            str(output),
        ],
        check=True,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Render scrimmage scene_status.jsonl as a 1:1 simulator-time replay.")
    parser.add_argument("run_dir", type=Path, help="Run directory containing scene_status.jsonl.")
    parser.add_argument("--output-dir", type=Path, default=Path("recordings"))
    parser.add_argument("--name", default=None)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--title", default="scrimmage replay")
    args = parser.parse_args()

    run_dir = args.run_dir
    scene_trace = run_dir / "scene_status.jsonl"
    rows = _read_rows(scene_trace)
    robot_traces = _read_robot_traces(run_dir)
    run_timestamp = _read_run_timestamp(run_dir)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    stem = args.name or f"{run_dir.name}_realtime"
    raw = args.output_dir / f"{stem}_raw.mp4"
    output = args.output_dir / f"{stem}_h264_{args.height}p.mp4"
    cover = args.output_dir / f"{stem}_cover.jpg"

    duration = _render_raw(
        rows,
        robot_traces,
        raw,
        cover,
        fps=args.fps,
        width=args.width,
        height=args.height,
        title=args.title,
        run_timestamp=run_timestamp,
    )
    _transcode_h264(raw, output)
    print(f"video={output}")
    print(f"cover={cover}")
    print(f"duration_s={duration:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
