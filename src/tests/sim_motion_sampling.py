"""Collect simulator motion response using the normal chassis command path.

This script is intentionally operational, not a pytest test. Start the Unity
simulator first, then run:

    python -m src.tests.sim_motion_sampling

It sends the same style of commands used by real sampling:
`chassis.RobotMoveVector(vx, vy, wz)`, records pose deltas, and writes a CSV
that can be compared with `Records/friction/friction_real_sampling_*.csv`.
"""

from __future__ import annotations

import csv
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, "src")
os.environ.setdefault("ROBOCUPX_BITS", "SIM")

from LunaPre import Bits, UnitedPosition, chassis, logger  # noqa: E402


INVALID_POSE = [0xFFF, 0xFFF, 0xFFF]


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except Exception:
        return float(default)


def _env_csv_floats(name: str, default: list[float]) -> list[float]:
    raw = str(os.environ.get(name, "")).strip()
    if not raw:
        return list(default)
    out: list[float] = []
    for part in raw.split(","):
        try:
            out.append(float(part.strip()))
        except Exception:
            pass
    return out or list(default)


def _env_axes(name: str, default: list[str]) -> list[str]:
    raw = str(os.environ.get(name, "")).strip().lower()
    axes = [x.strip() for x in raw.split(",") if x.strip()] if raw else list(default)
    return [axis for axis in axes if axis in {"x", "y", "w"}] or list(default)


def _angle_diff_deg(current: float, start: float) -> float:
    return (float(current) - float(start) + 180.0) % 360.0 - 180.0


def _safe_pose() -> list[float]:
    pose = UnitedPosition.FieldRobotPos()
    if not isinstance(pose, list) or len(pose) < 3 or pose == INVALID_POSE:
        return list(INVALID_POSE)
    return [float(pose[0]), float(pose[1]), float(pose[2])]


def _command(axis: str, sign: int, level: float) -> tuple[float, float, float]:
    value = float(sign) * float(level)
    if axis == "x":
        return value, 0.0, 0.0
    if axis == "y":
        return 0.0, value, 0.0
    if axis == "w":
        return 0.0, 0.0, value
    raise ValueError(axis)


def wait_for_sim(timeout_s: float) -> bool:
    start_rx = int(getattr(Bits, "_rx_count", 0))
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        pose = _safe_pose()
        rx = int(getattr(Bits, "_rx_count", 0))
        if pose != INVALID_POSE and rx > start_rx:
            logger.info("[SimMotion] connected: pose={} rx_delta={}", pose, rx - start_rx)
            return True
        time.sleep(0.2)
    logger.error("[SimMotion] no simulator status packets received")
    return False


def write_phase(
    writer: csv.DictWriter,
    *,
    axis: str,
    level: float,
    phase: str,
    cmd: tuple[float, float, float],
    duration_s: float,
    sample_hz: float,
    sample_index_start: int,
) -> int:
    start_pose = _safe_pose()
    prev_pose = start_pose
    prev_t = time.time()
    phase_start = prev_t
    sample_index = sample_index_start
    interval = 1.0 / max(1.0, sample_hz)

    while time.time() - phase_start < duration_s:
        chassis.RobotMoveVector(*cmd)
        now = time.time()
        pose = _safe_pose()
        status = getattr(Bits, "_status", None)
        if pose != INVALID_POSE and prev_pose != INVALID_POSE:
            dt = max(1e-6, now - prev_t)
            dx = pose[0] - start_pose[0]
            dy = pose[1] - start_pose[1]
            dyaw = _angle_diff_deg(pose[2], start_pose[2])
            step_dx = pose[0] - prev_pose[0]
            step_dy = pose[1] - prev_pose[1]
            step_dyaw = _angle_diff_deg(pose[2], prev_pose[2])
            writer.writerow(
                {
                    "axis": axis,
                    "level": level,
                    "phase": phase,
                    "sample_index": sample_index,
                    "t": round(now, 6),
                    "phase_elapsed_s": round(now - phase_start, 6),
                    "cmd_vx": round(cmd[0], 4),
                    "cmd_vy": round(cmd[1], 4),
                    "cmd_wz": round(cmd[2], 4),
                    "x_cm": round(pose[0], 4),
                    "y_cm": round(pose[1], 4),
                    "yaw_deg": round(pose[2], 4),
                    "status_vx_mps": round(float(getattr(status, "vx_mps", 0.0)), 5),
                    "status_vy_mps": round(float(getattr(status, "vy_mps", 0.0)), 5),
                    "status_wz_rps": round(float(getattr(status, "wz_rps", 0.0)), 5),
                    "phase_dx_cm": round(dx, 4),
                    "phase_dy_cm": round(dy, 4),
                    "phase_dyaw_deg": round(dyaw, 4),
                    "est_vx_cm_s": round(step_dx / dt, 4),
                    "est_vy_cm_s": round(step_dy / dt, 4),
                    "est_wz_deg_s": round(step_dyaw / dt, 4),
                    "est_speed_cm_s": round((step_dx * step_dx + step_dy * step_dy) ** 0.5 / dt, 4),
                }
            )
            sample_index += 1
        prev_pose = pose
        prev_t = now
        time.sleep(interval)
    return sample_index


def main() -> int:
    axes = _env_axes("SIM_MOTION_AXES", ["x", "y", "w"])
    levels = _env_csv_floats("SIM_MOTION_LEVELS", [8, 12, 16, 20, 24])
    sample_hz = _env_float("SIM_MOTION_SAMPLE_HZ", 20.0)
    hold_s = _env_float("SIM_MOTION_HOLD_S", 0.45)
    coast_s = _env_float("SIM_MOTION_COAST_S", 0.35)
    settle_s = _env_float("SIM_MOTION_SETTLE_S", 0.25)
    warmup_s = _env_float("SIM_MOTION_WARMUP_S", 5.0)
    out_dir = Path(os.environ.get("SIM_MOTION_OUTPUT", "Records/sim_motion")).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / f"sim_motion_sampling_{time.strftime('%Y%m%d_%H%M%S')}.csv"

    if not wait_for_sim(warmup_s):
        return 2

    fields = [
        "axis",
        "level",
        "phase",
        "sample_index",
        "t",
        "phase_elapsed_s",
        "cmd_vx",
        "cmd_vy",
        "cmd_wz",
        "x_cm",
        "y_cm",
        "yaw_deg",
        "status_vx_mps",
        "status_vy_mps",
        "status_wz_rps",
        "phase_dx_cm",
        "phase_dy_cm",
        "phase_dyaw_deg",
        "est_vx_cm_s",
        "est_vy_cm_s",
        "est_wz_deg_s",
        "est_speed_cm_s",
    ]

    logger.info("[SimMotion] writing {}", csv_path)
    sample_index = 0
    try:
        with csv_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for axis in axes:
                for level in levels:
                    for sign, phase in ((1, "forward"), (-1, "backward")):
                        cmd = _command(axis, sign, level)
                        logger.info("[SimMotion] phase axis={} level={} phase={} cmd={}", axis, level, phase, cmd)
                        sample_index = write_phase(
                            writer,
                            axis=axis,
                            level=level,
                            phase=phase,
                            cmd=cmd,
                            duration_s=hold_s,
                            sample_hz=sample_hz,
                            sample_index_start=sample_index,
                        )
                        chassis.stop()
                        sample_index = write_phase(
                            writer,
                            axis=axis,
                            level=level,
                            phase=f"{phase}_coast",
                            cmd=(0.0, 0.0, 0.0),
                            duration_s=coast_s,
                            sample_hz=sample_hz,
                            sample_index_start=sample_index,
                        )
                        chassis.stop()
                        time.sleep(settle_s)
    finally:
        chassis.stop()
        UnitedPosition.Stop()

    logger.info("[SimMotion] done: {}", csv_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
