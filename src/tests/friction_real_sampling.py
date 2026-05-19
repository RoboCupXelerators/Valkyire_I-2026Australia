"""真机底盘摩擦 / 运动响应安全采样脚本。

目标：
- 采集真机对 vx / vy / wz 阶跃指令的响应数据
- 估算静摩擦起动阈值、速度比例、滑行尾巴
- 输出 CSV，供后续把模拟器的 deadzone / friction / damping / speed scale 调到接近真机

安全策略：
- 默认低速短时采样，避免冲到边缘
- 每个幅值都执行“正向一段 + 反向一段”往返采样
- 每个幅值结束后，使用低速 Pos2Pos 回到本轮采样起始原点

使用方法：
    python -m src.tests.friction_real_sampling

常用环境变量：
    FRICTION_WARMUP_S=8                # 等待定位稳定时间
    FRICTION_SAMPLE_HZ=20              # 采样频率
    FRICTION_SETTLE_S=0.6              # 每段动作前静止时间
    FRICTION_HOLD_S=0.45               # 每段指令保持时间，越大位移越大
    FRICTION_COAST_S=0.35              # 收指令后观察滑行时间
    FRICTION_MOVE_EPS_CM=1.2           # 判定“开始动了”的位移阈值
    FRICTION_YAW_EPS_DEG=3.0           # 判定“开始转了”的角度阈值
    FRICTION_LEVELS=8,12,16,20,24      # 阶跃指令档位，默认很保守
    FRICTION_AXES=x,y,w                # 采样轴，可改成 x 或 x,y
    FRICTION_RETURN_SPEED_GAIN=0.6     # 回原点时 Pos2Pos 的低速系数
    FRICTION_RETURN_TIMEOUT_S=5        # 回原点超时时间
    FRICTION_OUTPUT=Records/friction   # 输出目录

如果空间很小，建议先跑：
    FRICTION_AXES=x FRICTION_LEVELS=8,12,16 FRICTION_HOLD_S=0.3 python -m src.tests.friction_real_sampling
"""

from __future__ import annotations

import csv
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, "src")

from LunaPre import UnitedPosition, chassis, logger, Settings  # noqa: E402


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


def _safe_pose() -> list[float]:
    pose = UnitedPosition.FieldRobotPos()
    if not isinstance(pose, list) or len(pose) < 3:
        return list(INVALID_POSE)
    if pose == INVALID_POSE:
        return list(INVALID_POSE)
    return [float(pose[0]), float(pose[1]), float(pose[2])]


def _angle_diff_deg(current: float, start: float) -> float:
    return (float(current) - float(start) + 180.0) % 360.0 - 180.0


def wait_for_valid_pose(timeout_s: float, stable_reads: int = 3) -> list[float] | None:
    start = time.time()
    good_reads = 0
    last_pose = None
    while time.time() - start < timeout_s:
        pose = _safe_pose()
        if pose != INVALID_POSE:
            good_reads += 1
            last_pose = pose
            logger.info(f"[FrictionSample] Valid pose {good_reads}/{stable_reads}: {pose}")
            if good_reads >= stable_reads:
                return pose
        else:
            good_reads = 0
        time.sleep(0.2)
    return last_pose


@dataclass(frozen=True)
class MotionStep:
    axis: str
    sign: int
    level: float

    @property
    def label(self) -> str:
        prefix = "+" if self.sign >= 0 else "-"
        return f"{prefix}{self.axis}_{int(self.level)}"

    def to_command(self) -> tuple[float, float, float]:
        value = float(self.sign) * float(self.level)
        if self.axis == "x":
            return value, 0.0, 0.0
        if self.axis == "y":
            return 0.0, value, 0.0
        if self.axis == "w":
            return 0.0, 0.0, value
        raise ValueError(f"Unsupported axis: {self.axis}")


def build_steps(axes: list[str], levels: list[float]) -> list[MotionStep]:
    return [MotionStep(axis=axis, sign=1, level=level) for axis in axes for level in levels]


def ensure_parent(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def estimate_motion_metrics(start_pose: list[float], pose: list[float], dt: float) -> dict[str, float]:
    dx = float(pose[0]) - float(start_pose[0])
    dy = float(pose[1]) - float(start_pose[1])
    dyaw = _angle_diff_deg(pose[2], start_pose[2])
    disp_cm = math.hypot(dx, dy)
    if dt <= 1e-6:
        vel_x = vel_y = yaw_rate = speed_cm_s = 0.0
    else:
        vel_x = dx / dt
        vel_y = dy / dt
        yaw_rate = dyaw / dt
        speed_cm_s = disp_cm / dt
    return {
        "dx_cm": dx,
        "dy_cm": dy,
        "dyaw_deg": dyaw,
        "disp_cm": disp_cm,
        "est_vx_cm_s": vel_x,
        "est_vy_cm_s": vel_y,
        "est_wz_deg_s": yaw_rate,
        "est_speed_cm_s": speed_cm_s,
    }


def settle_robot(duration_s: float, sample_hz: float) -> None:
    chassis.stop()
    end_t = time.time() + max(0.0, duration_s)
    sleep_s = 1.0 / max(1.0, sample_hz)
    while time.time() < end_t:
        chassis.stop()
        time.sleep(min(0.05, sleep_s))


def return_to_origin(origin_pose: list[float], speed_gain: float, timeout_s: float) -> bool:
    target = [float(origin_pose[0]), float(origin_pose[1]), float(origin_pose[2])]
    end_t = time.time() + max(0.5, timeout_s)
    while time.time() < end_t:
        pose = _safe_pose()
        if pose == INVALID_POSE:
            chassis.stop()
            time.sleep(0.1)
            continue
        try:
            if UnitedPosition.Pos2Pos(target, speed_gain=speed_gain):
                chassis.stop()
                return True
        except Exception as e:
            logger.warning(f"[FrictionSample] return_to_origin failed: {e}")
            break
        time.sleep(0.05)
    chassis.stop()
    return False


def sample_phase(
    *,
    writer: csv.DictWriter,
    fieldnames: list[str],
    session_id: str,
    robot_id: int,
    pair_index: int,
    step: MotionStep,
    phase: str,
    phase_sign: int,
    pair_origin: list[float],
    phase_origin: list[float],
    hold_s: float,
    coast_s: float,
    sample_dt: float,
    move_eps_cm: float,
    yaw_eps_deg: float,
    sample_index: int,
) -> tuple[int, bool, float, int]:
    phase_step = MotionStep(axis=step.axis, sign=phase_sign, level=step.level)
    cmd_vx, cmd_vy, cmd_wz = phase_step.to_command()
    phase_start_t = time.time()
    moving_started = False
    start_latency_s = -1.0
    rows = 0

    def write_row(row_phase: str, now: float, pose: list[float], phase_elapsed: float, cmd: tuple[float, float, float]) -> None:
        nonlocal rows, sample_index, moving_started, start_latency_s
        rel_metrics = estimate_motion_metrics(phase_origin, pose, phase_elapsed)
        pair_metrics = estimate_motion_metrics(pair_origin, pose, max(phase_elapsed, 1e-6))
        movement_metric = abs(rel_metrics["dyaw_deg"]) if step.axis == "w" else rel_metrics["disp_cm"]
        row = {
            "session_id": session_id,
            "robot_id": robot_id,
            "pair_index": pair_index,
            "axis": step.axis,
            "sign": phase_sign,
            "level": round(step.level, 4),
            "phase": row_phase,
            "sample_index": sample_index,
            "cmd_vx": round(cmd[0], 4),
            "cmd_vy": round(cmd[1], 4),
            "cmd_wz": round(cmd[2], 4),
            "ts": round(now, 6),
            "phase_elapsed_s": round(phase_elapsed, 6),
            "x_cm": round(float(pose[0]), 4),
            "y_cm": round(float(pose[1]), 4),
            "yaw_deg": round(float(pose[2]), 4),
            "phase_dx_cm": round(rel_metrics["dx_cm"], 4),
            "phase_dy_cm": round(rel_metrics["dy_cm"], 4),
            "phase_dyaw_deg": round(rel_metrics["dyaw_deg"], 4),
            "phase_disp_cm": round(rel_metrics["disp_cm"], 4),
            "pair_dx_cm": round(pair_metrics["dx_cm"], 4),
            "pair_dy_cm": round(pair_metrics["dy_cm"], 4),
            "pair_dyaw_deg": round(pair_metrics["dyaw_deg"], 4),
            "pair_disp_cm": round(pair_metrics["disp_cm"], 4),
            "est_vx_cm_s": round(rel_metrics["est_vx_cm_s"], 4),
            "est_vy_cm_s": round(rel_metrics["est_vy_cm_s"], 4),
            "est_wz_deg_s": round(rel_metrics["est_wz_deg_s"], 4),
            "est_speed_cm_s": round(rel_metrics["est_speed_cm_s"], 4),
            "started_moving": int(moving_started),
            "start_latency_s": round(start_latency_s, 6),
            "movement_metric": round(movement_metric, 4),
        }
        writer.writerow({name: row[name] for name in fieldnames})
        rows += 1
        sample_index += 1

    hold_end_t = phase_start_t + hold_s
    while time.time() < hold_end_t:
        now = time.time()
        chassis.RobotMoveVector(cmd_vx, cmd_vy, cmd_wz)
        pose = _safe_pose()
        if pose == INVALID_POSE:
            chassis.stop()
            time.sleep(sample_dt)
            continue
        phase_elapsed = now - phase_start_t
        rel_metrics = estimate_motion_metrics(phase_origin, pose, phase_elapsed)
        movement_metric = abs(rel_metrics["dyaw_deg"]) if step.axis == "w" else rel_metrics["disp_cm"]
        threshold = yaw_eps_deg if step.axis == "w" else move_eps_cm
        if (not moving_started) and movement_metric >= threshold:
            moving_started = True
            start_latency_s = phase_elapsed
            logger.info(
                "[FrictionSample] %s %s moved: metric=%.3f latency=%.3fs",
                phase,
                phase_step.label,
                movement_metric,
                start_latency_s,
            )
        write_row(phase, now, pose, phase_elapsed, (cmd_vx, cmd_vy, cmd_wz))
        time.sleep(sample_dt)

    chassis.stop()
    coast_start_t = time.time()
    while time.time() - coast_start_t < coast_s:
        now = time.time()
        pose = _safe_pose()
        if pose != INVALID_POSE:
            write_row(f"{phase}_coast", now, pose, now - phase_start_t, (0.0, 0.0, 0.0))
        time.sleep(sample_dt)

    return rows, moving_started, start_latency_s, sample_index


def run_sampling(csv_path: Path) -> int:
    warmup_s = max(3.0, min(20.0, _env_float("FRICTION_WARMUP_S", 8.0)))
    sample_hz = max(5.0, min(60.0, _env_float("FRICTION_SAMPLE_HZ", 20.0)))
    settle_s = max(0.15, min(2.0, _env_float("FRICTION_SETTLE_S", 0.6)))
    hold_s = max(0.1, min(1.5, _env_float("FRICTION_HOLD_S", 0.45)))
    coast_s = max(0.1, min(1.0, _env_float("FRICTION_COAST_S", 0.35)))
    move_eps_cm = max(0.2, min(8.0, _env_float("FRICTION_MOVE_EPS_CM", 1.2)))
    yaw_eps_deg = max(0.5, min(15.0, _env_float("FRICTION_YAW_EPS_DEG", 3.0)))
    return_speed_gain = max(0.2, min(1.0, _env_float("FRICTION_RETURN_SPEED_GAIN", 0.6)))
    return_timeout_s = max(1.0, min(12.0, _env_float("FRICTION_RETURN_TIMEOUT_S", 5.0)))
    levels = [max(0.0, float(v)) for v in _env_csv_floats("FRICTION_LEVELS", [8, 12, 16, 20, 24])]
    axes = _env_axes("FRICTION_AXES", ["x", "y", "w"])
    steps = build_steps(axes, levels)
    sample_dt = 1.0 / sample_hz
    session_id = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    robot_id = int(getattr(Settings.RoboInfo, "number", 1))

    logger.info(
        "[FrictionSample] params: warmup=%.1fs hz=%.1f settle=%.2fs hold=%.2fs coast=%.2fs move_eps=%.2f yaw_eps=%.2f return_gain=%.2f return_timeout=%.1fs axes=%s levels=%s",
        warmup_s,
        sample_hz,
        settle_s,
        hold_s,
        coast_s,
        move_eps_cm,
        yaw_eps_deg,
        return_speed_gain,
        return_timeout_s,
        axes,
        levels,
    )

    origin = wait_for_valid_pose(timeout_s=warmup_s, stable_reads=3)
    if origin is None or origin == INVALID_POSE:
        logger.error("[FrictionSample] No valid pose, abort.")
        return 2

    logger.info(f"[FrictionSample] Sampling origin pose: {origin}")
    ensure_parent(csv_path)

    fieldnames = [
        "session_id", "robot_id", "pair_index", "axis", "sign", "level", "phase",
        "sample_index", "cmd_vx", "cmd_vy", "cmd_wz",
        "ts", "phase_elapsed_s",
        "x_cm", "y_cm", "yaw_deg",
        "phase_dx_cm", "phase_dy_cm", "phase_dyaw_deg", "phase_disp_cm",
        "pair_dx_cm", "pair_dy_cm", "pair_dyaw_deg", "pair_disp_cm",
        "est_vx_cm_s", "est_vy_cm_s", "est_wz_deg_s", "est_speed_cm_s",
        "started_moving", "start_latency_s", "movement_metric",
    ]

    total_rows = 0
    sample_index = 0
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for pair_index, step in enumerate(steps, start=1):
            logger.info(
                "[FrictionSample] Pair %d/%d axis=%s level=%.1f: forward then backward",
                pair_index,
                len(steps),
                step.axis,
                step.level,
            )
            settle_robot(settle_s, sample_hz)
            pair_origin = _safe_pose()
            if pair_origin == INVALID_POSE:
                logger.warning("[FrictionSample] Invalid pose before pair, skip")
                continue

            rows, fwd_started, fwd_latency, sample_index = sample_phase(
                writer=writer,
                fieldnames=fieldnames,
                session_id=session_id,
                robot_id=robot_id,
                pair_index=pair_index,
                step=step,
                phase="forward",
                phase_sign=1,
                pair_origin=pair_origin,
                phase_origin=pair_origin,
                hold_s=hold_s,
                coast_s=coast_s,
                sample_dt=sample_dt,
                move_eps_cm=move_eps_cm,
                yaw_eps_deg=yaw_eps_deg,
                sample_index=sample_index,
            )
            total_rows += rows

            settle_robot(min(0.3, settle_s), sample_hz)
            reverse_origin = _safe_pose()
            if reverse_origin != INVALID_POSE:
                rows, back_started, back_latency, sample_index = sample_phase(
                    writer=writer,
                    fieldnames=fieldnames,
                    session_id=session_id,
                    robot_id=robot_id,
                    pair_index=pair_index,
                    step=step,
                    phase="backward",
                    phase_sign=-1,
                    pair_origin=pair_origin,
                    phase_origin=reverse_origin,
                    hold_s=hold_s,
                    coast_s=coast_s,
                    sample_dt=sample_dt,
                    move_eps_cm=move_eps_cm,
                    yaw_eps_deg=yaw_eps_deg,
                    sample_index=sample_index,
                )
                total_rows += rows
            else:
                back_started = False
                back_latency = -1.0
                logger.warning("[FrictionSample] Invalid pose before backward phase, skip backward")

            returned = return_to_origin(origin, speed_gain=return_speed_gain, timeout_s=return_timeout_s)
            logger.info(
                "[FrictionSample] Pair %s summary: fwd_started=%s fwd_latency=%.3fs back_started=%s back_latency=%.3fs return=%s",
                step.label,
                fwd_started,
                fwd_latency,
                back_started,
                back_latency,
                "ok" if returned else "timeout",
            )
            if not returned:
                logger.warning("[FrictionSample] Return failed; continuing with next pair from current pose, please monitor field boundary.")

    logger.info(f"[FrictionSample] Sampling completed, rows={total_rows}, output={csv_path}")
    return 0


def main() -> int:
    out_dir = Path(os.environ.get("FRICTION_OUTPUT", "Records/friction")).expanduser()
    if not out_dir.is_absolute():
        repo_root = Path(__file__).resolve().parent.parent.parent
        out_dir = repo_root / out_dir
    csv_path = out_dir / f"friction_real_sampling_{time.strftime('%Y%m%d_%H%M%S', time.localtime())}.csv"
    return run_sampling(csv_path)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    finally:
        try:
            chassis.stop()
        except Exception:
            pass
        try:
            UnitedPosition.Stop()
        except Exception:
            pass
