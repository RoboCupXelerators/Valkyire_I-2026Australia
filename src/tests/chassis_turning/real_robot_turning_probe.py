"""Conservative real-robot probe for moving while turning.

Default mode is dry-run and does not import LunaPre or move hardware.

Run from Software:
    python src/tests/chassis_turning/real_robot_turning_probe.py
    python src/tests/chassis_turning/real_robot_turning_probe.py --armed --duration-s 0.8 --speed-scale 0.8
"""

from __future__ import annotations

import argparse
import csv
import time
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class ProbeStep:
    name: str
    vx: float
    vy: float
    wz: float

    def scaled(self, scale: float) -> "ProbeStep":
        return ProbeStep(self.name, self.vx * scale, self.vy * scale, self.wz * scale)


BASE_STEPS = [
    ProbeStep("forward_left_turn", 45.0, 0.0, 35.0),
    ProbeStep("forward_right_turn", 45.0, 0.0, -35.0),
    ProbeStep("strafe_left_turn", 0.0, 35.0, 30.0),
    ProbeStep("strafe_right_turn", 0.0, -35.0, -30.0),
    ProbeStep("diagonal_left_turn", 35.0, 25.0, 30.0),
    ProbeStep("diagonal_right_turn", 35.0, -25.0, -30.0),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--armed", action="store_true", help="actually command the robot")
    parser.add_argument("--duration-s", type=float, default=0.8, help="command duration per step")
    parser.add_argument("--coast-s", type=float, default=0.35, help="stop-and-observe time after each step")
    parser.add_argument("--settle-s", type=float, default=0.4, help="stop time before each step")
    parser.add_argument("--sample-hz", type=float, default=20.0, help="pose sample rate")
    parser.add_argument("--speed-scale", type=float, default=0.8, help="scale all vx/vy/wz commands")
    parser.add_argument("--output-dir", default="Records/chassis_turning", help="CSV output directory")
    parser.add_argument("--allow-invalid-pose", action="store_true", help="continue even if pose is unavailable")
    return parser.parse_args()


def _safe_pose(united_position) -> list[float]:
    try:
        pose = united_position.FieldRobotPos()
        if isinstance(pose, list) and len(pose) >= 3:
            return [float(pose[0]), float(pose[1]), float(pose[2])]
    except Exception:
        pass
    return [4095.0, 4095.0, 4095.0]


def _valid_pose(pose: list[float]) -> bool:
    return pose != [4095.0, 4095.0, 4095.0]


def wait_for_valid_pose(united_position, timeout_s: float = 8.0) -> list[float] | None:
    deadline = time.time() + timeout_s
    good = 0
    last_pose = None
    while time.time() < deadline:
        pose = _safe_pose(united_position)
        if _valid_pose(pose):
            good += 1
            last_pose = pose
            if good >= 3:
                return pose
        else:
            good = 0
        time.sleep(0.2)
    return last_pose


def open_writer(output_dir: str) -> tuple[Path, csv.DictWriter, object]:
    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"turning_probe_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    fh = path.open("w", newline="", encoding="utf-8")
    writer = csv.DictWriter(
        fh,
        fieldnames=[
            "t_s",
            "phase",
            "step",
            "cmd_vx",
            "cmd_vy",
            "cmd_wz",
            "pose_x",
            "pose_y",
            "pose_yaw",
        ],
    )
    writer.writeheader()
    return path, writer, fh


def write_sample(writer, start_t: float, phase: str, step: ProbeStep, pose: list[float]) -> None:
    writer.writerow(
        {
            "t_s": round(time.time() - start_t, 4),
            "phase": phase,
            "step": step.name,
            "cmd_vx": round(step.vx, 3),
            "cmd_vy": round(step.vy, 3),
            "cmd_wz": round(step.wz, 3),
            "pose_x": round(pose[0], 3),
            "pose_y": round(pose[1], 3),
            "pose_yaw": round(pose[2], 3),
        }
    )


def hold_stop(chassis, united_position, writer, start_t: float, step: ProbeStep, phase: str, duration_s: float, sample_hz: float) -> None:
    deadline = time.time() + max(0.0, duration_s)
    sleep_s = 1.0 / max(1.0, sample_hz)
    while time.time() < deadline:
        chassis.stop()
        write_sample(writer, start_t, phase, step, _safe_pose(united_position))
        time.sleep(sleep_s)


def run_armed(args: argparse.Namespace) -> int:
    from LunaPre import UnitedPosition, chassis, logger  # noqa: PLC0415

    duration_s = max(0.2, min(2.0, float(args.duration_s)))
    coast_s = max(0.0, min(1.5, float(args.coast_s)))
    settle_s = max(0.0, min(1.5, float(args.settle_s)))
    sample_hz = max(5.0, min(50.0, float(args.sample_hz)))
    speed_scale = max(0.1, min(1.5, float(args.speed_scale)))
    steps = [step.scaled(speed_scale) for step in BASE_STEPS]

    pose = wait_for_valid_pose(UnitedPosition)
    if pose is None and not args.allow_invalid_pose:
        logger.error("[TurningProbe] no valid pose, aborting. Use --allow-invalid-pose only for wheel-output smoke tests.")
        return 2

    csv_path, writer, fh = open_writer(args.output_dir)
    logger.info(f"[TurningProbe] writing {csv_path}")
    logger.info(
        "[TurningProbe] duration=%.2fs coast=%.2fs settle=%.2fs sample_hz=%.1f speed_scale=%.2f",
        duration_s,
        coast_s,
        settle_s,
        sample_hz,
        speed_scale,
    )

    start_t = time.time()
    try:
        for step in steps:
            logger.info(f"[TurningProbe] step {step}")
            hold_stop(chassis, UnitedPosition, writer, start_t, step, "settle", settle_s, sample_hz)
            deadline = time.time() + duration_s
            sleep_s = 1.0 / sample_hz
            while time.time() < deadline:
                chassis.RobotMoveVector(step.vx, step.vy, step.wz)
                write_sample(writer, start_t, "cmd", step, _safe_pose(UnitedPosition))
                time.sleep(sleep_s)
            hold_stop(chassis, UnitedPosition, writer, start_t, step, "coast", coast_s, sample_hz)
    finally:
        try:
            chassis.stop()
        finally:
            fh.close()
            try:
                UnitedPosition.Stop()
            except Exception:
                pass

    logger.info(f"[TurningProbe] done: {csv_path}")
    return 0


def main() -> int:
    args = parse_args()
    if not args.armed:
        print("DRY RUN: no hardware will be imported or moved.")
        print("Planned steps:")
        for step in BASE_STEPS:
            scaled = step.scaled(max(0.1, min(1.5, float(args.speed_scale))))
            print(f"  {scaled.name}: vx={scaled.vx:.1f}, vy={scaled.vy:.1f}, wz={scaled.wz:.1f}")
        print("Run with --armed only when the robot is on a clear field and someone can stop it.")
        return 0
    return run_armed(args)


if __name__ == "__main__":
    raise SystemExit(main())
