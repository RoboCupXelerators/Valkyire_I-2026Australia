"""Run one simulator robot role for multi-robot debugging.

This script is intentionally single-robot. Multi-robot runs should start one
process per robot so each process owns its own LunaPre globals and UDP ports.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path


INVALID_POSE = [0xFFF, 0xFFF, 0xFFF]


def _set_env(robot_id: int) -> None:
    os.environ.setdefault("ROBOCUPX_BITS", "SIM")
    os.environ.setdefault("XEL_UNITY_IP", "127.0.0.1")
    os.environ["XEL_ROBOT_ID"] = str(int(robot_id))
    attack_sign = "-1" if int(robot_id) >= 3 else "1"
    os.environ.setdefault("OFFENCE_ATTACK_SIGN", attack_sign)
    os.environ.setdefault("DEFENCE_ATTACK_SIGN", attack_sign)


def _pose(UnitedPosition) -> list[float]:
    pose = UnitedPosition.FieldRobotPos()
    if not isinstance(pose, list) or len(pose) < 3 or pose == INVALID_POSE:
        return list(INVALID_POSE)
    return [float(pose[0]), float(pose[1]), float(pose[2])]


def _wait_for_sim(Bits, UnitedPosition, timeout_s: float = 6.0) -> bool:
    start_rx = int(getattr(Bits, "_rx_count", 0))
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if _pose(UnitedPosition) != INVALID_POSE and int(getattr(Bits, "_rx_count", 0)) > start_rx:
            return True
        time.sleep(0.2)
    return False


def _write_jsonl(path: Path | None, row: dict) -> None:
    if path is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as fh:
        fh.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser(description="Run one SIM robot role.")
    parser.add_argument("--robot-id", type=int, required=True)
    parser.add_argument("--role", choices=["offence", "defence", "idle", "alpha", "lambda"], default="offence")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--trace", default="")
    parser.add_argument("--tick", type=float, default=0.03)
    args = parser.parse_args()

    _set_env(args.robot_id)
    sys.path.insert(0, "src")

    from LunaPre import Bits, UnitedPosition, chassis, logger, peripheral  # noqa: E402

    trace_path = Path(args.trace) if args.trace else None
    ok = False
    strategy = None
    started = time.time()

    try:
        connected = _wait_for_sim(Bits, UnitedPosition)
        if not connected:
            logger.error("[SimRobotRole] robot_id={} simulator connection failed", args.robot_id)
            return 2

        if args.role == "offence":
            from offence import RuleOffenceStrategy  # noqa: E402

            strategy = RuleOffenceStrategy()
        elif args.role == "defence":
            from defence import RuleDefenceStrategy  # noqa: E402

            strategy = RuleDefenceStrategy()
        elif args.role == "alpha":
            from alpha import AlphaController  # noqa: E402

            strategy = AlphaController(attack_dir=-1 if int(args.robot_id) >= 3 else 1)
        elif args.role == "lambda":
            from lambda_strategy import LambdaController  # noqa: E402

            strategy = LambdaController(attack_dir=-1 if int(args.robot_id) >= 3 else 1)

        deadline = time.time() + max(0.1, float(args.duration))
        while time.time() < deadline:
            if args.role == "offence":
                strategy.step()
                state = getattr(strategy, "state", "offence")
                tick_s = float(getattr(strategy.cfg, "loop_dt", args.tick))
            elif args.role == "defence":
                strategy.step()
                state = getattr(strategy, "state", "defence")
                tick_s = float(getattr(strategy.cfg, "loop_dt", args.tick))
            elif args.role == "alpha":
                strategy.step()
                state_attr = getattr(strategy, "current_mode_name", None)
                state = state_attr() if callable(state_attr) else (state_attr or getattr(strategy, "state", "alpha"))
                tick_s = float(args.tick)
            elif args.role == "lambda":
                strategy.step()
                state = getattr(strategy, "state", "lambda")
                tick_s = float(args.tick)
            else:
                chassis.stop()
                peripheral.Dribble(False)
                state = "idle"
                tick_s = float(args.tick)

            row = {
                "t_s": round(time.time() - started, 3),
                "robot_id": int(args.robot_id),
                "role": args.role,
                "state": state,
                "pose": _pose(UnitedPosition),
                "ball": UnitedPosition.RobotBallPos(),
                "enemies": UnitedPosition.FieldChassisPos(),
                "cmd_vx": round(float(getattr(chassis, "_last_vx", 0.0)), 3),
                "cmd_vy": round(float(getattr(chassis, "_last_vy", 0.0)), 3),
                "cmd_wz": round(float(getattr(chassis, "_last_wz", 0.0)), 3),
                "rx_count": int(getattr(Bits, "_rx_count", 0)),
                "tx_count": int(getattr(Bits, "_tx_count", 0)),
            }
            _write_jsonl(trace_path, row)
            time.sleep(max(0.005, tick_s))

        ok = True
        return 0
    finally:
        try:
            if strategy is not None:
                strategy.stop()
            else:
                peripheral.Dribble(False)
                chassis.stop()
        except Exception:
            pass
        try:
            UnitedPosition.Stop()
        except Exception:
            pass
        _write_jsonl(
            trace_path,
            {
                "t_s": round(time.time() - started, 3),
                "robot_id": int(args.robot_id),
                "role": args.role,
                "event": "exit",
                "ok": ok,
            },
        )


if __name__ == "__main__":
    raise SystemExit(main())
