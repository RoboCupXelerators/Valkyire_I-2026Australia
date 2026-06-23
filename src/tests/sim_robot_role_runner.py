"""运行一个模拟器机器人角色，便于多机器人调试。

这个脚本有意只绑定一台机器人。多机器人运行时应为每台机器人启动一个
独立进程，确保每个进程独占自己的 LunaPre 全局对象和 UDP 端口。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path


INVALID_POSE = [0xFFF, 0xFFF, 0xFFF]


def _set_env(robot_id: int) -> None:
    """配置当前进程，让 LunaPre 绑定到指定模拟器机器人。"""
    os.environ.setdefault("ROBOCUPX_BITS", "SIM")
    os.environ.setdefault("XEL_UNITY_IP", "127.0.0.1")
    os.environ["XEL_ROBOT_ID"] = str(int(robot_id))
    attack_sign = "-1" if int(robot_id) >= 3 else "1"
    os.environ.setdefault("OFFENCE_ATTACK_SIGN", attack_sign)
    os.environ.setdefault("DEFENCE_ATTACK_SIGN", attack_sign)


def _pose(UnitedPosition) -> list[float]:
    """返回规范化场地位姿；无效时返回模拟器无效位姿哨兵值。"""
    pose = UnitedPosition.FieldRobotPos()
    if not isinstance(pose, list) or len(pose) < 3 or pose == INVALID_POSE:
        return list(INVALID_POSE)
    return [float(pose[0]), float(pose[1]), float(pose[2])]


def _wait_for_sim(Bits, UnitedPosition, timeout_s: float = 6.0) -> bool:
    """等待 Unity 同时发来有效位姿和新的状态帧。"""
    start_rx = int(getattr(Bits, "_rx_count", 0))
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if _pose(UnitedPosition) != INVALID_POSE and int(getattr(Bits, "_rx_count", 0)) > start_rx:
            return True
        time.sleep(0.2)
    return False


def _write_jsonl(path: Path | None, row: dict) -> None:
    """开启 trace 时追加一行紧凑 JSON。"""
    if path is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as fh:
        fh.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n")


def _write_json(path: Path | None, payload: dict) -> None:
    """配置 summary 路径时写入缩进格式的 JSON 报告。"""
    if path is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def _motion_summary(frames: list[dict]) -> dict:
    """汇总角色循环是否产生了模拟器运动和底盘命令。"""
    pose_frames = [row for row in frames if row.get("pose") and row.get("pose") != INVALID_POSE]
    if not pose_frames:
        return {
            "frames": len(frames),
            "valid_pose_frames": 0,
            "states": [],
            "motion_span": 0.0,
            "x_span": 0.0,
            "y_span": 0.0,
            "yaw_span": 0.0,
            "nonzero_cmd_frames": 0,
            "rx_delta": 0,
            "tx_delta": 0,
        }

    xs = [row["pose"][0] for row in pose_frames]
    ys = [row["pose"][1] for row in pose_frames]
    yaws = [row["pose"][2] for row in pose_frames]
    start_x, start_y = pose_frames[0]["pose"][:2]
    motion_span = max(math.hypot(row["pose"][0] - start_x, row["pose"][1] - start_y) for row in pose_frames)
    nonzero_cmd_frames = sum(
        1
        for row in pose_frames
        if any(abs(float(row.get(name) or 0.0)) > 1e-6 for name in ("cmd_vx", "cmd_vy", "cmd_wz"))
    )
    rx_values = [int(row.get("rx_count") or 0) for row in pose_frames]
    tx_values = [int(row.get("tx_count") or 0) for row in pose_frames]

    return {
        "frames": len(frames),
        "valid_pose_frames": len(pose_frames),
        "states": sorted({row.get("state") for row in pose_frames if row.get("state") is not None}),
        "first_pose": pose_frames[0]["pose"],
        "last_pose": pose_frames[-1]["pose"],
        "motion_span": round(float(motion_span), 3),
        "x_span": round(float(max(xs) - min(xs)), 3),
        "y_span": round(float(max(ys) - min(ys)), 3),
        "yaw_span": round(float(max(yaws) - min(yaws)), 3),
        "nonzero_cmd_frames": nonzero_cmd_frames,
        "rx_delta": rx_values[-1] - rx_values[0],
        "tx_delta": tx_values[-1] - tx_values[0],
    }


def main() -> int:
    """运行一个角色循环，并可在机器人没有实际运动时失败退出。"""
    parser = argparse.ArgumentParser(description="运行一个 SIM 机器人角色。")
    parser.add_argument("--robot-id", type=int, required=True)
    parser.add_argument("--role", choices=["offence", "defence", "idle", "alpha", "lambda", "mlp"], default="offence")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--trace", default="")
    parser.add_argument("--tick", type=float, default=0.03)
    parser.add_argument("--summary", default="", help="可选 JSON 汇总文件，记录运动和命令统计。")
    parser.add_argument("--require-motion", action="store_true", help="模拟器机器人没有实际运动时失败退出。")
    parser.add_argument("--min-motion", type=float, default=5.0, help="--require-motion 使用的最小平面运动跨度。")
    parser.add_argument(
        "--min-command-frames",
        type=int,
        default=3,
        help="--require-motion 使用的最小非零命令帧数量。",
    )
    parser.add_argument(
        "--mlp-model",
        default=os.environ.get(
            "MLP_MODEL_PATH",
            "logs/mlp/offline_bc/bc_20260608_161239/model_best.pt",
        ),
    )
    args = parser.parse_args()

    _set_env(args.robot_id)
    sys.path.insert(0, "src")

    from LunaPre import Bits, UnitedPosition, chassis, logger, peripheral  # noqa: E402

    trace_path = Path(args.trace) if args.trace else None
    summary_path = Path(args.summary) if args.summary else None
    ok = False
    strategy = None
    mlp_last_action = None
    frames: list[dict] = []
    started = time.time()

    try:
        connected = _wait_for_sim(Bits, UnitedPosition)
        if not connected:
            logger.error("[SimRobotRole] robot_id={} simulator connection failed", args.robot_id)
            return 2

        if args.role == "offence":
            import offence  # noqa: E402

            strategy = offence
        elif args.role == "defence":
            from defence import RuleDefenceStrategy  # noqa: E402

            strategy = RuleDefenceStrategy()
        elif args.role == "alpha":
            from alpha import AlphaController  # noqa: E402

            strategy = AlphaController(attack_dir=-1 if int(args.robot_id) >= 3 else 1)
        elif args.role == "lambda":
            from lambda_strategy import LambdaController  # noqa: E402

            strategy = LambdaController(attack_dir=-1 if int(args.robot_id) >= 3 else 1)
        elif args.role == "mlp":
            from mlp.config import MLPConfig  # noqa: E402
            from mlp.inference import MLPController  # noqa: E402

            model_path = Path(args.mlp_model)
            if not model_path.is_absolute():
                model_path = Path.cwd() / model_path
            if not model_path.exists():
                logger.error("[SimRobotRole] MLP model not found: {}", model_path)
                return 3
            strategy = MLPController(str(model_path), config=MLPConfig(), role="offence", device="cpu")

        deadline = time.time() + max(0.1, float(args.duration))
        while time.time() < deadline:
            if args.role == "offence":
                strategy.OffenceStep()
                state = getattr(strategy, "state", "offence")
                tick_s = float(getattr(strategy, "OFFENCE_DT", args.tick))
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
            elif args.role == "mlp":
                obs = strategy.inference._get_state_from_lunapre()
                mlp_last_action = strategy.inference.infer(obs, deterministic=True)
                strategy._execute_action(mlp_last_action)
                state = "mlp"
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
            if mlp_last_action is not None:
                row["mlp_action"] = {
                    "vx": round(float(mlp_last_action.get("vx", 0.0)), 3),
                    "vy": round(float(mlp_last_action.get("vy", 0.0)), 3),
                    "wz": round(float(mlp_last_action.get("wz", 0.0)), 3),
                    "kick": bool(mlp_last_action.get("kick", False)),
                    "dribble": bool(mlp_last_action.get("dribble", False)),
                }
            if args.role == "offence":
                # offence 会记录本 tick 的射门门控、线路质量和动作原因，方便真实场景复盘。
                row["offence_decision"] = dict(getattr(strategy, "last_decision", {}) or {})
            frames.append(row)
            _write_jsonl(trace_path, row)
            time.sleep(max(0.005, tick_s))

        summary = _motion_summary(frames)
        summary.update(
            {
                "robot_id": int(args.robot_id),
                "role": args.role,
                "duration_s": round(time.time() - started, 3),
                "require_motion": bool(args.require_motion),
                "min_motion": float(args.min_motion),
                "min_command_frames": int(args.min_command_frames),
            }
        )
        summary["motion_ok"] = (
            summary["valid_pose_frames"] > 0
            and summary["motion_span"] >= float(args.min_motion)
            and summary["nonzero_cmd_frames"] >= int(args.min_command_frames)
            and summary["rx_delta"] > 0
            and summary["tx_delta"] > 0
        )
        _write_json(summary_path, summary)
        if args.require_motion and not summary["motion_ok"]:
            logger.error("[SimRobotRole] motion validation failed: {}", summary)
            return 4

        ok = True
        return 0
    finally:
        try:
            if strategy is not None:
                if args.role == "offence":
                    strategy.OffenceStop()
                else:
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
