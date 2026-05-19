"""Pos2Pos 真机低速调参与采样脚本。

设计目标：
- 机器人可从随机初始位置启动
- 先读取当前位置，再围绕当前位置生成短距离安全轨迹
- 低速执行 Pos2Pos，顺便记录 PID / 位置 / 控制输出

使用方法：
    python -m src.tests.pos2pos_real_tuning

可选环境变量：
    POS2POS_STEP_CM=20            # 单步位移，默认 20cm
    POS2POS_SPEED_GAIN=1.2        # speed_gain，默认 1.2
    POS2POS_TIMEOUT_S=8           # 单点超时，默认 8s
    POS2POS_SETTLE_S=0.4          # 每个点到达后等待时间，默认 0.4s
    POS2POS_ENABLE_DIAGONAL=0     # 是否增加对角点，默认 0
"""

from __future__ import annotations

import math
import os
import sys
import time
from typing import Iterable

sys.path.insert(0, "src")

from LunaPre import UnitedPosition, chassis, logger, Settings  # noqa: E402


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except Exception:
        return float(default)


def _env_bool(name: str, default: bool = False) -> bool:
    value = str(os.environ.get(name, "1" if default else "0")).strip().lower()
    return value in {"1", "true", "yes", "on"}


def wait_for_valid_pose(timeout_s: float, stable_reads: int = 3) -> list[float] | None:
    """等待激光/定位稳定，连续拿到若干帧有效位姿后返回。"""
    start = time.time()
    good_reads = 0
    last_pose = None
    while time.time() - start < timeout_s:
        pose = UnitedPosition.FieldRobotPos()
        if pose != [0xFFF, 0xFFF, 0xFFF]:
            good_reads += 1
            last_pose = pose
            logger.info(f"[Pos2PosTune] Valid pose {good_reads}/{stable_reads}: {pose}")
            if good_reads >= stable_reads:
                return pose
        else:
            good_reads = 0
        time.sleep(0.2)
    return last_pose


def build_relative_targets(origin: list[float], step_cm: float, include_diagonal: bool) -> list[list[float]]:
    ox, oy, oz = origin
    base = [
        [ox + step_cm, oy, oz],
        [ox, oy + step_cm, oz],
        [ox - step_cm, oy, oz],
        [ox, oy - step_cm, oz],
        [ox, oy, oz],
    ]
    if include_diagonal:
        diag = step_cm / math.sqrt(2)
        base = [
            [ox + diag, oy + diag, oz],
            [ox - diag, oy + diag, oz],
            [ox - diag, oy - diag, oz],
            [ox + diag, oy - diag, oz],
        ] + base
    return [[round(x, 2), round(y, 2), round(z, 2)] for x, y, z in base]


def wait_until_arrived(target: list[float], speed_gain: float, timeout_s: float) -> tuple[bool, float, list[float]]:
    start = time.time()
    last_pos = [0xFFF, 0xFFF, 0xFFF]
    while time.time() - start < timeout_s:
        last_pos = UnitedPosition.FieldRobotPos()
        if UnitedPosition.Pos2Pos(target, speed_gain=speed_gain):
            return True, time.time() - start, last_pos
        time.sleep(0.02)
    chassis.stop()
    return False, time.time() - start, last_pos


def run_sequence(targets: Iterable[list[float]], speed_gain: float, timeout_s: float, settle_s: float) -> list[dict]:
    results: list[dict] = []
    for idx, target in enumerate(targets, start=1):
        logger.info(f"[Pos2PosTune] Target {idx}: {target}")
        ok, cost_s, last_pos = wait_until_arrived(target, speed_gain=speed_gain, timeout_s=timeout_s)
        result = {
            "index": idx,
            "target": target,
            "success": ok,
            "elapsed_s": round(cost_s, 3),
            "last_pos": [round(float(v), 2) for v in last_pos],
        }
        results.append(result)
        logger.info(f"[Pos2PosTune] Result {idx}: {result}")
        if not ok:
            logger.warning("[Pos2PosTune] Target timeout, stop sequence for safety.")
            break
        time.sleep(settle_s)
    return results


def main() -> int:
    step_cm = max(8.0, min(30.0, _env_float("POS2POS_STEP_CM", 20.0)))
    speed_gain = max(0.6, min(2.0, _env_float("POS2POS_SPEED_GAIN", 1.2)))
    timeout_s = max(3.0, min(20.0, _env_float("POS2POS_TIMEOUT_S", 8.0)))
    settle_s = max(0.1, min(2.0, _env_float("POS2POS_SETTLE_S", 0.4)))
    include_diagonal = _env_bool("POS2POS_ENABLE_DIAGONAL", False)
    warmup_s = max(5.0, min(20.0, _env_float("POS2POS_WARMUP_S", 12.0)))

    logger.info("[Pos2PosTune] Starting real tuning sequence")
    logger.info(
        "[Pos2PosTune] params: step=%.1fcm speed_gain=%.2f timeout=%.1fs settle=%.1fs diagonal=%s",
        step_cm,
        speed_gain,
        timeout_s,
        settle_s,
        include_diagonal,
    )
    logger.info(f"[Pos2PosTune] lidar warmup timeout={warmup_s:.1f}s")

    try:
        UnitedPosition.RefreshPos2PosPID()
    except Exception as e:
        logger.warning(f"[Pos2PosTune] RefreshPos2PosPID failed: {e}")

    origin = wait_for_valid_pose(timeout_s=warmup_s, stable_reads=3)
    if origin is None or origin == [0xFFF, 0xFFF, 0xFFF]:
        logger.error("[Pos2PosTune] No valid position data, abort.")
        return 2

    logger.info(f"[Pos2PosTune] Origin pose: {origin}")
    targets = build_relative_targets(origin, step_cm, include_diagonal)
    logger.info(f"[Pos2PosTune] Planned targets: {targets}")

    results = run_sequence(targets, speed_gain=speed_gain, timeout_s=timeout_s, settle_s=settle_s)
    success_count = sum(1 for item in results if item["success"])
    logger.info(f"[Pos2PosTune] Sequence done: success={success_count}/{len(results)}")

    for item in results:
        logger.info(f"[Pos2PosTune] Summary: {item}")

    return 0 if results and all(item["success"] for item in results) else 1


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