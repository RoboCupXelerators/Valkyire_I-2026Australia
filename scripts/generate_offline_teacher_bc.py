"""Generate offline teacher transitions for behavior cloning.

This script deliberately does not import LunaPre, Unity helpers, or the
hardware-facing data collector. It synthesizes DeployObservationV1 samples,
asks the pure RuleTeacher for decisions, and writes the .npz format consumed by
mlp.behavior_cloning.
"""

from __future__ import annotations

import argparse
import json
import math
import random
import sys
from collections import Counter
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
from typing import Iterable

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from mlp.config import MLPConfig  # noqa: E402
from mlp.contracts import DeployObservationV1, DribbleMode, EntityObservationV1, IntentId  # noqa: E402
from mlp.state_encoder import StateEncoder  # noqa: E402
from teacher.rule_teacher import RuleTeacher, TeacherConfig  # noqa: E402


def _clip01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def encode_teacher_action(packet, obs: DeployObservationV1, config: MLPConfig) -> np.ndarray:
    """Encode a teacher decision packet into [vx, vy, wz, kick, dribble]."""
    vx = 0.0
    vy = 0.0
    wz = 0.0
    kick = 0.0
    dribble = 0.0

    intent = IntentId(packet.intent_id)
    if intent in (IntentId.SEARCH_BALL, IntentId.CARRY_TO_POSE, IntentId.AVOID_WALL):
        dx = float(packet.target_x) - float(obs.self_x)
        dy = float(packet.target_y) - float(obs.self_y)
        dist = math.hypot(dx, dy)
        if dist > 1e-6:
            gain = _clip01(float(packet.move_gain) / 2.2)
            vx = (dx / dist) * gain
            vy = (dy / dist) * gain
    elif intent == IntentId.APPROACH_BALL:
        dist = math.hypot(float(obs.ball_rel_x), float(obs.ball_rel_y))
        if dist > 1e-6:
            gain = min(1.0, dist / 120.0)
            vx = (float(obs.ball_rel_x) / dist) * gain
            vy = (float(obs.ball_rel_y) / dist) * gain
    elif intent == IntentId.SHOOT:
        kick = 1.0

    yaw_error = (float(packet.target_yaw) - float(obs.self_yaw) + 180.0) % 360.0 - 180.0
    wz = max(-1.0, min(1.0, yaw_error / max(config.max_angular_speed, 1e-6)))

    dribble_mode = DribbleMode(packet.dribble_mode)
    if dribble_mode == DribbleMode.WEAK:
        dribble = 0.35
    elif dribble_mode == DribbleMode.STRONG:
        dribble = 0.8

    return np.array([vx, vy, wz, kick, dribble], dtype=np.float32)


def _base_obs(rng: random.Random, x: float, y: float, yaw: float) -> DeployObservationV1:
    obs = DeployObservationV1(
        self_x=float(x),
        self_y=float(y),
        self_yaw=float(yaw % 360.0),
        self_vx=rng.uniform(-30.0, 30.0),
        self_vy=rng.uniform(-20.0, 20.0),
        self_wz=rng.uniform(-35.0, 35.0),
        velocity_valid=True,
        wall_front=max(0.0, 130.0 - x),
        wall_left=max(0.0, 100.0 - y),
        wall_back=max(0.0, 130.0 + x),
        wall_right=max(0.0, 100.0 + y),
        wall_valid=True,
        ball_age_ms=0,
        max_enemies=4,
    )

    for _ in range(rng.randint(0, 3)):
        obs.enemies.append(
            EntityObservationV1(
                rel_x=rng.uniform(20.0, 140.0),
                rel_y=rng.uniform(-65.0, 65.0),
                visible=True,
                age_ms=rng.randint(0, 120),
                confidence=rng.uniform(0.65, 1.0),
            )
        )
    return obs


def search_samples(rng: random.Random, n: int) -> Iterable[DeployObservationV1]:
    for _ in range(n):
        obs = _base_obs(
            rng,
            rng.uniform(-78.0, 28.0),
            rng.uniform(-58.0, 58.0),
            rng.uniform(0.0, 360.0),
        )
        obs.ball_visible = False
        obs.ball_age_ms = rng.randint(450, 1500)
        yield obs


def approach_samples(rng: random.Random, n: int) -> Iterable[DeployObservationV1]:
    for _ in range(n):
        obs = _base_obs(
            rng,
            rng.uniform(-70.0, 55.0),
            rng.uniform(-55.0, 55.0),
            rng.uniform(0.0, 360.0),
        )
        obs.ball_visible = True
        obs.has_ball = False
        obs.ball_rel_x = rng.uniform(18.0, 145.0)
        obs.ball_rel_y = rng.uniform(-55.0, 55.0)
        yield obs


def carry_samples(rng: random.Random, n: int) -> Iterable[DeployObservationV1]:
    for _ in range(n):
        obs = _base_obs(
            rng,
            rng.uniform(-35.0, 72.0),
            rng.uniform(-62.0, 62.0),
            rng.uniform(0.0, 360.0),
        )
        obs.ball_visible = True
        obs.has_ball = True
        obs.ball_rel_x = 0.0
        obs.ball_rel_y = 0.0
        yield obs


def shoot_samples(rng: random.Random, n: int) -> Iterable[DeployObservationV1]:
    for _ in range(n):
        x = rng.uniform(58.5, 64.5)
        y = rng.uniform(-22.0, 22.0)
        goal_y = -29.25
        yaw = math.degrees(math.atan2(goal_y - y, 130.0 - x)) + rng.uniform(-4.0, 4.0)
        obs = _base_obs(rng, x, y, yaw)
        obs.enemies.clear()
        obs.ball_visible = True
        obs.has_ball = True
        obs.ball_rel_x = 0.0
        obs.ball_rel_y = 0.0
        yield obs


def avoid_wall_samples(rng: random.Random, n: int) -> Iterable[DeployObservationV1]:
    zones = [
        (rng.uniform(91.0, 108.0), rng.uniform(-68.0, 68.0)),
        (rng.uniform(-108.0, -91.0), rng.uniform(-68.0, 68.0)),
        (rng.uniform(-88.0, 88.0), rng.uniform(71.0, 88.0)),
        (rng.uniform(-88.0, 88.0), rng.uniform(-88.0, -71.0)),
        (rng.choice([-1.0, 1.0]) * rng.uniform(66.0, 88.0), rng.uniform(-38.0, 38.0)),
    ]
    for idx in range(n):
        x, y = zones[idx % len(zones)]
        obs = _base_obs(rng, x, y, rng.uniform(0.0, 360.0))
        obs.ball_visible = bool(rng.getrandbits(1))
        obs.has_ball = False
        if obs.ball_visible:
            obs.ball_rel_x = rng.uniform(20.0, 120.0)
            obs.ball_rel_y = rng.uniform(-40.0, 40.0)
        else:
            obs.ball_age_ms = rng.randint(500, 1500)
        yield obs


def build_observations(samples: int, seed: int) -> list[DeployObservationV1]:
    rng = random.Random(seed)
    counts = {
        "search": int(samples * 0.16),
        "approach": int(samples * 0.32),
        "carry": int(samples * 0.24),
        "shoot": int(samples * 0.12),
    }
    counts["avoid_wall"] = samples - sum(counts.values())

    obs = []
    obs.extend(search_samples(rng, counts["search"]))
    obs.extend(approach_samples(rng, counts["approach"]))
    obs.extend(carry_samples(rng, counts["carry"]))
    obs.extend(shoot_samples(rng, counts["shoot"]))
    obs.extend(avoid_wall_samples(rng, counts["avoid_wall"]))
    rng.shuffle(obs)
    return obs


def write_jsonl(path: Path, rows: Iterable[dict]) -> None:
    with path.open("w", encoding="utf-8") as f:
        for row in rows:
            f.write(json.dumps(row, ensure_ascii=False) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate offline teacher BC data")
    parser.add_argument("--samples", type=int, default=5000, help="number of transitions to generate")
    parser.add_argument("--seed", type=int, default=20260608, help="random seed")
    parser.add_argument("--output-dir", type=str, default="Records/offline_teacher_bc", help="output directory")
    args = parser.parse_args()

    if args.samples <= 0:
        raise ValueError("--samples must be positive")

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    npz_path = output_dir / f"teacher_bc_{timestamp}.npz"
    jsonl_path = output_dir / f"teacher_decisions_{timestamp}.jsonl"
    summary_path = output_dir / f"summary_{timestamp}.json"

    mlp_config = MLPConfig()
    teacher = RuleTeacher(TeacherConfig())
    encoder = StateEncoder(mlp_config)

    states = []
    actions = []
    rewards = []
    dones = []
    trace_rows = []
    state_counts: Counter[str] = Counter()
    intent_counts: Counter[str] = Counter()

    for idx, obs in enumerate(build_observations(args.samples, args.seed)):
        decision = teacher.decide(obs)
        action = encode_teacher_action(decision.packet, obs, mlp_config)

        states.append(encoder.encode(obs))
        actions.append(action)
        rewards.append(1.0 if decision.state == "shoot" else 0.0)
        dones.append(False)

        state_counts[decision.state] += 1
        intent_counts[IntentId(decision.packet.intent_id).name] += 1
        trace_rows.append(
            {
                "index": idx,
                "observation": obs.to_dict(),
                "decision": decision.to_dict(),
                "action": action.tolist(),
            }
        )

    np.savez_compressed(
        npz_path,
        states=np.asarray(states, dtype=np.float32),
        actions=np.asarray(actions, dtype=np.float32),
        rewards=np.asarray(rewards, dtype=np.float32),
        dones=np.asarray(dones, dtype=np.bool_),
    )
    write_jsonl(jsonl_path, trace_rows)

    summary = {
        "samples": args.samples,
        "seed": args.seed,
        "npz": str(npz_path),
        "jsonl": str(jsonl_path),
        "state_shape": list(np.asarray(states).shape),
        "action_shape": list(np.asarray(actions).shape),
        "teacher_states": dict(sorted(state_counts.items())),
        "intents": dict(sorted(intent_counts.items())),
        "mlp_config": mlp_config.to_dict(),
        "teacher_config": asdict(TeacherConfig()),
    }
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"[offline-teacher] wrote {npz_path}")
    print(f"[offline-teacher] wrote {jsonl_path}")
    print(f"[offline-teacher] wrote {summary_path}")
    print(f"[offline-teacher] states={summary['state_shape']} actions={summary['action_shape']}")
    print(f"[offline-teacher] teacher_states={summary['teacher_states']}")


if __name__ == "__main__":
    main()
