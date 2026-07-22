"""Canonical conversion from policy tensors to executable decisions."""

from __future__ import annotations

from typing import Mapping, Sequence

from .contracts import CONTROL_PERIOD_MS, DecisionPacket


def decode_policy_action(
    continuous: Sequence[float],
    categories: Mapping[str, int],
    confidence: float,
    control_period_ms: int = CONTROL_PERIOD_MS,
) -> DecisionPacket:
    if len(continuous) != 4:
        raise ValueError(f"continuous action must have 4 values, got {len(continuous)}")
    values = [max(-1.0, min(1.0, float(value))) for value in continuous]
    return DecisionPacket(
        intent_id=int(categories["intent"]),
        target_x=values[0] * 130.0,
        target_y=values[1] * 100.0,
        target_yaw=values[2] * 180.0,
        move_gain=(values[3] + 1.0) * 1.5,
        face_mode=int(categories["face"]),
        dribble_mode=int(categories["dribble"]),
        shoot_intent=int(categories["shoot"]),
        decision_horizon_ms=int(control_period_ms),
        confidence=max(0.0, min(1.0, float(confidence))),
    )
