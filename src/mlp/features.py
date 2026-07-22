"""Deterministic 49-dimensional feature encoding."""

from __future__ import annotations

from typing import List, Sequence

from .contracts import FEATURE_DIM, MAX_ENEMIES, EntityObservation, FeatureSpec, PolicyObservation


class FeatureEncoder:
    FIELD_HALF_LENGTH_CM = 130.0
    FIELD_HALF_WIDTH_CM = 100.0
    MAX_SPEED_CM_S = 300.0
    MAX_ANGULAR_SPEED_DEG_S = 360.0
    MAX_DISTANCE_CM = 300.0
    MAX_AGE_MS = 1000.0
    MATCH_DURATION_S = 300.0
    PENALTY_DURATION_S = 60.0

    def __init__(self, spec: FeatureSpec | None = None):
        self.spec = spec or FeatureSpec()

    @staticmethod
    def _clip(value: float, low: float = -1.0, high: float = 1.0) -> float:
        return max(low, min(high, float(value)))

    def encode(self, obs: PolicyObservation) -> List[float]:
        features: List[float] = [
            self._clip(obs.self_x / self.FIELD_HALF_LENGTH_CM),
            self._clip(obs.self_y / self.FIELD_HALF_WIDTH_CM),
            self._clip(((obs.self_yaw + 180.0) % 360.0 - 180.0) / 180.0),
            self._clip(obs.self_vx / self.MAX_SPEED_CM_S) if obs.velocity_valid else 0.0,
            self._clip(obs.self_vy / self.MAX_SPEED_CM_S) if obs.velocity_valid else 0.0,
            self._clip(obs.self_wz / self.MAX_ANGULAR_SPEED_DEG_S) if obs.velocity_valid else 0.0,
            float(obs.velocity_valid),
            self._clip(obs.ball_rel_x / self.MAX_DISTANCE_CM) if obs.ball_visible and not obs.has_ball else 0.0,
            self._clip(obs.ball_rel_y / self.MAX_DISTANCE_CM) if obs.ball_visible and not obs.has_ball else 0.0,
            float(obs.has_ball),
            float(obs.ball_visible),
            self._age(obs.ball_age_ms),
            self._clip(obs.ball_confidence, 0.0, 1.0),
        ]

        for index in range(MAX_ENEMIES):
            enemy = obs.enemies[index] if index < len(obs.enemies) else EntityObservation()
            features.extend(self._encode_enemy(enemy))

        walls = (obs.wall_front, obs.wall_left, obs.wall_back, obs.wall_right)
        features.extend(
            [self._clip(distance / self.MAX_DISTANCE_CM, 0.0, 1.0) for distance in walls]
            if obs.wall_valid else [0.0, 0.0, 0.0, 0.0]
        )
        features.extend([
            float(obs.wall_valid),
            self._clip((obs.opponent_goal_x - obs.self_x) / (2.0 * self.FIELD_HALF_LENGTH_CM)),
            self._clip((obs.opponent_goal_y - obs.self_y) / (2.0 * self.FIELD_HALF_WIDTH_CM)),
            self._clip(float(obs.referee_state) / 4.0, 0.0, 1.0),
            float(obs.control_enabled),
            float(obs.penalty_active),
            self._clip(obs.penalty_remaining_s / self.PENALTY_DURATION_S, 0.0, 1.0),
            self._clip(obs.score_diff / 10.0),
            self._clip(obs.match_remaining_s / self.MATCH_DURATION_S, 0.0, 1.0),
            self._clip(obs.last_decision_id / 9.0, 0.0, 1.0),
            self._clip(obs.last_action_age_ms / self.MAX_AGE_MS, 0.0, 1.0),
            float(obs.stuck_flag),
        ])
        if len(features) != FEATURE_DIM:
            raise RuntimeError(f"encoded {len(features)} features, expected {FEATURE_DIM}")
        return features

    def _age(self, age_ms: int) -> float:
        if age_ms < 0:
            return 1.0
        return self._clip(age_ms / self.MAX_AGE_MS, 0.0, 1.0)

    def _encode_enemy(self, enemy: EntityObservation) -> Sequence[float]:
        if not enemy.visible:
            return (0.0, 0.0, 0.0, self._age(enemy.age_ms), self._clip(enemy.confidence, 0.0, 1.0))
        return (
            self._clip(enemy.rel_x / self.MAX_DISTANCE_CM),
            self._clip(enemy.rel_y / self.MAX_DISTANCE_CM),
            1.0,
            self._age(enemy.age_ms),
            self._clip(enemy.confidence, 0.0, 1.0),
        )
