"""Canonical V2 observation and action contracts shared with training."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import IntEnum
import hashlib
import json
from typing import Any, Dict, List, Sequence, Tuple


CONTRACT_VERSION = "2.0.1"
CONTROL_PERIOD_MS = 30
FEATURE_DIM = 49
MAX_ENEMIES = 4


class IntentId(IntEnum):
    SEARCH_BALL = 0
    APPROACH_BALL = 1
    SECURE_BALL = 2
    CARRY_TO_POSE = 3
    SHOOT = 4
    BLOCK_GOAL = 5
    MARK_ENEMY = 6
    RETURN_HOME = 7
    AVOID_WALL = 8
    SUPPORT_MATE = 9


class FaceMode(IntEnum):
    FACE_BALL = 0
    FACE_GOAL = 1
    FACE_TARGET = 2
    FACE_KEEP = 3


class DribbleMode(IntEnum):
    OFF = 0
    WEAK = 1
    STRONG = 2


class ShootIntent(IntEnum):
    NONE = 0
    TRY_SHOOT = 1


class RoleId(IntEnum):
    DEFENDER = 0
    ATTACKER = 1


class RefereeState(IntEnum):
    STOP = 0
    PLAY = 1
    PENALIZED = 2
    HALF_TIME = 3
    GAME_OVER = 4


class PenaltyReason:
    UNKNOWN = "UNKNOWN"
    PUSHING = "PUSHING"
    ILLEGAL_DEFENCE = "ILLEGAL_DEFENCE"
    OUT_OF_BOUNDS = "OUT_OF_BOUNDS"
    MANUAL = "MANUAL"


@dataclass(frozen=True)
class EntityObservation:
    rel_x: float = 0.0
    rel_y: float = 0.0
    visible: bool = False
    age_ms: int = -1
    confidence: float = 0.0

    @classmethod
    def from_dict(cls, values: Dict[str, Any]) -> "EntityObservation":
        return cls(
            rel_x=float(values.get("rel_x", 0.0)),
            rel_y=float(values.get("rel_y", 0.0)),
            visible=bool(values.get("visible", False)),
            age_ms=int(values.get("age_ms", -1)),
            confidence=float(values.get("confidence", 0.0)),
        )


@dataclass
class PolicyObservation:
    self_x: float = 0.0
    self_y: float = 0.0
    self_yaw: float = 0.0
    self_vx: float = 0.0
    self_vy: float = 0.0
    self_wz: float = 0.0
    velocity_valid: bool = False
    ball_rel_x: float = 0.0
    ball_rel_y: float = 0.0
    ball_visible: bool = False
    ball_age_ms: int = -1
    ball_confidence: float = 0.0
    has_ball: bool = False
    enemies: List[EntityObservation] = field(default_factory=list)
    wall_front: float = 0.0
    wall_left: float = 0.0
    wall_back: float = 0.0
    wall_right: float = 0.0
    wall_valid: bool = False
    opponent_goal_x: float = 130.0
    opponent_goal_y: float = 0.0
    referee_state: int = int(RefereeState.PLAY)
    control_enabled: bool = True
    penalty_active: bool = False
    penalty_reason: str = PenaltyReason.UNKNOWN
    penalty_remaining_s: float = 0.0
    score_diff: int = 0
    match_remaining_s: float = 300.0
    last_decision_id: int = 0
    last_action_age_ms: int = 0
    stuck_flag: bool = False
    role: int = int(RoleId.ATTACKER)
    observed_at_ms: int = 0
    tick: int = 0
    match_id: str = ""

    def to_dict(self) -> Dict[str, Any]:
        values = asdict(self)
        return values

    @classmethod
    def from_dict(cls, values: Dict[str, Any]) -> "PolicyObservation":
        copy = dict(values)
        copy["enemies"] = [
            item if isinstance(item, EntityObservation) else EntityObservation.from_dict(item)
            for item in values.get("enemies", [])
        ]
        known = cls.__dataclass_fields__
        return cls(**{key: value for key, value in copy.items() if key in known})


@dataclass(frozen=True)
class DecisionPacket:
    intent_id: int = int(IntentId.SEARCH_BALL)
    target_x: float = 0.0
    target_y: float = 0.0
    target_yaw: float = 0.0
    face_mode: int = int(FaceMode.FACE_BALL)
    move_gain: float = 0.0
    dribble_mode: int = int(DribbleMode.OFF)
    shoot_intent: int = int(ShootIntent.NONE)
    decision_horizon_ms: int = CONTROL_PERIOD_MS
    confidence: float = 0.0
    control_enabled: bool = True

    @classmethod
    def stopped(cls) -> "DecisionPacket":
        return cls(control_enabled=False)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


FEATURE_NAMES: Tuple[str, ...] = (
    "self_x", "self_y", "self_yaw", "self_vx", "self_vy", "self_wz",
    "velocity_valid", "ball_rel_x", "ball_rel_y", "has_ball", "ball_visible",
    "ball_age", "ball_confidence",
    *(f"enemy_{index}_{name}" for index in range(MAX_ENEMIES) for name in ("rel_x", "rel_y", "visible", "age", "confidence")),
    "wall_front", "wall_left", "wall_back", "wall_right", "wall_valid",
    "opponent_goal_dx", "opponent_goal_dy", "referee_state", "control_enabled",
    "penalty_active", "penalty_remaining", "score_diff", "match_remaining",
    "last_decision", "last_action_age", "stuck",
)

if len(FEATURE_NAMES) != FEATURE_DIM:
    raise RuntimeError(f"FeatureSpec has {len(FEATURE_NAMES)} fields, expected {FEATURE_DIM}")


@dataclass(frozen=True)
class FeatureSpec:
    version: str = CONTRACT_VERSION
    names: Tuple[str, ...] = FEATURE_NAMES
    dimension: int = FEATURE_DIM

    def to_dict(self) -> Dict[str, Any]:
        return {"version": self.version, "names": list(self.names), "dimension": self.dimension}

    @property
    def contract_hash(self) -> str:
        encoded = json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":")).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()


@dataclass(frozen=True)
class ActionSpec:
    version: str = CONTRACT_VERSION
    continuous_names: Tuple[str, ...] = ("target_x", "target_y", "target_yaw", "move_gain")
    intent_classes: int = len(IntentId)
    face_classes: int = len(FaceMode)
    dribble_classes: int = len(DribbleMode)
    shoot_classes: int = len(ShootIntent)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @property
    def contract_hash(self) -> str:
        encoded = json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":")).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()


def combined_contract_hash(feature_spec: FeatureSpec | None = None, action_spec: ActionSpec | None = None) -> str:
    feature_spec = feature_spec or FeatureSpec()
    action_spec = action_spec or ActionSpec()
    payload = f"{feature_spec.contract_hash}:{action_spec.contract_hash}".encode("ascii")
    return hashlib.sha256(payload).hexdigest()
