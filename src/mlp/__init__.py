"""Xelerators MLP V2 inference package.

The base package intentionally avoids importing torch. Contract and feature
utilities remain available on deployment tools that only validate bundles.
"""

from .config import RuntimeConfig
from .actions import decode_policy_action
from .contracts import (
    ActionSpec,
    DecisionPacket,
    DribbleMode,
    EntityObservation,
    FaceMode,
    FeatureSpec,
    IntentId,
    PolicyObservation,
    RefereeState,
    RoleId,
    ShootIntent,
    combined_contract_hash,
)
from .features import FeatureEncoder

__all__ = [
    "ActionSpec", "DecisionPacket", "DribbleMode", "EntityObservation", "FaceMode",
    "FeatureEncoder", "FeatureSpec", "IntentId", "PolicyObservation", "RefereeState",
    "RoleId", "RuntimeConfig", "ShootIntent", "combined_contract_hash", "decode_policy_action",
]
