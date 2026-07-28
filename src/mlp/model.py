"""Role-conditioned MLP policy shared by inference and RLlib training."""

from __future__ import annotations

import math
from collections import OrderedDict
from typing import Dict, Mapping, Tuple

import torch
from torch import Tensor, nn
from torch.distributions import Categorical, Normal

from .contracts import ActionSpec, CONTROL_INTERFACE_WIDTH, FEATURE_DIM


def _atanh(value: Tensor) -> Tensor:
    value = value.clamp(-0.999999, 0.999999)
    return 0.5 * (torch.log1p(value) - torch.log1p(-value))


class SquashedNormal:
    """Tanh-squashed normal with a matching log-probability."""

    def __init__(self, mean: Tensor, std: Tensor):
        self.base = Normal(mean, std)

    def rsample(self) -> Tensor:
        return torch.tanh(self.base.rsample())

    @property
    def mode(self) -> Tensor:
        return torch.tanh(self.base.mean)

    def log_prob(self, action: Tensor) -> Tensor:
        raw = _atanh(action)
        correction = torch.log(1.0 - action.square() + 1e-6)
        return (self.base.log_prob(raw) - correction).sum(dim=-1)

    def entropy(self) -> Tensor:
        # This stable approximation is sufficient for confidence and PPO logging.
        return self.base.entropy().sum(dim=-1)


class _Head(nn.Module):
    def __init__(self, output_dim: int):
        super().__init__()
        self.layers = nn.Sequential(nn.Linear(256, 128), nn.SiLU(), nn.Linear(128, output_dim))

    def forward(self, features: Tensor) -> Tensor:
        return self.layers(features)


class VectorObservationEncoder(nn.Module):
    """Maps normalized V2 features and role to the fixed 256-wide control interface."""

    def __init__(self, feature_dim: int, role_count: int, role_embedding_dim: int):
        super().__init__()
        self.feature_dim = feature_dim
        self.role_embedding = nn.Embedding(role_count, role_embedding_dim)
        self.projection = nn.Linear(feature_dim + role_embedding_dim, CONTROL_INTERFACE_WIDTH)
        self.normalization = nn.LayerNorm(CONTROL_INTERFACE_WIDTH)
        self.activation = nn.SiLU()

    def forward(self, features: Tensor, role: Tensor) -> Tensor:
        if features.shape[-1] != self.feature_dim:
            raise ValueError(f"expected feature width {self.feature_dim}, got {features.shape[-1]}")
        if features.dtype != torch.float32:
            raise ValueError(f"expected float32 features, got {features.dtype}")
        role = role.long().reshape(-1)
        return self.activation(self.normalization(self.projection(torch.cat((features, self.role_embedding(role)), dim=-1))))


class ControlCore(nn.Module):
    """Shared post-interface controller that V3 must warm-start from V2."""

    def __init__(self):
        super().__init__()
        self.projection = nn.Linear(CONTROL_INTERFACE_WIDTH, CONTROL_INTERFACE_WIDTH)
        self.normalization = nn.LayerNorm(CONTROL_INTERFACE_WIDTH)
        self.activation = nn.SiLU()

    def forward(self, control_interface: Tensor) -> Tensor:
        return self.activation(self.normalization(self.projection(control_interface)))


class ActionHeads(nn.Module):
    """All actor heads with stable state-dict prefixes under ``action_heads``."""

    def __init__(self, action_spec: ActionSpec):
        super().__init__()
        self.continuous = _Head(len(action_spec.continuous_names))
        self.intent = _Head(action_spec.intent_classes)
        self.face = _Head(action_spec.face_classes)
        self.dribble = _Head(action_spec.dribble_classes)
        self.shoot = _Head(action_spec.shoot_classes)

    def forward(self, features: Tensor) -> Dict[str, Tensor]:
        return {
            "continuous_mean": self.continuous(features),
            "intent_logits": self.intent(features),
            "face_logits": self.face(features),
            "dribble_logits": self.dribble(features),
            "shoot_logits": self.shoot(features),
        }


class ValueHead(_Head):
    """Dedicated critic head kept separate from the actor head collection."""

    def __init__(self):
        super().__init__(1)


class PolicyNetwork(nn.Module):
    def __init__(self, feature_dim: int = FEATURE_DIM, role_count: int = 2, role_embedding_dim: int = 8):
        super().__init__()
        self.feature_dim = feature_dim
        self.action_spec = ActionSpec()
        self.vector_observation_encoder = VectorObservationEncoder(feature_dim, role_count, role_embedding_dim)
        self.control_core = ControlCore()
        self.action_heads = ActionHeads(self.action_spec)
        self.value_head = ValueHead()
        self.log_std = nn.Parameter(torch.full((len(self.action_spec.continuous_names),), -0.5))

    def forward(self, features: Tensor, role: Tensor) -> Dict[str, Tensor]:
        control_interface = self.encode_control_interface(features, role)
        encoded = self.control_core(control_interface)
        outputs = self.action_heads(encoded)
        outputs.update({
            "continuous_log_std": self.log_std.expand(features.shape[0], -1),
            "value": self.value_head(encoded).squeeze(-1),
        })
        return outputs

    def encode_control_interface(self, features: Tensor, role: Tensor) -> Tensor:
        """Return the V2/V3 float32 256-wide warm-start representation."""

        return self.vector_observation_encoder(features, role)

    def sample(self, features: Tensor, role: Tensor, deterministic: bool = False) -> Tuple[Dict[str, Tensor], Tensor, Tensor, Tensor]:
        outputs = self.forward(features, role)
        continuous_dist = SquashedNormal(outputs["continuous_mean"], outputs["continuous_log_std"].exp())
        continuous = continuous_dist.mode if deterministic else continuous_dist.rsample()
        action: Dict[str, Tensor] = {"continuous": continuous}
        log_prob = continuous_dist.log_prob(continuous)
        entropy = continuous_dist.entropy()
        for name in ("intent", "face", "dribble", "shoot"):
            dist = Categorical(logits=outputs[f"{name}_logits"])
            choice = outputs[f"{name}_logits"].argmax(dim=-1) if deterministic else dist.sample()
            action[name] = choice
            log_prob = log_prob + dist.log_prob(choice)
            entropy = entropy + dist.entropy()
        return action, log_prob, entropy, outputs["value"]

    def evaluate_actions(self, features: Tensor, role: Tensor, action: Dict[str, Tensor]) -> Tuple[Tensor, Tensor, Tensor]:
        outputs = self.forward(features, role)
        continuous_dist = SquashedNormal(outputs["continuous_mean"], outputs["continuous_log_std"].exp())
        log_prob = continuous_dist.log_prob(action["continuous"])
        entropy = continuous_dist.entropy()
        for name in ("intent", "face", "dribble", "shoot"):
            dist = Categorical(logits=outputs[f"{name}_logits"])
            log_prob = log_prob + dist.log_prob(action[name].long())
            entropy = entropy + dist.entropy()
        return log_prob, entropy, outputs["value"]


_LEGACY_V2_KEY_PREFIXES = {
    "role_embedding.": "vector_observation_encoder.role_embedding.",
    "trunk.0.": "vector_observation_encoder.projection.",
    "trunk.1.": "vector_observation_encoder.normalization.",
    "trunk.3.": "control_core.projection.",
    "trunk.4.": "control_core.normalization.",
    "continuous_head.": "action_heads.continuous.",
    "intent_head.": "action_heads.intent.",
    "face_head.": "action_heads.face.",
    "dribble_head.": "action_heads.dribble.",
    "shoot_head.": "action_heads.shoot.",
    "value_head.": "value_head.",
    "log_std": "log_std",
}


def migrate_legacy_v2_state_dict(state_dict: Mapping[str, Tensor]) -> "OrderedDict[str, Tensor]":
    """Convert the pre-ControlInterface V2 layout with an explicit, strict mapping."""

    migrated: "OrderedDict[str, Tensor]" = OrderedDict()
    unmatched = []
    for key, value in state_dict.items():
        for source_prefix, destination_prefix in _LEGACY_V2_KEY_PREFIXES.items():
            if key.startswith(source_prefix):
                migrated[destination_prefix + key[len(source_prefix):]] = value
                break
        else:
            unmatched.append(key)
    if unmatched:
        raise ValueError(f"unrecognized legacy V2 checkpoint keys: {sorted(unmatched)}")
    return migrated


def load_compatible_policy_state_dict(model: PolicyNetwork, state_dict: Mapping[str, Tensor]) -> bool:
    """Load modular or legacy V2 weights strictly; return whether migration was used."""

    is_legacy = any(key.startswith("trunk.") for key in state_dict)
    model.load_state_dict(migrate_legacy_v2_state_dict(state_dict) if is_legacy else state_dict, strict=True)
    return is_legacy


class OnnxPolicyWrapper(nn.Module):
    def __init__(self, policy: PolicyNetwork):
        super().__init__()
        self.policy = policy

    def forward(self, features: Tensor, role: Tensor) -> Tuple[Tensor, ...]:
        output = self.policy(features, role)
        return (
            torch.tanh(output["continuous_mean"]), output["intent_logits"], output["face_logits"],
            output["dribble_logits"], output["shoot_logits"], output["value"], output["continuous_log_std"],
        )
