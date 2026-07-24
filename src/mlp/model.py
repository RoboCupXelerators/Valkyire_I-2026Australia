"""Role-conditioned MLP policy shared by inference and RLlib training."""

from __future__ import annotations

import math
from typing import Dict, Tuple

import torch
from torch import Tensor, nn
from torch.distributions import Categorical, Normal

from .contracts import ActionSpec, FEATURE_DIM


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


class PolicyNetwork(nn.Module):
    def __init__(self, feature_dim: int = FEATURE_DIM, role_count: int = 2, role_embedding_dim: int = 8):
        super().__init__()
        self.feature_dim = feature_dim
        self.action_spec = ActionSpec()
        self.role_embedding = nn.Embedding(role_count, role_embedding_dim)
        self.trunk = nn.Sequential(
            nn.Linear(feature_dim + role_embedding_dim, 256),
            nn.LayerNorm(256),
            nn.SiLU(),
            nn.Linear(256, 256),
            nn.LayerNorm(256),
            nn.SiLU(),
        )
        self.continuous_head = _Head(len(self.action_spec.continuous_names))
        self.intent_head = _Head(self.action_spec.intent_classes)
        self.face_head = _Head(self.action_spec.face_classes)
        self.dribble_head = _Head(self.action_spec.dribble_classes)
        self.shoot_head = _Head(self.action_spec.shoot_classes)
        self.value_head = _Head(1)
        self.log_std = nn.Parameter(torch.full((len(self.action_spec.continuous_names),), -0.5))

    def forward(self, features: Tensor, role: Tensor) -> Dict[str, Tensor]:
        if features.shape[-1] != self.feature_dim:
            raise ValueError(f"expected feature width {self.feature_dim}, got {features.shape[-1]}")
        role = role.long().reshape(-1)
        encoded = self.trunk(torch.cat((features, self.role_embedding(role)), dim=-1))
        return {
            "continuous_mean": self.continuous_head(encoded),
            "continuous_log_std": self.log_std.expand(features.shape[0], -1),
            "intent_logits": self.intent_head(encoded),
            "face_logits": self.face_head(encoded),
            "dribble_logits": self.dribble_head(encoded),
            "shoot_logits": self.shoot_head(encoded),
            "value": self.value_head(encoded).squeeze(-1),
        }

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
