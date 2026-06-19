"""MLP神经网络模型

定义Actor-Critic网络结构，用于PPO强化学习训练。
"""

import math
import numpy as np
from typing import List, Tuple, Optional, Dict

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Normal

from .config import MLPConfig


class MLPActor(nn.Module):
    """MLP Actor网络

    输出连续动作分布的参数（均值和标准差）。
    使用高斯策略，支持连续动作空间。
    """

    def __init__(self, config: MLPConfig):
        super().__init__()
        self.config = config
        self.output_mode = getattr(config, "output_mode", "legacy_action")

        # 构建隐藏层
        layers = []
        input_dim = config.state_dim

        for hidden_dim in config.hidden_dims:
            layers.append(nn.Linear(input_dim, hidden_dim))
            layers.append(self._get_activation(config.activation))
            input_dim = hidden_dim

        self.shared_layers = nn.Sequential(*layers)

        if self.output_mode == "decision_packet":
            self.continuous_head = nn.Linear(input_dim, config.decision_continuous_dim)
            self.intent_head = nn.Linear(input_dim, config.decision_intent_classes)
            self.face_head = nn.Linear(input_dim, config.decision_face_classes)
            self.dribble_head = nn.Linear(input_dim, config.decision_dribble_classes)
            self.shoot_head = nn.Linear(input_dim, config.decision_shoot_classes)
            self.log_std = nn.Parameter(torch.zeros(config.decision_continuous_dim))
        else:
            # 动作输出层
            # 连续动作: vx, vy, wz (使用tanh输出到[-1, 1])
            # 离散动作: kick, dribble (使用sigmoid输出概率)

            # 连续动作头 (3维: vx, vy, wz)
            self.continuous_head = nn.Linear(input_dim, 3)

            # 离散动作头 (2维: kick, dribble)
            self.discrete_head = nn.Linear(input_dim, 2)

            # 可学习的标准差参数（用于连续动作）
            self.log_std = nn.Parameter(torch.zeros(3))

    def _get_activation(self, name: str) -> nn.Module:
        """获取激活函数"""
        if name == "relu":
            return nn.ReLU()
        elif name == "tanh":
            return nn.Tanh()
        elif name == "elu":
            return nn.ELU()
        elif name == "leaky_relu":
            return nn.LeakyReLU(0.2)
        else:
            return nn.ReLU()

    def forward(self, x: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """前向传播

        Args:
            x: 状态张量 (batch_size, state_dim)

        Returns:
            continuous_mean: 连续动作均值 (batch_size, 3)
            continuous_std: 连续动作标准差 (batch_size, 3)
            discrete_logits: 离散动作logits (batch_size, 2)
        """
        features = self.shared_layers(x)

        if self.output_mode == "decision_packet":
            continuous_mean = torch.tanh(self.continuous_head(features))
            continuous_std = torch.exp(self.log_std.clamp(-20, 2)).expand_as(continuous_mean)
            discrete_logits = torch.cat(
                [
                    self.intent_head(features),
                    self.face_head(features),
                    self.dribble_head(features),
                    self.shoot_head(features),
                ],
                dim=-1,
            )
            return continuous_mean, continuous_std, discrete_logits

        # 连续动作均值 (使用tanh限制到[-1, 1])
        continuous_mean = torch.tanh(self.continuous_head(features))

        # 连续动作标准差 (使用softplus保证正数)
        continuous_std = torch.exp(self.log_std.clamp(-20, 2)).expand_as(continuous_mean)

        # 离散动作概率 (使用sigmoid)
        discrete_logits = torch.sigmoid(self.discrete_head(features))

        return continuous_mean, continuous_std, discrete_logits

    def get_action(self, x: torch.Tensor, deterministic: bool = False) -> Tuple[torch.Tensor, torch.Tensor]:
        """采样动作

        Args:
            x: 状态张量
            deterministic: 是否使用确定性策略

        Returns:
            action: 采样得到的动作
            log_prob: 动作的对数概率
        """
        continuous_mean, continuous_std, discrete_logits = self.forward(x)

        if self.output_mode == "decision_packet":
            if deterministic:
                continuous_action = continuous_mean
            else:
                dist = Normal(continuous_mean, continuous_std)
                continuous_action = torch.clamp(dist.rsample(), -1, 1)

            slices = self._decision_logit_slices(discrete_logits)
            class_actions = []
            class_log_probs = []
            for logits in slices:
                probs = torch.softmax(logits, dim=-1)
                if deterministic:
                    action_idx = torch.argmax(probs, dim=-1, keepdim=True)
                else:
                    action_idx = torch.distributions.Categorical(probs=probs).sample().unsqueeze(-1)
                class_actions.append(action_idx.float())
                class_log_probs.append(torch.log(probs.gather(-1, action_idx) + 1e-8))

            action = torch.cat([continuous_action] + class_actions, dim=-1)
            continuous_log_prob = Normal(continuous_mean, continuous_std).log_prob(continuous_action).sum(dim=-1, keepdim=True)
            log_prob = continuous_log_prob + torch.cat(class_log_probs, dim=-1).sum(dim=-1, keepdim=True)
            return action, log_prob

        if deterministic:
            # 确定性动作：使用均值
            continuous_action = continuous_mean
            discrete_action = (discrete_logits > 0.5).float()
        else:
            # 随机采样
            dist = Normal(continuous_mean, continuous_std)
            continuous_action = dist.rsample()  # 使用重参数化技巧
            continuous_action = torch.clamp(continuous_action, -1, 1)

            # 离散动作使用伯努利分布采样
            discrete_action = (torch.rand_like(discrete_logits) < discrete_logits).float()

        # 组合动作
        action = torch.cat([continuous_action, discrete_action], dim=-1)

        # 计算对数概率
        log_prob = self._compute_log_prob(continuous_mean, continuous_std, continuous_action,
                                          discrete_logits, discrete_action)

        return action, log_prob

    def _compute_log_prob(self, mean, std, action, discrete_logits, discrete_action):
        """计算动作的对数概率"""
        # 连续部分
        dist = Normal(mean, std)
        continuous_log_prob = dist.log_prob(action)

        # 离散部分（伯努利分布）
        # P(x=1) = p, P(x=0) = 1-p
        discrete_log_prob = torch.where(
            discrete_action == 1,
            torch.log(discrete_logits + 1e-8),
            torch.log(1 - discrete_logits + 1e-8)
        )

        # 总对数概率
        log_prob = torch.cat([continuous_log_prob, discrete_log_prob], dim=-1)
        return log_prob.sum(dim=-1, keepdim=True)

    def evaluate_actions(self, x: torch.Tensor, actions: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        """评估给定动作的概率

        Args:
            x: 状态张量
            actions: 动作张量

        Returns:
            log_prob: 对数概率
            entropy: 熵
        """
        continuous_mean, continuous_std, discrete_logits = self.forward(x)

        if self.output_mode == "decision_packet":
            continuous_dim = self.config.decision_continuous_dim
            continuous_action = actions[:, :continuous_dim]
            class_actions = actions[:, continuous_dim:continuous_dim + 4].long()

            dist = Normal(continuous_mean, continuous_std)
            continuous_log_prob = dist.log_prob(continuous_action).sum(dim=-1, keepdim=True)
            continuous_entropy = dist.entropy().sum(dim=-1, keepdim=True)

            discrete_log_prob = torch.zeros_like(continuous_log_prob)
            discrete_entropy = torch.zeros_like(continuous_entropy)
            for i, logits in enumerate(self._decision_logit_slices(discrete_logits)):
                probs = torch.softmax(logits, dim=-1)
                idx = class_actions[:, i:i + 1].clamp(0, logits.shape[-1] - 1)
                discrete_log_prob = discrete_log_prob + torch.log(probs.gather(-1, idx) + 1e-8)
                discrete_entropy = discrete_entropy - (probs * torch.log(probs + 1e-8)).sum(dim=-1, keepdim=True)

            return continuous_log_prob + discrete_log_prob, continuous_entropy + discrete_entropy

        # 分离连续和离散动作
        continuous_action = actions[:, :3]
        discrete_action = actions[:, 3:5]

        # 连续动作的log_prob和entropy
        dist = Normal(continuous_mean, continuous_std)
        continuous_log_prob = dist.log_prob(continuous_action)
        continuous_entropy = dist.entropy()

        # 离散动作的log_prob和entropy
        discrete_log_prob = torch.where(
            discrete_action == 1,
            torch.log(discrete_logits + 1e-8),
            torch.log(1 - discrete_logits + 1e-8)
        )
        # 伯努利分布的熵: H = -p*log(p) - (1-p)*log(1-p)
        discrete_entropy = -(
            discrete_logits * torch.log(discrete_logits + 1e-8) +
            (1 - discrete_logits) * torch.log(1 - discrete_logits + 1e-8)
        )

        # 合并
        log_prob = torch.cat([continuous_log_prob, discrete_log_prob], dim=-1).sum(dim=-1)
        entropy = torch.cat([continuous_entropy, discrete_entropy], dim=-1).sum(dim=-1)

        return log_prob.unsqueeze(-1), entropy.unsqueeze(-1)

    def _decision_logit_slices(self, logits: torch.Tensor):
        intent_end = self.config.decision_intent_classes
        face_end = intent_end + self.config.decision_face_classes
        dribble_end = face_end + self.config.decision_dribble_classes
        shoot_end = dribble_end + self.config.decision_shoot_classes
        return (
            logits[:, :intent_end],
            logits[:, intent_end:face_end],
            logits[:, face_end:dribble_end],
            logits[:, dribble_end:shoot_end],
        )


class MLPCritic(nn.Module):
    """MLP Critic网络

    输出状态价值估计。
    """

    def __init__(self, config: MLPConfig):
        super().__init__()
        self.config = config

        # 构建隐藏层
        layers = []
        input_dim = config.state_dim

        for hidden_dim in config.hidden_dims:
            layers.append(nn.Linear(input_dim, hidden_dim))
            layers.append(self._get_activation(config.activation))
            input_dim = hidden_dim

        layers.append(nn.Linear(input_dim, 1))

        self.network = nn.Sequential(*layers)

    def _get_activation(self, name: str) -> nn.Module:
        """获取激活函数"""
        if name == "relu":
            return nn.ReLU()
        elif name == "tanh":
            return nn.Tanh()
        elif name == "elu":
            return nn.ELU()
        elif name == "leaky_relu":
            return nn.LeakyReLU(0.2)
        else:
            return nn.ReLU()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """前向传播

        Args:
            x: 状态张量 (batch_size, state_dim)

        Returns:
            value: 状态价值估计 (batch_size, 1)
        """
        return self.network(x)


class ActorCritic(nn.Module):
    """Actor-Critic组合网络

    将Actor和Critic组合在一起，便于训练和推理。
    """

    def __init__(self, config: MLPConfig):
        super().__init__()
        self.config = config

        self.actor = MLPActor(config)
        self.critic = MLPCritic(config)

    def forward(self, x: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """前向传播

        Args:
            x: 状态张量

        Returns:
            continuous_mean, continuous_std, discrete_logits, value
        """
        continuous_mean, continuous_std, discrete_logits = self.actor(x)
        value = self.critic(x)
        return continuous_mean, continuous_std, discrete_logits, value

    def get_action(self, x: torch.Tensor, deterministic: bool = False) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """采样动作并返回价值估计

        Args:
            x: 状态张量
            deterministic: 是否使用确定性策略

        Returns:
            action: 动作
            log_prob: 对数概率
            value: 状态价值
        """
        action, log_prob = self.actor.get_action(x, deterministic)
        value = self.critic(x)
        return action, log_prob, value

    def evaluate_actions(self, x: torch.Tensor, actions: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """评估动作

        Args:
            x: 状态张量
            actions: 动作张量

        Returns:
            log_prob: 对数概率
            entropy: 熵
            value: 状态价值
        """
        log_prob, entropy = self.actor.evaluate_actions(x, actions)
        value = self.critic(x)
        return log_prob, entropy, value

    def get_value(self, x: torch.Tensor) -> torch.Tensor:
        """获取状态价值"""
        return self.critic(x)


def create_model(config: MLPConfig) -> ActorCritic:
    """创建Actor-Critic模型

    Args:
        config: MLP配置

    Returns:
        ActorCritic模型实例
    """
    return ActorCritic(config)


def load_model(path: str, config: MLPConfig) -> ActorCritic:
    """加载模型权重

    Args:
        path: 模型权重路径
        config: MLP配置

    Returns:
        加载了权重的ActorCritic模型
    """
    model = ActorCritic(config)
    state_dict = torch.load(path, map_location='cpu')
    model.load_state_dict(state_dict)
    model.eval()
    return model


def save_model(model: ActorCritic, path: str):
    """保存模型权重

    Args:
        model: 要保存的模型
        path: 保存路径
    """
    torch.save(model.state_dict(), path)
