"""MLP决策模块

用于机器人对抗的神经网络决策系统，基于强化学习训练。

模块结构：
- config.py: 配置参数
- state_encoder.py: 状态特征提取器
- model.py: MLP网络定义（Actor-Critic）
- gym_env.py: Gym环境包装器，封装仿真器为RL可用接口
- train.py: 训练脚本（PPO算法）
- inference.py: 推理引擎

使用方法：
    # 训练模型
    python -m mlp.train --mock --timesteps 100000
    
    # 推理
    python -m mlp.inference --model logs/mlp/ppo_xxx/model_best.pt --mock
"""

from importlib import import_module

from .config import MLPConfig
from .contracts import (
    DeployObservationV1,
    TruthStateV1,
    DecisionPacketV1,
    EpisodeStatusV1,
    ResetScenarioV1,
)
from .state_encoder import StateEncoder, RobotState
from .model import ActorCritic, create_model, load_model, save_model
from .skill_executor import SkillExecutor, SkillExecutionResult


_LAZY_EXPORTS = {
    "SoccerSimEnv": (".gym_env", "SoccerSimEnv"),
    "MockSoccerSimEnv": (".gym_env", "MockSoccerSimEnv"),
    "make_env": (".gym_env", "make_env"),
    "PPOTrainer": (".train", "PPOTrainer"),
    "MLPInference": (".inference", "MLPInference"),
    "MLPController": (".inference", "MLPController"),
    "run_mlp_strategy": (".inference", "run_mlp_strategy"),
}


def __getattr__(name: str):
    if name not in _LAZY_EXPORTS:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

    module_name, attr_name = _LAZY_EXPORTS[name]
    module = import_module(module_name, __name__)
    value = getattr(module, attr_name)
    globals()[name] = value
    return value

__all__ = [
    "MLPConfig",
    "DeployObservationV1",
    "TruthStateV1",
    "DecisionPacketV1",
    "EpisodeStatusV1",
    "ResetScenarioV1",
    "StateEncoder",
    "RobotState",
    "ActorCritic",
    "create_model",
    "load_model",
    "save_model",
    "SoccerSimEnv",
    "MockSoccerSimEnv",
    "make_env",
    "PPOTrainer",
    "MLPInference",
    "MLPController",
    "run_mlp_strategy",
    "SkillExecutor",
    "SkillExecutionResult",
]
