"""MLP决策模块

用于机器人对抗的神经网络决策系统。

模块结构（推理框架）：
- config.py: 配置参数
- contracts.py: 数据契约定义
- state_encoder.py: 状态特征提取器
- model.py: MLP网络定义
- inference.py: 推理引擎
- skill_executor.py: 技能执行器

训练相关代码已迁移至 TreeDiagramDataset 项目。

使用方法：
    # 推理
    from mlp import MLPInference, MLPController
    inference = MLPInference(model_path)
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
    "MLPInference",
    "MLPController",
    "run_mlp_strategy",
    "SkillExecutor",
    "SkillExecutionResult",
]