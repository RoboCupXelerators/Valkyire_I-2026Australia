"""MLP决策模块配置

定义网络结构、训练参数、环境参数等配置。
"""

from dataclasses import dataclass, field
from typing import List, Optional
import os


@dataclass
class MLPConfig:
    """MLP决策系统配置"""
    
    # ==================== 网络结构 ====================
    # 输入维度（状态特征数）
    # 当前 DeployObservationV1 编码维度：
    # 自身位姿(3) + 速度(3) + 球位置(2) + 持球状态(1) +
    # 球可见性/年龄/置信度(3) + 敌方(最大4个 × 5) + 边界距离(4) +
    # 球门方向(2) + 比赛状态(3) + 历史信息(3) = 44
    state_dim: int = 44
    
    # 隐藏层结构
    hidden_dims: List[int] = field(default_factory=lambda: [128, 64, 32])
    
    # 输出维度（动作空间）
    # vx, vy, wz, kick(0/1), dribble(0/1) = 5
    action_dim: int = 5

    # Policy output contract.
    # - legacy_action: vx, vy, wz, kick, dribble
    # - decision_packet: native DecisionPacketV1 action vector
    output_mode: str = "legacy_action"
    decision_continuous_dim: int = 6  # target_x, target_y, target_yaw, move_gain, horizon, confidence
    decision_intent_classes: int = 10
    decision_face_classes: int = 4
    decision_dribble_classes: int = 3
    decision_shoot_classes: int = 2
    
    # 激活函数
    activation: str = "relu"  # relu, tanh, elu
    
    # ==================== 训练参数 ====================
    # 学习率
    learning_rate: float = 3e-4
    
    # PPO参数
    gamma: float = 0.99           # 折扣因子
    gae_lambda: float = 0.95      # GAE lambda
    clip_epsilon: float = 0.2     # PPO clip范围
    entropy_coef: float = 0.01    # 熵正则化系数
    value_coef: float = 0.5       # 价值损失系数
    max_grad_norm: float = 0.5    # 梯度裁剪
    
    # 训练循环
    batch_size: int = 64
    n_epochs: int = 10            # 每次更新的epoch数
    n_steps: int = 2048           # 每次收集的步数
    total_timesteps: int = 1_000_000  # 总训练步数
    
    # ==================== 环境参数 ====================
    # 场地尺寸 (cm)
    field_length: float = 260.0   # 场地长度（前后方向）
    field_width: float = 200.0    # 场地宽度（左右方向）
    
    # 机器人参数
    max_speed: float = 300.0      # 最大速度 (cm/s)
    max_angular_speed: float = 180.0  # 最大角速度 (deg/s)
    
    # 检测范围
    ball_detection_range: float = 150.0  # 球检测范围 (cm)
    enemy_detection_range: float = 200.0  # 敌方检测范围 (cm)
    max_enemies: int = 4          # 最多追踪敌方数量
    
    # 目标位置（球门）
    goal_x: float = 130.0         # 对方球门X坐标
    goal_y: float = 0.0           # 对方球门Y坐标
    goal_width: float = 60.0      # 球门宽度
    
    # ==================== 奖励函数参数 ====================
    # 进球奖励
    reward_goal: float = 100.0
    
    # 持球奖励（每步）
    reward_has_ball: float = 0.1
    
    # 接近球奖励
    reward_approach_ball: float = 0.01  # 每减少1cm距离的奖励系数
    
    # 射门奖励
    reward_shoot: float = 1.0
    
    # 被抢球惩罚
    reward_lost_ball: float = -1.0
    
    # 出界惩罚
    reward_out_of_bounds: float = -50.0  # 出界重罚
    
    # 罚下时间（秒）
    reward_penalty_time: float = 120.0
    
    # 每步时间惩罚（鼓励快速行动）
    reward_time_penalty: float = -0.001
    
    # ==================== 仿真器连接 ====================
    sim_ip: str = "127.0.0.1"
    sim_cmd_port: int = 20001
    sim_status_port: int = 20002
    robot_id: int = 1
    
    # ==================== 保存与日志 ====================
    # 模型保存路径
    model_dir: str = "models/mlp"
    save_every: int = 10000       # 每N步保存一次模型
    
    # 日志
    log_dir: str = "logs/mlp"
    log_every: int = 100          # 每N步记录一次
    
    # ==================== 推理参数 ====================
    # 推理时的动作平滑
    action_smoothing: float = 0.3  # 0=无平滑, 1=完全平滑
    
    # 安全边界
    safe_margin: float = 10.0     # 与边界的最小距离 (cm)
    
    def __post_init__(self):
        """初始化后处理"""
        self.output_mode = str(self.output_mode or "legacy_action").lower()
        if self.output_mode not in ("legacy_action", "decision_packet"):
            raise ValueError(f"Unsupported MLP output_mode: {self.output_mode}")
        if self.output_mode == "decision_packet":
            self.action_dim = self.decision_continuous_dim + 4
        # 确保目录存在
        os.makedirs(self.model_dir, exist_ok=True)
        os.makedirs(self.log_dir, exist_ok=True)
    
    @classmethod
    def from_dict(cls, config_dict: dict) -> "MLPConfig":
        """从字典创建配置"""
        return cls(**{k: v for k, v in config_dict.items() if k in cls.__dataclass_fields__})
    
    def to_dict(self) -> dict:
        """转换为字典"""
        return {
            "state_dim": self.state_dim,
            "hidden_dims": self.hidden_dims,
            "action_dim": self.action_dim,
            "output_mode": self.output_mode,
            "decision_continuous_dim": self.decision_continuous_dim,
            "decision_intent_classes": self.decision_intent_classes,
            "decision_face_classes": self.decision_face_classes,
            "decision_dribble_classes": self.decision_dribble_classes,
            "decision_shoot_classes": self.decision_shoot_classes,
            "activation": self.activation,
            "learning_rate": self.learning_rate,
            "gamma": self.gamma,
            "gae_lambda": self.gae_lambda,
            "clip_epsilon": self.clip_epsilon,
            "entropy_coef": self.entropy_coef,
            "value_coef": self.value_coef,
            "max_grad_norm": self.max_grad_norm,
            "batch_size": self.batch_size,
            "n_epochs": self.n_epochs,
            "n_steps": self.n_steps,
            "total_timesteps": self.total_timesteps,
            "field_length": self.field_length,
            "field_width": self.field_width,
            "max_speed": self.max_speed,
            "max_angular_speed": self.max_angular_speed,
            "ball_detection_range": self.ball_detection_range,
            "enemy_detection_range": self.enemy_detection_range,
            "max_enemies": self.max_enemies,
            "goal_x": self.goal_x,
            "goal_y": self.goal_y,
            "goal_width": self.goal_width,
            "reward_goal": self.reward_goal,
            "reward_has_ball": self.reward_has_ball,
            "reward_approach_ball": self.reward_approach_ball,
            "reward_shoot": self.reward_shoot,
            "reward_lost_ball": self.reward_lost_ball,
            "reward_out_of_bounds": self.reward_out_of_bounds,
            "reward_time_penalty": self.reward_time_penalty,
            "sim_ip": self.sim_ip,
            "sim_cmd_port": self.sim_cmd_port,
            "sim_status_port": self.sim_status_port,
            "robot_id": self.robot_id,
            "model_dir": self.model_dir,
            "save_every": self.save_every,
            "log_dir": self.log_dir,
            "log_every": self.log_every,
            "action_smoothing": self.action_smoothing,
            "safe_margin": self.safe_margin,
        }


# 默认配置实例
default_config = MLPConfig()
