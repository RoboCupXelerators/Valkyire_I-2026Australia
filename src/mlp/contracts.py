"""MLP 策略契约模块

定义部署对齐的观测/决策/训练协议契约。
所有结构优先使用 dataclass 实现。

契约版本命名规范:
- V1 后缀表示第一版稳定契约
- 契约一旦发布，字段只能增加，不能删除或修改语义
- 如需破坏性变更，创建新版本契约

参考: docs/MLP_POLICY_ROADMAP.md
"""

from dataclasses import dataclass, field, asdict
from typing import List, Optional, Dict, Any
from enum import IntEnum
import json

from LunaPre.strategy_tools import BALL_HELD, BALL_NOT_FOUND


# ==================== 枚举定义 ====================

class IntentId(IntEnum):
    """高层技能意图枚举
    
    对应路线图 4.4 节的 intent_id 初始集合。
    """
    SEARCH_BALL = 0       # 搜索球
    APPROACH_BALL = 1     # 靠近球
    SECURE_BALL = 2       # 控球
    CARRY_TO_POSE = 3     # 带球到目标点
    SHOOT = 4             # 射门
    BLOCK_GOAL = 5        # 封堵球门
    MARK_ENEMY = 6        # 盯防敌人
    RETURN_HOME = 7       # 回归初始位
    AVOID_WALL = 8        # 避墙
    SUPPORT_MATE = 9      # 支援队友


class FaceMode(IntEnum):
    """面向模式枚举"""
    FACE_BALL = 0         # 面向球
    FACE_GOAL = 1         # 面向球门
    FACE_TARGET = 2       # 面向目标点
    FACE_KEEP = 3         # 保持当前朝向


class DribbleMode(IntEnum):
    """运球模式枚举"""
    OFF = 0               # 关闭
    WEAK = 1              # 弱运球
    STRONG = 2            # 强运球


class ShootIntent(IntEnum):
    """射门意图枚举"""
    NONE = 0              # 不射门
    TRY_SHOOT = 1         # 尝试射门


class RefereeState(IntEnum):
    """裁判状态枚举 (占位，待对接真实裁判盒协议)"""
    NORMAL = 0            # 正常比赛
    KICKOFF = 1           # 开球
    FREEKICK = 2          # 任意球
    PENALTY = 3           # 点球
    GOAL = 4              # 进球
    TIMEOUT = 5           # 暂停


class RoleId(IntEnum):
    """角色ID枚举 (占位)"""
    GOALIE = 0            # 守门员
    DEFENDER = 1          # 后卫
    ATTACKER = 2          # 前锋
    SUPPORT = 3           # 支援


# ==================== 观测契约 ====================

@dataclass
class EntityObservationV1:
    """单个实体的观测信息
    
    用于球、敌方、队友等实体的观测。
    包含可见性、数据年龄、置信度等部署必需字段。
    """
    # 相对位置 (机器人坐标系)
    rel_x: float = 0.0           # 相对X (cm)
    rel_y: float = 0.0           # 相对Y (cm)
    
    # 观测质量标记
    visible: bool = False        # 当前是否可见
    age_ms: int = -1             # 数据年龄 (ms), -1 表示未知/无效
    confidence: float = 0.0      # 检测置信度 [0, 1]
    
    def to_dict(self) -> Dict[str, Any]:
        """转换为字典"""
        return asdict(self)
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'EntityObservationV1':
        """从字典创建"""
        return cls(
            rel_x=float(data.get('rel_x', 0.0)),
            rel_y=float(data.get('rel_y', 0.0)),
            visible=bool(data.get('visible', False)),
            age_ms=int(data.get('age_ms', -1)),
            confidence=float(data.get('confidence', 0.0)),
        )


@dataclass
class GoalObservationV1:
    """球门观测信息"""
    # 球门中心相对位置
    rel_x: float = 0.0
    rel_y: float = 0.0
    
    # 观测质量
    visible: bool = False
    age_ms: int = -1
    
    # 球门宽度相关 (可选)
    left_post_x: float = 0.0     # 左门柱相对位置
    left_post_y: float = 0.0
    right_post_x: float = 0.0    # 右门柱相对位置
    right_post_y: float = 0.0
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'GoalObservationV1':
        return cls(
            rel_x=float(data.get('rel_x', 0.0)),
            rel_y=float(data.get('rel_y', 0.0)),
            visible=bool(data.get('visible', False)),
            age_ms=int(data.get('age_ms', -1)),
            left_post_x=float(data.get('left_post_x', 0.0)),
            left_post_y=float(data.get('left_post_y', 0.0)),
            right_post_x=float(data.get('right_post_x', 0.0)),
            right_post_y=float(data.get('right_post_y', 0.0)),
        )


@dataclass
class DeployObservationV1:
    """部署对齐观测契约 V1
    
    这是 actor (策略网络) 最终应该看到的观测结构。
    所有字段必须能对应到真实部署环境 (IL + AI/TI + LunaPre)。
    
    参考: MLP_POLICY_ROADMAP.md 4.2 节
    """
    # ==================== 自身状态 ====================
    # 场地绝对位姿 (来自 FieldRobotPose)
    self_x: float = 0.0          # 场地X坐标 (cm)
    self_y: float = 0.0          # 场地Y坐标 (cm)
    self_yaw: float = 0.0        # 航向角 (deg)
    
    # 自身速度 (来自 IL / chassis / estimator)
    # 注意: 真实侧可能不稳定，标记为可选
    self_vx: float = 0.0         # X方向速度 (cm/s)
    self_vy: float = 0.0         # Y方向速度 (cm/s)
    self_wz: float = 0.0         # 角速度 (deg/s)
    velocity_valid: bool = False # 速度数据是否有效
    
    # ==================== 球观测 ====================
    # 相对位置 (来自 RobotBallPos)
    ball_rel_x: float = 0.0      # 球相对X (cm)
    ball_rel_y: float = 0.0      # 球相对Y (cm)
    
    # 观测质量
    ball_visible: bool = False   # 球是否可见 (非真值，由观测层推导)
    ball_age_ms: int = -1        # 球数据年龄 (ms)
    ball_confidence: float = 0.0 # 球观测置信度 [0, 1]
    has_ball: bool = False       # 是否持球 (RobotBallPos == [0xDDD, 0xDDD])
    
    # ==================== 边界距离 ====================
    # 来自 DirectDistance (lidar / stub)
    wall_front: float = 0.0      # 前方距离 (cm)
    wall_left: float = 0.0       # 左侧距离 (cm)
    wall_back: float = 0.0       # 后方距离 (cm)
    wall_right: float = 0.0      # 右侧距离 (cm)
    wall_valid: bool = False     # 边界数据是否有效
    
    # ==================== 敌方观测 ====================
    # 最多 N 个敌方 (来自 RobotChassisPos 或 FieldChassisPos)
    # 列表长度可变，编码时需固定维度
    enemies: List[EntityObservationV1] = field(default_factory=list)
    max_enemies: int = 4         # 最大敌方数量 (用于编码维度)
    
    # ==================== 球门观测 ====================
    # 来自 FieldGoalsPos / vision
    blue_goal: GoalObservationV1 = field(default_factory=GoalObservationV1)
    yellow_goal: GoalObservationV1 = field(default_factory=GoalObservationV1)
    
    # ==================== 队友观测 (占位) ====================
    # 来自队内通信，多机器人策略必需
    teammates: List[EntityObservationV1] = field(default_factory=list)
    max_teammates: int = 3       # 最大队友数量
    
    # ==================== 比赛状态 (部分占位) ====================
    referee_state: int = 0       # 裁判状态 (RefereeState 枚举)
    penalty_state: int = 0       # 点球状态 (占位)
    role: int = 0                # 当前角色 (RoleId 枚举)
    
    # ==================== 历史信息 (占位) ====================
    # 用于时序建模
    last_decision_id: int = 0    # 上一步决策 intent_id
    last_action_age_ms: int = 0  # 上一步动作已执行时间
    stuck_flag: bool = False     # 是否卡住
    
    def to_dict(self) -> Dict[str, Any]:
        """转换为字典 (用于 JSON 序列化)"""
        return {
            # 自身状态
            'self_x': self.self_x,
            'self_y': self.self_y,
            'self_yaw': self.self_yaw,
            'self_vx': self.self_vx,
            'self_vy': self.self_vy,
            'self_wz': self.self_wz,
            'velocity_valid': self.velocity_valid,
            
            # 球观测
            'ball_rel_x': self.ball_rel_x,
            'ball_rel_y': self.ball_rel_y,
            'ball_visible': self.ball_visible,
            'ball_age_ms': self.ball_age_ms,
            'ball_confidence': self.ball_confidence,
            'has_ball': self.has_ball,
            
            # 边界
            'wall_front': self.wall_front,
            'wall_left': self.wall_left,
            'wall_back': self.wall_back,
            'wall_right': self.wall_right,
            'wall_valid': self.wall_valid,
            
            # 敌方
            'enemies': [e.to_dict() for e in self.enemies],
            'max_enemies': self.max_enemies,
            
            # 球门
            'blue_goal': self.blue_goal.to_dict(),
            'yellow_goal': self.yellow_goal.to_dict(),
            
            # 队友
            'teammates': [t.to_dict() for t in self.teammates],
            'max_teammates': self.max_teammates,
            
            # 比赛状态
            'referee_state': self.referee_state,
            'penalty_state': self.penalty_state,
            'role': self.role,
            
            # 历史
            'last_decision_id': self.last_decision_id,
            'last_action_age_ms': self.last_action_age_ms,
            'stuck_flag': self.stuck_flag,
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'DeployObservationV1':
        """从字典创建 (用于 JSON 反序列化)"""
        enemies_data = data.get('enemies', [])
        teammates_data = data.get('teammates', [])
        
        return cls(
            # 自身状态
            self_x=float(data.get('self_x', 0.0)),
            self_y=float(data.get('self_y', 0.0)),
            self_yaw=float(data.get('self_yaw', 0.0)),
            self_vx=float(data.get('self_vx', 0.0)),
            self_vy=float(data.get('self_vy', 0.0)),
            self_wz=float(data.get('self_wz', 0.0)),
            velocity_valid=bool(data.get('velocity_valid', False)),
            
            # 球观测
            ball_rel_x=float(data.get('ball_rel_x', 0.0)),
            ball_rel_y=float(data.get('ball_rel_y', 0.0)),
            ball_visible=bool(data.get('ball_visible', False)),
            ball_age_ms=int(data.get('ball_age_ms', -1)),
            ball_confidence=float(data.get('ball_confidence', 0.0)),
            has_ball=bool(data.get('has_ball', False)),
            
            # 边界
            wall_front=float(data.get('wall_front', 0.0)),
            wall_left=float(data.get('wall_left', 0.0)),
            wall_back=float(data.get('wall_back', 0.0)),
            wall_right=float(data.get('wall_right', 0.0)),
            wall_valid=bool(data.get('wall_valid', False)),
            
            # 敌方
            enemies=[EntityObservationV1.from_dict(e) for e in enemies_data],
            max_enemies=int(data.get('max_enemies', 4)),
            
            # 球门
            blue_goal=GoalObservationV1.from_dict(data.get('blue_goal', {})),
            yellow_goal=GoalObservationV1.from_dict(data.get('yellow_goal', {})),
            
            # 队友
            teammates=[EntityObservationV1.from_dict(t) for t in teammates_data],
            max_teammates=int(data.get('max_teammates', 3)),
            
            # 比赛状态
            referee_state=int(data.get('referee_state', 0)),
            penalty_state=int(data.get('penalty_state', 0)),
            role=int(data.get('role', 0)),
            
            # 历史
            last_decision_id=int(data.get('last_decision_id', 0)),
            last_action_age_ms=int(data.get('last_action_age_ms', 0)),
            stuck_flag=bool(data.get('stuck_flag', False)),
        )
    
    def to_json(self) -> str:
        """序列化为 JSON 字符串"""
        return json.dumps(self.to_dict(), ensure_ascii=False)
    
    @classmethod
    def from_json(cls, json_str: str) -> 'DeployObservationV1':
        """从 JSON 字符串反序列化"""
        return cls.from_dict(json.loads(json_str))


# ==================== 真值契约 ====================

@dataclass
class TruthStateV1:
    """仿真真值契约 V1
    
    用于 trainer / critic / reward / teacher / debug。
    actor 不应直接访问此结构。
    
    参考: MLP_POLICY_ROADMAP.md 4.3 节
    """
    # ==================== 球真值 ====================
    ball_true_x: float = 0.0     # 球绝对X (cm)
    ball_true_y: float = 0.0     # 球绝对Y (cm)
    ball_true_vx: float = 0.0    # 球速度X (cm/s)
    ball_true_vy: float = 0.0    # 球速度Y (cm/s)
    
    # ==================== 敌方真值 ====================
    # [(x, y, vx, vy, yaw), ...]
    enemies_true: List[Dict[str, float]] = field(default_factory=list)
    
    # ==================== 队友真值 ====================
    teammates_true: List[Dict[str, float]] = field(default_factory=list)
    
    # ==================== Episode 状态 ====================
    episode_id: int = 0          # 当前 episode ID
    elapsed_ms: int = 0          # 已运行时间 (ms)
    is_goal: bool = False        # 是否进球
    is_lose_ball: bool = False   # 是否丢球
    is_out_of_bounds: bool = False  # 是否出界
    is_timeout: bool = False     # 是否超时
    
    # ==================== 场景信息 ====================
    scenario_id: int = 0         # 场景 ID
    field_side: int = 0          # 半场 (0: 蓝方, 1: 黄方)
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            'ball_true_x': self.ball_true_x,
            'ball_true_y': self.ball_true_y,
            'ball_true_vx': self.ball_true_vx,
            'ball_true_vy': self.ball_true_vy,
            'enemies_true': self.enemies_true,
            'teammates_true': self.teammates_true,
            'episode_id': self.episode_id,
            'elapsed_ms': self.elapsed_ms,
            'is_goal': self.is_goal,
            'is_lose_ball': self.is_lose_ball,
            'is_out_of_bounds': self.is_out_of_bounds,
            'is_timeout': self.is_timeout,
            'scenario_id': self.scenario_id,
            'field_side': self.field_side,
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'TruthStateV1':
        return cls(
            ball_true_x=float(data.get('ball_true_x', 0.0)),
            ball_true_y=float(data.get('ball_true_y', 0.0)),
            ball_true_vx=float(data.get('ball_true_vx', 0.0)),
            ball_true_vy=float(data.get('ball_true_vy', 0.0)),
            enemies_true=data.get('enemies_true', []),
            teammates_true=data.get('teammates_true', []),
            episode_id=int(data.get('episode_id', 0)),
            elapsed_ms=int(data.get('elapsed_ms', 0)),
            is_goal=bool(data.get('is_goal', False)),
            is_lose_ball=bool(data.get('is_lose_ball', False)),
            is_out_of_bounds=bool(data.get('is_out_of_bounds', False)),
            is_timeout=bool(data.get('is_timeout', False)),
            scenario_id=int(data.get('scenario_id', 0)),
            field_side=int(data.get('field_side', 0)),
        )
    
    def to_json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False)
    
    @classmethod
    def from_json(cls, json_str: str) -> 'TruthStateV1':
        return cls.from_dict(json.loads(json_str))


# ==================== 决策契约 ====================

@dataclass
class DecisionPacketV1:
    """决策包契约 V1
    
    策略网络的输出结构，不再是低层速度。
    由 Skill Executor 翻译为具体控制指令。
    
    参考: MLP_POLICY_ROADMAP.md 4.4 节
    """
    # ==================== 核心决策 ====================
    intent_id: int = 0           # 技能意图 (IntentId 枚举)
    
    # ==================== 目标参数 ====================
    target_x: float = 0.0        # 目标点X (cm)
    target_y: float = 0.0        # 目标点Y (cm)
    target_yaw: float = 0.0      # 目标朝向 (deg)
    
    # ==================== 执行参数 ====================
    face_mode: int = 0           # 面向模式 (FaceMode 枚举)
    move_gain: float = 1.0       # 移动强度权重 [0, 1]
    dribble_mode: int = 0        # 运球模式 (DribbleMode 枚举)
    shoot_intent: int = 0        # 射门意图 (ShootIntent 枚举)
    
    # ==================== 元信息 ====================
    decision_horizon_ms: int = 100  # 建议持续时长 (ms)
    confidence: float = 1.0      # 决策置信度 [0, 1]
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            'intent_id': self.intent_id,
            'target_x': self.target_x,
            'target_y': self.target_y,
            'target_yaw': self.target_yaw,
            'face_mode': self.face_mode,
            'move_gain': self.move_gain,
            'dribble_mode': self.dribble_mode,
            'shoot_intent': self.shoot_intent,
            'decision_horizon_ms': self.decision_horizon_ms,
            'confidence': self.confidence,
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'DecisionPacketV1':
        return cls(
            intent_id=int(data.get('intent_id', 0)),
            target_x=float(data.get('target_x', 0.0)),
            target_y=float(data.get('target_y', 0.0)),
            target_yaw=float(data.get('target_yaw', 0.0)),
            face_mode=int(data.get('face_mode', 0)),
            move_gain=float(data.get('move_gain', 1.0)),
            dribble_mode=int(data.get('dribble_mode', 0)),
            shoot_intent=int(data.get('shoot_intent', 0)),
            decision_horizon_ms=int(data.get('decision_horizon_ms', 100)),
            confidence=float(data.get('confidence', 1.0)),
        )
    
    def to_json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False)
    
    @classmethod
    def from_json(cls, json_str: str) -> 'DecisionPacketV1':
        return cls.from_dict(json.loads(json_str))


# ==================== Episode 状态契约 ====================

@dataclass
class EpisodeStatusV1:
    """Episode 状态契约 V1
    
    用于训练控制，查询当前 episode 的状态。
    """
    episode_id: int = 0          # Episode ID
    is_running: bool = False     # 是否正在运行
    is_done: bool = False        # 是否结束
    result: int = 0              # 结果码 (0: 无结果, 1: 成功, -1: 失败)
    elapsed_ms: int = 0          # 已运行时间 (ms)
    
    # 终止原因
    end_reason: str = ""         # 结束原因描述
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'EpisodeStatusV1':
        return cls(
            episode_id=int(data.get('episode_id', 0)),
            is_running=bool(data.get('is_running', False)),
            is_done=bool(data.get('is_done', False)),
            result=int(data.get('result', 0)),
            elapsed_ms=int(data.get('elapsed_ms', 0)),
            end_reason=str(data.get('end_reason', '')),
        )
    
    def to_json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False)
    
    @classmethod
    def from_json(cls, json_str: str) -> 'EpisodeStatusV1':
        return cls.from_dict(json.loads(json_str))


# ==================== Reset 场景契约 ====================

@dataclass
class ResetScenarioV1:
    """Reset 场景契约 V1
    
    用于精确控制训练场景的初始化。
    Python 侧发送此结构给 Unity 进行场景重置。
    
    参考: MLP_POLICY_ROADMAP.md 5.2 节
    """
    # ==================== 随机种子 ====================
    seed: int = 0                # 随机种子 (0 表示使用时间)
    
    # ==================== 本机位姿 ====================
    self_pose: Dict[str, float] = field(default_factory=lambda: {
        'x': 0.0, 'y': 0.0, 'yaw': 0.0
    })
    
    # ==================== 球状态 ====================
    ball_state: Dict[str, float] = field(default_factory=lambda: {
        'x': 0.0, 'y': 0.0, 'vx': 0.0, 'vy': 0.0
    })
    
    # ==================== 敌方配置 ====================
    # [{'x': 0, 'y': 0, 'yaw': 0, 'vx': 0, 'vy': 0}, ...]
    enemies: List[Dict[str, float]] = field(default_factory=list)
    
    # ==================== 队友配置 ====================
    teammates: List[Dict[str, float]] = field(default_factory=list)
    
    # ==================== 场地配置 ====================
    field_side: int = 0          # 半场 (0: 蓝方, 1: 黄方)
    referee_state: int = 0       # 初始裁判状态
    
    # ==================== 物理参数 (可选) ====================
    physics_profile: Dict[str, float] = field(default_factory=dict)
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            'seed': self.seed,
            'self_pose': self.self_pose,
            'ball_state': self.ball_state,
            'enemies': self.enemies,
            'teammates': self.teammates,
            'field_side': self.field_side,
            'referee_state': self.referee_state,
            'physics_profile': self.physics_profile,
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'ResetScenarioV1':
        return cls(
            seed=int(data.get('seed', 0)),
            self_pose=data.get('self_pose', {'x': 0.0, 'y': 0.0, 'yaw': 0.0}),
            ball_state=data.get('ball_state', {'x': 0.0, 'y': 0.0, 'vx': 0.0, 'vy': 0.0}),
            enemies=data.get('enemies', []),
            teammates=data.get('teammates', []),
            field_side=int(data.get('field_side', 0)),
            referee_state=int(data.get('referee_state', 0)),
            physics_profile=data.get('physics_profile', {}),
        )
    
    def to_json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False)
    
    @classmethod
    def from_json(cls, json_str: str) -> 'ResetScenarioV1':
        return cls.from_dict(json.loads(json_str))


# ==================== 辅助函数 ====================

_BALL_NOT_VISIBLE_SENTINEL = tuple(BALL_NOT_FOUND)
_BALL_HAS_POSSESSION_SENTINEL = tuple(BALL_HELD)
_INVALID_WALL_SENTINEL = BALL_NOT_FOUND[0]


def _is_pair(value: Any) -> bool:
    """Return True when value looks like a 2-element coordinate pair."""
    return isinstance(value, (list, tuple)) and len(value) >= 2


def _normalize_ball_observation(ball_pos: Any) -> Dict[str, Any]:
    """Map LunaPre/Simulator RobotBallPos semantics to contract fields.

    RobotBallPos is already in robot-relative coordinates. Sentinel values:
    - [0xFFF, 0xFFF]: ball not visible
    - [0xDDD, 0xDDD]: robot already has possession
    """
    result = {
        'ball_rel_x': 0.0,
        'ball_rel_y': 0.0,
        'ball_visible': False,
        'ball_age_ms': -1,
        'ball_confidence': 0.0,
        'has_ball': False,
    }

    if not _is_pair(ball_pos):
        return result

    bx = ball_pos[0]
    by = ball_pos[1]
    if (bx, by) == _BALL_NOT_VISIBLE_SENTINEL:
        return result
    if (bx, by) == _BALL_HAS_POSSESSION_SENTINEL:
        result['has_ball'] = True
        result['ball_age_ms'] = 0
        result['ball_confidence'] = 1.0
        return result

    result['ball_rel_x'] = float(bx)
    result['ball_rel_y'] = float(by)
    result['ball_visible'] = not (bx == 0 and by == 0)
    result['ball_age_ms'] = 0 if result['ball_visible'] else -1
    result['ball_confidence'] = 1.0 if result['ball_visible'] else 0.0
    return result


def _apply_wall_distances(obs: 'DeployObservationV1', wall_dist: Any) -> None:
    """Populate wall distances when the source data is valid.

    DirectDistance keeps historical unit `mm`. Any `0xFFF` entry means the lidar
    distance is invalid/unavailable and must not be treated as a real wall reading.
    """
    if not isinstance(wall_dist, (list, tuple)) or len(wall_dist) < 4:
        return
    if any(dist == _INVALID_WALL_SENTINEL for dist in wall_dist[:4]):
        return

    obs.wall_front = float(wall_dist[0]) / 10.0  # mm -> cm
    obs.wall_left = float(wall_dist[1]) / 10.0
    obs.wall_back = float(wall_dist[2]) / 10.0
    obs.wall_right = float(wall_dist[3]) / 10.0
    obs.wall_valid = True

def observation_from_lunapre() -> DeployObservationV1:
    """从 LunaPre 创建部署对齐观测
    
    当前实现是最小可用骨架：只读取已存在的稳定接口，
    并保留球门、裁判状态等后续扩展位。

    注意当前约定:
    - `RobotBallPos()` 已经是机器人坐标系相对坐标。
    - `[0xFFF, 0xFFF]` 表示球不可见。
    - `[0xDDD, 0xDDD]` 表示已经持球。
    - `DirectDistance()` 任一方向为 `0xFFF` 时，整组墙距视为无效。
    
    Returns:
        DeployObservationV1 实例
    """
    obs = DeployObservationV1()
    
    try:
        # 尝试从 LunaPre 获取数据
        # 注意: 这里是占位实现，实际需要根据 LunaPre 接口调整
        from LunaPre import UnitedPosition
        
        # 自身位姿
        pos = UnitedPosition.FieldRobotPose()
        if pos:
            obs.self_x = float(pos[0])
            obs.self_y = float(pos[1])
            obs.self_yaw = float(pos[2])
        
        # 球位置: LunaPre / simulator 都返回机器人坐标系相对坐标。
        ball_pos = UnitedPosition.RobotBallPos()
        if ball_pos is not None:
            for key, value in _normalize_ball_observation(ball_pos).items():
                setattr(obs, key, value)
        
        # 边界距离
        wall_dist = UnitedPosition.DirectDistance()  # [front, left, back, right] in mm
        _apply_wall_distances(obs, wall_dist)
        
        # 敌方位置
        chassis_pos = UnitedPosition.FieldChassisPos()
        if chassis_pos:
            for i, c in enumerate(chassis_pos[:obs.max_enemies]):
                enemy = EntityObservationV1(
                    rel_x=float(c[0]) - obs.self_x,  # 简化为相对坐标
                    rel_y=float(c[1]) - obs.self_y,
                    visible=True,
                    age_ms=0,
                    confidence=1.0,
                )
                obs.enemies.append(enemy)
        
        # 球门位置
        goals = UnitedPosition.FieldGoalsPos()
        if goals:
            # 根据实际 LunaPre 接口填充
            pass
            
    except ImportError:
        # LunaPre 不可用，返回默认值
        pass
    except Exception as e:
        # 其他错误，记录但不抛出
        pass
    
    return obs


def observation_from_simulator(
    self_x: float, self_y: float, self_yaw: float,
    ball_x: float, ball_y: float, has_ball: bool,
    wall_dist: List[float],
    enemies: List[tuple] = None,
    **kwargs
) -> DeployObservationV1:
    """从模拟器数据创建观测
    
    将模拟器的原始数据转换为部署对齐的观测契约。
    这是 gym_env 应该使用的主要转换函数。
    
    Args:
        self_x, self_y, self_yaw: 自身位姿
        ball_x, ball_y: 球相对位置 (机器人坐标系, cm)
        has_ball: 是否持球
        wall_dist: 边界距离 [front, left, back, right] (mm)
        enemies: 敌方位置列表 [(x, y), ...]
            注意: 这里要求传入的敌方坐标已经是机器人坐标系相对坐标。
            例如 `XeleratorSimBits.GetEnemyPos()` 的返回值就可直接透传。
        **kwargs: 其他可选参数
    
    Returns:
        DeployObservationV1 实例
    """
    obs = DeployObservationV1(self_x=self_x, self_y=self_y, self_yaw=self_yaw)
    for key, value in _normalize_ball_observation(
        _BALL_HAS_POSSESSION_SENTINEL if has_ball else (ball_x, ball_y)
    ).items():
        setattr(obs, key, value)
    
    # 边界距离
    _apply_wall_distances(obs, wall_dist)
    
    # 敌方
    if enemies:
        for i, (ex, ey) in enumerate(enemies[:obs.max_enemies]):
            # Simulator API already returns robot-frame relative coordinates.
            enemy = EntityObservationV1(
                rel_x=float(ex),
                rel_y=float(ey),
                visible=True,
                age_ms=0,
                confidence=1.0,
            )
            obs.enemies.append(enemy)
    
    # 其他参数
    for key, value in kwargs.items():
        if hasattr(obs, key):
            setattr(obs, key, value)
    
    return obs


def observation_from_lunapre_stub() -> DeployObservationV1:
    """Backward-compatible alias for older callers/tests."""
    return observation_from_lunapre()
