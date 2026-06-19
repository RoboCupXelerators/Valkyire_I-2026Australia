"""状态特征编码器

将多源传感器数据编码为MLP输入向量。

支持两种输入模式:
1. 传统 RobotState (向后兼容)
2. 部署对齐的 DeployObservationV1 (推荐)

参考: docs/MLP_POLICY_ROADMAP.md
"""

import math
import numpy as np
from typing import List, Optional, Tuple, Dict, Any, Union
from dataclasses import dataclass

from .config import MLPConfig
from .contracts import (
    DecisionPacketV1,
    DeployObservationV1,
    DribbleMode,
    EntityObservationV1,
    FaceMode,
    IntentId,
    ShootIntent,
)


@dataclass
class RobotState:
    """机器人状态数据结构"""
    # 自身位姿
    x: float = 0.0          # 场地X坐标 (cm)
    y: float = 0.0          # 场地Y坐标 (cm)
    yaw: float = 0.0        # 航向角 (deg)
    
    # 自身速度
    vx: float = 0.0         # X方向速度 (cm/s)
    vy: float = 0.0         # Y方向速度 (cm/s)
    wz: float = 0.0         # 角速度 (deg/s)
    
    # 球信息
    ball_x: float = 0.0     # 球相对X位置 (cm)
    ball_y: float = 0.0     # 球相对Y位置 (cm)
    has_ball: bool = False  # 是否持球
    
    # 敌方机器人位置（场地坐标）
    enemies: List[Tuple[float, float]] = None  # [(x1, y1), (x2, y2), ...]
    
    # 边界距离 [front, left, back, right] (mm -> cm)
    wall_dist: List[float] = None
    
    def __post_init__(self):
        if self.enemies is None:
            self.enemies = []
        if self.wall_dist is None:
            self.wall_dist = [0.0, 0.0, 0.0, 0.0]


class StateEncoder:
    """状态特征编码器
    
    将原始传感器数据编码为固定维度的特征向量，
    用于MLP网络输入。
    """
    
    def __init__(self, config: MLPConfig):
        self.config = config
        
        # 归一化参数
        self.pos_scale = max(config.field_length, config.field_width) / 2.0
        self.speed_scale = config.max_speed
        self.angle_scale = 180.0  # 角度归一化到 [-1, 1]
        self.dist_scale = max(config.field_length, config.field_width) / 2.0
    
    def encode(self, state: Union[RobotState, DeployObservationV1]) -> np.ndarray:
        """编码状态为特征向量
        
        支持两种输入类型:
        - RobotState: 传统状态结构 (向后兼容)
        - DeployObservationV1: 部署对齐观测契约 (推荐)
        
        Args:
            state: 机器人状态或部署观测
            
        Returns:
            归一化的特征向量，维度为 config.state_dim
        """
        # 判断输入类型并路由到对应的编码方法
        if isinstance(state, DeployObservationV1):
            return self._encode_observation_v1(state)
        else:
            return self._encode_robot_state(state)
    
    def _encode_robot_state(self, state: RobotState) -> np.ndarray:
        """编码传统 RobotState 为特征向量 (向后兼容)
        
        Args:
            state: 机器人状态
            
        Returns:
            归一化的特征向量，维度为 config.state_dim
        """
        features = []
        
        # 1. 自身位姿 (3维)
        features.extend([
            self._normalize_pos(state.x),
            self._normalize_pos(state.y),
            self._normalize_angle(state.yaw),
        ])
        
        # 2. 自身速度 (3维)
        features.extend([
            self._normalize_speed(state.vx),
            self._normalize_speed(state.vy),
            self._normalize_angular_speed(state.wz),
        ])
        
        # 3. 球位置（相对坐标） (2维)
        ball_dist = math.sqrt(state.ball_x**2 + state.ball_y**2)
        if state.has_ball:
            # 持球时，球位置为特殊值
            features.extend([0.0, 0.0])
        elif ball_dist > self.config.ball_detection_range:
            # 球超出检测范围，归一化为边界值
            features.extend([1.0, 1.0])  # 特殊标记
        else:
            features.extend([
                self._normalize_dist(state.ball_x),
                self._normalize_dist(state.ball_y),
            ])
        
        # 4. 持球状态 (1维)
        features.append(1.0 if state.has_ball else 0.0)
        
        # 5. 敌方机器人位置 (max_enemies × 2 维)
        enemy_features = self._encode_enemies(state.enemies)
        features.extend(enemy_features)
        
        # 6. 边界距离 (4维)
        features.extend([
            self._normalize_dist(state.wall_dist[0] / 10.0),  # mm -> cm
            self._normalize_dist(state.wall_dist[1] / 10.0),
            self._normalize_dist(state.wall_dist[2] / 10.0),
            self._normalize_dist(state.wall_dist[3] / 10.0),
        ])
        
        # 7. 球门方向（归一化向量）(2维)
        goal_dx = self.config.goal_x - state.x
        goal_dy = self.config.goal_y - state.y
        goal_dist = math.sqrt(goal_dx**2 + goal_dy**2)
        if goal_dist > 0:
            features.extend([
                goal_dx / self.pos_scale,
                goal_dy / self.pos_scale,
            ])
        else:
            features.extend([0.0, 0.0])
        
        return self._finalize_legacy_features(features)
    
    def _encode_observation_v1(self, obs: DeployObservationV1) -> np.ndarray:
        """编码部署对齐观测契约 V1 为特征向量
        
        这是 actor 应该使用的主要编码方法。
        所有字段都来自部署对齐的观测契约，确保训练与部署一致。
        
        Args:
            obs: 部署对齐观测契约 V1
            
        Returns:
            归一化的特征向量，维度为 config.state_dim
        """
        features = []
        
        # 1. 自身位姿 (3维) - 来自 FieldRobotPos
        features.extend([
            self._normalize_pos(obs.self_x),
            self._normalize_pos(obs.self_y),
            self._normalize_angle(obs.self_yaw),
        ])
        
        # 2. 自身速度 (3维) - 来自 IL / chassis / estimator
        # 注意: 速度数据可能无效，需要检查
        if obs.velocity_valid:
            features.extend([
                self._normalize_speed(obs.self_vx),
                self._normalize_speed(obs.self_vy),
                self._normalize_angular_speed(obs.self_wz),
            ])
        else:
            features.extend([0.0, 0.0, 0.0])
        
        # 3. 球位置（相对坐标）(2维) - 来自 RobotBallPos
        if obs.has_ball:
            # 持球时，球位置为特殊值
            features.extend([0.0, 0.0])
        elif not obs.ball_visible:
            # 球不可见，使用特殊标记
            features.extend([1.0, 1.0])
        else:
            features.extend([
                self._normalize_dist(obs.ball_rel_x),
                self._normalize_dist(obs.ball_rel_y),
            ])
        
        # 4. 持球状态 (1维)
        features.append(1.0 if obs.has_ball else 0.0)
        
        # 5. 球可见性/年龄/置信度 (3维) - 视觉质量字段
        features.append(1.0 if obs.ball_visible else 0.0)
        # 数据年龄归一化 (假设最大 1000ms)
        features.append(min(obs.ball_age_ms / 1000.0, 1.0) if obs.ball_age_ms >= 0 else 1.0)
        features.append(float(np.clip(obs.ball_confidence, 0.0, 1.0)))
        
        # 6. 敌方机器人位置 (max_enemies × 5 维) - 包含可见性/年龄/置信度
        # 固定使用 config.max_enemies，确保输入维度由模型配置决定，而不是由单次观测决定。
        enemy_features = self._encode_enemies_v1(obs.enemies, self.config.max_enemies)
        features.extend(enemy_features)
        
        # 7. 边界距离 (4维) - 来自 DirectDistance
        if obs.wall_valid:
            features.extend([
                self._normalize_dist(obs.wall_front),
                self._normalize_dist(obs.wall_left),
                self._normalize_dist(obs.wall_back),
                self._normalize_dist(obs.wall_right),
            ])
        else:
            features.extend([0.0, 0.0, 0.0, 0.0])
        
        # 8. 球门方向（归一化向量）(2维)
        goal_dx = self.config.goal_x - obs.self_x
        goal_dy = self.config.goal_y - obs.self_y
        goal_dist = math.sqrt(goal_dx**2 + goal_dy**2)
        if goal_dist > 0:
            features.extend([
                goal_dx / self.pos_scale,
                goal_dy / self.pos_scale,
            ])
        else:
            features.extend([0.0, 0.0])
        
        # 9. 比赛状态 (3维) - 占位
        features.extend([
            obs.referee_state / 10.0,  # 归一化
            obs.penalty_state / 10.0,
            obs.role / 10.0,
        ])
        
        # 10. 历史信息 (3维) - 占位
        features.extend([
            obs.last_decision_id / 10.0,
            obs.last_action_age_ms / 1000.0,
            1.0 if obs.stuck_flag else 0.0,
        ])
        
        return self._finalize_contract_features(features)

    def _finalize_legacy_features(self, features: List[float]) -> np.ndarray:
        """Finalize legacy RobotState features.

        Legacy paths stay permissive for backward compatibility: they are padded
        or truncated to the configured width.
        """
        features_array = np.array(features, dtype=np.float32)
        if len(features_array) > self.config.state_dim:
            features_array = features_array[:self.config.state_dim]
        elif len(features_array) < self.config.state_dim:
            padding = np.zeros(self.config.state_dim - len(features_array), dtype=np.float32)
            features_array = np.concatenate([features_array, padding])
        return features_array

    def _finalize_contract_features(self, features: List[float]) -> np.ndarray:
        """Finalize DeployObservationV1 features.

        Contract-driven paths must never silently truncate. If the configured
        model width diverges from the contract width, fail fast so training and
        deployment do not proceed with a partially dropped observation.
        """
        features_array = np.array(features, dtype=np.float32)
        expected_dim = self.get_observation_dim()
        actual_dim = len(features_array)

        if actual_dim != expected_dim:
            raise ValueError(
                f"DeployObservationV1 encoded dim mismatch: actual={actual_dim}, "
                f"expected={expected_dim}"
            )

        if self.config.state_dim != expected_dim:
            raise ValueError(
                f"MLPConfig.state_dim ({self.config.state_dim}) does not match "
                f"DeployObservationV1 encoded dim ({expected_dim})."
            )

        return features_array
    
    def _encode_enemies_v1(self, enemies: List[EntityObservationV1], max_enemies: int) -> List[float]:
        """编码敌方观测 (V1 契约版本)
        
        包含位置、可见性、数据年龄等信息。
        
        Args:
            enemies: 敌方观测列表
            max_enemies: 最大敌方数量
            
        Returns:
            归一化的敌方特征列表 (每个敌人 5 维: rel_x, rel_y, visible, age, confidence)
        """
        features = []
        
        for i in range(max_enemies):
            if i < len(enemies) and enemies[i].visible:
                enemy = enemies[i]
                features.extend([
                    self._normalize_dist(enemy.rel_x),
                    self._normalize_dist(enemy.rel_y),
                    1.0,  # visible
                    min(enemy.age_ms / 1000.0, 1.0) if enemy.age_ms >= 0 else 1.0,
                    float(np.clip(enemy.confidence, 0.0, 1.0)),
                ])
            elif i < len(enemies):
                # 有数据但不可见
                enemy = enemies[i]
                features.extend([
                    self._normalize_dist(enemy.rel_x),
                    self._normalize_dist(enemy.rel_y),
                    0.0,  # not visible
                    min(enemy.age_ms / 1000.0, 1.0) if enemy.age_ms >= 0 else 1.0,
                    float(np.clip(enemy.confidence, 0.0, 1.0)),
                ])
            else:
                # 填充无效位置 (5维)
                features.extend([2.0, 2.0, 0.0, 1.0, 0.0])  # 特殊值表示无效
        
        return features
    
    def get_observation_dim(self) -> int:
        """获取 DeployObservationV1 编码后的特征维度
        
        用于验证 config.state_dim 是否正确配置。
        这里返回的是“当前 encoder 实际编码进去的字段维度”，而不是
        契约 dataclass 所有字段的总数。
        
        Returns:
            理论特征维度
        """
        # 计算理论维度
        dim = 0
        dim += 3   # 自身位姿
        dim += 3   # 自身速度
        dim += 2   # 球位置
        dim += 1   # 持球状态
        dim += 3   # 球可见性/年龄/置信度
        dim += self.config.max_enemies * 5  # 敌方 (位置 + 可见性 + 年龄 + 置信度)
        dim += 4   # 边界距离
        dim += 2   # 球门方向
        dim += 3   # 比赛状态
        dim += 3   # 历史信息
        return dim
    
    def _encode_enemies(self, enemies: List[Tuple[float, float]]) -> List[float]:
        """编码敌方机器人位置
        
        Args:
            enemies: 敌方位置列表 [(x1, y1), (x2, y2), ...]
            
        Returns:
            归一化的敌方位置特征列表
        """
        features = []
        max_enemies = self.config.max_enemies
        
        for i in range(max_enemies):
            if i < len(enemies):
                ex, ey = enemies[i]
                features.extend([
                    self._normalize_pos(ex),
                    self._normalize_pos(ey),
                ])
            else:
                # 填充无效位置
                features.extend([2.0, 2.0])  # 特殊值表示无效
        
        return features
    
    def _normalize_pos(self, value: float) -> float:
        """归一化位置到 [-1, 1]"""
        return np.clip(value / self.pos_scale, -1.0, 1.0)
    
    def _normalize_speed(self, value: float) -> float:
        """归一化速度到 [-1, 1]"""
        return np.clip(value / self.speed_scale, -1.0, 1.0)
    
    def _normalize_angle(self, value: float) -> float:
        """归一化角度到 [-1, 1]"""
        # 先归一化到 [-180, 180]
        value = ((value + 180.0) % 360.0) - 180.0
        return value / self.angle_scale
    
    def _normalize_angular_speed(self, value: float) -> float:
        """归一化角速度到 [-1, 1]"""
        return np.clip(value / self.config.max_angular_speed, -1.0, 1.0)
    
    def _normalize_dist(self, value: float) -> float:
        """归一化距离到 [-1, 1]"""
        return np.clip(value / self.dist_scale, -1.0, 1.0)
    
    def decode_action(self, action: np.ndarray) -> Dict[str, float]:
        """解码网络输出为动作指令
        
        Args:
            action: 网络输出向量 [vx, vy, wz, kick, dribble]
            
        Returns:
            动作字典 {'vx': float, 'vy': float, 'wz': float, 'kick': bool, 'dribble': bool}
        """
        # 确保action是正确维度
        if len(action) < 5:
            action = np.concatenate([action, np.zeros(5 - len(action))])
        
        # 速度分量（反归一化）
        vx = float(action[0]) * self.config.max_speed
        vy = float(action[1]) * self.config.max_speed
        wz = float(action[2]) * self.config.max_angular_speed
        
        # 离散动作（使用阈值）
        kick = float(action[3]) > 0.5
        dribble = float(action[4]) > 0.5
        
        return {
            'vx': vx,
            'vy': vy,
            'wz': wz,
            'kick': kick,
            'dribble': dribble,
        }

    def decode_decision_packet(self, action: np.ndarray) -> DecisionPacketV1:
        """Decode native decision-packet action vector.

        Layout:
            [target_x, target_y, target_yaw, move_gain, horizon, confidence,
             intent_id, face_mode, dribble_mode, shoot_intent]

        Continuous values are normalized to [-1, 1] by the model and mapped
        back to practical field/control units here.
        """
        action = np.asarray(action, dtype=np.float32)
        required = self.config.decision_continuous_dim + 4
        if len(action) < required:
            action = np.concatenate([action, np.zeros(required - len(action), dtype=np.float32)])

        target_x = float(np.clip(action[0], -1.0, 1.0) * self.pos_scale)
        target_y = float(np.clip(action[1], -1.0, 1.0) * self.pos_scale)
        target_yaw = float(np.clip(action[2], -1.0, 1.0) * 180.0)
        move_gain = float((np.clip(action[3], -1.0, 1.0) + 1.0) * 0.5)
        horizon01 = float((np.clip(action[4], -1.0, 1.0) + 1.0) * 0.5)
        confidence = float((np.clip(action[5], -1.0, 1.0) + 1.0) * 0.5)
        decision_horizon_ms = int(round(50 + horizon01 * 450))

        intent_id = self._clamp_class(action[6], len(IntentId))
        face_mode = self._clamp_class(action[7], len(FaceMode))
        dribble_mode = self._clamp_class(action[8], len(DribbleMode))
        shoot_intent = self._clamp_class(action[9], len(ShootIntent))

        return DecisionPacketV1(
            intent_id=int(intent_id),
            target_x=target_x,
            target_y=target_y,
            target_yaw=target_yaw,
            face_mode=int(face_mode),
            move_gain=move_gain,
            dribble_mode=int(dribble_mode),
            shoot_intent=int(shoot_intent),
            decision_horizon_ms=decision_horizon_ms,
            confidence=confidence,
        )

    @staticmethod
    def _clamp_class(value: float, class_count: int) -> int:
        try:
            idx = int(round(float(value)))
        except Exception:
            idx = 0
        return max(0, min(max(0, class_count - 1), idx))
    
    @staticmethod
    def from_lunapre(state_dict: Dict[str, Any]) -> RobotState:
        """从LunaPre状态字典创建RobotState
        
        Args:
            state_dict: 从UnitedPosition获取的状态字典
            
        Returns:
            RobotState实例
        """
        # 获取位置信息
        try:
            from LunaPre import UnitedPosition
            pos = UnitedPosition.FieldRobotPos()
            x, y, yaw = pos[0], pos[1], pos[2]
        except:
            x, y, yaw = 0.0, 0.0, 0.0
        
        # 获取球位置
        try:
            from LunaPre import UnitedPosition
            ball_pos = UnitedPosition.RobotBallPos()
            ball_x, ball_y = ball_pos[0], ball_pos[1]
            has_ball = (ball_pos == [0xDDD, 0xDDD])
        except:
            ball_x, ball_y = 0.0, 0.0
            has_ball = False
        
        # 获取边界距离
        try:
            from LunaPre import UnitedPosition
            wall_dist = UnitedPosition.DirectDistance()  # [front, left, back, right] in mm
        except:
            wall_dist = [0.0, 0.0, 0.0, 0.0]
        
        # 敌方位置（目前仿真器未实现，返回空列表）
        enemies = []
        try:
            from LunaPre import UnitedPosition
            chassis_pos = UnitedPosition.FieldChassisPos()
            if chassis_pos:
                enemies = [(c[0], c[1]) for c in chassis_pos]
        except:
            pass
        
        return RobotState(
            x=x, y=y, yaw=yaw,
            vx=state_dict.get('vx', 0.0),
            vy=state_dict.get('vy', 0.0),
            wz=state_dict.get('wz', 0.0),
            ball_x=ball_x, ball_y=ball_y,
            has_ball=has_ball,
            enemies=enemies,
            wall_dist=list(wall_dist) if wall_dist else [0.0, 0.0, 0.0, 0.0],
        )
