"""状态特征编码器

将多源传感器数据编码为MLP输入向量。
"""

import math
import numpy as np
from typing import List, Optional, Tuple, Dict, Any
from dataclasses import dataclass

from .config import MLPConfig


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
    
    def encode(self, state: RobotState) -> np.ndarray:
        """编码状态为特征向量
        
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
        
        # 转换为numpy数组并确保维度正确
        features_array = np.array(features, dtype=np.float32)
        
        # 裁剪或填充到指定维度
        if len(features_array) > self.config.state_dim:
            features_array = features_array[:self.config.state_dim]
        elif len(features_array) < self.config.state_dim:
            padding = np.zeros(self.config.state_dim - len(features_array), dtype=np.float32)
            features_array = np.concatenate([features_array, padding])
        
        return features_array
    
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