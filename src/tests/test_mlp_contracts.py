"""MLP 契约模块测试

测试内容:
- dataclass 创建
- JSON round-trip (序列化/反序列化)
- observation -> encoder 输入维度正确

参考: docs/MLP_POLICY_ROADMAP.md
"""

import pytest
import numpy as np
import sys
import os

# 添加 src 目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mlp.contracts import (
    DeployObservationV1,
    TruthStateV1,
    DecisionPacketV1,
    EpisodeStatusV1,
    ResetScenarioV1,
    EntityObservationV1,
    GoalObservationV1,
    IntentId,
    FaceMode,
    DribbleMode,
    ShootIntent,
    RefereeState,
    RoleId,
    observation_from_simulator,
)
from mlp.state_encoder import StateEncoder, RobotState
from mlp.config import MLPConfig


class TestEntityObservationV1:
    """测试 EntityObservationV1 数据结构"""
    
    def test_create_default(self):
        """测试默认创建"""
        entity = EntityObservationV1()
        assert entity.rel_x == 0.0
        assert entity.rel_y == 0.0
        assert entity.visible == False
        assert entity.age_ms == -1
        assert entity.confidence == 0.0
    
    def test_create_with_values(self):
        """测试带值创建"""
        entity = EntityObservationV1(
            rel_x=100.0,
            rel_y=50.0,
            visible=True,
            age_ms=100,
            confidence=0.9,
        )
        assert entity.rel_x == 100.0
        assert entity.rel_y == 50.0
        assert entity.visible == True
        assert entity.age_ms == 100
        assert entity.confidence == 0.9
    
    def test_to_dict(self):
        """测试转换为字典"""
        entity = EntityObservationV1(rel_x=10.0, rel_y=20.0, visible=True)
        d = entity.to_dict()
        assert d['rel_x'] == 10.0
        assert d['rel_y'] == 20.0
        assert d['visible'] == True
    
    def test_from_dict(self):
        """测试从字典创建"""
        d = {'rel_x': 30.0, 'rel_y': 40.0, 'visible': False, 'age_ms': 50}
        entity = EntityObservationV1.from_dict(d)
        assert entity.rel_x == 30.0
        assert entity.rel_y == 40.0
        assert entity.visible == False
        assert entity.age_ms == 50
    
    def test_json_roundtrip(self):
        """测试 JSON 序列化/反序列化"""
        entity = EntityObservationV1(
            rel_x=100.0,
            rel_y=-50.0,
            visible=True,
            age_ms=200,
            confidence=0.85,
        )
        # 通过 dict 进行 round-trip
        d = entity.to_dict()
        entity2 = EntityObservationV1.from_dict(d)
        assert entity2.rel_x == entity.rel_x
        assert entity2.rel_y == entity.rel_y
        assert entity2.visible == entity.visible
        assert entity2.age_ms == entity.age_ms
        assert entity2.confidence == entity.confidence


class TestDeployObservationV1:
    """测试 DeployObservationV1 数据结构"""
    
    def test_create_default(self):
        """测试默认创建"""
        obs = DeployObservationV1()
        assert obs.self_x == 0.0
        assert obs.self_y == 0.0
        assert obs.self_yaw == 0.0
        assert obs.ball_visible == False
        assert obs.has_ball == False
        assert obs.enemies == []
        assert obs.max_enemies == 4
    
    def test_create_with_values(self):
        """测试带值创建"""
        enemies = [
            EntityObservationV1(rel_x=100.0, rel_y=50.0, visible=True),
            EntityObservationV1(rel_x=-200.0, rel_y=100.0, visible=False),
        ]
        obs = DeployObservationV1(
            self_x=150.0,
            self_y=200.0,
            self_yaw=45.0,
            ball_rel_x=30.0,
            ball_rel_y=40.0,
            ball_visible=True,
            has_ball=False,
            enemies=enemies,
        )
        assert obs.self_x == 150.0
        assert obs.self_y == 200.0
        assert obs.self_yaw == 45.0
        assert obs.ball_rel_x == 30.0
        assert obs.ball_rel_y == 40.0
        assert obs.ball_visible == True
        assert len(obs.enemies) == 2
        assert obs.enemies[0].rel_x == 100.0
    
    def test_to_dict(self):
        """测试转换为字典"""
        obs = DeployObservationV1(
            self_x=100.0,
            self_y=200.0,
            ball_visible=True,
            enemies=[EntityObservationV1(rel_x=50.0, visible=True)],
        )
        d = obs.to_dict()
        assert d['self_x'] == 100.0
        assert d['self_y'] == 200.0
        assert d['ball_visible'] == True
        assert len(d['enemies']) == 1
        assert d['enemies'][0]['rel_x'] == 50.0
    
    def test_from_dict(self):
        """测试从字典创建"""
        d = {
            'self_x': 300.0,
            'self_y': 400.0,
            'self_yaw': 90.0,
            'ball_rel_x': 50.0,
            'ball_rel_y': 60.0,
            'ball_visible': True,
            'ball_age_ms': 100,
            'has_ball': False,
            'enemies': [
                {'rel_x': 100.0, 'rel_y': 50.0, 'visible': True, 'age_ms': 50}
            ],
        }
        obs = DeployObservationV1.from_dict(d)
        assert obs.self_x == 300.0
        assert obs.self_y == 400.0
        assert obs.self_yaw == 90.0
        assert obs.ball_rel_x == 50.0
        assert obs.ball_visible == True
        assert len(obs.enemies) == 1
        assert obs.enemies[0].rel_x == 100.0
    
    def test_json_roundtrip(self):
        """测试 JSON 序列化/反序列化"""
        obs = DeployObservationV1(
            self_x=150.0,
            self_y=250.0,
            self_yaw=30.0,
            self_vx=50.0,
            self_vy=30.0,
            self_wz=10.0,
            velocity_valid=True,
            ball_rel_x=40.0,
            ball_rel_y=60.0,
            ball_visible=True,
            ball_age_ms=150,
            has_ball=False,
            wall_front=100.0,
            wall_left=50.0,
            wall_back=80.0,
            wall_right=60.0,
            wall_valid=True,
            enemies=[
                EntityObservationV1(rel_x=100.0, rel_y=50.0, visible=True, age_ms=100),
                EntityObservationV1(rel_x=-50.0, rel_y=30.0, visible=False, age_ms=200),
            ],
            referee_state=1,
            role=2,
        )
        
        # JSON round-trip
        json_str = obs.to_json()
        obs2 = DeployObservationV1.from_json(json_str)
        
        assert obs2.self_x == obs.self_x
        assert obs2.self_y == obs.self_y
        assert obs2.self_yaw == obs.self_yaw
        assert obs2.velocity_valid == obs.velocity_valid
        assert obs2.ball_visible == obs.ball_visible
        assert obs2.ball_age_ms == obs.ball_age_ms
        assert obs2.wall_valid == obs.wall_valid
        assert len(obs2.enemies) == len(obs.enemies)
        assert obs2.enemies[0].rel_x == obs.enemies[0].rel_x
        assert obs2.enemies[1].visible == obs.enemies[1].visible
    
    def test_nested_structures(self):
        """测试嵌套结构 (球门、队友)"""
        obs = DeployObservationV1(
            blue_goal=GoalObservationV1(rel_x=100.0, rel_y=50.0, visible=True),
            yellow_goal=GoalObservationV1(rel_x=-100.0, rel_y=-50.0, visible=False),
            teammates=[
                EntityObservationV1(rel_x=200.0, rel_y=100.0, visible=True),
            ],
        )
        
        json_str = obs.to_json()
        obs2 = DeployObservationV1.from_json(json_str)
        
        assert obs2.blue_goal.rel_x == 100.0
        assert obs2.blue_goal.visible == True
        assert obs2.yellow_goal.rel_x == -100.0
        assert obs2.yellow_goal.visible == False
        assert len(obs2.teammates) == 1


class TestTruthStateV1:
    """测试 TruthStateV1 数据结构"""
    
    def test_create_default(self):
        """测试默认创建"""
        truth = TruthStateV1()
        assert truth.ball_true_x == 0.0
        assert truth.episode_id == 0
        assert truth.is_goal == False
    
    def test_create_with_values(self):
        """测试带值创建"""
        truth = TruthStateV1(
            ball_true_x=100.0,
            ball_true_y=200.0,
            ball_true_vx=50.0,
            ball_true_vy=30.0,
            episode_id=5,
            elapsed_ms=3000,
            is_goal=True,
        )
        assert truth.ball_true_x == 100.0
        assert truth.episode_id == 5
        assert truth.is_goal == True
    
    def test_json_roundtrip(self):
        """测试 JSON round-trip"""
        truth = TruthStateV1(
            ball_true_x=150.0,
            ball_true_y=250.0,
            enemies_true=[{'x': 100.0, 'y': 50.0, 'vx': 10.0}],
            episode_id=10,
            is_timeout=True,
        )
        json_str = truth.to_json()
        truth2 = TruthStateV1.from_json(json_str)
        assert truth2.ball_true_x == 150.0
        assert truth2.episode_id == 10
        assert truth2.is_timeout == True
        assert len(truth2.enemies_true) == 1


class TestDecisionPacketV1:
    """测试 DecisionPacketV1 数据结构"""
    
    def test_create_default(self):
        """测试默认创建"""
        decision = DecisionPacketV1()
        assert decision.intent_id == 0
        assert decision.target_x == 0.0
        assert decision.move_gain == 1.0
    
    def test_create_with_intent(self):
        """测试带意图创建"""
        decision = DecisionPacketV1(
            intent_id=IntentId.APPROACH_BALL,
            target_x=100.0,
            target_y=50.0,
            face_mode=FaceMode.FACE_BALL,
            dribble_mode=DribbleMode.WEAK,
        )
        assert decision.intent_id == IntentId.APPROACH_BALL
        assert decision.target_x == 100.0
        assert decision.face_mode == FaceMode.FACE_BALL
    
    def test_json_roundtrip(self):
        """测试 JSON round-trip"""
        decision = DecisionPacketV1(
            intent_id=IntentId.SHOOT,
            target_x=200.0,
            target_y=100.0,
            target_yaw=45.0,
            face_mode=FaceMode.FACE_GOAL,
            move_gain=0.8,
            dribble_mode=DribbleMode.OFF,
            shoot_intent=ShootIntent.TRY_SHOOT,
            decision_horizon_ms=200,
        )
        json_str = decision.to_json()
        decision2 = DecisionPacketV1.from_json(json_str)
        assert decision2.intent_id == IntentId.SHOOT
        assert decision2.target_x == 200.0
        assert decision2.shoot_intent == ShootIntent.TRY_SHOOT


class TestEpisodeStatusV1:
    """测试 EpisodeStatusV1 数据结构"""
    
    def test_create_default(self):
        """测试默认创建"""
        status = EpisodeStatusV1()
        assert status.episode_id == 0
        assert status.is_running == False
        assert status.is_done == False
    
    def test_create_with_values(self):
        """测试带值创建"""
        status = EpisodeStatusV1(
            episode_id=100,
            is_running=True,
            elapsed_ms=5000,
        )
        assert status.episode_id == 100
        assert status.is_running == True
        assert status.elapsed_ms == 5000
    
    def test_json_roundtrip(self):
        """测试 JSON round-trip"""
        status = EpisodeStatusV1(
            episode_id=50,
            is_done=True,
            result=1,
            end_reason="goal",
        )
        json_str = status.to_json()
        status2 = EpisodeStatusV1.from_json(json_str)
        assert status2.episode_id == 50
        assert status2.is_done == True
        assert status2.end_reason == "goal"


class TestResetScenarioV1:
    """测试 ResetScenarioV1 数据结构"""
    
    def test_create_default(self):
        """测试默认创建"""
        scenario = ResetScenarioV1()
        assert scenario.seed == 0
        assert scenario.self_pose['x'] == 0.0
        assert scenario.enemies == []
    
    def test_create_with_values(self):
        """测试带值创建"""
        scenario = ResetScenarioV1(
            seed=42,
            self_pose={'x': 100.0, 'y': 200.0, 'yaw': 45.0},
            ball_state={'x': 150.0, 'y': 250.0, 'vx': 10.0, 'vy': 5.0},
            enemies=[
                {'x': 300.0, 'y': 400.0, 'yaw': 0.0},
            ],
        )
        assert scenario.seed == 42
        assert scenario.self_pose['x'] == 100.0
        assert scenario.ball_state['x'] == 150.0
        assert len(scenario.enemies) == 1
    
    def test_json_roundtrip(self):
        """测试 JSON round-trip"""
        scenario = ResetScenarioV1(
            seed=123,
            self_pose={'x': 50.0, 'y': 100.0, 'yaw': 30.0},
            ball_state={'x': 200.0, 'y': 300.0, 'vx': 20.0, 'vy': 10.0},
            enemies=[
                {'x': 400.0, 'y': 500.0, 'yaw': 90.0, 'vx': 5.0, 'vy': 5.0},
            ],
            teammates=[
                {'x': 100.0, 'y': 150.0, 'yaw': 0.0},
            ],
            field_side=1,
        )
        json_str = scenario.to_json()
        scenario2 = ResetScenarioV1.from_json(json_str)
        assert scenario2.seed == 123
        assert scenario2.self_pose['x'] == 50.0
        assert scenario2.ball_state['vx'] == 20.0
        assert len(scenario2.enemies) == 1
        assert len(scenario2.teammates) == 1


class TestObservationFromSimulator:
    """测试 observation_from_simulator 辅助函数"""
    
    def test_basic_creation(self):
        """测试基本创建"""
        obs = observation_from_simulator(
            self_x=100.0,
            self_y=200.0,
            self_yaw=45.0,
            ball_x=30.0,
            ball_y=40.0,
            has_ball=False,
            wall_dist=[1000.0, 500.0, 800.0, 600.0],  # mm
        )
        assert obs.self_x == 100.0
        assert obs.self_y == 200.0
        assert obs.ball_rel_x == 30.0
        assert obs.ball_rel_y == 40.0
        assert obs.has_ball == False
        assert obs.ball_visible == True
        # wall_dist 转换为 cm
        assert obs.wall_front == 100.0  # 1000mm -> 100cm
        assert obs.wall_left == 50.0
        assert obs.wall_valid == True
    
    def test_with_enemies(self):
        """测试带敌方"""
        obs = observation_from_simulator(
            self_x=100.0,
            self_y=200.0,
            self_yaw=0.0,
            ball_x=0.0,
            ball_y=0.0,
            has_ball=True,
            wall_dist=[0, 0, 0, 0],
            enemies=[(300.0, 400.0), (500.0, 600.0)],
        )
        assert obs.has_ball == True
        assert obs.ball_visible == False
        assert len(obs.enemies) == 2
        # 敌方输入已经是相对机器人坐标，不应再减 self_x/self_y
        assert obs.enemies[0].rel_x == 300.0
        assert obs.enemies[0].rel_y == 400.0
    
    def test_ball_not_visible(self):
        """测试球不可见"""
        obs = observation_from_simulator(
            self_x=0.0,
            self_y=0.0,
            self_yaw=0.0,
            ball_x=0.0,
            ball_y=0.0,
            has_ball=False,
            wall_dist=[0, 0, 0, 0],
        )
        assert obs.ball_visible == False
        assert obs.ball_age_ms == -1


class TestStateEncoderWithContracts:
    """测试 StateEncoder 与契约的集成"""
    
    @pytest.fixture
    def encoder(self):
        """创建编码器实例"""
        config = MLPConfig()
        return StateEncoder(config)

    def assert_deploy_observation_dim_matches_config(self, encoder):
        expected_dim = encoder.get_observation_dim()
        actual_dim = encoder.config.state_dim
        assert actual_dim == expected_dim, (
            f"MLPConfig.state_dim={actual_dim} does not match "
            f"DeployObservationV1 theoretical dim={expected_dim}; "
            "encoding would otherwise be silently truncated or padded."
        )
    
    def test_encode_robot_state_backward_compat(self, encoder):
        """测试向后兼容 RobotState 编码"""
        state = RobotState(
            x=100.0,
            y=200.0,
            yaw=45.0,
            ball_x=30.0,
            ball_y=40.0,
            has_ball=False,
        )
        features = encoder.encode(state)
        assert features.shape[0] == encoder.config.state_dim
        assert features.dtype == np.float32
    
    def test_encode_deploy_observation_v1(self, encoder):
        """测试 DeployObservationV1 编码"""
        obs = DeployObservationV1(
            self_x=150.0,
            self_y=250.0,
            self_yaw=30.0,
            self_vx=50.0,
            self_vy=30.0,
            self_wz=10.0,
            velocity_valid=True,
            ball_rel_x=40.0,
            ball_rel_y=60.0,
            ball_visible=True,
            ball_age_ms=100,
            has_ball=False,
            wall_front=100.0,
            wall_left=50.0,
            wall_back=80.0,
            wall_right=60.0,
            wall_valid=True,
            enemies=[
                EntityObservationV1(rel_x=100.0, rel_y=50.0, visible=True),
            ],
        )
        features = encoder.encode(obs)
        assert features.shape[0] == encoder.config.state_dim
        assert features.dtype == np.float32
    
    def test_encode_observation_dimension(self, encoder):
        """测试编码输出维度正确"""
        # 计算理论维度
        expected_dim = encoder.get_observation_dim()
        
        # 创建完整观测
        obs = DeployObservationV1(
            self_x=100.0,
            self_y=200.0,
            self_yaw=45.0,
            velocity_valid=True,
            ball_visible=True,
            wall_valid=True,
            enemies=[
                EntityObservationV1(rel_x=50.0, rel_y=30.0, visible=True)
                for _ in range(encoder.config.max_enemies)
            ],
        )
        
        features = encoder.encode(obs)

        # 契约维度、配置维度、实际编码维度必须严格一致。
        assert expected_dim == encoder.config.state_dim
        assert features.shape[0] == expected_dim
        assert not np.any(np.isnan(features))

    def test_contract_dim_guard_uses_current_config(self, encoder):
        """测试当前配置与 DeployObservationV1 编码维度严格对齐。"""
        assert encoder.get_observation_dim() == MLPConfig().state_dim
    
    def test_encode_with_velocity_invalid(self, encoder):
        """测试速度无效时的编码"""
        obs = DeployObservationV1(
            self_x=100.0,
            self_y=200.0,
            self_yaw=45.0,
            self_vx=50.0,
            self_vy=30.0,
            self_wz=10.0,
            velocity_valid=False,  # 速度无效
        )
        features = encoder.encode(obs)
        # 速度部分应该为 0
        # 位置 3维 + 速度 3维 (速度无效时为 0)
        assert features.shape[0] == encoder.config.state_dim
    
    def test_encode_with_ball_not_visible(self, encoder):
        """测试球不可见时的编码"""
        obs = DeployObservationV1(
            self_x=100.0,
            self_y=200.0,
            self_yaw=45.0,
            ball_rel_x=0.0,
            ball_rel_y=0.0,
            ball_visible=False,
            ball_age_ms=500,
            has_ball=False,
        )
        features = encoder.encode(obs)
        assert features.shape[0] == encoder.config.state_dim
    
    def test_encode_with_has_ball(self, encoder):
        """测试持球时的编码"""
        obs = DeployObservationV1(
            self_x=100.0,
            self_y=200.0,
            self_yaw=45.0,
            ball_rel_x=0.0,
            ball_rel_y=0.0,
            has_ball=True,
        )
        features = encoder.encode(obs)
        assert features.shape[0] == encoder.config.state_dim


class TestEnums:
    """测试枚举定义"""
    
    def test_intent_id_values(self):
        """测试 IntentId 枚举值"""
        assert IntentId.SEARCH_BALL == 0
        assert IntentId.APPROACH_BALL == 1
        assert IntentId.SHOOT == 4
        assert IntentId.SUPPORT_MATE == 9
    
    def test_face_mode_values(self):
        """测试 FaceMode 枚举值"""
        assert FaceMode.FACE_BALL == 0
        assert FaceMode.FACE_GOAL == 1
        assert FaceMode.FACE_TARGET == 2
        assert FaceMode.FACE_KEEP == 3
    
    def test_dribble_mode_values(self):
        """测试 DribbleMode 枚举值"""
        assert DribbleMode.OFF == 0
        assert DribbleMode.WEAK == 1
        assert DribbleMode.STRONG == 2
    
    def test_referee_state_values(self):
        """测试 RefereeState 枚举值"""
        assert RefereeState.NORMAL == 0
        assert RefereeState.KICKOFF == 1
        assert RefereeState.GOAL == 4


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
