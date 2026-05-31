"""教师模型数据采集模块

自动采集教师决策轨迹，用于行为克隆训练。
支持：
- 多回合自动重置
- 状态-动作对记录
- 奖励计算
- 轨迹保存
"""

from __future__ import annotations

import os
import sys
import json
import time
import math
import random
from dataclasses import dataclass, asdict, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
from datetime import datetime

import numpy as np

# 设置环境变量
os.environ["ROBOCUPX_BITS"] = "SIM"
os.environ["ROBOCUPX_USE_MOCK"] = "0"

from LunaPre import UnitedPosition, chassis, peripheral
from BasicFuc import lockball

from teacher.rule_teacher import RuleTeacher, TeacherConfig, TeacherDecision
from mlp.contracts import (
    DeployObservationV1,
    EntityObservationV1,
    DecisionPacketV1,
    IntentId,
    FaceMode,
    DribbleMode,
    ShootIntent,
)
from mlp.skill_executor import SkillExecutor
from mlp.state_encoder import StateEncoder
from mlp.config import MLPConfig


@dataclass
class Transition:
    """单个状态转移"""
    # 状态
    state: np.ndarray  # 编码后的状态向量
    raw_obs: Dict[str, Any]  # 原始观测 (用于调试)
    
    # 动作
    action: np.ndarray  # 编码后的动作向量
    raw_packet: Dict[str, Any]  # 原始决策包
    
    # 奖励和终止
    reward: float
    done: bool
    
    # 元信息
    step: int
    episode: int
    timestamp: float
    
    # 教师信息
    teacher_state: str
    teacher_reasons: List[str]
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "state": self.state.tolist(),
            "raw_obs": self.raw_obs,
            "action": self.action.tolist(),
            "raw_packet": self.raw_packet,
            "reward": self.reward,
            "done": self.done,
            "step": self.step,
            "episode": self.episode,
            "timestamp": self.timestamp,
            "teacher_state": self.teacher_state,
            "teacher_reasons": self.teacher_reasons,
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Transition":
        return cls(
            state=np.array(data["state"], dtype=np.float32),
            raw_obs=data["raw_obs"],
            action=np.array(data["action"], dtype=np.float32),
            raw_packet=data["raw_packet"],
            reward=data["reward"],
            done=data["done"],
            step=data["step"],
            episode=data["episode"],
            timestamp=data["timestamp"],
            teacher_state=data["teacher_state"],
            teacher_reasons=data["teacher_reasons"],
        )


@dataclass
class Episode:
    """单个回合数据"""
    episode_id: int
    transitions: List[Transition] = field(default_factory=list)
    total_reward: float = 0.0
    total_steps: int = 0
    outcome: str = "running"  # running, goal, out_of_bounds, timeout
    start_time: float = 0.0
    end_time: float = 0.0
    
    def add_transition(self, t: Transition):
        self.transitions.append(t)
        self.total_reward += t.reward
        self.total_steps = t.step
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "episode_id": self.episode_id,
            "transitions": [t.to_dict() for t in self.transitions],
            "total_reward": self.total_reward,
            "total_steps": self.total_steps,
            "outcome": self.outcome,
            "start_time": self.start_time,
            "end_time": self.end_time,
        }


@dataclass
class CollectionStats:
    """采集统计"""
    total_episodes: int = 0
    total_transitions: int = 0
    total_goals: int = 0
    total_out_of_bounds: int = 0
    total_timeouts: int = 0
    avg_episode_reward: float = 0.0
    avg_episode_length: float = 0.0
    goal_rate: float = 0.0
    
    def update(self, episode: Episode):
        self.total_episodes += 1
        self.total_transitions += len(episode.transitions)
        
        if episode.outcome == "goal":
            self.total_goals += 1
        elif episode.outcome == "out_of_bounds":
            self.total_out_of_bounds += 1
        elif episode.outcome == "timeout":
            self.total_timeouts += 1
        
        # 更新平均值
        n = self.total_episodes
        self.avg_episode_reward = (self.avg_episode_reward * (n - 1) + episode.total_reward) / n
        self.avg_episode_length = (self.avg_episode_length * (n - 1) + episode.total_steps) / n
        self.goal_rate = self.total_goals / n if n > 0 else 0.0
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class DataCollector:
    """数据采集器"""
    
    def __init__(
        self,
        mlp_config: MLPConfig = None,
        teacher_config: TeacherConfig = None,
        output_dir: str = "Records/teacher_trajectories",
    ):
        self.mlp_config = mlp_config or MLPConfig()
        self.teacher_config = teacher_config or TeacherConfig()
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # 初始化组件
        self.teacher = RuleTeacher(self.teacher_config)
        self.executor = SkillExecutor()
        self.encoder = StateEncoder(self.mlp_config)
        
        # 统计
        self.stats = CollectionStats()
        self.episodes: List[Episode] = []
        self.current_episode: Optional[Episode] = None
        
        # 奖励参数
        self.reward_goal = 100.0
        self.reward_has_ball = 0.1
        self.reward_approach_ball = 0.01
        self.reward_out_of_bounds = -50.0
        self.reward_time_penalty = -0.001
        
        # 状态跟踪
        self._last_ball_dist = 200.0
        self._last_has_ball = False
        self._max_episode_steps = 1000
        self._episode_timeout_s = 60.0
    
    def get_observation(self) -> DeployObservationV1:
        """获取当前观测"""
        obs = DeployObservationV1()
        
        try:
            # 自身位姿
            pos = UnitedPosition.FieldRobotPos()
            if pos and len(pos) >= 3:
                obs.self_x = float(pos[0])
                obs.self_y = float(pos[1])
                obs.self_yaw = float(pos[2])
            
            # 球位置
            ball_pos = UnitedPosition.RobotBallPos()
            if ball_pos:
                bx, by = ball_pos[0], ball_pos[1]
                if (bx, by) == (0xFFF, 0xFFF):
                    obs.ball_visible = False
                    obs.has_ball = False
                elif (bx, by) == (0xDDD, 0xDDD):
                    obs.has_ball = True
                    obs.ball_visible = True
                    obs.ball_rel_x = 0.0
                    obs.ball_rel_y = 0.0
                else:
                    obs.ball_rel_x = float(bx)
                    obs.ball_rel_y = float(by)
                    obs.ball_visible = True
                    obs.has_ball = False
            
            # 边界距离
            wall_dist = UnitedPosition.DirectDistance()
            if wall_dist and len(wall_dist) >= 4:
                if all(d != 0xFFF for d in wall_dist[:4]):
                    obs.wall_front = float(wall_dist[0]) / 10.0
                    obs.wall_left = float(wall_dist[1]) / 10.0
                    obs.wall_back = float(wall_dist[2]) / 10.0
                    obs.wall_right = float(wall_dist[3]) / 10.0
                    obs.wall_valid = True
            
            # 敌方位置
            enemies_raw = UnitedPosition.FieldChassisPos()
            if enemies_raw:
                for enemy_pos in enemies_raw[:obs.max_enemies]:
                    if enemy_pos and len(enemy_pos) >= 2:
                        rel_x = float(enemy_pos[0]) - obs.self_x
                        rel_y = float(enemy_pos[1]) - obs.self_y
                        enemy = EntityObservationV1(
                            rel_x=rel_x,
                            rel_y=rel_y,
                            visible=True,
                            age_ms=0,
                            confidence=1.0,
                        )
                        obs.enemies.append(enemy)
                        
        except Exception as e:
            print(f"[ERROR] 获取观测失败: {e}")
        
        return obs
    
    def encode_action(self, packet: DecisionPacketV1) -> np.ndarray:
        """将决策包编码为动作向量
        
        输出: [vx_norm, vy_norm, wz_norm, kick, dribble]
        """
        # 连续动作: 从 intent 和 target 推导
        # 这里简化处理，实际应该根据执行器的输出来编码
        vx = 0.0
        vy = 0.0
        wz = 0.0
        kick = 0.0
        dribble = 0.0
        
        # 根据 intent 设置动作
        intent = IntentId(packet.intent_id)
        
        if intent == IntentId.SEARCH_BALL:
            # 搜索: 向目标点移动
            vx = min(1.0, packet.move_gain / 2.0)
        elif intent == IntentId.APPROACH_BALL:
            # 接近球: 前进
            vx = 0.5
        elif intent == IntentId.CARRY_TO_POSE:
            # 带球: 向目标移动
            vx = min(1.0, packet.move_gain / 2.0)
        elif intent == IntentId.SHOOT:
            # 射门
            kick = 1.0
        elif intent == IntentId.AVOID_WALL:
            # 避墙: 向目标移动
            vx = min(1.0, packet.move_gain / 2.0)
        
        # 运球
        dribble_mode = DribbleMode(packet.dribble_mode)
        if dribble_mode == DribbleMode.WEAK:
            dribble = 0.35
        elif dribble_mode == DribbleMode.STRONG:
            dribble = 0.8
        
        return np.array([vx, vy, wz, kick, dribble], dtype=np.float32)
    
    def compute_reward(self, obs: DeployObservationV1) -> Tuple[float, bool, str]:
        """计算奖励
        
        Returns:
            reward: 奖励值
            done: 是否终止
            outcome: 终止原因
        """
        reward = 0.0
        done = False
        outcome = "running"
        
        # 持球奖励
        if obs.has_ball:
            reward += self.reward_has_ball
        
        # 接近球奖励
        if obs.ball_visible and not obs.has_ball:
            ball_dist = math.hypot(obs.ball_rel_x, obs.ball_rel_y)
            dist_delta = self._last_ball_dist - ball_dist
            reward += dist_delta * self.reward_approach_ball
            self._last_ball_dist = ball_dist
        
        # 获得球奖励
        if obs.has_ball and not self._last_has_ball:
            reward += 1.0
        
        # 失去球惩罚
        if not obs.has_ball and self._last_has_ball:
            reward += -1.0
        
        self._last_has_ball = obs.has_ball
        
        # 进球检测
        if obs.has_ball:
            goal_x = self.mlp_config.goal_x
            goal_y = self.mlp_config.goal_y
            goal_width = self.mlp_config.goal_width
            
            if abs(obs.self_x - goal_x) < 20 and abs(obs.self_y - goal_y) < goal_width / 2:
                reward += self.reward_goal
                done = True
                outcome = "goal"
        
        # 出界检测
        half_length = self.mlp_config.field_length / 2.0 - self.mlp_config.safe_margin
        half_width = self.mlp_config.field_width / 2.0 - self.mlp_config.safe_margin
        
        if abs(obs.self_x) > half_length or abs(obs.self_y) > half_width:
            reward += self.reward_out_of_bounds
            done = True
            outcome = "out_of_bounds"
        
        # 时间惩罚
        reward += self.reward_time_penalty
        
        return reward, done, outcome
    
    def reset_episode(self, episode_id: int) -> Episode:
        """重置回合"""
        # 重置状态
        self._last_ball_dist = 200.0
        self._last_has_ball = False
        
        # 创建新回合
        episode = Episode(
            episode_id=episode_id,
            start_time=time.time(),
        )
        self.current_episode = episode
        
        # 尝试重置 Unity 场景
        try:
            # 通过 SceneControl 重置
            from utils.XeleratorSceneControl import XeleratorSceneControl
            ctl = XeleratorSceneControl(timeout_s=2.0)
            ctl.set_ball(
                random.uniform(-50, 50),
                random.uniform(-30, 30),
            )
            ctl.set_robot(1, -60.0, 0.0, 0.0)
        except Exception as e:
            print(f"[WARN] 场景重置失败: {e}")
        
        return episode
    
    def collect_step(self, step: int) -> Optional[Transition]:
        """采集单步数据"""
        if self.current_episode is None:
            return None
        
        # 获取观测
        obs = self.get_observation()
        
        # 教师决策
        decision = self.teacher.decide(obs)
        
        # 编码状态和动作
        state = self.encoder.encode(obs)
        action = self.encode_action(decision.packet)
        
        # 计算奖励
        reward, done, outcome = self.compute_reward(obs)
        
        # 创建转移
        transition = Transition(
            state=state,
            raw_obs=obs.to_dict(),
            action=action,
            raw_packet=decision.packet.to_dict(),
            reward=reward,
            done=done,
            step=step,
            episode=self.current_episode.episode_id,
            timestamp=time.time(),
            teacher_state=decision.state,
            teacher_reasons=decision.reasons,
        )
        
        # 执行决策
        try:
            self.executor.execute(decision.packet)
        except Exception as e:
            print(f"[ERROR] 执行决策失败: {e}")
        
        # 更新回合
        self.current_episode.add_transition(transition)
        
        # 检查终止条件
        if done:
            self.current_episode.outcome = outcome
            self.current_episode.end_time = time.time()
        elif step >= self._max_episode_steps:
            self.current_episode.outcome = "timeout"
            self.current_episode.end_time = time.time()
            done = True
        elif time.time() - self.current_episode.start_time > self._episode_timeout_s:
            self.current_episode.outcome = "timeout"
            self.current_episode.end_time = time.time()
            done = True
        
        return transition if not done else None
    
    def collect_episode(self, episode_id: int, verbose: bool = True) -> Episode:
        """采集单个回合"""
        episode = self.reset_episode(episode_id)
        
        if verbose:
            print(f"[Episode {episode_id}] 开始采集...")
        
        step = 0
        while True:
            transition = self.collect_step(step)
            step += 1
            
            if transition is None:
                break
            
            time.sleep(0.02)  # 50Hz
        
        # 更新统计
        self.stats.update(episode)
        self.episodes.append(episode)
        
        if verbose:
            print(f"[Episode {episode_id}] 完成: {episode.outcome}, "
                  f"步数={episode.total_steps}, 奖励={episode.total_reward:.2f}")
        
        return episode
    
    def collect_episodes(
        self,
        n_episodes: int,
        save_every: int = 10,
        verbose: bool = True,
    ) -> CollectionStats:
        """采集多个回合"""
        print(f"[DataCollector] 开始采集 {n_episodes} 个回合...")
        print(f"[DataCollector] 输出目录: {self.output_dir}")
        
        start_time = time.time()
        
        for i in range(n_episodes):
            episode_id = self.stats.total_episodes
            self.collect_episode(episode_id, verbose=verbose)
            
            # 定期保存
            if (i + 1) % save_every == 0:
                self.save_checkpoint()
                if verbose:
                    elapsed = time.time() - start_time
                    print(f"[DataCollector] 进度: {i+1}/{n_episodes}, "
                          f"进球率={self.stats.goal_rate:.1%}, "
                          f"平均奖励={self.stats.avg_episode_reward:.2f}, "
                          f"耗时={elapsed:.1f}s")
        
        # 最终保存
        self.save_final()
        
        elapsed = time.time() - start_time
        print(f"\n[DataCollector] 采集完成!")
        print(f"  - 总回合数: {self.stats.total_episodes}")
        print(f"  - 总转移数: {self.stats.total_transitions}")
        print(f"  - 进球数: {self.stats.total_goals}")
        print(f"  - 进球率: {self.stats.goal_rate:.1%}")
        print(f"  - 平均奖励: {self.stats.avg_episode_reward:.2f}")
        print(f"  - 平均步数: {self.stats.avg_episode_length:.1f}")
        print(f"  - 总耗时: {elapsed:.1f}s")
        
        return self.stats
    
    def save_checkpoint(self):
        """保存检查点"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = self.output_dir / f"checkpoint_{timestamp}.json"
        
        data = {
            "stats": self.stats.to_dict(),
            "episodes": [e.to_dict() for e in self.episodes[-100:]],  # 只保存最近100个
        }
        
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
    
    def save_final(self):
        """保存最终数据"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 保存统计
        stats_path = self.output_dir / f"stats_{timestamp}.json"
        with open(stats_path, 'w', encoding='utf-8') as f:
            json.dump(self.stats.to_dict(), f, ensure_ascii=False, indent=2)
        
        # 保存所有转移 (用于训练)
        transitions_path = self.output_dir / f"transitions_{timestamp}.npz"
        all_states = []
        all_actions = []
        all_rewards = []
        all_dones = []
        
        for episode in self.episodes:
            for t in episode.transitions:
                all_states.append(t.state)
                all_actions.append(t.action)
                all_rewards.append(t.reward)
                all_dones.append(t.done)
        
        np.savez_compressed(
            transitions_path,
            states=np.array(all_states, dtype=np.float32),
            actions=np.array(all_actions, dtype=np.float32),
            rewards=np.array(all_rewards, dtype=np.float32),
            dones=np.array(all_dones, dtype=np.bool_),
        )
        
        print(f"[DataCollector] 数据已保存:")
        print(f"  - 统计: {stats_path}")
        print(f"  - 转移: {transitions_path}")


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description="采集教师模型数据")
    parser.add_argument("--episodes", type=int, default=100, help="采集回合数")
    parser.add_argument("--output", type=str, default="Records/teacher_trajectories", help="输出目录")
    parser.add_argument("--save-every", type=int, default=10, help="保存频率")
    parser.add_argument("--quiet", action="store_true", help="静默模式")
    
    args = parser.parse_args()
    
    # 创建采集器
    collector = DataCollector(output_dir=args.output)
    
    try:
        # 采集数据
        collector.collect_episodes(
            n_episodes=args.episodes,
            save_every=args.save_every,
            verbose=not args.quiet,
        )
    except KeyboardInterrupt:
        print("\n[DataCollector] 用户中断")
        collector.save_checkpoint()
    finally:
        # 停止机器人
        try:
            chassis.stop()
        except:
            pass


if __name__ == "__main__":
    main()