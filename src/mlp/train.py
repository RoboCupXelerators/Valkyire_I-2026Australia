"""PPO训练脚本

使用Proximal Policy Optimization算法训练机器人决策模型。
"""

import os
import sys
import time
import json
import argparse
from datetime import datetime
from pathlib import Path
from typing import List, Tuple, Optional

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.tensorboard import SummaryWriter

from .config import MLPConfig
from .model import ActorCritic, create_model, save_model, load_model
from .gym_env import make_env
from .state_encoder import StateEncoder
from .training_plots import export_results


class RolloutBuffer:
    """经验回放缓冲区
    
    存储一个rollout周期内的轨迹数据。
    """
    
    def __init__(self):
        self.states: List[np.ndarray] = []
        self.actions: List[np.ndarray] = []
        self.rewards: List[float] = []
        self.values: List[float] = []
        self.log_probs: List[float] = []
        self.dones: List[bool] = []
    
    def add(
        self,
        state: np.ndarray,
        action: np.ndarray,
        reward: float,
        value: float,
        log_prob: float,
        done: bool,
    ):
        """添加一条经验"""
        self.states.append(state)
        self.actions.append(action)
        self.rewards.append(reward)
        self.values.append(value)
        self.log_probs.append(log_prob)
        self.dones.append(done)
    
    def clear(self):
        """清空缓冲区"""
        self.states.clear()
        self.actions.clear()
        self.rewards.clear()
        self.values.clear()
        self.log_probs.clear()
        self.dones.clear()
    
    def __len__(self):
        return len(self.states)


class PPOTrainer:
    """PPO训练器"""
    
    def __init__(
        self,
        config: MLPConfig,
        device: str = "auto",
        use_mock_env: bool = False,
    ):
        """初始化训练器
        
        Args:
            config: MLP配置
            device: 计算设备 ("auto", "cuda", "cpu")
            use_mock_env: 是否使用mock环境（用于测试）
        """
        self.config = config
        self.use_mock_env = use_mock_env
        
        # 设置设备
        if device == "auto":
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        else:
            self.device = torch.device(device)
        
        print(f"[PPO] Using device: {self.device}")
        
        # 创建模型
        self.model = create_model(config).to(self.device)
        self.optimizer = optim.Adam(self.model.parameters(), lr=config.learning_rate)
        
        # 创建环境
        self.env = make_env(config, mock=use_mock_env)
        
        # 经验缓冲区
        self.buffer = RolloutBuffer()
        
        # 训练统计
        self.global_step = 0
        self.episode_rewards: List[float] = []
        self.episode_lengths: List[int] = []
        self.best_reward = float('-inf')
        
        # TensorBoard
        self.writer: Optional[SummaryWriter] = None
        self.log_path: Optional[Path] = None
        self.training_history: List[dict] = []
        self.results_csv_path: Optional[Path] = None
        self.results_png_path: Optional[Path] = None
    
    def setup_logging(self, log_dir: str = None):
        """设置日志"""
        if log_dir is None:
            log_dir = self.config.log_dir
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = Path(log_dir) / f"ppo_{timestamp}"
        log_path.mkdir(parents=True, exist_ok=True)
        
        self.writer = SummaryWriter(str(log_path))
        self.log_path = log_path
        
        # 保存配置
        config_path = log_path / "config.json"
        with open(config_path, 'w') as f:
            json.dump(self.config.to_dict(), f, indent=2)
        
        print(f"[PPO] Logging to {log_path}")
    
    def collect_rollouts(self, n_steps: int) -> Tuple[float, int, int]:
        """收集rollout数据
        
        Args:
            n_steps: 收集步数
            
        Returns:
            total_reward: 总奖励
            n_episodes: 完成的episode数
            n_steps_actual: 实际收集的步数
        """
        self.buffer.clear()
        
        # 重置环境
        state, _ = self.env.reset()
        
        total_reward = 0.0
        n_episodes = 0
        episode_reward = 0.0
        episode_length = 0
        
        for step in range(n_steps):
            # 转换为tensor
            state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
            
            # 获取动作
            with torch.no_grad():
                action, log_prob, value = self.model.get_action(state_tensor)
            
            action_np = action.cpu().numpy()[0]
            log_prob_np = log_prob.cpu().numpy()[0, 0]
            value_np = value.cpu().numpy()[0, 0]
            
            # 执行动作
            next_state, reward, terminated, truncated, info = self.env.step(action_np)
            done = terminated or truncated
            
            # 存储经验
            self.buffer.add(
                state=state,
                action=action_np,
                reward=reward,
                value=value_np,
                log_prob=log_prob_np,
                done=done,
            )
            
            # 更新统计
            total_reward += reward
            episode_reward += reward
            episode_length += 1
            self.global_step += 1
            
            # 记录到TensorBoard
            if self.writer and self.global_step % self.config.log_every == 0:
                self.writer.add_scalar("train/episode_reward", reward, self.global_step)
            
            # 处理episode结束
            if done:
                self.episode_rewards.append(episode_reward)
                self.episode_lengths.append(episode_length)
                n_episodes += 1
                
                # 记录episode统计
                if self.writer:
                    self.writer.add_scalar("train/episode_total_reward", episode_reward, self.global_step)
                    self.writer.add_scalar("train/episode_length", episode_length, self.global_step)
                
                episode_reward = 0.0
                episode_length = 0
                
                # 重置环境
                state, _ = self.env.reset()
            else:
                state = next_state
        
        return total_reward, n_episodes, len(self.buffer)
    
    def compute_gae(
        self,
        rewards: np.ndarray,
        values: np.ndarray,
        dones: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """计算Generalized Advantage Estimation
        
        Args:
            rewards: 奖励序列
            values: 价值估计序列
            dones: 终止标志序列
            
        Returns:
            advantages: 优势函数
            returns: 回报
        """
        n = len(rewards)
        advantages = np.zeros(n, dtype=np.float32)
        returns = np.zeros(n, dtype=np.float32)
        
        last_advantage = 0
        last_value = values[-1] if not dones[-1] else 0
        
        for t in reversed(range(n)):
            if dones[t]:
                next_value = 0
                delta = rewards[t] - values[t]
            else:
                next_value = values[t + 1] if t + 1 < n else last_value
                delta = rewards[t] + self.config.gamma * next_value - values[t]
            
            advantages[t] = last_advantage = delta + self.config.gamma * self.config.gae_lambda * (1 - dones[t]) * last_advantage
            returns[t] = advantages[t] + values[t]
        
        return advantages, returns
    
    def update(self) -> dict:
        """执行PPO更新
        
        Returns:
            训练统计字典
        """
        if len(self.buffer) == 0:
            return {}
        
        # 转换为数组
        states = np.array(self.buffer.states)
        actions = np.array(self.buffer.actions)
        rewards = np.array(self.buffer.rewards)
        values = np.array(self.buffer.values)
        old_log_probs = np.array(self.buffer.log_probs)
        dones = np.array(self.buffer.dones)
        
        # 计算GAE
        advantages, returns = self.compute_gae(rewards, values, dones)
        
        # 标准化优势
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        
        # 转换为tensor
        states_t = torch.FloatTensor(states).to(self.device)
        actions_t = torch.FloatTensor(actions).to(self.device)
        advantages_t = torch.FloatTensor(advantages).unsqueeze(1).to(self.device)
        returns_t = torch.FloatTensor(returns).unsqueeze(1).to(self.device)
        old_log_probs_t = torch.FloatTensor(old_log_probs).unsqueeze(1).to(self.device)
        
        # PPO更新
        total_loss = 0.0
        policy_loss_total = 0.0
        value_loss_total = 0.0
        entropy_total = 0.0
        
        n_samples = len(states)
        batch_size = self.config.batch_size
        
        for epoch in range(self.config.n_epochs):
            # 随机打乱
            indices = np.random.permutation(n_samples)
            
            for start in range(0, n_samples, batch_size):
                end = start + batch_size
                batch_indices = indices[start:end]
                
                # 获取batch
                batch_states = states_t[batch_indices]
                batch_actions = actions_t[batch_indices]
                batch_advantages = advantages_t[batch_indices]
                batch_returns = returns_t[batch_indices]
                batch_old_log_probs = old_log_probs_t[batch_indices]
                
                # 评估动作
                new_log_probs, entropy, new_values = self.model.evaluate_actions(
                    batch_states, batch_actions
                )
                
                # Policy loss (PPO clip)
                ratio = torch.exp(new_log_probs - batch_old_log_probs)
                surr1 = ratio * batch_advantages
                surr2 = torch.clamp(
                    ratio,
                    1 - self.config.clip_epsilon,
                    1 + self.config.clip_epsilon,
                ) * batch_advantages
                policy_loss = -torch.min(surr1, surr2).mean()
                
                # Value loss
                value_loss = nn.functional.mse_loss(new_values, batch_returns)
                
                # Entropy loss
                entropy_loss = -entropy.mean()
                
                # Total loss
                loss = (
                    policy_loss +
                    self.config.value_coef * value_loss +
                    self.config.entropy_coef * entropy_loss
                )
                
                # 优化
                self.optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(self.model.parameters(), self.config.max_grad_norm)
                self.optimizer.step()
                
                total_loss += loss.item()
                policy_loss_total += policy_loss.item()
                value_loss_total += value_loss.item()
                entropy_total += entropy.mean().item()
        
        n_updates = self.config.n_epochs * (n_samples // batch_size + 1)
        
        return {
            "total_loss": total_loss / n_updates,
            "policy_loss": policy_loss_total / n_updates,
            "value_loss": value_loss_total / n_updates,
            "entropy": entropy_total / n_updates,
        }
    
    def train(
        self,
        total_timesteps: int = None,
        save_every: int = None,
        log_every: int = None,
    ):
        """训练模型
        
        Args:
            total_timesteps: 总训练步数
            save_every: 保存频率
            log_every: 日志频率
        """
        total_timesteps = total_timesteps or self.config.total_timesteps
        save_every = save_every or self.config.save_every
        log_every = log_every or self.config.log_every
        
        # 设置日志
        if self.writer is None:
            self.setup_logging()
        
        print(f"[PPO] Starting training for {total_timesteps} timesteps")
        print(f"[PPO] Device: {self.device}, Mock env: {self.use_mock_env}")
        
        start_time = time.time()
        
        while self.global_step < total_timesteps:
            # 收集经验
            total_reward, n_episodes, n_steps = self.collect_rollouts(self.config.n_steps)
            
            # 更新模型
            stats = self.update()
            
            # 计算平均奖励
            mean_reward = np.mean(self.episode_rewards[-10:]) if self.episode_rewards else 0.0
            mean_length = np.mean(self.episode_lengths[-10:]) if self.episode_lengths else 0.0
            
            # 记录统计
            if self.writer:
                for key, value in stats.items():
                    self.writer.add_scalar(f"train/{key}", value, self.global_step)
                self.writer.add_scalar("train/mean_reward", mean_reward, self.global_step)
                self.writer.add_scalar("train/mean_length", mean_length, self.global_step)

            row = {
                "step": float(self.global_step),
                "mean_reward": float(mean_reward),
                "mean_length": float(mean_length),
                "lr": float(self.optimizer.param_groups[0]["lr"]),
            }
            for key, value in stats.items():
                row[key] = float(value)
            self.training_history.append(row)
            self.export_training_results()
            
            # 打印进度
            elapsed = time.time() - start_time
            fps = self.global_step / elapsed if elapsed > 0 else 0
            
            print(
                f"[PPO] Step {self.global_step}/{total_timesteps} | "
                f"Reward: {mean_reward:.2f} | "
                f"Length: {mean_length:.1f} | "
                f"Loss: {stats.get('total_loss', 0):.4f} | "
                f"FPS: {fps:.1f}"
            )
            
            # 保存模型
            if self.global_step % save_every < self.config.n_steps:
                self.save_checkpoint()
            
            # 保存最佳模型
            if mean_reward > self.best_reward:
                self.best_reward = mean_reward
                self.save_checkpoint(best=True)
        
        print(f"[PPO] Training completed!")
        print(f"[PPO] Best reward: {self.best_reward:.2f}")
        if self.results_csv_path:
            print(f"[PPO] Results CSV: {self.results_csv_path}")
        if self.results_png_path:
            print(f"[PPO] Results plot: {self.results_png_path}")
        
        # 最终保存
        self.save_checkpoint(final=True)
    
    def save_checkpoint(self, best: bool = False, final: bool = False):
        """保存检查点"""
        if self.log_path is None:
            return
        
        if final:
            path = self.log_path / "model_final.pt"
        elif best:
            path = self.log_path / "model_best.pt"
        else:
            path = self.log_path / f"model_step_{self.global_step}.pt"
        
        save_model(self.model, str(path))
        print(f"[PPO] Saved model to {path}")

    def export_training_results(self):
        """Export YOLO-like static training curves."""
        if self.log_path is None or not self.training_history:
            return

        csv_path, png_path = export_results(
            self.log_path,
            self.training_history,
            title="PPO Training Results",
        )
        self.results_csv_path = csv_path
        self.results_png_path = png_path
    
    def load_checkpoint(self, path: str):
        """加载检查点"""
        self.model = load_model(path, self.config).to(self.device)
        print(f"[PPO] Loaded model from {path}")
    
    def close(self):
        """关闭训练器"""
        self.env.close()
        if self.writer:
            self.writer.close()


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description="PPO Training for Robot Soccer")
    parser.add_argument("--timesteps", type=int, default=1_000_000, help="Total training timesteps")
    parser.add_argument("--lr", type=float, default=3e-4, help="Learning rate")
    parser.add_argument("--batch-size", type=int, default=64, help="Batch size")
    parser.add_argument("--n-steps", type=int, default=2048, help="Steps per rollout")
    parser.add_argument("--n-epochs", type=int, default=10, help="Epochs per update")
    parser.add_argument("--gamma", type=float, default=0.99, help="Discount factor")
    parser.add_argument("--gae-lambda", type=float, default=0.95, help="GAE lambda")
    parser.add_argument("--clip-eps", type=float, default=0.2, help="PPO clip epsilon")
    parser.add_argument("--entropy-coef", type=float, default=0.01, help="Entropy coefficient")
    parser.add_argument("--device", type=str, default="auto", help="Device (auto/cuda/cpu)")
    parser.add_argument("--mock", action="store_true", help="Use mock environment")
    parser.add_argument("--log-dir", type=str, default=None, help="Log directory")
    parser.add_argument("--resume", type=str, default=None, help="Resume from checkpoint")
    
    args = parser.parse_args()
    
    # 创建配置
    config = MLPConfig(
        learning_rate=args.lr,
        batch_size=args.batch_size,
        n_steps=args.n_steps,
        n_epochs=args.n_epochs,
        gamma=args.gamma,
        gae_lambda=args.gae_lambda,
        clip_epsilon=args.clip_eps,
        entropy_coef=args.entropy_coef,
    )
    
    # 创建训练器
    trainer = PPOTrainer(config, device=args.device, use_mock_env=args.mock)
    
    # 设置日志
    if args.log_dir:
        trainer.setup_logging(args.log_dir)
    else:
        trainer.setup_logging()
    
    # 加载检查点
    if args.resume:
        trainer.load_checkpoint(args.resume)
    
    try:
        # 开始训练
        trainer.train(total_timesteps=args.timesteps)
    except KeyboardInterrupt:
        print("\n[PPO] Training interrupted")
        trainer.save_checkpoint()
    finally:
        trainer.close()


if __name__ == "__main__":
    main()
