"""行为克隆训练模块

使用教师模型采集的数据训练 MLP 策略网络。
支持：
- 从 npz 文件加载数据
- 监督学习训练
- 训练/验证集划分
- TensorBoard 日志
"""

from __future__ import annotations

import os
import json
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from datetime import datetime

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader, random_split
from torch.utils.tensorboard import SummaryWriter

from .config import MLPConfig
from .model import ActorCritic, create_model, save_model, load_model


class TransitionDataset(Dataset):
    """转移数据集"""
    
    def __init__(
        self,
        states: np.ndarray,
        actions: np.ndarray,
        rewards: np.ndarray = None,
        dones: np.ndarray = None,
    ):
        self.states = torch.FloatTensor(states)
        self.actions = torch.FloatTensor(actions)
        self.rewards = torch.FloatTensor(rewards) if rewards is not None else None
        self.dones = torch.BoolTensor(dones) if dones is not None else None
    
    def __len__(self):
        return len(self.states)
    
    def __getitem__(self, idx):
        if self.rewards is not None:
            return (
                self.states[idx],
                self.actions[idx],
                self.rewards[idx],
                self.dones[idx] if self.dones is not None else False,
            )
        return self.states[idx], self.actions[idx]


class BehaviorCloningTrainer:
    """行为克隆训练器"""
    
    def __init__(
        self,
        config: MLPConfig = None,
        device: str = "auto",
    ):
        self.config = config or MLPConfig()
        
        # 设置设备
        if device == "auto":
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        else:
            self.device = torch.device(device)
        
        print(f"[BC] Using device: {self.device}")
        
        # 创建模型
        self.model = create_model(self.config).to(self.device)
        self.optimizer = optim.Adam(self.model.parameters(), lr=self.config.learning_rate)
        
        # 损失函数
        self.mse_loss = nn.MSELoss()
        self.bce_loss = nn.BCELoss()
        
        # 训练统计
        self.global_step = 0
        self.best_loss = float('inf')
        
        # TensorBoard
        self.writer: Optional[SummaryWriter] = None
        self.log_path: Optional[Path] = None
    
    def setup_logging(self, log_dir: str = None):
        """设置日志"""
        if log_dir is None:
            log_dir = self.config.log_dir
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = Path(log_dir) / f"bc_{timestamp}"
        log_path.mkdir(parents=True, exist_ok=True)
        
        self.writer = SummaryWriter(str(log_path))
        self.log_path = log_path
        
        print(f"[BC] Logging to {log_path}")
    
    def load_data(
        self,
        data_path: str,
        val_ratio: float = 0.1,
    ) -> Tuple[DataLoader, DataLoader]:
        """加载数据"""
        print(f"[BC] Loading data from {data_path}")
        
        data = np.load(data_path)
        states = data["states"]
        actions = data["actions"]
        rewards = data.get("rewards", None)
        dones = data.get("dones", None)
        
        print(f"[BC] Loaded {len(states)} transitions")
        print(f"[BC] State shape: {states.shape}, Action shape: {actions.shape}")
        
        # 创建数据集
        dataset = TransitionDataset(states, actions, rewards, dones)
        
        # 划分训练集和验证集
        n_val = int(len(dataset) * val_ratio)
        n_train = len(dataset) - n_val
        
        train_dataset, val_dataset = random_split(
            dataset, [n_train, n_val],
            generator=torch.Generator().manual_seed(42),
        )
        
        print(f"[BC] Train: {len(train_dataset)}, Val: {len(val_dataset)}")
        
        # 创建数据加载器
        train_loader = DataLoader(
            train_dataset,
            batch_size=self.config.batch_size,
            shuffle=True,
            num_workers=0,
        )
        
        val_loader = DataLoader(
            val_dataset,
            batch_size=self.config.batch_size,
            shuffle=False,
            num_workers=0,
        )
        
        return train_loader, val_loader
    
    def compute_loss(
        self,
        states: torch.Tensor,
        actions: torch.Tensor,
    ) -> Tuple[torch.Tensor, Dict[str, float]]:
        """计算损失"""
        # 前向传播
        continuous_mean, continuous_std, discrete_logits, value = self.model(states)
        
        # 分离连续和离散动作
        target_continuous = actions[:, :3]  # vx, vy, wz
        target_discrete = actions[:, 3:5]   # kick, dribble
        
        # 连续动作损失 (MSE)
        continuous_loss = self.mse_loss(continuous_mean, target_continuous)
        
        # 离散动作损失 (BCE)
        discrete_loss = self.bce_loss(discrete_logits, target_discrete)
        
        # 总损失
        total_loss = continuous_loss + discrete_loss
        
        # 统计
        stats = {
            "total_loss": total_loss.item(),
            "continuous_loss": continuous_loss.item(),
            "discrete_loss": discrete_loss.item(),
        }
        
        return total_loss, stats
    
    def train_epoch(
        self,
        train_loader: DataLoader,
        epoch: int,
    ) -> Dict[str, float]:
        """训练一个 epoch"""
        self.model.train()
        
        total_loss = 0.0
        total_continuous_loss = 0.0
        total_discrete_loss = 0.0
        n_batches = 0
        
        for batch_idx, batch in enumerate(train_loader):
            if len(batch) == 4:
                states, actions, rewards, dones = batch
            else:
                states, actions = batch
            
            states = states.to(self.device)
            actions = actions.to(self.device)
            
            # 前向传播
            self.optimizer.zero_grad()
            loss, stats = self.compute_loss(states, actions)
            
            # 反向传播
            loss.backward()
            nn.utils.clip_grad_norm_(self.model.parameters(), self.config.max_grad_norm)
            self.optimizer.step()
            
            # 累计统计
            total_loss += stats["total_loss"]
            total_continuous_loss += stats["continuous_loss"]
            total_discrete_loss += stats["discrete_loss"]
            n_batches += 1
            
            self.global_step += 1
            
            # 记录到 TensorBoard
            if self.writer and self.global_step % 100 == 0:
                self.writer.add_scalar("train/loss", stats["total_loss"], self.global_step)
                self.writer.add_scalar("train/continuous_loss", stats["continuous_loss"], self.global_step)
                self.writer.add_scalar("train/discrete_loss", stats["discrete_loss"], self.global_step)
        
        return {
            "loss": total_loss / n_batches,
            "continuous_loss": total_continuous_loss / n_batches,
            "discrete_loss": total_discrete_loss / n_batches,
        }
    
    def validate(
        self,
        val_loader: DataLoader,
    ) -> Dict[str, float]:
        """验证"""
        self.model.eval()
        
        total_loss = 0.0
        total_continuous_loss = 0.0
        total_discrete_loss = 0.0
        n_batches = 0
        
        with torch.no_grad():
            for batch in val_loader:
                if len(batch) == 4:
                    states, actions, rewards, dones = batch
                else:
                    states, actions = batch
                
                states = states.to(self.device)
                actions = actions.to(self.device)
                
                loss, stats = self.compute_loss(states, actions)
                
                total_loss += stats["total_loss"]
                total_continuous_loss += stats["continuous_loss"]
                total_discrete_loss += stats["discrete_loss"]
                n_batches += 1
        
        return {
            "val_loss": total_loss / n_batches,
            "val_continuous_loss": total_continuous_loss / n_batches,
            "val_discrete_loss": total_discrete_loss / n_batches,
        }
    
    def train(
        self,
        data_path: str,
        n_epochs: int = 100,
        val_ratio: float = 0.1,
        save_every: int = 10,
        early_stopping_patience: int = 10,
    ):
        """训练模型"""
        # 设置日志
        if self.writer is None:
            self.setup_logging()
        
        # 加载数据
        train_loader, val_loader = self.load_data(data_path, val_ratio)
        
        print(f"[BC] Starting training for {n_epochs} epochs")
        
        best_val_loss = float('inf')
        patience_counter = 0
        
        for epoch in range(n_epochs):
            # 训练
            train_stats = self.train_epoch(train_loader, epoch)
            
            # 验证
            val_stats = self.validate(val_loader)
            
            # 记录到 TensorBoard
            if self.writer:
                self.writer.add_scalar("epoch/train_loss", train_stats["loss"], epoch)
                self.writer.add_scalar("epoch/val_loss", val_stats["val_loss"], epoch)
            
            # 打印进度
            print(f"[BC] Epoch {epoch+1}/{n_epochs} | "
                  f"Train Loss: {train_stats['loss']:.4f} | "
                  f"Val Loss: {val_stats['val_loss']:.4f}")
            
            # 保存最佳模型
            if val_stats["val_loss"] < best_val_loss:
                best_val_loss = val_stats["val_loss"]
                patience_counter = 0
                self.save_model("model_best.pt")
                print(f"[BC] New best model saved! Val Loss: {best_val_loss:.4f}")
            else:
                patience_counter += 1
            
            # 定期保存
            if (epoch + 1) % save_every == 0:
                self.save_model(f"model_epoch_{epoch+1}.pt")
            
            # 早停
            if patience_counter >= early_stopping_patience:
                print(f"[BC] Early stopping at epoch {epoch+1}")
                break
        
        # 保存最终模型
        self.save_model("model_final.pt")
        
        print(f"[BC] Training completed!")
        print(f"[BC] Best Val Loss: {best_val_loss:.4f}")
    
    def save_model(self, filename: str):
        """保存模型"""
        if self.log_path is None:
            return
        
        path = self.log_path / filename
        save_model(self.model, str(path))
    
    def load_model(self, path: str):
        """加载模型"""
        self.model = load_model(path, self.config).to(self.device)
        print(f"[BC] Loaded model from {path}")
    
    def close(self):
        """关闭训练器"""
        if self.writer:
            self.writer.close()


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description="行为克隆训练")
    parser.add_argument("--data", type=str, required=True, help="数据文件路径 (.npz)")
    parser.add_argument("--epochs", type=int, default=100, help="训练轮数")
    parser.add_argument("--batch-size", type=int, default=64, help="批次大小")
    parser.add_argument("--lr", type=float, default=1e-3, help="学习率")
    parser.add_argument("--val-ratio", type=float, default=0.1, help="验证集比例")
    parser.add_argument("--device", type=str, default="auto", help="设备")
    parser.add_argument("--log-dir", type=str, default=None, help="日志目录")
    
    args = parser.parse_args()
    
    # 创建配置
    config = MLPConfig(
        learning_rate=args.lr,
        batch_size=args.batch_size,
    )
    
    # 创建训练器
    trainer = BehaviorCloningTrainer(config, device=args.device)
    
    # 设置日志
    if args.log_dir:
        trainer.setup_logging(args.log_dir)
    else:
        trainer.setup_logging()
    
    try:
        # 开始训练
        trainer.train(
            data_path=args.data,
            n_epochs=args.epochs,
            val_ratio=args.val_ratio,
        )
    except KeyboardInterrupt:
        print("\n[BC] Training interrupted")
        trainer.save_model("model_interrupted.pt")
    finally:
        trainer.close()


if __name__ == "__main__":
    main()