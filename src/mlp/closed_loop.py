"""闭环迭代训练系统

自动化训练流程:
1. 数据采集 (教师模型)
2. 行为克隆训练
3. 模型评估 (对比教师)
4. 迭代优化

用法:
    cd D:\\Github\\Software
    python src/mlp/closed_loop.py --iterations 5 --episodes-per-iter 100
"""

from __future__ import annotations

import os
import sys
import json
import time
import argparse
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from datetime import datetime

import numpy as np
import torch

# 设置环境变量
os.environ["ROBOCUPX_BITS"] = "SIM"
os.environ["ROBOCUPX_USE_MOCK"] = "0"

from LunaPre import UnitedPosition, chassis

from mlp.config import MLPConfig
from mlp.data_collector import DataCollector, CollectionStats
from mlp.behavior_cloning import BehaviorCloningTrainer
from mlp.model import create_model, save_model, load_model
from mlp.state_encoder import StateEncoder
from mlp.skill_executor import SkillExecutor
from mlp.inference import PolicyInference
from teacher.rule_teacher import RuleTeacher, TeacherConfig
from mlp.contracts import DeployObservationV1


@dataclass
class IterationStats:
    """迭代统计"""
    iteration: int
    data_stats: Dict[str, float]
    train_stats: Dict[str, float]
    eval_stats: Dict[str, float]
    timestamp: str
    
    def to_dict(self) -> Dict:
        return {
            "iteration": self.iteration,
            "data_stats": self.data_stats,
            "train_stats": self.train_stats,
            "eval_stats": self.eval_stats,
            "timestamp": self.timestamp,
        }


class ClosedLoopTrainer:
    """闭环迭代训练器"""
    
    def __init__(
        self,
        output_dir: str = "Records/closed_loop",
        mlp_config: MLPConfig = None,
        teacher_config: TeacherConfig = None,
        device: str = "auto",
    ):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        self.mlp_config = mlp_config or MLPConfig()
        self.teacher_config = teacher_config or TeacherConfig()
        
        # 设备
        if device == "auto":
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        else:
            self.device = torch.device(device)
        
        print(f"[ClosedLoop] Output directory: {self.output_dir}")
        print(f"[ClosedLoop] Device: {self.device}")
        
        # 组件
        self.teacher = RuleTeacher(self.teacher_config)
        self.encoder = StateEncoder(self.mlp_config)
        self.executor = SkillExecutor()
        
        # 当前最佳模型
        self.best_model_path: Optional[str] = None
        self.iteration_stats: List[IterationStats] = []
    
    def step1_collect_data(
        self,
        iteration: int,
        n_episodes: int = 100,
        verbose: bool = True,
    ) -> Tuple[str, Dict[str, float]]:
        """步骤1: 数据采集"""
        print(f"\n{'='*60}")
        print(f"[Iteration {iteration}] Step 1: Data Collection")
        print(f"{'='*60}")
        
        # 创建采集目录
        data_dir = self.output_dir / f"iter_{iteration}" / "data"
        data_dir.mkdir(parents=True, exist_ok=True)
        
        # 采集数据
        collector = DataCollector(
            mlp_config=self.mlp_config,
            teacher_config=self.teacher_config,
            output_dir=str(data_dir),
        )
        
        stats = collector.collect_episodes(
            n_episodes=n_episodes,
            save_every=10,
            verbose=verbose,
        )
        
        # 找到最新的转移文件
        transitions_files = list(data_dir.glob("transitions_*.npz"))
        if not transitions_files:
            raise RuntimeError("No transitions file found!")
        
        latest_file = max(transitions_files, key=lambda p: p.stat().st_mtime)
        
        # 停止机器人
        try:
            chassis.stop()
        except:
            pass
        
        print(f"[Iteration {iteration}] Data collected: {stats.total_transitions} transitions")
        print(f"[Iteration {iteration}] Goal rate: {stats.goal_rate:.1%}")
        
        return str(latest_file), stats.to_dict()
    
    def step2_train_bc(
        self,
        iteration: int,
        data_path: str,
        n_epochs: int = 50,
        verbose: bool = True,
    ) -> Dict[str, float]:
        """步骤2: 行为克隆训练"""
        print(f"\n{'='*60}")
        print(f"[Iteration {iteration}] Step 2: Behavior Cloning")
        print(f"{'='*60}")
        
        # 创建训练目录
        train_dir = self.output_dir / f"iter_{iteration}" / "train"
        train_dir.mkdir(parents=True, exist_ok=True)
        
        # 创建训练器
        trainer = BehaviorCloningTrainer(
            config=self.mlp_config,
            device=str(self.device),
        )
        
        trainer.setup_logging(str(train_dir))
        
        # 训练
        try:
            trainer.train(
                data_path=data_path,
                n_epochs=n_epochs,
                val_ratio=0.1,
                save_every=10,
                early_stopping_patience=10,
            )
        except KeyboardInterrupt:
            print("[ClosedLoop] Training interrupted")
            trainer.save_model("model_interrupted.pt")
        
        trainer.close()
        
        # 找到最佳模型
        best_model = train_dir / "bc_*" / "model_best.pt"
        best_models = list(train_dir.glob("bc_*/model_best.pt"))
        
        if best_models:
            self.best_model_path = str(max(best_models, key=lambda p: p.stat().st_mtime))
        
        # 返回训练统计
        train_stats = {
            "epochs": n_epochs,
            "best_model_path": self.best_model_path,
        }
        
        print(f"[Iteration {iteration}] BC training completed")
        print(f"[Iteration {iteration}] Best model: {self.best_model_path}")
        
        return train_stats
    
    def step3_evaluate(
        self,
        iteration: int,
        model_path: str,
        n_episodes: int = 20,
        verbose: bool = True,
    ) -> Dict[str, float]:
        """步骤3: 模型评估"""
        print(f"\n{'='*60}")
        print(f"[Iteration {iteration}] Step 3: Evaluation")
        print(f"{'='*60}")
        
        # 加载模型
        model = load_model(model_path, self.mlp_config).to(self.device)
        inference = PolicyInference(model, self.encoder, self.executor, self.device)
        
        # 评估 MLP 模型
        print(f"[Iteration {iteration}] Evaluating MLP model...")
        mlp_stats = self._evaluate_policy(inference, n_episodes, verbose=verbose)
        
        # 评估教师模型 (基准)
        print(f"[Iteration {iteration}] Evaluating Teacher (baseline)...")
        teacher_stats = self._evaluate_policy(None, n_episodes, verbose=verbose, use_teacher=True)
        
        # 对比
        print(f"\n[Iteration {iteration}] Evaluation Results:")
        print(f"  MLP    - Goal Rate: {mlp_stats['goal_rate']:.1%}, Avg Reward: {mlp_stats['avg_reward']:.2f}")
        print(f"  Teacher - Goal Rate: {teacher_stats['goal_rate']:.1%}, Avg Reward: {teacher_stats['avg_reward']:.2f}")
        
        # 停止机器人
        try:
            chassis.stop()
        except:
            pass
        
        return {
            "mlp_goal_rate": mlp_stats["goal_rate"],
            "mlp_avg_reward": mlp_stats["avg_reward"],
            "teacher_goal_rate": teacher_stats["goal_rate"],
            "teacher_avg_reward": teacher_stats["avg_reward"],
            "improvement": mlp_stats["goal_rate"] - teacher_stats["goal_rate"],
        }
    
    def _evaluate_policy(
        self,
        inference: Optional[PolicyInference],
        n_episodes: int,
        verbose: bool = True,
        use_teacher: bool = False,
    ) -> Dict[str, float]:
        """评估策略"""
        total_reward = 0.0
        total_goals = 0
        total_steps = 0
        
        for ep in range(n_episodes):
            reward, steps, outcome = self._run_episode(inference, use_teacher=use_teacher)
            total_reward += reward
            total_steps += steps
            if outcome == "goal":
                total_goals += 1
            
            if verbose:
                print(f"  Episode {ep}: outcome={outcome}, reward={reward:.2f}, steps={steps}")
        
        return {
            "goal_rate": total_goals / n_episodes,
            "avg_reward": total_reward / n_episodes,
            "avg_steps": total_steps / n_episodes,
        }
    
    def _run_episode(
        self,
        inference: Optional[PolicyInference],
        max_steps: int = 500,
        timeout_s: float = 30.0,
        use_teacher: bool = False,
    ) -> Tuple[float, int, str]:
        """运行单个回合"""
        # 重置
        self._reset_scene()
        
        reward = 0.0
        outcome = "timeout"
        
        start_time = time.time()
        
        for step in range(max_steps):
            # 获取观测
            obs = self._get_observation()
            
            # 决策
            if use_teacher:
                decision = self.teacher.decide(obs)
                packet = decision.packet
            else:
                packet = inference.decide(obs)
            
            # 执行
            self.executor.execute(packet)
            
            # 计算奖励
            r, done, outcome = self._compute_reward(obs)
            reward += r
            
            if done:
                break
            
            if time.time() - start_time > timeout_s:
                outcome = "timeout"
                break
            
            time.sleep(0.02)
        
        return reward, step + 1, outcome
    
    def _reset_scene(self):
        """重置场景"""
        try:
            from utils.XeleratorSceneControl import XeleratorSceneControl
            ctl = XeleratorSceneControl(timeout_s=2.0)
            ctl.set_ball(0.0, 0.0)
            ctl.set_robot(1, -60.0, 0.0, 0.0)
        except Exception as e:
            print(f"[WARN] Scene reset failed: {e}")
    
    def _get_observation(self) -> DeployObservationV1:
        """获取观测"""
        obs = DeployObservationV1()
        
        try:
            pos = UnitedPosition.FieldRobotPos()
            if pos and len(pos) >= 3:
                obs.self_x = float(pos[0])
                obs.self_y = float(pos[1])
                obs.self_yaw = float(pos[2])
            
            ball_pos = UnitedPosition.RobotBallPos()
            if ball_pos:
                bx, by = ball_pos[0], ball_pos[1]
                if (bx, by) == (0xFFF, 0xFFF):
                    obs.ball_visible = False
                elif (bx, by) == (0xDDD, 0xDDD):
                    obs.has_ball = True
                else:
                    obs.ball_rel_x = float(bx)
                    obs.ball_rel_y = float(by)
                    obs.ball_visible = True
        except Exception as e:
            print(f"[ERROR] Get observation failed: {e}")
        
        return obs
    
    def _compute_reward(self, obs: DeployObservationV1) -> Tuple[float, bool, str]:
        """计算奖励"""
        reward = 0.0
        done = False
        outcome = "running"
        
        if obs.has_ball:
            reward += 0.1
        
        # 进球检测
        if obs.has_ball and obs.self_x > 100:
            reward += 100.0
            done = True
            outcome = "goal"
        
        # 出界检测
        half_length = self.mlp_config.field_length / 2.0 - self.mlp_config.safe_margin
        half_width = self.mlp_config.field_width / 2.0 - self.mlp_config.safe_margin
        
        if abs(obs.self_x) > half_length or abs(obs.self_y) > half_width:
            reward += -50.0
            done = True
            outcome = "out_of_bounds"
        
        reward += -0.001  # 时间惩罚
        
        return reward, done, outcome
    
    def run_iteration(
        self,
        iteration: int,
        n_episodes_data: int = 100,
        n_epochs_bc: int = 50,
        n_episodes_eval: int = 20,
        verbose: bool = True,
    ) -> IterationStats:
        """运行单次迭代"""
        print(f"\n{'='*60}")
        print(f"[ClosedLoop] Starting Iteration {iteration}")
        print(f"{'='*60}\n")
        
        start_time = time.time()
        
        # Step 1: 数据采集
        data_path, data_stats = self.step1_collect_data(
            iteration, n_episodes_data, verbose
        )
        
        # Step 2: 行为克隆训练
        train_stats = self.step2_train_bc(
            iteration, data_path, n_epochs_bc, verbose
        )
        
        # Step 3: 模型评估
        if self.best_model_path:
            eval_stats = self.step3_evaluate(
                iteration, self.best_model_path, n_episodes_eval, verbose
            )
        else:
            eval_stats = {"error": "No model trained"}
        
        # 记录统计
        stats = IterationStats(
            iteration=iteration,
            data_stats=data_stats,
            train_stats=train_stats,
            eval_stats=eval_stats,
            timestamp=datetime.now().isoformat(),
        )
        
        self.iteration_stats.append(stats)
        self._save_stats()
        
        elapsed = time.time() - start_time
        print(f"\n[ClosedLoop] Iteration {iteration} completed in {elapsed:.1f}s")
        
        return stats
    
    def run(
        self,
        n_iterations: int = 5,
        n_episodes_data: int = 100,
        n_epochs_bc: int = 50,
        n_episodes_eval: int = 20,
        verbose: bool = True,
    ):
        """运行完整迭代流程"""
        print(f"\n{'='*60}")
        print(f"[ClosedLoop] Starting Closed-Loop Training")
        print(f"{'='*60}")
        print(f"  Iterations: {n_iterations}")
        print(f"  Episodes per iteration: {n_episodes_data}")
        print(f"  BC epochs: {n_epochs_bc}")
        print(f"  Eval episodes: {n_episodes_eval}")
        print(f"{'='*60}\n")
        
        start_time = time.time()
        
        try:
            for i in range(n_iterations):
                self.run_iteration(
                    iteration=i + 1,
                    n_episodes_data=n_episodes_data,
                    n_epochs_bc=n_epochs_bc,
                    n_episodes_eval=n_episodes_eval,
                    verbose=verbose,
                )
                
                # 保存最佳模型
                if self.best_model_path:
                    final_model = self.output_dir / f"model_iter_{i+1}.pt"
                    import shutil
                    shutil.copy(self.best_model_path, str(final_model))
        
        except KeyboardInterrupt:
            print("\n[ClosedLoop] Training interrupted")
        
        finally:
            # 停止机器人
            try:
                chassis.stop()
            except:
                pass
        
        elapsed = time.time() - start_time
        
        # 打印总结
        print(f"\n{'='*60}")
        print(f"[ClosedLoop] Training Summary")
        print(f"{'='*60}")
        
        for stats in self.iteration_stats:
            print(f"\nIteration {stats.iteration}:")
            print(f"  Data: {stats.data_stats['total_transitions']} transitions, "
                  f"goal_rate={stats.data_stats['goal_rate']:.1%}")
            if "mlp_goal_rate" in stats.eval_stats:
                print(f"  Eval: MLP={stats.eval_stats['mlp_goal_rate']:.1%}, "
                      f"Teacher={stats.eval_stats['teacher_goal_rate']:.1%}")
        
        print(f"\nTotal time: {elapsed:.1f}s")
        print(f"{'='*60}")
    
    def _save_stats(self):
        """保存统计"""
        stats_path = self.output_dir / "iteration_stats.json"
        with open(stats_path, 'w', encoding='utf-8') as f:
            json.dump(
                [s.to_dict() for s in self.iteration_stats],
                f, ensure_ascii=False, indent=2
            )


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description="闭环迭代训练")
    parser.add_argument("--iterations", type=int, default=5, help="迭代次数")
    parser.add_argument("--episodes-per-iter", type=int, default=100, help="每次迭代采集回合数")
    parser.add_argument("--epochs", type=int, default=50, help="BC 训练轮数")
    parser.add_argument("--eval-episodes", type=int, default=20, help="评估回合数")
    parser.add_argument("--output", type=str, default="Records/closed_loop", help="输出目录")
    parser.add_argument("--device", type=str, default="auto", help="设备")
    parser.add_argument("--quiet", action="store_true", help="静默模式")
    
    args = parser.parse_args()
    
    trainer = ClosedLoopTrainer(
        output_dir=args.output,
        device=args.device,
    )
    
    try:
        trainer.run(
            n_iterations=args.iterations,
            n_episodes_data=args.episodes_per_iter,
            n_epochs_bc=args.epochs,
            n_episodes_eval=args.eval_episodes,
            verbose=not args.quiet,
        )
    except KeyboardInterrupt:
        print("\n[ClosedLoop] Interrupted by user")


if __name__ == "__main__":
    main()
