"""推理引擎

将训练好的MLP模型嵌入到机器人控制循环中，替换或辅助现有硬编码策略。
"""

import os
import sys
import time
import math
import threading
import queue
from pathlib import Path
from typing import Optional, Dict, Any, Callable, Union, Tuple

import numpy as np
import torch

from .config import MLPConfig
from .contracts import (
    DecisionPacketV1,
    DeployObservationV1,
    DribbleMode,
    FaceMode,
    IntentId,
    ShootIntent,
    observation_from_lunapre,
)
from .model import ActorCritic, load_model
from .state_encoder import StateEncoder, RobotState
from .skill_executor import SkillExecutor


class MLPInference:
    """MLP推理引擎
    
    加载训练好的模型，实时推理并输出动作指令。
    可嵌入到现有的机器人控制循环中。
    """
    
    def __init__(
        self,
        model_path: str,
        config: MLPConfig = None,
        device: str = "auto",
        action_callback: Callable[[Dict[str, Any]], None] = None,
    ):
        """初始化推理引擎
        
        Args:
            model_path: 模型权重文件路径
            config: MLP配置
            device: 计算设备
            action_callback: 动作输出回调函数
        """
        self.config = config or MLPConfig()
        self.model_path = model_path
        self.action_callback = action_callback
        
        # 设置设备
        if device == "auto":
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        else:
            self.device = torch.device(device)
        
        # 状态编码器
        self.encoder = StateEncoder(self.config)
        
        # 加载模型
        self.model = self._load_model()
        self.model.eval()
        
        # 推理状态
        self._running = False
        self._last_action: Optional[np.ndarray] = None
        self._action_queue: queue.Queue = queue.Queue(maxsize=1)
        
        # 动作平滑
        self._smoothed_action = np.zeros(self.config.action_dim, dtype=np.float32)
        
        # 状态获取函数（可由外部设置）
        self._state_getter: Optional[Callable[[], Union[RobotState, DeployObservationV1]]] = None
        
        # 动作执行函数（可由外部设置）
        self._action_executor: Optional[Callable[[Dict[str, Any]], None]] = None
        
        print(f"[MLPInference] Loaded model from {model_path}")
        print(f"[MLPInference] Device: {self.device}")
    
    def _load_model(self) -> ActorCritic:
        """加载模型"""
        return load_model(self.model_path, self.config)
    
    def set_state_getter(self, getter: Callable[[], Union[RobotState, DeployObservationV1]]):
        """设置状态获取函数
        
        Args:
            getter: 返回RobotState的函数
        """
        self._state_getter = getter
    
    def set_action_executor(self, executor: Callable[[Dict[str, Any]], None]):
        """设置动作执行函数
        
        Args:
            executor: 执行动作的函数，接收解码后的动作字典
        """
        self._action_executor = executor
    
    def infer(self, state: Union[RobotState, DeployObservationV1], deterministic: bool = True) -> Dict[str, Any]:
        """执行推理
        
        Args:
            state: 机器人状态
            deterministic: 是否使用确定性策略
            
        Returns:
            解码后的动作字典
        """
        # 编码状态
        state_vec = self.encoder.encode(state)
        
        # 转换为tensor
        state_tensor = torch.FloatTensor(state_vec).unsqueeze(0).to(self.device)
        
        # 推理
        with torch.no_grad():
            action, log_prob, value = self.model.get_action(state_tensor, deterministic=deterministic)
        
        action_np = action.cpu().numpy()[0]
        
        # 动作平滑
        self._smoothed_action = (
            self.config.action_smoothing * self._smoothed_action +
            (1 - self.config.action_smoothing) * action_np
        )
        
        # 解码动作
        decoded = self.encoder.decode_action(self._smoothed_action)
        
        self._last_action = action_np
        
        return decoded

    def infer_packet(self, state: Union[RobotState, DeployObservationV1], deterministic: bool = True) -> DecisionPacketV1:
        """Temporary adapter: map legacy low-level action output into a decision packet.

        This keeps the new packet execution chain available before the model
        architecture is upgraded to emit native high-level intents.
        """
        action = self.infer(state, deterministic=deterministic)
        return self._legacy_action_to_packet(state, action)
    
    def infer_from_obs(self, obs: np.ndarray, deterministic: bool = True) -> Dict[str, Any]:
        """从观测向量推理
        
        Args:
            obs: 已编码的观测向量
            deterministic: 是否使用确定性策略
            
        Returns:
            解码后的动作字典
        """
        state_tensor = torch.FloatTensor(obs).unsqueeze(0).to(self.device)
        
        with torch.no_grad():
            action, _, _ = self.model.get_action(state_tensor, deterministic=deterministic)
        
        action_np = action.cpu().numpy()[0]
        
        # 动作平滑
        self._smoothed_action = (
            self.config.action_smoothing * self._smoothed_action +
            (1 - self.config.action_smoothing) * action_np
        )
        
        return self.encoder.decode_action(self._smoothed_action)
    
    def get_action(self) -> Optional[np.ndarray]:
        """获取最近的动作（用于外部调用）"""
        return self._last_action
    
    def start_loop(self, hz: float = 30.0):
        """启动推理循环
        
        Args:
            hz: 推理频率
        """
        if self._running:
            return
        
        self._running = True
        self._loop_thread = threading.Thread(target=self._inference_loop, args=(hz,))
        self._loop_thread.daemon = True
        self._loop_thread.start()
        
        print(f"[MLPInference] Started inference loop at {hz} Hz")
    
    def stop_loop(self):
        """停止推理循环"""
        self._running = False
        if hasattr(self, '_loop_thread'):
            self._loop_thread.join(timeout=1.0)
        
        print("[MLPInference] Stopped inference loop")
    
    def _inference_loop(self, hz: float):
        """推理循环"""
        period = 1.0 / hz
        
        while self._running:
            start_time = time.time()
            
            try:
                # 获取状态
                if self._state_getter:
                    state = self._state_getter()
                else:
                    # 从LunaPre获取状态
                    state = self._get_state_from_lunapre()
                
                # 推理
                action = self.infer(state, deterministic=True)
                
                # 执行动作
                if self._action_executor:
                    self._action_executor(action)
                elif self.action_callback:
                    self.action_callback(action)
                
            except Exception as e:
                print(f"[MLPInference] Error in inference loop: {e}")
            
            # 控制频率
            elapsed = time.time() - start_time
            if elapsed < period:
                time.sleep(period - elapsed)
    
    def _get_state_from_lunapre(self) -> DeployObservationV1:
        """从LunaPre获取部署对齐观测"""
        try:
            obs = observation_from_lunapre()
            try:
                from LunaPre import chassis
                obs.self_vx = float(getattr(chassis, '_last_vx', 0.0))
                obs.self_vy = float(getattr(chassis, '_last_vy', 0.0))
                obs.self_wz = float(getattr(chassis, '_last_wz', 0.0))
                obs.velocity_valid = True
            except Exception:
                obs.velocity_valid = False
            return obs
        except Exception as e:
            print(f"[MLPInference] Error getting state from LunaPre: {e}")
            return DeployObservationV1()

    def _legacy_action_to_packet(
        self,
        state: Union[RobotState, DeployObservationV1],
        action: Dict[str, Any],
    ) -> DecisionPacketV1:
        """Map current low-level action dict to a coarse high-level decision packet."""
        try:
            vx = float(action.get("vx", 0.0))
            vy = float(action.get("vy", 0.0))
            wz = float(action.get("wz", 0.0))
        except Exception:
            vx, vy, wz = 0.0, 0.0, 0.0

        self_x = float(getattr(state, "self_x", getattr(state, "x", 0.0)))
        self_y = float(getattr(state, "self_y", getattr(state, "y", 0.0)))
        self_yaw = float(getattr(state, "self_yaw", getattr(state, "yaw", 0.0)))

        # Approximate a short horizon target in field frame from the old robot-frame command.
        horizon_s = 0.25
        yaw_rad = math.radians(self_yaw)
        dx = (vx * math.cos(yaw_rad) - vy * math.sin(yaw_rad)) * horizon_s
        dy = (vx * math.sin(yaw_rad) + vy * math.cos(yaw_rad)) * horizon_s
        target_x = self_x + dx
        target_y = self_y + dy
        target_yaw = self_yaw + wz * horizon_s

        if action.get("kick"):
            intent = IntentId.SHOOT
            shoot_intent = ShootIntent.TRY_SHOOT
        elif action.get("dribble"):
            intent = IntentId.CARRY_TO_POSE
            shoot_intent = ShootIntent.NONE
        elif abs(vx) > 1e-3 or abs(vy) > 1e-3:
            intent = IntentId.CARRY_TO_POSE
            shoot_intent = ShootIntent.NONE
        else:
            intent = IntentId.SEARCH_BALL
            shoot_intent = ShootIntent.NONE

        packet = DecisionPacketV1(
            intent_id=int(intent),
            target_x=target_x,
            target_y=target_y,
            target_yaw=target_yaw,
            face_mode=int(FaceMode.FACE_TARGET),
            move_gain=1.0,
            dribble_mode=int(DribbleMode.WEAK if action.get("dribble") else DribbleMode.OFF),
            shoot_intent=int(shoot_intent),
            )
        return packet
    
    def close(self):
        """关闭推理引擎"""
        self.stop_loop()


class MLPController:
    """MLP控制器
    
    更高级的封装，提供完整的机器人控制功能，
    可以直接替换offence.py/defence.py的策略。
    """
    
    def __init__(
        self,
        model_path: str,
        config: MLPConfig = None,
        role: str = "offence",
        device: str = "auto",
    ):
        """初始化控制器
        
        Args:
            model_path: 模型路径
            config: MLP配置
            role: 角色 ("offence" 或 "defence")
            device: 计算设备
        """
        self.config = config or MLPConfig()
        self.role = role
        self.device = device
        
        # 创建推理引擎
        self.inference = MLPInference(
            model_path=model_path,
            config=config,
            device=device,
        )
        self.skill_executor = SkillExecutor(self.config)
        
        # 设置动作执行器
        self.inference.set_action_executor(self._execute_action)
        
        # 控制状态
        self._enabled = False
        self._override_safety = False  # 是否启用安全覆盖
    
    def _execute_action(self, action: Dict[str, Any]):
        """执行动作"""
        try:
            from LunaPre import chassis, peripheral
            
            # 速度命令
            vx = action['vx']
            vy = action['vy']
            wz = action['wz']
            
            # 安全检查
            if self._override_safety:
                vx, vy, wz = self._apply_safety_limits(vx, vy, wz)
            
            # 执行移动
            chassis.RobotMoveVector(vx, vy, wz)
            
            # 盘球
            if action['dribble']:
                peripheral.Dribble(True)
            else:
                peripheral.Dribble(False)
            
            # 射门
            if action['kick']:
                peripheral.ShootBall()
                
        except Exception as e:
            print(f"[MLPController] Error executing action: {e}")

    def execute_decision_packet(self, packet: DecisionPacketV1):
        """Execute a high-level decision packet via the new skill executor."""
        result = self.skill_executor.execute(packet)
        if not result.executed:
            print(f"[MLPController] Decision packet execution failed: {result.status}")

    def run_packet_once(self, deterministic: bool = True) -> DecisionPacketV1:
        """Infer one decision packet and execute it immediately."""
        state = self.inference._get_state_from_lunapre()
        packet = self.inference.infer_packet(state, deterministic=deterministic)
        self.execute_decision_packet(packet)
        return packet
    
    def _apply_safety_limits(self, vx: float, vy: float, wz: float) -> Tuple[float, float, float]:
        """应用安全限制"""
        # 限制最大速度
        max_speed = self.config.max_speed
        max_angular = self.config.max_angular_speed
        
        vx = np.clip(vx, -max_speed, max_speed)
        vy = np.clip(vy, -max_speed, max_speed)
        wz = np.clip(wz, -max_angular, max_angular)
        
        # 检查边界（如果有位置信息）
        try:
            from LunaPre import UnitedPosition
            pos = UnitedPosition.FieldRobotPos()
            if pos != [0xfff, 0xfff, 0xfff]:
                x, y = pos[0], pos[1]
                half_length = self.config.field_length / 2.0 - self.config.safe_margin
                half_width = self.config.field_width / 2.0 - self.config.safe_margin
                
                # 如果接近边界，减速
                if x > half_length - 20:
                    vx = min(vx, 50)  # 减速
                elif x < -half_length + 20:
                    vx = max(vx, -50)
                
                if y > half_width - 20:
                    vy = min(vy, 50)
                elif y < -half_width + 20:
                    vy = max(vy, -50)
        except:
            pass
        
        return vx, vy, wz
    
    def enable(self):
        """启用MLP控制"""
        self._enabled = True
        self.inference.start_loop(hz=30.0)
        print(f"[MLPController] MLP control enabled (role={self.role})")
    
    def disable(self):
        """禁用MLP控制"""
        self._enabled = False
        self.inference.stop_loop()
        print("[MLPController] MLP control disabled")
    
    def is_enabled(self) -> bool:
        """检查是否启用"""
        return self._enabled
    
    def set_role(self, role: str):
        """设置角色"""
        self.role = role
        print(f"[MLPController] Role set to {role}")
    
    def main_loop(self):
        """主控制循环（用于替换offence.py/defence.py的main函数）"""
        self.enable()
        
        try:
            while self._enabled:
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.disable()
        finally:
            self.disable()
    
    def close(self):
        """关闭控制器"""
        self.disable()
        self.inference.close()


def run_mlp_strategy(
    model_path: str,
    role: str = "offence",
    config: MLPConfig = None,
):
    """运行MLP策略
    
    可用于替换现有的offence.py/defence.py策略。
    
    Args:
        model_path: 模型路径
        role: 角色
        config: 配置
    """
    controller = MLPController(
        model_path=model_path,
        config=config,
        role=role,
    )
    
    try:
        controller.main_loop()
    finally:
        controller.close()


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description="MLP Inference for Robot Soccer")
    parser.add_argument("--model", type=str, required=True, help="Model path")
    parser.add_argument("--role", type=str, default="offence", help="Role (offence/defence)")
    parser.add_argument("--device", type=str, default="auto", help="Device")
    parser.add_argument("--hz", type=float, default=30.0, help="Inference frequency")
    parser.add_argument("--mock", action="store_true", help="Use mock state getter")
    
    args = parser.parse_args()
    
    config = MLPConfig()
    
    inference = MLPInference(
        model_path=args.model,
        config=config,
        device=args.device,
    )
    
    if args.mock:
        # 使用mock状态获取器
        def mock_state_getter():
            return RobotState(
                x=np.random.uniform(-50, 50),
                y=np.random.uniform(-50, 50),
                yaw=np.random.uniform(0, 360),
                ball_x=np.random.uniform(20, 100),
                ball_y=np.random.uniform(-50, 50),
                has_ball=False,
            )
        inference.set_state_getter(mock_state_getter)
    else:
        # 从LunaPre获取状态
        inference.set_state_getter(inference._get_state_from_lunapre)
    
    # 设置动作打印回调
    inference.action_callback = lambda action: print(
        f"[MLP] vx={action['vx']:.1f} vy={action['vy']:.1f} wz={action['wz']:.1f} "
        f"kick={action['kick']} dribble={action['dribble']}"
    )
    
    try:
        inference.start_loop(hz=args.hz)
        
        while True:
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\n[MLP] Interrupted")
        inference.stop_loop()
    finally:
        inference.close()


if __name__ == "__main__":
    main()
