"""Gym环境包装器

将Unity仿真器封装为OpenAI Gym接口，用于强化学习训练。
支持：
- 球门检测
- 回合重置（通过UDP命令）
- 时间限制（1分钟）
- 随机球位置
"""

import math
import json
import socket
import time
import threading
from typing import Dict, List, Optional, Tuple, Any

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:
    import gym
    from gym import spaces

from .config import MLPConfig
from .state_encoder import StateEncoder, RobotState
from .contracts import (
    DeployObservationV1,
    EpisodeStatusV1,
    ResetScenarioV1,
    observation_from_simulator,
)


class SoccerSimEnv(gym.Env):
    """机器人足球仿真环境
    
    OpenAI Gym接口封装的仿真环境，支持强化学习训练。
    """
    
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}
    
    def __init__(
        self,
        config: MLPConfig = None,
        render_mode: Optional[str] = None,
        headless: bool = True,
    ):
        """初始化环境
        
        Args:
            config: MLP配置
            render_mode: 渲染模式
            headless: 是否无头模式（不显示Unity窗口）
        """
        super().__init__()
        
        self.config = config or MLPConfig()
        self.render_mode = render_mode
        self.headless = headless
        
        # 状态编码器
        self.encoder = StateEncoder(self.config)
        
        # 动作空间：[vx, vy, wz, kick, dribble]
        # 连续动作: vx, vy, wz 归一化到 [-1, 1]
        # 离散动作: kick, dribble 归一化到 [0, 1]
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(self.config.action_dim,),
            dtype=np.float32
        )
        
        # 观测空间：状态特征向量
        self.observation_space = spaces.Box(
            low=-2.0,  # 允许一些特殊值超出[-1,1]
            high=2.0,
            shape=(self.config.state_dim,),
            dtype=np.float32
        )
        
        # 仿真器连接
        self._bits = None
        self._connected = False
        
        # 状态缓存
        self._last_state: Optional[RobotState] = None
        self._last_observation: Optional[DeployObservationV1] = None
        self._last_obs: Optional[np.ndarray] = None
        self._last_episode_status: Optional[EpisodeStatusV1] = None
        self._last_ball_dist: float = 200.0  # 200cm = 最大可能距离
        self._last_has_ball: bool = False
        self._goal_scored: bool = False
        self._episode_steps: int = 0
        self._max_episode_steps: int = 1000
        
        # 奖励计算相关
        self._cumulative_reward: float = 0.0
        
        # 线程安全
        self._lock = threading.Lock()
    
    def connect(self) -> bool:
        """连接到仿真器
        
        Returns:
            是否连接成功
        """
        try:
            # 导入仿真器Bits
            import os
            os.environ["ROBOCUPX_BITS"] = "SIM"
            
            from utils.XeleratorSimBits import XeleratorSimBits
            
            self._bits = XeleratorSimBits(
                unity_ip=self.config.sim_ip,
                cmd_port=self.config.sim_cmd_port,
                status_port=self.config.sim_status_port,
                robot_id=self.config.robot_id,
            )
            self._connected = True
            return True
            
        except Exception as e:
            print(f"[GymEnv] Failed to connect to simulator: {e}")
            self._connected = False
            return False
    
    def disconnect(self):
        """断开与仿真器的连接"""
        if self._bits is not None:
            try:
                self._bits.Close()
            except:
                pass
        self._connected = False
    
    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ) -> Tuple[np.ndarray, dict]:
        """重置环境
        
        Args:
            seed: 随机种子
            options: 额外选项
            
        Returns:
            observation: 初始观测
            info: 额外信息
        """
        super().reset(seed=seed)
        
        # 确保连接
        if not self._connected:
            if not self.connect():
                # 返回零状态
                return np.zeros(self.config.state_dim, dtype=np.float32), {"error": "not connected"}
        
        # 重置状态
        self._last_state = None
        self._last_observation = None
        self._last_obs = None
        self._last_episode_status = None
        self._last_ball_dist = 200.0  # 200cm = 最大可能距离
        self._last_has_ball = False
        self._goal_scored = False
        self._episode_steps = 0
        self._cumulative_reward = 0.0
        
        # 等待仿真器状态更新
        time.sleep(0.1)
        
        # 获取初始状态
        obs, info = self._get_observation()
        
        return obs, info
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, dict]:
        """执行动作
        
        Args:
            action: 动作向量 [vx, vy, wz, kick, dribble]
            
        Returns:
            observation: 新的观测
            reward: 奖励
            terminated: 是否终止（进球等）
            truncated: 是否截断（超时等）
            info: 额外信息
        """
        if not self._connected:
            return np.zeros(self.config.state_dim, dtype=np.float32), 0.0, True, False, {"error": "not connected"}
        
        # 解码并执行动作
        self._execute_action(action)
        
        # 等待物理模拟
        time.sleep(1.0 / 50.0)  # 50Hz
        
        # 获取新状态
        obs, info = self._get_observation()
        
        # 计算奖励
        reward, terminated = self._compute_reward(action)
        
        # 检查截断条件
        self._episode_steps += 1
        truncated = self._episode_steps >= self._max_episode_steps
        
        # 更新状态
        self._last_obs = obs
        
        return obs, reward, terminated, truncated, info
    
    def _get_observation(self) -> Tuple[np.ndarray, dict]:
        """获取当前观测

        训练时优先走 DeployObservationV1 契约，再派生兼容的 RobotState，
        确保后续 reward/调试逻辑还能沿用旧字段。
        """
        deploy_obs = self._get_deploy_observation()
        self._last_observation = deploy_obs

        state = self._robot_state_from_observation(deploy_obs)
        self._last_state = state
        obs = self.encoder.encode(deploy_obs)

        info = {
            "x": deploy_obs.self_x,
            "y": deploy_obs.self_y,
            "yaw": deploy_obs.self_yaw,
            "has_ball": deploy_obs.has_ball,
            "ball_visible": deploy_obs.ball_visible,
            "ball_dist": math.sqrt(deploy_obs.ball_rel_x**2 + deploy_obs.ball_rel_y**2),
            "enemy_count": len(deploy_obs.enemies),
        }
        if self._last_episode_status is not None:
            info["episode_status"] = self._last_episode_status.to_dict()

        return obs, info
    
    def _get_deploy_observation(self) -> DeployObservationV1:
        """从仿真器获取部署对齐观测。"""
        if self._bits is None:
            return DeployObservationV1()

        try:
            x, y, yaw = self._bits.GetPose()
            ball_pos = self._bits.GetBallPos()
            has_ball = (ball_pos == [0xDDD, 0xDDD])
            ball_missing = (ball_pos == [0xFFF, 0xFFF])
            ball_x = 0.0 if (has_ball or ball_missing) else float(ball_pos[0])
            ball_y = 0.0 if (has_ball or ball_missing) else float(ball_pos[1])

            wall_dist_mm = self._estimate_wall_distances_mm(x, y)
            enemies = []
            if hasattr(self._bits, "GetEnemyPos"):
                try:
                    enemies = list(self._bits.GetEnemyPos() or [])
                except Exception:
                    enemies = []

            obs = observation_from_simulator(
                self_x=float(x),
                self_y=float(y),
                self_yaw=float(yaw),
                ball_x=ball_x,
                ball_y=ball_y,
                has_ball=has_ball,
                wall_dist=wall_dist_mm,
                enemies=enemies,
            )

            vx, vy, wz_deg_s, valid = self._read_bits_velocity()
            obs.self_vx = vx
            obs.self_vy = vy
            obs.self_wz = wz_deg_s
            obs.velocity_valid = valid
            return obs
        except Exception as e:
            print(f"[GymEnv] Error getting deploy observation: {e}")
            return DeployObservationV1()

    def _robot_state_from_observation(self, obs: DeployObservationV1) -> RobotState:
        """Build a backward-compatible RobotState from DeployObservationV1."""
        ball_x = 0.0 if obs.has_ball else obs.ball_rel_x
        ball_y = 0.0 if obs.has_ball else obs.ball_rel_y
        wall_dist_mm = [
            obs.wall_front * 10.0,
            obs.wall_left * 10.0,
            obs.wall_back * 10.0,
            obs.wall_right * 10.0,
        ] if obs.wall_valid else [0.0, 0.0, 0.0, 0.0]

        enemies = [(enemy.rel_x, enemy.rel_y) for enemy in obs.enemies if enemy.visible]
        return RobotState(
            x=obs.self_x,
            y=obs.self_y,
            yaw=obs.self_yaw,
            vx=obs.self_vx if obs.velocity_valid else 0.0,
            vy=obs.self_vy if obs.velocity_valid else 0.0,
            wz=obs.self_wz if obs.velocity_valid else 0.0,
            ball_x=ball_x,
            ball_y=ball_y,
            has_ball=obs.has_ball,
            enemies=enemies,
            wall_dist=wall_dist_mm,
        )

    def _estimate_wall_distances_mm(self, x: float, y: float) -> List[float]:
        """Estimate wall distances in mm when the simulator does not expose lidar."""
        half_length = self.config.field_length / 2.0
        half_width = self.config.field_width / 2.0
        return [
            (half_length - x) * 10.0,   # front
            (half_width + y) * 10.0,    # left
            (half_length + x) * 10.0,   # back
            (half_width - y) * 10.0,    # right
        ]

    def _read_bits_velocity(self) -> Tuple[float, float, float, bool]:
        """Best-effort readback of simulator velocity without changing base components."""
        try:
            status_lock = getattr(self._bits, "_status_lock", None)
            status = getattr(self._bits, "_status", None)
            if status_lock is None or status is None:
                return 0.0, 0.0, 0.0, False
            with status_lock:
                vx = float(getattr(status, "vx_mps", 0.0)) * 100.0
                vy = float(getattr(status, "vy_mps", 0.0)) * 100.0
                wz_deg_s = math.degrees(float(getattr(status, "wz_rps", 0.0)))
            return vx, vy, wz_deg_s, True
        except Exception:
            return 0.0, 0.0, 0.0, False
    
    def _execute_action(self, action: np.ndarray):
        """执行动作"""
        if self._bits is None:
            return
        
        try:
            # 解码动作
            # action: [vx_norm, vy_norm, wz_norm, kick, dribble]
            # 反归一化
            vx = float(action[0]) * self.config.max_speed
            vy = float(action[1]) * self.config.max_speed
            wz_deg_s = float(action[2]) * self.config.max_angular_speed
            kick = float(action[3]) > 0.5
            dribble = float(action[4]) > 0.5
            
            # 发送速度命令
            self._bits.SetTwist(vx / 100.0, vy / 100.0, math.radians(wz_deg_s))  # m/s, rad/s
            
            # 发送踢球/盘球命令
            if dribble:
                self._bits.VirtualPorts(4, 1.0)  # Dribble port
            else:
                self._bits.VirtualPorts(4, 0.0)
            
            if kick:
                self._bits.VirtualPorts(3, 0)  # Kick trigger
                self._bits.VirtualPorts(3, 1)  # Release
                
        except Exception as e:
            print(f"[GymEnv] Error executing action: {e}")
    
    def _compute_reward(self, action: np.ndarray) -> Tuple[float, bool]:
        """计算奖励
        
        Args:
            action: 执行的动作
            
        Returns:
            reward: 奖励值
            terminated: 是否终止
        """
        reward = 0.0
        terminated = False
        
        if self._last_state is None:
            return reward, terminated
        
        state = self._last_state
        cfg = self.config
        
        # 1. 持球奖励
        if state.has_ball:
            reward += cfg.reward_has_ball
        
        # 2. 接近球奖励
        ball_dist = math.sqrt(state.ball_x**2 + state.ball_y**2)
        if ball_dist < cfg.ball_detection_range:
            # 计算距离变化
            dist_delta = self._last_ball_dist - ball_dist
            reward += dist_delta * cfg.reward_approach_ball
        self._last_ball_dist = ball_dist
        
        # 3. 持球状态变化奖励/惩罚
        if state.has_ball and not self._last_has_ball:
            # 刚刚获得球
            reward += 1.0
        elif not state.has_ball and self._last_has_ball:
            # 刚刚失去球
            reward += cfg.reward_lost_ball
        self._last_has_ball = state.has_ball
        
        # 4. 射门奖励（如果持球且面向球门）
        if state.has_ball:
            # 计算到球门的角度
            goal_dx = cfg.goal_x - state.x
            goal_dy = cfg.goal_y - state.y
            goal_angle = math.degrees(math.atan2(goal_dy, goal_dx)) % 360
            yaw_error = abs((goal_angle - state.yaw + 180) % 360 - 180)
            
            # 如果面向球门且接近球门，给予奖励
            if yaw_error < 30:  # 30度误差内
                goal_dist = math.sqrt(goal_dx**2 + goal_dy**2)
                if goal_dist < 100:  # 100cm内
                    reward += 0.1
            
            # 检测进球（简化：如果机器人持球且在球门附近）
            if abs(state.x - cfg.goal_x) < 20 and abs(state.y - cfg.goal_y) < cfg.goal_width / 2:
                reward += cfg.reward_goal
                terminated = True
                self._goal_scored = True
        
        # 5. 出界惩罚（重罚）
        half_length = cfg.field_length / 2.0 - cfg.safe_margin
        half_width = cfg.field_width / 2.0 - cfg.safe_margin
        if abs(state.x) > half_length or abs(state.y) > half_width:
            reward += cfg.reward_out_of_bounds  # -50.0 重罚
            terminated = True  # 结束回合
        
        # 6. 时间惩罚
        reward += cfg.reward_time_penalty
        
        self._cumulative_reward += reward
        
        return reward, terminated
    
    def render(self):
        """渲染环境"""
        # 仿真器自带渲染，这里只返回信息
        if self._last_state is not None:
            print(f"[Step {self._episode_steps}] "
                  f"pos=({self._last_state.x:.1f}, {self._last_state.y:.1f}) "
                  f"yaw={self._last_state.yaw:.1f} "
                  f"ball=({self._last_state.ball_x:.1f}, {self._last_state.ball_y:.1f}) "
                  f"has_ball={self._last_state.has_ball} "
                  f"reward={self._cumulative_reward:.2f}")
    
    def close(self):
        """关闭环境"""
        self.disconnect()
    
    def get_state(self) -> Optional[RobotState]:
        """获取当前状态"""
        return self._last_state


class MockSoccerSimEnv(gym.Env):
    """Mock环境（用于无仿真器时的测试）
    
    提供一个简单的模拟环境，不需要Unity仿真器。
    """
    
    def __init__(self, config: MLPConfig = None):
        super().__init__()
        
        self.config = config or MLPConfig()
        self.encoder = StateEncoder(self.config)
        
        # 动作和观测空间
        self.action_space = spaces.Box(
            low=-1.0, high=1.0,
            shape=(self.config.action_dim,),
            dtype=np.float32
        )
        self.observation_space = spaces.Box(
            low=-2.0, high=2.0,
            shape=(self.config.state_dim,),
            dtype=np.float32
        )
        
        # 模拟状态
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._ball_x = 50.0
        self._ball_y = 30.0
        self._has_ball = False
        self._episode_steps = 0
    
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._ball_x = 50.0 + self.np_random.uniform(-30, 30)
        self._ball_y = 30.0 + self.np_random.uniform(-30, 30)
        self._has_ball = False
        self._episode_steps = 0
        return self._get_obs(), {}
    
    def step(self, action):
        # 简单物理模拟
        vx = action[0] * 100  # cm/s
        vy = action[1] * 100
        wz = action[2] * 30   # deg/s
        
        # 更新位置
        dt = 0.02
        self._yaw += wz * dt
        self._yaw = self._yaw % 360
        
        # 转换到场地坐标系
        yaw_rad = math.radians(self._yaw)
        dx = (vx * math.cos(yaw_rad) - vy * math.sin(yaw_rad)) * dt
        dy = (vx * math.sin(yaw_rad) + vy * math.cos(yaw_rad)) * dt
        
        self._x += dx
        self._y += dy
        
        # 边界限制
        self._x = np.clip(self._x, -130, 130)
        self._y = np.clip(self._y, -100, 100)
        
        # 更新球位置（如果持球）
        if self._has_ball:
            self._ball_x = 10  # 球在机器人前方
            self._ball_y = 0
        else:
            # 检查是否接近球
            ball_dist = math.sqrt(self._ball_x**2 + self._ball_y**2)
            if ball_dist < 20:
                self._has_ball = True
        
        # 计算奖励
        reward = 0.0
        terminated = False
        
        if self._has_ball:
            reward += 0.1
        
        # 进球检测
        if self._has_ball and self._x > 120:
            reward += 100.0
            terminated = True
        
        self._episode_steps += 1
        truncated = self._episode_steps >= 500
        
        return self._get_obs(), reward, terminated, truncated, {}
    
    def _get_obs(self):
        state = RobotState(
            x=self._x, y=self._y, yaw=self._yaw,
            ball_x=self._ball_x, ball_y=self._ball_y,
            has_ball=self._has_ball,
        )
        return self.encoder.encode(state)
    
    def close(self):
        pass


class TrainingEnvController:
    """训练环境控制器
    
    通过UDP与Unity的XelTrainingEnv通信，实现：
    - 发送重置命令
    - 查询回合状态
    - 设置随机种子
    """
    
    def __init__(self, ip: str = "127.0.0.1", port: int = 20003):
        self.ip = ip
        self.port = port
        self._socket = None
        self._remote = (self.ip, self.port)
        self._status_roundtrip_supported: Optional[bool] = None
        self._last_status_probe_ts: float = 0.0
        self._status_probe_interval_s: float = 2.0
    
    def connect(self):
        """创建UDP socket"""
        if self._socket is None:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._socket.settimeout(0.5)
    
    def close(self):
        """关闭socket"""
        if self._socket is not None:
            try:
                self._socket.close()
            except:
                pass
            self._socket = None
    
    def send_command(self, cmd: str) -> bool:
        """发送命令到Unity
        
        Args:
            cmd: 命令字符串 (RESET, START, STOP, STATUS, SEED N)
            
        Returns:
            是否发送成功
        """
        if self._socket is None:
            self.connect()
        
        try:
            data = cmd.encode('utf-8')
            self._socket.sendto(data, self._remote)
            return True
        except Exception as e:
            print(f"[TrainingEnvController] Error sending command: {e}")
            return False

    def send_json_command(self, cmd: str, payload: Dict[str, Any]) -> bool:
        """Send a JSON payload command.

        Protocol skeleton for M2:
        `COMMAND {"json":"payload"}`
        """
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        return self.send_command(f"{cmd} {body}")

    def request_text_response(self, cmd: str) -> Optional[str]:
        """Send a command and wait for a single UDP text response."""
        if not self.send_command(cmd):
            return None
        if self._socket is None:
            return None
        try:
            data, _addr = self._socket.recvfrom(4096)
            if not data:
                return None
            return data.decode("utf-8", errors="ignore").strip()
        except socket.timeout:
            self._status_roundtrip_supported = False
            return None
        except Exception as e:
            self._status_roundtrip_supported = False
            print(f"[TrainingEnvController] Error receiving response: {e}")
            return None

    def _parse_episode_status(self, text: str) -> Optional[EpisodeStatusV1]:
        """Parse either future JSON status or the current CSV debug format."""
        if not text:
            return None
        try:
            if text.startswith("{"):
                return EpisodeStatusV1.from_dict(json.loads(text))
        except Exception:
            pass

        parts = [p.strip() for p in text.split(",")]
        if len(parts) >= 3:
            try:
                running = parts[0] == "1"
                elapsed_s = float(parts[1])
                result = int(parts[2])
                return EpisodeStatusV1(
                    episode_id=0,
                    is_running=running,
                    is_done=not running and result != 0,
                    result=result,
                    elapsed_ms=int(elapsed_s * 1000.0),
                    end_reason="legacy_csv",
                )
            except Exception:
                return None
        return None
    
    def reset_episode(self) -> bool:
        """重置回合
        
        发送RESET命令到Unity，触发：
        - 机器人位置重置
        - 球位置随机化
        - 计时器重置
        
        Returns:
            是否发送成功
        """
        return self.send_command("RESET")

    def reset_scenario(self, scenario: ResetScenarioV1) -> bool:
        """Send structured reset payload for future Unity protocol support."""
        return self.send_json_command("RESET_SCENARIO", scenario.to_dict())
    
    def start_episode(self) -> bool:
        """开始新回合"""
        return self.send_command("START")
    
    def stop_episode(self) -> bool:
        """停止当前回合"""
        return self.send_command("STOP")
    
    def set_seed(self, seed: int) -> bool:
        """设置随机种子
        
        Args:
            seed: 随机种子值
            
        Returns:
            是否发送成功
        """
        return self.send_command(f"SEED {seed}")

    def get_episode_status(self) -> Optional[EpisodeStatusV1]:
        """Query episode status.

        Works with the future JSON protocol and degrades gracefully if Unity
        still uses the legacy CSV status response.
        """
        now = time.time()
        if self._status_roundtrip_supported is False:
            if (now - self._last_status_probe_ts) < self._status_probe_interval_s:
                return None

        self._last_status_probe_ts = now
        text = self.request_text_response("GET_EPISODE_STATUS")
        if text is None:
            text = self.request_text_response("STATUS")
        if text is None:
            return None
        self._status_roundtrip_supported = True
        return self._parse_episode_status(text)


class TrainingSoccerSimEnv(SoccerSimEnv):
    """训练专用环境
    
    扩展SoccerSimEnv，添加与Unity训练环境管理器的集成。
    """
    
    def __init__(
        self,
        config: MLPConfig = None,
        training_env_port: int = 20003,
    ):
        super().__init__(config)
        
        # 训练环境控制器
        self._training_controller = TrainingEnvController(
            ip=self.config.sim_ip,
            port=training_env_port,
        )
        
        # 时间限制（60秒）
        self._episode_time_limit = 60.0
        self._episode_start_time = None
    
    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ) -> Tuple[np.ndarray, dict]:
        """重置环境
        
        发送重置命令到Unity，让Unity管理：
        - 机器人位置重置
        - 球位置随机化
        """
        # 连接训练环境控制器
        self._training_controller.connect()
        
        # 设置随机种子
        if seed is not None:
            self._training_controller.set_seed(seed)

        # 优先使用结构化场景重置；失败时回退到传统 RESET。
        scenario_payload = None
        if options and isinstance(options, dict):
            scenario_payload = options.get("scenario")

        if isinstance(scenario_payload, ResetScenarioV1):
            ok = self._training_controller.reset_scenario(scenario_payload)
            if not ok:
                self._training_controller.reset_episode()
        else:
            self._training_controller.reset_episode()

        # 记录开始时间
        self._episode_start_time = time.time()
        
        # 调用父类reset
        obs, info = super().reset(seed=seed, options=options)
        
        # 添加时间限制信息
        info["episode_time_limit"] = self._episode_time_limit
        self._last_episode_status = self._training_controller.get_episode_status()
        if self._last_episode_status is not None:
            info["episode_status"] = self._last_episode_status.to_dict()
        
        return obs, info
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, dict]:
        """执行动作
        
        添加时间限制检查。
        """
        obs, reward, terminated, truncated, info = super().step(action)

        status = self._training_controller.get_episode_status()
        if status is not None:
            self._last_episode_status = status
            info["episode_status"] = status.to_dict()
            if status.is_done:
                terminated = bool(status.result > 0)
                truncated = not terminated
        
        # 检查时间限制
        if self._episode_start_time is not None:
            elapsed = time.time() - self._episode_start_time
            if elapsed >= self._episode_time_limit:
                truncated = True
                info["time_out"] = True
                info["elapsed_time"] = elapsed
        
        return obs, reward, terminated, truncated, info
    
    def close(self):
        """关闭环境"""
        self._training_controller.close()
        super().close()


def make_env(config: MLPConfig = None, mock: bool = False, training: bool = False) -> gym.Env:
    """创建环境
    
    Args:
        config: MLP配置
        mock: 是否使用mock环境
        training: 是否使用训练环境（带Unity环境管理器集成）
        
    Returns:
        Gym环境实例
    """
    if mock:
        return MockSoccerSimEnv(config)
    if training:
        return TrainingSoccerSimEnv(config)
    return SoccerSimEnv(config)
