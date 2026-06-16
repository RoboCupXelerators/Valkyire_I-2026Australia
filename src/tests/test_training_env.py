#!/usr/bin/env python
"""训练环境测试

测试Unity训练环境管理器的功能：
1. 球门检测
2. 回合重置
3. 时间限制
4. 随机球位置
"""

import os
import sys
import time
import socket

# 设置路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# 设置仿真模式
os.environ["ROBOCUPX_BITS"] = "SIM"


def test_udp_communication():
    """测试UDP通信"""
    print("=" * 60)
    print("测试 UDP 通信 (端口 20003)")
    print("=" * 60)
    
    # 创建UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)
    
    commands = ["RESET", "STATUS", "SEED 42"]
    
    for cmd in commands:
        print(f"\n发送命令: {cmd}")
        try:
            sock.sendto(cmd.encode('utf-8'), ("127.0.0.1", 20003))
            print("  发送成功!")
        except Exception as e:
            print(f"  发送失败: {e}")
    
    sock.close()
    print("\nUDP通信测试完成")


def test_env_controller():
    """测试训练环境控制器"""
    print("\n" + "=" * 60)
    print("测试 TrainingEnvController")
    print("=" * 60)
    
    from mlp.gym_env import TrainingEnvController
    
    controller = TrainingEnvController(ip="127.0.0.1", port=20003)
    controller.connect()
    
    print("\n测试 reset_episode()...")
    result = controller.reset_episode()
    print(f"  结果: {'成功' if result else '失败'}")
    
    print("\n测试 set_seed(12345)...")
    result = controller.set_seed(12345)
    print(f"  结果: {'成功' if result else '失败'}")
    
    controller.close()
    print("\n控制器测试完成")


def test_sim_env():
    """测试仿真环境"""
    print("\n" + "=" * 60)
    print("测试仿真环境")
    print("=" * 60)
    
    from mlp.gym_env import SoccerSimEnv
    from mlp.config import MLPConfig
    
    config = MLPConfig()
    env = SoccerSimEnv(config)
    
    print("\n初始化环境...")
    obs, info = env.reset(seed=42)
    print(f"  观测维度: {obs.shape}")
    print(f"  初始信息: {info}")
    
    print("\n执行随机动作...")
    for i in range(10):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        
        if i % 3 == 0:
            print(f"  Step {i+1}: reward={reward:.2f}, terminated={terminated}")
        
        if terminated or truncated:
            break
    
    env.close()
    print("\n仿真环境测试完成")


def test_training_env():
    """测试训练环境（需要Unity运行）"""
    print("\n" + "=" * 60)
    print("测试 TrainingSoccerSimEnv")
    print("=" * 60)
    print("\n请确保Unity场景正在运行!")
    print("等待5秒...")
    time.sleep(5)
    
    from mlp.gym_env import TrainingSoccerSimEnv
    from mlp.config import MLPConfig
    
    config = MLPConfig()
    env = TrainingSoccerSimEnv(config, training_env_port=20003)
    
    print("\n连接到Unity...")
    if not env.connect():
        print("  连接失败!")
        return
    
    print("  连接成功!")
    
    print("\n重置环境...")
    obs, info = env.reset(seed=42)
    print(f"  观测维度: {obs.shape}")
    print(f"  信息: {info}")
    
    print("\n执行动作 (10步)...")
    for i in range(10):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        
        print(f"  Step {i+1}: reward={reward:.2f}, terminated={terminated}, truncated={truncated}")
        
        if terminated or truncated:
            print(f"  回合结束: {info}")
            break
    
    env.close()
    print("\n训练环境测试完成")


def main():
    print("=" * 60)
    print("训练环境测试套件")
    print("=" * 60)
    
    # 1. 测试UDP通信
    test_udp_communication()
    
    # 2. 测试环境控制器
    test_env_controller()
    
    # 3. 测试仿真环境
    test_sim_env()
    
    # 4. 测试训练环境（需要Unity）
    print("\n是否测试Unity训练环境? (需要Unity场景运行)")
    response = input("输入 y 继续，其他键跳过: ").strip().lower()
    if response == 'y':
        test_training_env()
    
    print("\n" + "=" * 60)
    print("所有测试完成!")
    print("=" * 60)


if __name__ == "__main__":
    main()