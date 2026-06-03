# -*- coding: utf-8 -*-
"""
alpha_test.py - 测试 alpha.py 策略沙盒

用法：
    python src/tests/alpha_test.py --robot 1 --duration 30
    
需要先启动 Unity 仿真器（SoccerSimulator 场景）。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import logging
from pathlib import Path
from typing import Any

# 设置环境变量
SOFTWARE_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SOFTWARE_ROOT / "src"))

os.environ.setdefault("ROBOCUPX_BITS", "SIM")

# 设置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger("alpha_test")


def run_alpha_loop(robot_id: int, attack_dir: int, duration: float) -> dict:
    """
    运行 alpha.py 策略循环
    
    参数:
        robot_id: 机器人 ID (1-4)
        attack_dir: 进攻方向 (1 或 -1)
        duration: 运行时长 (秒)
    
    返回:
        统计数据字典
    """
    # 设置机器人 ID
    os.environ["XEL_ROBOT_ID"] = str(robot_id)
    
    # 导入 alpha.py 和基础组件
    from alpha import AlphaController, AlphaFeatures
    from LunaPre import UnitedPosition
    
    # 创建控制器
    controller = AlphaController(attack_dir=attack_dir)
    
    # 统计
    stats = {
        'robot_id': robot_id,
        'attack_dir': attack_dir,
        'duration': duration,
        'steps': 0,
        'modes': {0: 0, 1: 0, 2: 0},  # STRIKING, LATERAL, INTERCEPT
        'shots': 0,
        'goals': 0,
    }
    
    start_time = time.time()
    last_log_time = start_time
    
    logger.info(f"[Alpha Test] Robot {robot_id} | attack_dir={attack_dir} | duration={duration}s")
    
    while time.time() - start_time < duration:
        # 执行一步策略
        result = controller.run_step()
        stats['steps'] += 1
        stats['modes'][result['mode']] += 1
        
        # 每 5 秒打印状态
        if time.time() - last_log_time > 5.0:
            last_log_time = time.time()
            features = result['features']
            if features:
                logger.info(
                    f"[R{robot_id}] mode={result['mode_name']} "
                    f"pos=({features['robot_pos'][0]:.1f},{features['robot_pos'][1]:.1f}) "
                    f"ball=({features['ball_pos'][0]:.1f},{features['ball_pos'][1]:.1f}) "
                    f"rbg={features['rbg_angle']:.1f}° "
                    f"dist={features['dist_rb']:.1f}cm"
                )
        
        # 小延迟避免过快循环
        time.sleep(0.02)
    
    # 打印统计
    logger.info(f"[Alpha Test Complete] Robot {robot_id}")
    logger.info(f"  Steps: {stats['steps']}")
    logger.info(f"  Modes: STRIKING={stats['modes'][0]}, LATERAL={stats['modes'][1]}, INTERCEPT={stats['modes'][2]}")
    
    return stats


def main():
    parser = argparse.ArgumentParser(description="Alpha Strategy Test Runner")
    parser.add_argument("--robot", type=int, default=1, help="Robot ID (1-4)")
    parser.add_argument("--attack_dir", type=int, default=1, help="Attack direction (1 or -1)")
    parser.add_argument("--duration", type=float, default=30.0, help="Test duration (seconds)")
    args = parser.parse_args()
    
    # 检查仿真器是否运行
    logger.info("Checking simulator connection...")
    try:
        from LunaPre import UnitedPosition
        pose = UnitedPosition.FieldRobotPos()
        if pose is None or len(pose) < 2:
            logger.warning("Simulator not responding - is Unity running?")
    except Exception as e:
        logger.warning(f"Simulator check failed: {e}")
    
    # 运行测试
    stats = run_alpha_loop(args.robot, args.attack_dir, args.duration)
    
    # 打印最终统计
    print("\n" + "=" * 60)
    print("Alpha Test Summary")
    print("=" * 60)
    print(json.dumps(stats, indent=2))
    print("=" * 60)


if __name__ == "__main__":
    main()