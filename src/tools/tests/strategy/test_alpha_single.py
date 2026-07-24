# -*- coding: utf-8 -*-
"""
test_alpha_single.py - Alpha 单机器人测试脚本

测试单个 Alpha 控制器控制 Robot 1
"""

import time
import sys
import os
from pathlib import Path

SRC_DIR = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(SRC_DIR))

from loguru import logger
from LunaPre import UnitedPosition, chassis, peripheral
from alpha import AlphaController, FIELD_LONG, GOAL_HALF_WIDTH


def main():
    """主循环"""
    logger.info("=== Alpha 单机器人测试 ===")
    logger.info("Alpha: Robot 1 (attack +X)")
    logger.info("Press Ctrl+C to stop")
    
    # Alpha 控制器 (机器人 1 - 攻击 +X)
    alpha = AlphaController(attack_dir=1)
    
    stats = {
        'total_steps': 0,
        'shots': 0,
        'goals': 0,
    }
    
    try:
        step = 0
        while True:
            alpha.step()
            step += 1
            stats['total_steps'] = step
            
            # 每 100 步打印状态
            if step % 100 == 0:
                features = alpha.strategy.state_stats if hasattr(alpha.strategy, 'state_stats') else {}
                logger.info(f"[Step {step}] mode={alpha.strategy.get_mode_name()} state={alpha.strategy.state}")
            
            time.sleep(0.03)  # 30ms 周期
            
    except KeyboardInterrupt:
        logger.info("Stopping...")
    finally:
        alpha.stop()
        logger.info(f"=== Final Stats ===")
        logger.info(f"Total steps: {stats['total_steps']}")


if __name__ == "__main__":
    main()