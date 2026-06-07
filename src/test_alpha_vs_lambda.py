# -*- coding: utf-8 -*-
"""
test_alpha_vs_lambda.py - Alpha vs Lambda 对比测试

在仿真环境中对比两种策略的表现

作者: 克莱因  日期: 2026-06-06
"""

import sys
import time

# 添加 src 目录到路径
sys.path.insert(0, 'src')

from loguru import logger

# 导入控制器
from alpha import AlphaController
from lambda_strategy import LambdaController


def test_single_strategy(controller_class, name, attack_dir=1, duration_s=30):
    """测试单个策略"""
    logger.info(f"\n{'='*60}")
    logger.info(f"测试策略: {name}")
    logger.info(f"{'='*60}")
    
    controller = controller_class(attack_dir)
    
    start_time = time.time()
    step_count = 0
    
    try:
        while time.time() - start_time < duration_s:
            controller.step()
            step_count += 1
            
            # 每100步打印一次状态
            if step_count % 100 == 0:
                elapsed = time.time() - start_time
                logger.info(f"[{name}] step={step_count} state={controller.state} elapsed={elapsed:.1f}s")
            
            time.sleep(0.02)  # 50Hz
            
    except KeyboardInterrupt:
        logger.warning(f"[{name}] 测试被用户中断")
    finally:
        controller.stop()
    
    elapsed = time.time() - start_time
    logger.info(f"\n[{name}] 测试完成:")
    logger.info(f"  - 总步数: {step_count}")
    logger.info(f"  - 总时间: {elapsed:.1f}s")
    logger.info(f"  - 平均频率: {step_count/elapsed:.1f} Hz")
    
    return {
        'name': name,
        'steps': step_count,
        'duration': elapsed,
        'hz': step_count/elapsed
    }


def compare_strategies():
    """对比两种策略"""
    logger.info("\n" + "="*60)
    logger.info("Alpha vs Lambda 策略对比测试")
    logger.info("="*60)
    
    results = []
    
    # 测试 Alpha
    try:
        result = test_single_strategy(AlphaController, "Alpha", attack_dir=1, duration_s=20)
        results.append(result)
    except Exception as e:
        logger.error(f"Alpha 测试失败: {e}")
    
    # 短暂休息
    time.sleep(2)
    
    # 测试 Lambda
    try:
        result = test_single_strategy(LambdaController, "Lambda", attack_dir=1, duration_s=20)
        results.append(result)
    except Exception as e:
        logger.error(f"Lambda 测试失败: {e}")
    
    # 打印对比结果
    if len(results) == 2:
        logger.info("\n" + "="*60)
        logger.info("对比结果:")
        logger.info("="*60)
        for r in results:
            logger.info(f"\n{r['name']}:")
            logger.info(f"  步数: {r['steps']}")
            logger.info(f"  时长: {r['duration']:.1f}s")
            logger.info(f"  频率: {r['hz']:.1f} Hz")


if __name__ == "__main__":
    compare_strategies()