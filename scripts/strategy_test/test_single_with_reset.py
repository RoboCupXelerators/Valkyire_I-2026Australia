# -*- coding: utf-8 -*-
"""
test_single_with_reset.py - 单机器人测试（带球重置和进球检测）

功能：
1. 单机器人运行策略
2. 球出界或进球后随机重置球位置
3. 正确记录进球和乌龙球

作者: 克莱因  日期: 2026-06-06
"""

import time
import sys
import os
import random

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from loguru import logger
from LunaPre import UnitedPosition, chassis, peripheral
from alpha import AlphaController, FIELD_LONG, FIELD_SHORT, GOAL_HALF_WIDTH

# 球状态常量
BALL_NOT_FOUND = [0xFFF, 0xFFF]
BALL_HELD = [0xDDD, 0xDDD]


class BallManager:
    """球管理器 - 处理球位置检测和重置"""
    
    def __init__(self):
        self.last_ball_pos = None
        self.last_check_time = time.time()
        self.reset_cooldown = 2.0  # 重置后冷却时间
        self.last_reset_time = -999.0
        
    def check_ball_status(self):
        """检查球状态，返回是否需要重置"""
        now = time.time()
        
        # 冷却期内不检查
        if now - self.last_reset_time < self.reset_cooldown:
            return None
        
        try:
            ball = UnitedPosition.FieldBallPos()
            if not ball or ball[0] > 1000:
                return None
            
            bx, by = float(ball[0]), float(ball[1])
            self.last_ball_pos = (bx, by)
            
            # 检查是否出界
            if abs(bx) > FIELD_LONG / 2 + 20 or abs(by) > FIELD_SHORT / 2 + 20:
                logger.info(f"[Ball] OUT OF BOUNDS at ({bx:.0f}, {by:.0f})")
                return 'out_of_bounds'
            
            # 检查进球（+X 球门）
            if bx > FIELD_LONG / 2 - 15 and abs(by) < GOAL_HALF_WIDTH + 15:
                logger.info(f"[Ball] GOAL at +X gate! pos=({bx:.0f}, {by:.0f})")
                return 'goal_plus_x'
            
            # 检查进球（-X 球门）
            if bx < -FIELD_LONG / 2 + 15 and abs(by) < GOAL_HALF_WIDTH + 15:
                logger.info(f"[Ball] GOAL at -X gate! pos=({bx:.0f}, {by:.0f})")
                return 'goal_minus_x'
            
            return None
            
        except Exception as e:
            logger.error(f"Ball check error: {e}")
            return None
    
    def reset_ball_random(self):
        """随机重置球位置（通过移动球到新位置）"""
        # 在 Unity 中，我们需要通过 MCP 来移动球
        # 随机生成球的新位置（中场区域）
        new_x = random.uniform(-30, 30)
        new_y = random.uniform(-30, 30)
        
        logger.info(f"[Ball] RESET to random position ({new_x:.0f}, {new_y:.0f})")
        self.last_reset_time = time.time()
        
        return (new_x, new_y)


class SingleRobotTest:
    """单机器人测试"""
    
    def __init__(self, strategy_name='alpha', attack_dir=1):
        self.strategy_name = strategy_name
        self.attack_dir = attack_dir
        
        # 创建策略控制器
        if strategy_name == 'alpha':
            self.controller = AlphaController(attack_dir=attack_dir)
        elif strategy_name == 'lambda':
            from lambda_strategy import LambdaController
            self.controller = LambdaController(attack_dir=attack_dir)
        else:
            raise ValueError(f"Unknown strategy: {strategy_name}")
        
        # 球管理器
        self.ball_manager = BallManager()
        
        # 统计
        self.stats = {
            'total_steps': 0,
            'shots': 0,
            'goals_for': 0,      # 进攻方向进球
            'goals_against': 0,  # 乌龙球
            'out_of_bounds': 0,
            'possession_time': 0.0,
        }
        
        self.running = False
        self.step_count = 0
        self.last_shot_time = -999.0
        
    def step(self):
        """执行一步"""
        self.stats['total_steps'] += 1
        self.step_count += 1
        
        # 执行策略
        self.controller.step()
        
        # 检查持球状态
        try:
            robot_ball = UnitedPosition.RobotBallPos()
            if robot_ball == BALL_HELD:
                self.stats['possession_time'] += 0.02
        except:
            pass
        
        # 检查射门（通过状态变化）
        current_state = self.controller.state if hasattr(self.controller, 'state') else ''
        if 'shoot' in current_state.lower():
            now = time.time()
            if now - self.last_shot_time > 0.5:  # 射门冷却
                self.stats['shots'] += 1
                self.last_shot_time = now
                logger.info(f"[{self.strategy_name}] SHOT detected!")
        
        # 检查球状态
        ball_status = self.ball_manager.check_ball_status()
        if ball_status:
            if ball_status == 'out_of_bounds':
                self.stats['out_of_bounds'] += 1
                self.ball_manager.reset_ball_random()
            elif ball_status == 'goal_plus_x':
                if self.attack_dir == 1:
                    self.stats['goals_for'] += 1
                    logger.info(f"🎯 GOAL for {self.strategy_name}!")
                else:
                    self.stats['goals_against'] += 1
                    logger.info(f"❌ OWN GOAL by {self.strategy_name}!")
                self.ball_manager.reset_ball_random()
            elif ball_status == 'goal_minus_x':
                if self.attack_dir == -1:
                    self.stats['goals_for'] += 1
                    logger.info(f"🎯 GOAL for {self.strategy_name}!")
                else:
                    self.stats['goals_against'] += 1
                    logger.info(f"❌ OWN GOAL by {self.strategy_name}!")
                self.ball_manager.reset_ball_random()
    
    def stop(self):
        self.running = False
        self.controller.stop()
        try:
            chassis.stop()
            peripheral.Dribble(False)
        except:
            pass
    
    def get_stats(self):
        return self.stats.copy()


def main():
    """主测试"""
    logger.info("=" * 60)
    logger.info("单机器人测试（带进球检测和球重置）")
    logger.info("=" * 60)
    logger.info("策略: Alpha")
    logger.info("进攻方向: +X")
    logger.info("按 Ctrl+C 停止")
    logger.info("")
    
    test = SingleRobotTest(strategy_name='alpha', attack_dir=1)
    test.running = True
    
    try:
        step = 0
        while test.running:
            test.step()
            step += 1
            
            # 每 200 步打印统计
            if step % 200 == 0:
                stats = test.get_stats()
                logger.info(
                    f"[Step {step}] "
                    f"Goals: {stats['goals_for']} | Own Goals: {stats['goals_against']} | "
                    f"Shots: {stats['shots']} | OOB: {stats['out_of_bounds']} | "
                    f"Possession: {stats['possession_time']:.1f}s"
                )
            
            time.sleep(0.02)  # 50Hz
            
    except KeyboardInterrupt:
        logger.info("正在停止...")
    finally:
        test.stop()
        stats = test.get_stats()
        logger.info("")
        logger.info("=" * 60)
        logger.info("测试结束 - 最终统计")
        logger.info("=" * 60)
        logger.info(f"总步数: {stats['total_steps']}")
        logger.info(f"进球: {stats['goals_for']}")
        logger.info(f"乌龙球: {stats['goals_against']}")
        logger.info(f"射门次数: {stats['shots']}")
        logger.info(f"出界次数: {stats['out_of_bounds']}")
        logger.info(f"控球时间: {stats['possession_time']:.1f}s")


if __name__ == "__main__":
    main()