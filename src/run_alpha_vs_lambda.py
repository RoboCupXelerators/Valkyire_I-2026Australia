# -*- coding: utf-8 -*-
"""
run_alpha_vs_lambda.py - Alpha vs Lambda 对打测试脚本

用法:
    cd Software/src
    python run_alpha_vs_lambda.py

需要:
    - Unity 模拟器运行中
    - 场景中有机器人 (thesus_id=1~4)
    - 配置文件中 RoboInfo.Bit = "SIM"

作者: 克莱因  日期: 2026-06-06
"""

import time
import sys
import os

# 添加父目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from loguru import logger
from LunaPre import UnitedPosition, chassis, peripheral, Settings
from BasicFuc import lockball

from alpha import AlphaController, FIELD_LONG, FIELD_SHORT, GOAL_HALF_WIDTH
from lambda_strategy import LambdaController


class AlphaVsLambdaMatch:
    """Alpha vs Lambda 对打控制器"""
    
    # 球状态常量
    BALL_NOT_FOUND = [0xFFF, 0xFFF]
    BALL_HELD = [0xDDD, 0xDDD]
    
    def __init__(self):
        # Alpha 控制器 (攻击 +X 方向)
        self.alpha = AlphaController(attack_dir=1)
        
        # Lambda 控制器 (攻击 -X 方向)  
        self.lambda_ctrl = LambdaController(attack_dir=-1)
        
        # 比赛统计
        self.stats = {
            'alpha_shots': 0,
            'lambda_shots': 0,
            'alpha_goals': 0,        # Alpha 进攻方向进球
            'alpha_own_goals': 0,    # Alpha 乌龙球
            'lambda_goals': 0,       # Lambda 进攻方向进球
            'lambda_own_goals': 0,   # Lambda 乌龙球
            'alpha_possession_time': 0.0,
            'lambda_possession_time': 0.0,
            'total_steps': 0,
        }
        
        self.last_ball_pos = None
        self.last_goal_check = time.time()
        self.goal_cooldown = 2.0  # 进球后冷却时间
        self.last_goal_time = -999.0
        self.running = False
        self.current_controller = 'alpha'  # 当前控制机器人的策略
        self.switch_interval = 500  # 每500步切换控制权（模拟对战）
        self.step_count = 0
        
    def step(self):
        """执行一步比赛"""
        self.stats['total_steps'] += 1
        self.step_count += 1
        
        # 轮流控制：每 switch_interval 步切换一次
        # Alpha 控制机器人 1, Lambda 控制机器人 2
        # 实际上每个控制器都连接到同一个机器人，所以我们需要模拟对打
        
        # 当前简化实现：让两个策略轮流控制同一个机器人
        # 这不是真正的对打，但可以对比两种策略的行为
        
        try:
            # 获取球的状态判断持球方
            robot_ball = UnitedPosition.RobotBallPos()
            ball_held = robot_ball == [0xDDD, 0xDDD]
            
            if ball_held:
                # 有球的一方进攻
                if self.current_controller == 'alpha':
                    self.stats['alpha_possession_time'] += 0.03
                else:
                    self.stats['lambda_possession_time'] += 0.03
            
            # 执行当前控制器
            if self.current_controller == 'alpha':
                self.alpha.step()
            else:
                self.lambda_ctrl.step()
            
            # 定期切换控制器（模拟不同策略的回合）
            if self.step_count % self.switch_interval == 0:
                self.current_controller = 'lambda' if self.current_controller == 'alpha' else 'alpha'
                logger.info(f"[MATCH] Switch controller to: {self.current_controller}")
            
        except Exception as e:
            logger.error(f"Step error: {e}")
        
        # 检测进球
        self._check_goals()
        
    def _check_goals(self):
        """检测进球（包括乌龙球）"""
        now = time.time()
        
        # 进球冷却期内不检测
        if now - self.last_goal_time < self.goal_cooldown:
            return
            
        if now - self.last_goal_check < 0.3:
            return
        self.last_goal_check = now
        
        try:
            ball = UnitedPosition.FieldBallPos()
            if not ball or ball[0] > 1000:
                return
            
            bx, by = float(ball[0]), float(ball[1])
            self.last_ball_pos = (bx, by)
            
            # 场地参数
            goal_x_threshold = FIELD_LONG / 2.0 - 15.0  # 进球检测阈值
            goal_y_threshold = GOAL_HALF_WIDTH + 15.0
            
            # 检测 +X 球门进球 (X > 115)
            if bx > goal_x_threshold and abs(by) < goal_y_threshold:
                # +X 球门是 Lambda 的进攻目标（Lambda 攻击 -X 方向）
                # 所以球进 +X 球门 = Lambda 得分
                if self.current_controller == 'lambda':
                    self.stats['lambda_goals'] += 1
                    logger.info(f"🎯 [MATCH] GOAL for Lambda! Ball at ({bx:.0f}, {by:.0f})")
                else:
                    # Alpha 把球踢进自己防守的球门 = 乌龙球
                    self.stats['alpha_own_goals'] += 1
                    logger.info(f"❌ [MATCH] OWN GOAL by Alpha! Ball at ({bx:.0f}, {by:.0f})")
                self.last_goal_time = now
            
            # 检测 -X 球门进球 (X < -115)
            elif bx < -goal_x_threshold and abs(by) < goal_y_threshold:
                # -X 球门是 Alpha 的进攻目标（Alpha 攻击 +X 方向）
                # 所以球进 -X 球门 = Alpha 得分
                if self.current_controller == 'alpha':
                    self.stats['alpha_goals'] += 1
                    logger.info(f"🎯 [MATCH] GOAL for Alpha! Ball at ({bx:.0f}, {by:.0f})")
                else:
                    # Lambda 把球踢进自己防守的球门 = 乌龙球
                    self.stats['lambda_own_goals'] += 1
                    logger.info(f"❌ [MATCH] OWN GOAL by Lambda! Ball at ({bx:.0f}, {by:.0f})")
                self.last_goal_time = now
                
        except Exception as e:
            logger.error(f"Check goals error: {e}")
    
    def get_stats(self):
        return self.stats.copy()
    
    def stop(self):
        self.running = False
        self.alpha.stop()
        self.lambda_ctrl.stop()
        try:
            chassis.stop()
            peripheral.Dribble(False)
        except:
            pass


def main():
    """主循环"""
    logger.info("=" * 60)
    logger.info("Alpha vs Lambda 对打测试")
    logger.info("=" * 60)
    logger.info("Alpha: 攻击 +X 方向 (机器人 1)")
    logger.info("Lambda: 攻击 -X 方向 (机器人 2)")
    logger.info("按 Ctrl+C 停止")
    logger.info("")
    
    match = AlphaVsLambdaMatch()
    match.running = True
    
    try:
        step = 0
        while match.running:
            match.step()
            step += 1
            
            # 每 100 步打印统计
            if step % 100 == 0:
                stats = match.get_stats()
                logger.info(
                    f"[Step {step}] "
                    f"Alpha: goals={stats['alpha_goals']} possession={stats['alpha_possession_time']:.1f}s | "
                    f"Lambda: goals={stats['lambda_goals']} possession={stats['lambda_possession_time']:.1f}s"
                )
            
            time.sleep(0.02)  # 50Hz
            
    except KeyboardInterrupt:
        logger.info("正在停止...")
    finally:
        match.stop()
        stats = match.get_stats()
        logger.info("")
        logger.info("=" * 60)
        logger.info("比赛结束 - 最终统计")
        logger.info("=" * 60)
        logger.info(f"总步数: {stats['total_steps']}")
        logger.info(f"Alpha 进球: {stats['alpha_goals']} (乌龙: {stats['alpha_own_goals']})")
        logger.info(f"Lambda 进球: {stats['lambda_goals']} (乌龙: {stats['lambda_own_goals']})")
        logger.info(f"Alpha 控球时间: {stats['alpha_possession_time']:.1f}s")
        logger.info(f"Lambda 控球时间: {stats['lambda_possession_time']:.1f}s")
        
        # 计算净胜球
        alpha_net = stats['alpha_goals'] + stats['lambda_own_goals']  # Alpha 进球 + Lambda 乌龙
        lambda_net = stats['lambda_goals'] + stats['alpha_own_goals']  # Lambda 进球 + Alpha 乌龙
        
        # 判断胜负
        if alpha_net > lambda_net:
            logger.info(f"🏆 胜者: Alpha! (净胜 {alpha_net - lambda_net} 球)")
        elif lambda_net > alpha_net:
            logger.info(f"🏆 胜者: Lambda! (净胜 {lambda_net - alpha_net} 球)")
        else:
            logger.info("🤝 平局!")


if __name__ == "__main__":
    main()