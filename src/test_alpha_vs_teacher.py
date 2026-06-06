# -*- coding: utf-8 -*-
"""
test_alpha_vs_teacher.py - Alpha vs Teacher 对打测试脚本

用法:
    cd Software/src
    python test_alpha_vs_teacher.py

需要:
    - Unity 模拟器运行中 (MCP Unity 连接)
    - 场景中有 4 个机器人 (thesus_id=1~4)
"""

import time
import sys
import os

# 添加父目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from loguru import logger
from LunaPre import UnitedPosition, chassis, peripheral, Settings
from BasicFuc import lockball

from alpha import AlphaController, AlphaFeatures, MODE_STRIKING, MODE_LATERAL_DRIBBLE, MODE_INTERCEPT, MODE_RECOVER
from teacher.rule_teacher import RuleTeacher, TeacherConfig


class MatchController:
    """控制 Alpha vs Teacher 对打"""
    
    def __init__(self):
        # Alpha 控制器 (机器人 1 - 攻击 +X)
        # 注意：只使用一个 Alpha 实例，因为 AlphaController 内部已经连接到 Unity
        self.alpha1 = AlphaController(attack_dir=1)
        
        # Teacher 控制器 (机器人 3 - 攻击 -X)
        # 注意：Teacher 需要 Robot 3 的数据，但当前连接到 Robot 1
        # 暂时禁用 Teacher，只测试 Alpha 单机器人
        # cfg = TeacherConfig()
        # self.teacher1 = RuleTeacher(cfg)
        
        # 比赛统计
        self.stats = {
            'alpha_shots': 0,
            'teacher_shots': 0,
            'alpha_goals': 0,
            'teacher_goals': 0,
            'alpha_possession': 0,
            'teacher_possession': 0,
            'total_steps': 0,
        }
        
        self.last_ball_pos = None
        self.last_goal_check = time.time()
        self.running = False
        
    def step(self):
        """执行一步比赛"""
        self.stats['total_steps'] += 1
        
        # 获取所有机器人和球的状态
        poses = {}
        for i in [1, 2, 3, 4]:
            # 模拟获取机器人位姿
            try:
                pose = UnitedPosition.FieldRobotPos()
                poses[i] = pose
            except:
                poses[i] = None
        
        # Alpha 机器人 (1, 2)
        try:
            self.alpha1.step()
            self.alpha2.step()
        except Exception as e:
            logger.error(f"Alpha error: {e}")
        
        # Teacher 机器人 (3, 4) - 需要转换坐标系
        # Teacher 攻击 -X，需要镜像坐标
        try:
            # 构建 DeployObservationV1
            from mlp.contracts import DeployObservationV1
            obs = self._build_observation(3, attack_dir=-1)
            if obs:
                decision = self.teacher1.decide(obs)
                self._execute_teacher_decision(3, decision, attack_dir=-1)
        except Exception as e:
            logger.error(f"Teacher error: {e}")
        
        # 检测进球
        self._check_goals()
        
    def _build_observation(self, robot_id: int, attack_dir: int) -> object:
        """构建 Teacher 所需的观察数据"""
        try:
            pose = UnitedPosition.FieldRobotPos()
            if not pose or len(pose) < 3:
                return None
            
            # 镜像坐标（Teacher 攻击 -X）
            x = pose[0] * attack_dir
            y = pose[1] * attack_dir
            yaw = pose[2]
            if attack_dir < 0:
                yaw = (yaw + 180) % 360
            
            ball = UnitedPosition.RobotBallPos()
            ball_visible = ball != [0xFFF, 0xFFF] and ball != [0xDDD, 0xDDD]
            has_ball = ball == [0xDDD, 0xDDD]
            
            ball_rel_x = 0.0
            ball_rel_y = 0.0
            if ball and ball_visible:
                ball_rel_x = float(ball[0])
                ball_rel_y = float(ball[1])
            
            # 构建敌人列表（Alpha 机器人）
            enemies = []
            for eid in [1, 2]:
                # 简化：假设敌人可见
                enemies.append(type('Enemy', (), {
                    'visible': True,
                    'rel_x': 100.0,  # 简化
                    'rel_y': 0.0,
                })())
            
            from mlp.contracts import DeployObservationV1
            return DeployObservationV1(
                self_x=x,
                self_y=y,
                self_yaw=yaw,
                ball_visible=ball_visible,
                ball_rel_x=ball_rel_x,
                ball_rel_y=ball_rel_y,
                has_ball=has_ball,
                enemies=enemies,
            )
        except Exception as e:
            logger.error(f"Build observation error: {e}")
            return None
    
    def _execute_teacher_decision(self, robot_id: int, decision: object, attack_dir: int):
        """执行 Teacher 的决策"""
        packet = decision.packet
        
        # 镜像目标坐标
        target_x = packet.target_x * attack_dir
        target_y = packet.target_y * attack_dir
        target_yaw = packet.target_yaw
        if attack_dir < 0:
            target_yaw = (target_yaw + 180) % 360
        
        move_gain = packet.move_gain
        
        # 执行移动
        if packet.intent_id == 0:  # AVOID_WALL
            UnitedPosition.Pos2Pos([target_x, target_y, target_yaw], move_gain)
        elif packet.intent_id == 1:  # SEARCH_BALL
            UnitedPosition.Pos2Pos([target_x, target_y, target_yaw], move_gain)
        elif packet.intent_id == 2:  # APPROACH_BALL
            lockball.LockBallSlip()
        elif packet.intent_id == 3:  # CARRY_TO_POSE
            chassis.FieldMoveVector(
                (target_x - decision.packet.target_x) * move_gain,
                (target_y - decision.packet.target_y) * move_gain,
                target_yaw
            )
            peripheral.Dribble(0.75)
        elif packet.intent_id == 4:  # SHOOT
            peripheral.ShootBall()
            self.stats['teacher_shots'] += 1
            logger.info(f"[Teacher {robot_id}] SHOOT!")
    
    def _check_goals(self):
        """检测进球"""
        now = time.time()
        if now - self.last_goal_check < 0.5:
            return
        self.last_goal_check = now
        
        try:
            ball = UnitedPosition.FieldBallPos()
            if not ball or ball[0] > 1000:
                return
            
            bx, by = float(ball[0]), float(ball[1])
            
            # 检测 Alpha 进球 (球在 +X 球门)
            if bx > FIELD_LONG / 2.0 - 10.0 and abs(by) < GOAL_HALF_WIDTH + 10:
                self.stats['alpha_goals'] += 1
                logger.info(f"[MATCH] GOAL for Alpha! Ball at ({bx:.0f}, {by:.0f})")
                self._reset_ball()
            
            # 检测 Teacher 进球 (球在 -X 球门)
            if bx < -FIELD_LONG / 2.0 + 10.0 and abs(by) < GOAL_HALF_WIDTH + 10:
                self.stats['teacher_goals'] += 1
                logger.info(f"[MATCH] GOAL for Teacher! Ball at ({bx:.0f}, {by:.0f})")
                self._reset_ball()
            
            self.last_ball_pos = (bx, by)
        except Exception as e:
            logger.error(f"Check goals error: {e}")
    
    def _reset_ball(self):
        """重置球到中场"""
        # Unity 会自动处理，这里只是日志
        logger.info("[MATCH] Ball reset to center")
    
    def get_stats(self) -> dict:
        return self.stats.copy()
    
    def stop(self):
        self.running = False
        self.alpha1.stop()
        self.alpha2.stop()
        try:
            chassis.stop()
            peripheral.Dribble(False)
        except:
            pass


def main():
    """主循环"""
    logger.info("=== Alpha vs Teacher 对打测试 ===")
    logger.info("Alpha: Robot 1, 2 (attack +X)")
    logger.info("Teacher: Robot 3, 4 (attack -X)")
    logger.info("Press Ctrl+C to stop")
    
    match = MatchController()
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
                    f"Alpha: shots={stats['alpha_shots']} goals={stats['alpha_goals']} | "
                    f"Teacher: shots={stats['teacher_shots']} goals={stats['teacher_goals']} | "
                    f"possession: A={stats['alpha_possession']} T={stats['teacher_possession']}"
                )
            
            time.sleep(0.03)  # 30ms 周期
            
    except KeyboardInterrupt:
        logger.info("Stopping...")
    finally:
        match.stop()
        stats = match.get_stats()
        logger.info("=== Final Stats ===")
        logger.info(f"Total steps: {stats['total_steps']}")
        logger.info(f"Alpha goals: {stats['alpha_goals']}")
        logger.info(f"Teacher goals: {stats['teacher_goals']}")


if __name__ == "__main__":
    from alpha import FIELD_LONG, FIELD_SHORT, GOAL_HALF_WIDTH
    main()