# -*- coding: utf-8 -*-
"""
lambda.py - 快速响应策略 v1.0

核心设计原则（针对 Alpha 的弱点）：
  1. 快速抢球：比 Alpha 更激进地获取球权
  2. 直接射门：放宽射门条件，只要有角度就射
  3. 灵活运球：不依赖固定模式，实时决策
  4. 快速变向：遇到阻挡立即变向
  5. 预测拦截：提前预判球的位置

不新建数据类型，只使用 BasicFuc 和 LunaPre 提供的接口

作者: 克莱因  日期: 2026-06-06
"""

import math
import time

from loguru import logger

from LunaPre import UnitedPosition, chassis, peripheral
from LunaPre.strategy_tools import (
    BALL_HELD,
    BALL_NOT_FOUND,
    angle_error_deg,
    distance,
    robot_ball_distance,
    safe_stop,
)
from BasicFuc import lockball

# 场地参数
FIELD_LONG = 260.0
FIELD_SHORT = 200.0
GOAL_HALF_WIDTH = 24.0

# Lambda 策略参数（比 Alpha 更激进）
LAMBDA_INTERCEPT_GAIN = 2.0       # 更快的抢球速度
LAMBDA_CARRY_GAIN = 1.8           # 更快的带球速度
LAMBDA_SHOOT_PROGRESS_MIN = 35.0  # 更低的射门门槛
LAMBDA_SHOOT_YAW_TOL = 25.0       # 更宽的射门角度容差
LAMBDA_DRIBBLE_POWER = 0.85       # 更强的运球功率

# 快速变向参数
QUICK_TURN_FRAMES = 5
QUICK_TURN_DURATION_S = 0.3
STUCK_CHECK_FRAMES = 20
STUCK_MIN_STILL_S = 0.8
STUCK_POSITION_THRESHOLD_CM = 2.0
STUCK_NEAR_BALL_IGNORE_CM = 40.0

# 射门冷却
SHOT_COOLDOWN_S = 0.5


# ============================================================================
# Lambda 策略类
# ============================================================================
class LambdaController:
    """
    Lambda 控制器：快速响应策略
    
    设计思路：
    1. 更激进的抢球 - 优先获取球权
    2. 更快的射门 - 只要有角度就射
    3. 更灵活的脱困 - 快速变向不卡死
    """
    
    def __init__(self, attack_dir=1):
        self.attack_dir = attack_dir
        self.goal_x = attack_dir * FIELD_LONG / 2.0
        self.state = "init"
        
        # 状态变量
        self._last_shot_ts = -999.0
        self._last_dribble = None
        self._pos_history = []
        self._escaping = False
        self._escape_start = None
        self._escape_dir = 1
        self._stuck_since = None
        self._step_count = 0
        self._last_ball_pos = None
        self._last_ball_time = None
    
    def step(self):
        """执行一步策略"""
        self._step_count += 1
        
        # 获取传感器数据
        robot_pos = UnitedPosition.FieldRobotPos()
        robot_ball = UnitedPosition.RobotBallPos()
        field_ball = UnitedPosition.FieldBallPos()
        
        # 检查数据有效性
        if not robot_pos or robot_pos[0] > 500:
            self.state = "no_pos"
            return
        
        rx, ry, ryaw = float(robot_pos[0]), float(robot_pos[1]), float(robot_pos[2])
        
        # 记录位置历史（用于卡住检测）
        self._pos_history.append((rx, ry, time.time()))
        if len(self._pos_history) > STUCK_CHECK_FRAMES + 2:
            self._pos_history.pop(0)
        
        # 判断球的状态
        ball_held = (robot_ball == BALL_HELD)
        ball_found = (robot_ball != BALL_NOT_FOUND) or (field_ball and field_ball[0] < 500)
        robot_ball_dist = robot_ball_distance(robot_ball)
        
        # 计算进度（进攻方向）
        progress = self.attack_dir * rx
        
        # 计算到球门的距离和角度
        goal_dist = distance((rx, ry), (self.goal_x, 0))
        goal_angle = math.degrees(math.atan2(0 - ry, self.goal_x - rx)) % 360
        yaw_error = abs(angle_error_deg(ryaw, goal_angle))
        
        # ===== 卡住检测与脱困 =====
        if self._check_stuck(robot_ball_dist=robot_ball_dist, ball_held=ball_held):
            self._execute_escape(ryaw)
            return
        
        # ===== 边界保护 =====
        boundary_dist = min(
            FIELD_LONG / 2 - abs(rx),
            FIELD_SHORT / 2 - abs(ry)
        )
        if boundary_dist < 15 and not ball_held:
            self.state = "avoid_wall"
            safe = UnitedPosition.ProjectSafePose([rx, ry, ryaw])
            if safe:
                UnitedPosition.Pos2Pos(safe, 1.2)
            return
        
        # ===== 持球状态 =====
        if ball_held:
            self._escaping = False
            self._stuck_since = None
            self._pos_history = []
            self._execute_carry(rx, ry, ryaw, progress, yaw_error, goal_dist)
            return
        
        # ===== 无球状态 =====
        self._set_dribble(False)
        
        # 快速抢球
        if ball_found:
            self._execute_acquire(robot_ball, field_ball, rx, ry, ryaw)
        else:
            # 球完全丢失，回到中场
            self.state = "search"
            approach_yaw = 0 if self.attack_dir >= 0 else 180
            UnitedPosition.Pos2Pos([0, 0, approach_yaw], 1.0)
    
    def _execute_carry(self, rx, ry, ryaw, progress, yaw_error, goal_dist):
        """持球推进逻辑"""
        self.state = "carry"
        self._set_dribble(LAMBDA_DRIBBLE_POWER)
        
        # 射门判断 - 更激进
        can_shoot = (
            progress >= LAMBDA_SHOOT_PROGRESS_MIN and
            yaw_error <= LAMBDA_SHOOT_YAW_TOL and
            time.time() - self._last_shot_ts > SHOT_COOLDOWN_S
        )
        
        if can_shoot:
            # 射门！
            self.state = "shoot"
            chassis.stop()
            peripheral.ShootBall()
            self._set_dribble(False)
            self._last_shot_ts = time.time()
            if self._step_count % 50 == 0:
                logger.info(f"[lambda] SHOOT! pos=({rx:.0f},{ry:.0f}) progress={progress:.0f}")
            return
        
        # 带球推进
        # 目标：快速向球门推进，同时调整 Y 位置
        target_x = self.attack_dir * min(abs(rx) + 35, FIELD_LONG / 2 - 18)
        
        # 选择最佳射门角度
        aim_y = -GOAL_HALF_WIDTH * 0.5 if ry > 0 else GOAL_HALF_WIDTH * 0.5
        target_y = aim_y * 0.4 + ry * (-0.15)  # 向中间收缩
        
        speed_x = (target_x - rx) * LAMBDA_CARRY_GAIN
        speed_y = (target_y - ry) * LAMBDA_CARRY_GAIN
        
        # 限制速度
        max_speed = 80.0
        speed_mag = math.sqrt(speed_x**2 + speed_y**2)
        if speed_mag > max_speed:
            scale = max_speed / speed_mag
            speed_x *= scale
            speed_y *= scale
        
        # 目标朝向
        goal_yaw = math.degrees(math.atan2(0 - target_y, self.goal_x - target_x)) % 360
        
        chassis.FieldMoveVector(speed_x, speed_y, goal_yaw)
    
    def _execute_acquire(self, robot_ball, field_ball, rx, ry, ryaw):
        """抢球逻辑"""
        self.state = "acquire"
        
        # 使用 LockBallSlip 抢球
        if robot_ball != BALL_NOT_FOUND and robot_ball != BALL_HELD:
            ball_dist = robot_ball_distance(robot_ball)
            if ball_dist is not None and ball_dist < 60:
                # 球很近，用 LockBallSlip
                lockball.LockBallSlip()
                return
        
        # 球较远，快速接近
        if field_ball and field_ball[0] < 500:
            target_x = float(field_ball[0]) - self.attack_dir * 8
            target_y = float(field_ball[1])
            approach_yaw = 0 if self.attack_dir >= 0 else 180
            UnitedPosition.Pos2Pos([target_x, target_y, approach_yaw], LAMBDA_INTERCEPT_GAIN)
        else:
            # 用机体坐标球的相对位置
            if robot_ball != BALL_NOT_FOUND and robot_ball != BALL_HELD:
                bx, by = float(robot_ball[0]), float(robot_ball[1])
                # 预测拦截点
                target_x = rx + bx * 1.2
                target_y = ry + by * 1.2
                approach_yaw = 0 if self.attack_dir >= 0 else 180
                UnitedPosition.Pos2Pos([target_x, target_y, approach_yaw], LAMBDA_INTERCEPT_GAIN)
    
    def _execute_escape(self, ryaw):
        """脱困逻辑"""
        if not self._escaping:
            self._escaping = True
            self._escape_start = time.time()
            self._escape_dir *= -1
            if self._step_count % 20 == 0:
                logger.info(f"[lambda] STUCK! Escaping dir={self._escape_dir}")
        
        self.state = "escape"
        self._set_dribble(False)
        
        # 快速后退 + 横移
        speed_x = -self.attack_dir * 30
        speed_y = self._escape_dir * 50
        chassis.FieldMoveVector(speed_x, speed_y, ryaw)
        
        # 检查是否脱困完成
        if self._escape_start and time.time() - self._escape_start > QUICK_TURN_DURATION_S:
            self._escaping = False
            self._escape_start = None
            self._stuck_since = None
            self._pos_history = []
    
    def _check_stuck(self, *, robot_ball_dist=None, ball_held=False):
        """检测是否卡住"""
        if self._escaping:
            return True
        if ball_held:
            self._stuck_since = None
            return False
        if robot_ball_dist is not None and robot_ball_dist <= STUCK_NEAR_BALL_IGNORE_CM:
            self._stuck_since = None
            return False
        if len(self._pos_history) < STUCK_CHECK_FRAMES:
            return False
        
        recent = self._pos_history[-STUCK_CHECK_FRAMES:]
        x0, y0, t0 = recent[0]
        x1, y1, t1 = recent[-1]
        
        if t1 - t0 < STUCK_MIN_STILL_S:
            return False
        
        displacement = math.sqrt((x1-x0)**2 + (y1-y0)**2)
        if displacement < STUCK_POSITION_THRESHOLD_CM:
            self._stuck_since = self._stuck_since or time.time()
            return time.time() - self._stuck_since >= STUCK_MIN_STILL_S

        self._stuck_since = None
        return False
    
    def _set_dribble(self, power):
        """设置运球"""
        if self._last_dribble == power:
            return
        peripheral.Dribble(power)
        self._last_dribble = power
        try:
            lockball._lockball_dribble_state = power if power else False
        except Exception:
            pass
    
    def stop(self):
        """停止"""
        safe_stop(chassis, peripheral)


# ============================================================================
# 便捷函数
# ============================================================================
def lambda_main(attack_dir=1):
    """Lambda 策略主循环示例"""
    controller = LambdaController(attack_dir)
    
    while True:
        controller.step()
        time.sleep(0.02)


if __name__ == "__main__":
    lambda_main()
