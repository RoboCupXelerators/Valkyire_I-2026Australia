# -*- coding: utf-8 -*-
"""
alpha.py - 几何特征驱动的策略实验沙盒 v5.1

核心设计原则：
  1. 以 R-B-G angle（Robot-Ball-Goal 角度）为核心判别特征
  2. 多模式状态机：STRIKING / LATERAL_DRIBBLE / INTERCEPT / RECOVER / POSITIONING
  3. 不修改基础组件（LunaPre.py, chassis.py, peripheral）
  4. 几何特征层用于后续 MLP 训练标签
  5. 纯 Python，无 numpy 等外部依赖

v5.1 修复：移除 numpy 依赖，所有向量运算改为纯 Python

v5 新增逻辑：
  - RECOVER 模式：球丢失后主动用 FieldBallPos + FieldMoveVector 追球
  - POSITIONING 模式：无球时根据场地位置选择最佳站位
  - 动态运球功率：根据速度和转向调整 Dribble 功率
  - 卡死检测：连续 N 帧位置不变则触发脱困
  - 射门角度优化：选择球门最大空隙点
  - 球运动预测：根据球位置历史预测拦截点
  - 边界保护增强：多级边界距离响应
  - 持球稳定性检测：球在控球区边缘时降低速度
  - 进攻节奏控制：避免盲目冲入禁区
  - 紧急回防：球朝己方球门运动时优先拦截

作者: 克莱因  日期: 2026-06-03
"""

import math
import time
from typing import Optional, Dict, Any, List, Tuple

from loguru import logger

from LunaPre import UnitedPosition, chassis, peripheral
from BasicFuc import lockball

# ============================================================================
# 常量
# ============================================================================
BALL_NOT_FOUND = [0xFFF, 0xFFF]
BALL_HELD = [0xDDD, 0xDDD]

# 模式常量
MODE_STRIKING = 0
MODE_LATERAL_DRIBBLE = 1
MODE_INTERCEPT = 2
MODE_RECOVER = 3        # v5: 球丢失后主动追球
MODE_POSITIONING = 4    # v5: 无球时站位

# 几何特征阈值
RBG_ANGLE_STRIKE_THRESHOLD = 150.0
RBG_ANGLE_LATERAL_THRESHOLD = 120.0
BALL_CLOSE_DIST = 50.0
BALL_FAR_DIST = 120.0
BALL_MEDIUM_DIST = 80.0
DEEP_PROGRESS_THRESHOLD = 30.0
SHOOT_PROGRESS_THRESHOLD = 50.0
SHOOT_ALIGNMENT_THRESHOLD = 25.0
DIRECT_SHOT_ALIGNMENT_DEG = 15.0
DIRECT_SHOT_MIN_PROGRESS = 45.0
COMMITTED_SHOT_PROGRESS = 55.0
STALLED_SHOT_MIN_PROGRESS = 24.0
STALLED_SHOT_ALIGNMENT_DEG = 35.0

# 运动参数
STRIKE_SPEED_GAIN = 1.6
LATERAL_SPEED_GAIN = 1.4
INTERCEPT_SPEED_GAIN = 1.4
RECOVER_SPEED_GAIN = 1.6
POSITIONING_SPEED_GAIN = 1.0
CARRY_DRIBBLE_POWER = 0.72
SECURE_DRIBBLE_POWER = 0.82
APPROACH_DRIBBLE_POWER = 0.35
MAX_FIELD_SPEED = 72.0

# 场地参数
FIELD_LONG = 260.0
FIELD_SHORT = 200.0
GOAL_HALF_WIDTH = 24.0

# 模式切换滞后
MODE_SWITCH_FRAMES = 2

# 射门冷却
SHOT_COOLDOWN_S = 0.8
SHOOT_YAW_TOLERANCE_DEG = 12.0  # v5.2: 放宽角度容差

# v5: 卡死检测
STUCK_CHECK_FRAMES = 60
STUCK_POSITION_THRESHOLD = 3.0
STUCK_ESCAPE_DURATION_S = 0.5
CARRY_STALL_S = 1.0
CARRY_STALL_MOVE_CM = 3.0

# v5: 边界保护分级
BOUNDARY_DANGER_DIST = 15.0
BOUNDARY_CAUTION_DIST = 30.0
BOUNDARY_SOFT_DIST = 50.0

# v5: 球运动预测
BALL_PREDICTION_FRAMES = 5
BALL_PREDICTION_MAX_DIST = 80.0

# v5: 持球稳定性
BALL_EDGE_DIST = 12.0
STABILITY_SPEED_FACTOR = 0.6
POSSESSION_GRACE_S = 0.22
POSSESSION_GRACE_DIST = 28.0
POSSESSION_SETTLE_S = 0.45
SETTLE_CARRY_SPEED = 24.0
LATERAL_TARGET_Y = 12.0
LATERAL_CARRY_Y_LIMIT = 42.0
BLOCKING_ENEMY_FORWARD_CM = 55.0
BLOCKING_ENEMY_LATERAL_CM = 35.0
BLOCKING_ENEMY_DODGE_CM = 34.0
BLOCKING_ENEMY_FORWARD_CAP_CM = 28.0
BLOCKING_EDGE_FORWARD_CAP_CM = 36.0
BLOCKING_ENEMY_MIN_LATERAL_CM = 32.0

# v5: 站位参数
POSITIONING_DEFENSE_X = -40.0
POSITIONING_MIDFIELD_X = 0.0
POSITIONING_ATTACK_X = 40.0
POSITIONING_Y_SPREAD = 60.0


# ============================================================================
# 纯 Python 向量工具（替代 numpy）
# ============================================================================
def _vec_add(a, b):
    return (a[0] + b[0], a[1] + b[1])

def _vec_sub(a, b):
    return (a[0] - b[0], a[1] - b[1])

def _vec_dot(a, b):
    return a[0] * b[0] + a[1] * b[1]

def _vec_norm(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1])

def _angle_error_deg(current: float, target: float) -> float:
    return ((float(target) - float(current) + 180.0) % 360.0) - 180.0

def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))

def _normalize_angle(angle_deg: float) -> float:
    while angle_deg > 180: angle_deg -= 360
    while angle_deg < -180: angle_deg += 360
    return angle_deg

def _angle_between_vectors(v1, v2) -> float:
    """计算两个 2D 向量之间的夹角（度）"""
    n1, n2 = _vec_norm(v1), _vec_norm(v2)
    if n1 < 1e-6 or n2 < 1e-6:
        return 0.0
    cos_angle = _vec_dot(v1, v2) / (n1 * n2)
    cos_angle = max(-1.0, min(1.0, cos_angle))
    return math.degrees(math.acos(cos_angle))

def _dist(p1, p2) -> float:
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.sqrt(dx * dx + dy * dy)

def _limit_speed(sx, sy, mx) -> Tuple[float, float]:
    m = math.sqrt(sx * sx + sy * sy)
    if m > mx and m > 1e-6:
        s = mx / m
        return sx * s, sy * s
    return sx, sy


# ============================================================================
# 几何特征提取层
# ============================================================================
class AlphaFeatures:
    _ball_history: List[Tuple[float, float, float]] = []
    _max_history = 10

    @classmethod
    def reset_history(cls):
        cls._ball_history = []

    @staticmethod
    def get_features(attack_dir: int = 1) -> Optional[Dict[str, Any]]:
        pose = UnitedPosition.FieldRobotPos()
        if not pose or len(pose) < 3:
            return None

        robot_x, robot_y, robot_yaw = float(pose[0]), float(pose[1]), float(pose[2])
        robot_pos = (robot_x, robot_y)

        goal_x = attack_dir * (FIELD_LONG / 2.0)
        goal_pos = (goal_x, 0.0)
        own_goal_x = -attack_dir * (FIELD_LONG / 2.0)
        own_goal_pos = (own_goal_x, 0.0)

        robot_ball = UnitedPosition.RobotBallPos()
        field_ball = UnitedPosition.FieldBallPos()

        ball_held = (robot_ball == BALL_HELD)
        ball_found = (robot_ball != BALL_NOT_FOUND) or (field_ball and field_ball[0] < 1000)

        ball_field_pos = None
        if field_ball and len(field_ball) >= 2 and field_ball[0] < 1000:
            ball_field_pos = [float(field_ball[0]), float(field_ball[1])]
        elif ball_held:
            ball_field_pos = [robot_x, robot_y]
        elif robot_ball and robot_ball != BALL_NOT_FOUND and robot_ball != BALL_HELD:
            br, bt = float(robot_ball[0]), float(robot_ball[1])
            yaw_rad = math.radians(robot_yaw)
            ball_field_pos = [
                robot_x + br * math.cos(yaw_rad) - bt * math.sin(yaw_rad),
                robot_y + br * math.sin(yaw_rad) + bt * math.cos(yaw_rad)
            ]

        # 球位置历史
        now = time.time()
        if ball_field_pos:
            AlphaFeatures._ball_history.append((ball_field_pos[0], ball_field_pos[1], now))
            if len(AlphaFeatures._ball_history) > AlphaFeatures._max_history:
                AlphaFeatures._ball_history.pop(0)

        # 球运动速度
        ball_velocity = [0.0, 0.0]
        ball_speed = 0.0
        if len(AlphaFeatures._ball_history) >= 3:
            h = AlphaFeatures._ball_history
            dt = h[-1][2] - h[-3][2]
            if dt > 0.01:
                ball_velocity = [(h[-1][0] - h[-3][0]) / dt, (h[-1][1] - h[-3][1]) / dt]
                ball_speed = math.sqrt(ball_velocity[0]**2 + ball_velocity[1]**2)

        # 球运动预测
        ball_predicted_pos = None
        if ball_field_pos and ball_speed > 5.0:
            pred_dt = BALL_PREDICTION_FRAMES * 0.03
            pred_x = ball_field_pos[0] + ball_velocity[0] * pred_dt
            pred_y = ball_field_pos[1] + ball_velocity[1] * pred_dt
            pred_dist = _dist(ball_field_pos, (pred_x, pred_y))
            if pred_dist <= BALL_PREDICTION_MAX_DIST:
                ball_predicted_pos = [pred_x, pred_y]
            else:
                scale = BALL_PREDICTION_MAX_DIST / pred_dist
                ball_predicted_pos = [
                    ball_field_pos[0] + (pred_x - ball_field_pos[0]) * scale,
                    ball_field_pos[1] + (pred_y - ball_field_pos[1]) * scale
                ]

        # R-B-G angle
        rbg_angle = 0.0
        dist_rb = 999.0
        if ball_field_pos:
            bp = (ball_field_pos[0], ball_field_pos[1])
            vec_br = _vec_sub(robot_pos, bp)  # Ball -> Robot
            vec_bg = _vec_sub(goal_pos, bp)    # Ball -> Goal
            rbg_angle = _angle_between_vectors(vec_br, vec_bg)
            dist_rb = _dist(robot_pos, bp)

        progress = attack_dir * robot_x

        vec_to_goal = _vec_sub(goal_pos, robot_pos)
        goal_angle = math.degrees(math.atan2(vec_to_goal[1], vec_to_goal[0]))
        alignment_to_goal = abs(_normalize_angle(robot_yaw - goal_angle))

        dist_to_goal = _dist(robot_pos, goal_pos)
        dist_to_own_goal = _dist(robot_pos, own_goal_pos)

        # 边界危险等级
        dist_to_boundary = min(FIELD_LONG / 2.0 - abs(robot_x), FIELD_SHORT / 2.0 - abs(robot_y))
        if dist_to_boundary < BOUNDARY_DANGER_DIST:
            boundary_danger_level = 2
        elif dist_to_boundary < BOUNDARY_CAUTION_DIST:
            boundary_danger_level = 1
        elif dist_to_boundary < BOUNDARY_SOFT_DIST:
            boundary_danger_level = 0
        else:
            boundary_danger_level = -1

        # 持球稳定性
        ball_stability = 1.0
        if robot_ball and robot_ball != BALL_HELD and robot_ball != BALL_NOT_FOUND:
            bx = float(robot_ball[0])
            by = abs(float(robot_ball[1]))
            lateral_margin = max(0.0, by - BALL_EDGE_DIST)
            forward_margin = max(0.0, abs(bx - 8.0) - 18.0)
            ball_stability = max(0.3, 1.0 - (lateral_margin + 0.5 * forward_margin) / 32.0)

        # 球是否朝己方球门运动
        ball_toward_own_goal = False
        if ball_field_pos and ball_speed > 10.0:
            vec_ball_to_own = _vec_sub(own_goal_pos, (ball_field_pos[0], ball_field_pos[1]))
            vec_ball_vel = (ball_velocity[0], ball_velocity[1])
            n1, n2 = _vec_norm(vec_ball_to_own), _vec_norm(vec_ball_vel)
            if n1 > 1e-6 and n2 > 1e-6:
                ball_toward_own_goal = _vec_dot(vec_ball_vel, vec_ball_to_own) / (n1 * n2) > 0.5

        # 进攻角度质量
        attack_quality = 0.0
        if ball_held:
            angle_quality = min(1.0, rbg_angle / 180.0)
            depth_quality = min(1.0, max(0.0, progress / (FIELD_LONG / 2.0)))
            attack_quality = 0.6 * angle_quality + 0.4 * depth_quality

        enemies = UnitedPosition.FieldChassisPos()
        blocking_enemy = None
        best_block_dist = 9999.0
        for enemy in enemies or []:
            if not enemy or len(enemy) < 2:
                continue
            ex, ey = float(enemy[0]), float(enemy[1])
            forward_gap = attack_dir * (ex - robot_x)
            lateral_gap = ey - robot_y
            if forward_gap < -5.0 or forward_gap > BLOCKING_ENEMY_FORWARD_CM:
                continue
            if abs(lateral_gap) > BLOCKING_ENEMY_LATERAL_CM:
                continue
            dist = math.hypot(forward_gap, lateral_gap)
            if dist < best_block_dist:
                best_block_dist = dist
                blocking_enemy = {
                    'x': ex,
                    'y': ey,
                    'forward_gap': forward_gap,
                    'lateral_gap': lateral_gap,
                    'dist': dist,
                }

        return {
            'robot_pos': [robot_x, robot_y, robot_yaw],
            'ball_field_pos': ball_field_pos,
            'ball_predicted_pos': ball_predicted_pos,
            'goal_pos': [goal_x, 0.0],
            'own_goal_pos': [own_goal_x, 0.0],
            'rbg_angle': rbg_angle,
            'dist_rb': dist_rb,
            'progress': progress,
            'alignment_to_goal': alignment_to_goal,
            'ball_held': ball_held,
            'ball_found': ball_found,
            'robot_yaw': robot_yaw,
            'attack_dir': attack_dir,
            'dist_to_goal': dist_to_goal,
            'dist_to_own_goal': dist_to_own_goal,
            'dist_to_boundary': dist_to_boundary,
            'boundary_danger_level': boundary_danger_level,
            'ball_velocity': ball_velocity,
            'ball_speed': ball_speed,
            'ball_stability': ball_stability,
            'ball_toward_own_goal': ball_toward_own_goal,
            'attack_quality': attack_quality,
            'enemies': enemies or [],
            'blocking_enemy': blocking_enemy,
        }


# ============================================================================
# 策略层
# ============================================================================
class AlphaStrategy:
    def __init__(self, attack_dir: int = 1):
        self.attack_dir = attack_dir
        self.current_mode = MODE_INTERCEPT
        self.state = "init"

        self._mode_history: List[int] = []
        self._pending_mode: Optional[int] = None
        self._last_shot_ts = -999.0
        self._last_dribble = None

        # 卡死检测
        self._position_history: List[Tuple[float, float, float]] = []
        self._stuck_since: Optional[float] = None
        self._escaping_stuck = False
        self._escape_start_time: Optional[float] = None
        self._escape_direction = 1
        self._carry_stall_since: Optional[float] = None
        self._carry_stall_direction = 1

        # 球丢失计时
        self._ball_lost_since: Optional[float] = None
        self._last_possession_ts = -999.0
        self._possession_started_ts: Optional[float] = None
        self._had_possession = False

        # 子状态
        self._sub_state = ""
        self._lateral_phase = 0
        self._approach_retries = 0

        self.state_stats = {}
        self._log_counter = 0
        self.goal_x = attack_dir * FIELD_LONG / 2.0

    def _set_mode(self, mode: int) -> None:
        self._mode_history.append(mode)
        if len(self._mode_history) > MODE_SWITCH_FRAMES:
            self._mode_history.pop(0)
        if len(self._mode_history) == MODE_SWITCH_FRAMES and all(m == mode for m in self._mode_history):
            if self.current_mode != mode:
                self.current_mode = mode
                self.state_stats[self.get_mode_name(mode)] = self.state_stats.get(self.get_mode_name(mode), 0) + 1
                self._sub_state = ""
                self._lateral_phase = 0
        else:
            self._pending_mode = mode

    def _set_dribble(self, power) -> None:
        if self._last_dribble == power: return
        peripheral.Dribble(power)
        self._last_dribble = power
        # LockBallSlip caches its last Dribble() command. Alpha sometimes
        # releases the dribbler directly before handing acquisition back to
        # LockBallSlip; keep that cache truthful so LockBallSlip will re-enable
        # the dribbler instead of silently assuming it is still on.
        try:
            lockball._lockball_dribble_state = power if power else False
        except Exception:
            pass

    def get_mode_name(self, mode: int = None) -> str:
        if mode is None: mode = self.current_mode
        return {MODE_STRIKING: "STRIKING", MODE_LATERAL_DRIBBLE: "LATERAL_DRIBBLE",
                MODE_INTERCEPT: "INTERCEPT", MODE_RECOVER: "RECOVER",
                MODE_POSITIONING: "POSITIONING"}.get(mode, "UNKNOWN")

    def _attack_yaw(self) -> float:
        return 0.0 if self.attack_dir >= 0 else 180.0

    def _possession_age(self, now: Optional[float] = None) -> float:
        if self._possession_started_ts is None:
            return 999.0
        return (time.time() if now is None else now) - self._possession_started_ts

    def _apply_blocking_enemy_dodge(self, target_y: float, f: Dict[str, Any]) -> float:
        blocker = f.get('blocking_enemy')
        if not blocker:
            return target_y

        robot_y = float(f['robot_pos'][1])
        if abs(robot_y) > LATERAL_CARRY_Y_LIMIT * 0.95:
            self._sub_state = "enemy_dodge_infield"
            return _clamp(
                robot_y - math.copysign(BLOCKING_ENEMY_DODGE_CM, robot_y),
                -LATERAL_CARRY_Y_LIMIT,
                LATERAL_CARRY_Y_LIMIT,
            )

        lateral_gap = float(blocker.get('lateral_gap', 0.0))
        if abs(lateral_gap) < 6.0:
            side = -1.0 if robot_y > 0 else 1.0
        else:
            side = -1.0 if lateral_gap > 0 else 1.0

        self._sub_state = "enemy_dodge"
        return _clamp(robot_y + side * BLOCKING_ENEMY_DODGE_CM, -LATERAL_CARRY_Y_LIMIT, LATERAL_CARRY_Y_LIMIT)

    def _apply_blocking_enemy_speed(self, speed_x: float, speed_y: float, f: Dict[str, Any]) -> Tuple[float, float]:
        if not f.get('blocking_enemy'):
            return speed_x, speed_y

        robot_y = float(f['robot_pos'][1])
        max_forward = BLOCKING_EDGE_FORWARD_CAP_CM if abs(robot_y) > LATERAL_CARRY_Y_LIMIT * 0.95 else BLOCKING_ENEMY_FORWARD_CAP_CM
        if speed_x * self.attack_dir > max_forward:
            speed_x = self.attack_dir * max_forward

        if abs(speed_y) < BLOCKING_ENEMY_MIN_LATERAL_CM:
            lateral_gap = float(f['blocking_enemy'].get('lateral_gap', 0.0))
            if abs(robot_y) > LATERAL_CARRY_Y_LIMIT * 0.95:
                side = -1.0 if robot_y > 0 else 1.0
            elif abs(lateral_gap) < 6.0:
                side = -1.0 if robot_y > 0 else 1.0
            else:
                side = -1.0 if lateral_gap > 0 else 1.0
            speed_y = side * BLOCKING_ENEMY_MIN_LATERAL_CM
        return speed_x, speed_y

    def _clear_possession_tracking(self) -> None:
        self._had_possession = False
        self._possession_started_ts = None

    def _apply_possession_grace(self, f: Dict[str, Any], now: float) -> None:
        f['raw_ball_held'] = f['ball_held']
        f['possession_grace'] = False
        if f['ball_held']:
            if not self._had_possession:
                self._possession_started_ts = now
            self._had_possession = True
            self._last_possession_ts = now
            return

        carry_context = (
            self.current_mode in (MODE_STRIKING, MODE_LATERAL_DRIBBLE)
            or self._pending_mode in (MODE_STRIKING, MODE_LATERAL_DRIBBLE)
            or self.state in ("striking", "lateral_dribble")
        )
        if not carry_context:
            self._clear_possession_tracking()
            return
        if now - self._last_possession_ts > POSSESSION_GRACE_S:
            self._clear_possession_tracking()
            return
        if f.get('dist_rb', 999.0) > POSSESSION_GRACE_DIST:
            self._clear_possession_tracking()
            return
        if f.get('ball_stability', 1.0) < 0.45:
            self._clear_possession_tracking()
            return

        f['ball_held'] = True
        f['possession_grace'] = True
        if not self._had_possession:
            self._possession_started_ts = now
        self._had_possession = True

    # ====================================================================
    # 核心循环
    # ====================================================================
    def step(self) -> int:
        self._log_counter += 1
        features = AlphaFeatures.get_features(self.attack_dir)
        if not features:
            self._execute_intercept_emergency()
            return self.current_mode

        now = time.time()
        self._apply_possession_grace(features, now)
        self._position_history.append((features['robot_pos'][0], features['robot_pos'][1], now))
        if len(self._position_history) > STUCK_CHECK_FRAMES + 10:
            self._position_history.pop(0)

        if features['ball_held']:
            self._stuck_since = None
        elif features['ball_found'] and features['dist_rb'] <= BALL_CLOSE_DIST:
            self._stuck_since = None
        elif self._check_stuck():
            self._execute_stuck_escape(features)
            return self.current_mode

        # 边界保护
        if features['boundary_danger_level'] >= 2:
            self._execute_avoid_wall(features)
            self._set_mode(MODE_INTERCEPT)
            return self.current_mode

        # 紧急回防
        if (
            features['ball_toward_own_goal']
            and not features['ball_held']
            and features['dist_rb'] > BALL_CLOSE_DIST
        ):
            self._execute_defensive_intercept(features)
            self._set_mode(MODE_RECOVER)
            return self.current_mode

        mode = self._decide_mode(features)
        self._execute_mode(mode, features)
        self._set_mode(mode)

        if self._log_counter % 100 == 0:
            logger.info(
                f"[alpha] mode={self.get_mode_name():15s} sub={self._sub_state:10s} "
                f"pos=({features['robot_pos'][0]:6.1f},{features['robot_pos'][1]:6.1f}) "
                f"rbg={features['rbg_angle']:5.1f}° dist_rb={features['dist_rb']:5.1f} "
                f"progress={features['progress']:5.1f} held={features['ball_held']} "
                f"stability={features['ball_stability']:.2f} boundary={features['boundary_danger_level']} "
                f"ball_speed={features['ball_speed']:.0f}")
        return self.current_mode

    # ====================================================================
    # 模式决策（v5 大幅增强）
    # ====================================================================
    def _decide_mode(self, f: Dict[str, Any]) -> int:
        # 1. 球丢失 → RECOVER
        if not f['ball_found']:
            self._ball_lost_since = self._ball_lost_since or time.time()
            return MODE_RECOVER
        self._ball_lost_since = None

        # 2. 球远 + 球在运动 → 预测拦截
        if f['dist_rb'] > BALL_FAR_DIST and f['ball_speed'] > 10.0:
            return MODE_RECOVER

        # 3. 球远 + 球静止 → 标准追球
        if f['dist_rb'] > BALL_FAR_DIST:
            return MODE_INTERCEPT

        # 4-9. 持球状态
        if f['ball_held']:
            # 4a. 边界警告 → 先调整
            if f['boundary_danger_level'] >= 1:
                return MODE_LATERAL_DRIBBLE
            # 4b. 持球不稳定 → 稳住
            if f['ball_stability'] < 0.5:
                return MODE_LATERAL_DRIBBLE
            # 4c. 已经正对球门时直接推进。持球状态下 R-B-G 角会退化，
            # 所以这里优先用机器人朝向来避免无意义的横向盘带。
            if f['alignment_to_goal'] <= 18.0 and f['ball_stability'] >= 0.7:
                return MODE_STRIKING
            # 5. 进攻质量高 → 冲刺
            if f['attack_quality'] >= 0.7:
                return MODE_STRIKING
            # 6. 进攻深度足够 → 冲刺
            if f['progress'] >= DEEP_PROGRESS_THRESHOLD:
                return MODE_STRIKING
            # 7. R-B-G 角大 → 冲刺
            if f['rbg_angle'] >= RBG_ANGLE_STRIKE_THRESHOLD:
                return MODE_STRIKING
            # 8. R-B-G 角小 → 绕行
            if f['rbg_angle'] <= RBG_ANGLE_LATERAL_THRESHOLD:
                return MODE_LATERAL_DRIBBLE
            # 9. 中间角度
            if f['progress'] >= 0 and f['ball_stability'] >= 0.7:
                return MODE_STRIKING
            elif f['progress'] < -20.0:
                return MODE_LATERAL_DRIBBLE
            else:
                return MODE_STRIKING if f['alignment_to_goal'] < 45.0 else MODE_LATERAL_DRIBBLE

        # 10. 球接近但未控住
        if f['dist_rb'] <= BALL_CLOSE_DIST:
            return MODE_INTERCEPT
        if f['dist_rb'] <= BALL_MEDIUM_DIST:
            return MODE_INTERCEPT

        return MODE_INTERCEPT

    # ====================================================================
    # 模式执行
    # ====================================================================
    def _execute_mode(self, mode: int, f: Dict[str, Any]) -> None:
        if mode == MODE_STRIKING:
            self._execute_striking(f)
        elif mode == MODE_LATERAL_DRIBBLE:
            self._execute_lateral_dribble(f)
        elif mode == MODE_INTERCEPT:
            self._execute_intercept(f)
        elif mode == MODE_RECOVER:
            self._execute_recover(f)
        elif mode == MODE_POSITIONING:
            self._execute_positioning(f)
        else:
            self._execute_intercept(f)

    # ---- STRIKING ----
    def _execute_striking(self, f: Dict[str, Any]) -> None:
        self.state = "striking"
        if not f['ball_held']:
            self._carry_stall_since = None
            self._execute_intercept(f)
            return

        carry_stalled = self._is_carry_stalled(f)
        if self._can_shoot(f, carry_stalled=carry_stalled) and self._execute_shot(f):
            return

        if self._possession_age() < POSSESSION_SETTLE_S and f['boundary_danger_level'] < 1:
            self._sub_state = "settle_carry"
            self._set_dribble(SECURE_DRIBBLE_POWER)
            speed_x = self.attack_dir * SETTLE_CARRY_SPEED
            speed_y = _clamp(-f['robot_pos'][1] * 0.25, -12.0, 12.0)
            chassis.FieldMoveVector(speed_x, speed_y, f['robot_yaw'], KpW=0.35)
            return

        dribble_power = self._calc_dynamic_dribble_power(f)
        self._set_dribble(dribble_power)

        speed_factor = 1.0
        if f['ball_stability'] < 0.7:
            speed_factor = STABILITY_SPEED_FACTOR + 0.4 * f['ball_stability']

        boundary_adjust_y = 0.0
        if f['boundary_danger_level'] == 0:
            boundary_adjust_y = -f['robot_pos'][1] * 0.1

        target_x = self.attack_dir * min(abs(f['robot_pos'][0]) + 30.0, FIELD_LONG / 2.0 - 20.0)
        aim_y = self._best_goal_aim_y(f['robot_pos'])
        target_y = aim_y * 0.3 + boundary_adjust_y
        target_y = self._apply_blocking_enemy_dodge(target_y, f)
        if carry_stalled:
            self._sub_state = "carry_unblock"
            target_y = _clamp(
                f['robot_pos'][1] + self._carry_stall_direction * 28.0,
                -LATERAL_CARRY_Y_LIMIT,
                LATERAL_CARRY_Y_LIMIT,
            )
            target_x = f['robot_pos'][0] + self.attack_dir * 12.0

        speed_x = (target_x - f['robot_pos'][0]) * STRIKE_SPEED_GAIN * speed_factor
        speed_y = (target_y - f['robot_pos'][1]) * STRIKE_SPEED_GAIN * speed_factor
        speed_x, speed_y = self._apply_blocking_enemy_speed(speed_x, speed_y, f)
        speed_x, speed_y = _limit_speed(speed_x, speed_y, MAX_FIELD_SPEED)

        goal_yaw = self._calc_goal_yaw(f['robot_pos'], aim_y)
        chassis.FieldMoveVector(speed_x, speed_y, goal_yaw)

    # ---- LATERAL_DRIBBLE ----
    def _execute_lateral_dribble(self, f: Dict[str, Any]) -> None:
        self.state = "lateral_dribble"
        if not f['ball_held']:
            self._carry_stall_since = None
            self._execute_intercept(f)
            return

        self._lateral_phase += 1

        if self._can_shoot(f) and self._execute_shot(f):
            return

        dribble_power = self._calc_dynamic_dribble_power(f)
        self._set_dribble(dribble_power)

        if self._lateral_phase < 30:
            # 阶段1：横向调整
            self._sub_state = "lateral_adjust"
            if abs(f['robot_pos'][1]) > LATERAL_CARRY_Y_LIMIT:
                target_y = _clamp(f['robot_pos'][1] * 0.45, -LATERAL_CARRY_Y_LIMIT, LATERAL_CARRY_Y_LIMIT)
            else:
                target_y = LATERAL_TARGET_Y if f['robot_pos'][1] <= 0 else -LATERAL_TARGET_Y
            speed_x = self.attack_dir * 12.0
            speed_y = _clamp((target_y - f['robot_pos'][1]) * LATERAL_SPEED_GAIN, -24.0, 24.0)

        elif self._lateral_phase < 80:
            # 阶段2：斜向推进
            self._sub_state = "lateral_advance"
            target_x = self.attack_dir * min(abs(f['robot_pos'][0]) + 20.0, FIELD_LONG / 2.0 - 20.0)
            target_y = f['robot_pos'][1] * 0.6
            target_y = self._apply_blocking_enemy_dodge(target_y, f)
            speed_x = (target_x - f['robot_pos'][0]) * LATERAL_SPEED_GAIN
            speed_y = (target_y - f['robot_pos'][1]) * LATERAL_SPEED_GAIN
        else:
            # 阶段3：检查角度
            self._sub_state = "lateral_check"
            self._lateral_phase = 0
            if f['rbg_angle'] >= RBG_ANGLE_STRIKE_THRESHOLD:
                speed_x, speed_y = self.attack_dir * 20.0, 0.0
            else:
                target_x = self.attack_dir * min(abs(f['robot_pos'][0]) + 15.0, FIELD_LONG / 2.0 - 20.0)
                target_y = f['robot_pos'][1] * 0.7
                speed_x = (target_x - f['robot_pos'][0]) * LATERAL_SPEED_GAIN
                speed_y = (target_y - f['robot_pos'][1]) * LATERAL_SPEED_GAIN

        if f['ball_stability'] < 0.7:
            speed_x *= STABILITY_SPEED_FACTOR
            speed_y *= STABILITY_SPEED_FACTOR
        speed_x, speed_y = self._apply_blocking_enemy_speed(speed_x, speed_y, f)
        speed_x, speed_y = _limit_speed(speed_x, speed_y, MAX_FIELD_SPEED)

        if self._possession_age() < POSSESSION_SETTLE_S or f['boundary_danger_level'] >= 1:
            goal_yaw = f['robot_yaw']
        else:
            goal_yaw = self._calc_goal_yaw(f['robot_pos'], f['robot_pos'][1] * 0.5)
        chassis.FieldMoveVector(speed_x, speed_y, goal_yaw)

    # ---- INTERCEPT ----
    def _execute_intercept(self, f: Dict[str, Any]) -> None:
        self.state = "intercept"
        # LockBallSlip owns the dribbler while acquiring the ball. Alpha should
        # not also write Dribble() here, otherwise the IO oscillates every tick.
        self._last_dribble = None
        approach_yaw = self._attack_yaw()

        if not f['ball_found'] or f['dist_rb'] > BALL_FAR_DIST:
            ball_field = UnitedPosition.FieldBallPos()
            if ball_field and ball_field[0] < 1000:
                target_x = float(ball_field[0]) - self.attack_dir * 10.0
                target_y = float(ball_field[1])
                UnitedPosition.Pos2Pos([target_x, target_y, approach_yaw], INTERCEPT_SPEED_GAIN)
            return

        if f['dist_rb'] > BALL_CLOSE_DIST and f['ball_field_pos']:
            target_x = f['ball_field_pos'][0] - self.attack_dir * 5.0
            target_y = f['ball_field_pos'][1]
            UnitedPosition.Pos2Pos([target_x, target_y, approach_yaw], INTERCEPT_SPEED_GAIN)
            self._sub_state = "approaching"
            return

        lockball.LockBallSlip()
        self._sub_state = "locking"

    # ---- RECOVER: v5 球丢失后主动追球 ----
    def _execute_recover(self, f: Dict[str, Any]) -> None:
        self.state = "recover"
        if f.get('dist_rb', 999.0) <= BALL_CLOSE_DIST:
            self._execute_intercept(f)
            return

        self._set_dribble(False)
        approach_yaw = self._attack_yaw()

        # 优先用预测位置拦截
        if f['ball_predicted_pos']:
            target_x = f['ball_predicted_pos'][0] - self.attack_dir * 5.0
            target_y = f['ball_predicted_pos'][1]
            self._sub_state = "predict_intercept"
            UnitedPosition.Pos2Pos([target_x, target_y, approach_yaw], RECOVER_SPEED_GAIN)
            return

        # 用 FieldBallPos 追球
        ball_field = UnitedPosition.FieldBallPos()
        if ball_field and ball_field[0] < 1000:
            target_x = float(ball_field[0]) - self.attack_dir * 8.0
            target_y = float(ball_field[1])
            self._sub_state = "field_chase"
            UnitedPosition.Pos2Pos([target_x, target_y, approach_yaw], RECOVER_SPEED_GAIN)
            return

        # 球完全不可见 → 向中场移动
        self._sub_state = "search"
        UnitedPosition.Pos2Pos([0.0, 0.0, approach_yaw], POSITIONING_SPEED_GAIN)

    # ---- POSITIONING: v5 无球站位 ----
    def _execute_positioning(self, f: Dict[str, Any]) -> None:
        self.state = "positioning"
        self._set_dribble(False)

        if f['ball_field_pos']:
            ball_x = f['ball_field_pos'][0]
            if ball_x * self.attack_dir < 0:
                target_x = self.attack_dir * POSITIONING_DEFENSE_X
            else:
                target_x = self.attack_dir * POSITIONING_ATTACK_X
        else:
            target_x = self.attack_dir * POSITIONING_MIDFIELD_X

        target_y = _clamp(f['robot_pos'][1], -POSITIONING_Y_SPREAD, POSITIONING_Y_SPREAD)
        UnitedPosition.Pos2Pos([target_x, target_y, self._attack_yaw()], POSITIONING_SPEED_GAIN)
        self._sub_state = "positioning"

    # ---- 紧急回防 ----
    def _execute_defensive_intercept(self, f: Dict[str, Any]) -> None:
        self.state = "defensive_intercept"
        self._set_dribble(False)

        if f['ball_predicted_pos']:
            target_x, target_y = f['ball_predicted_pos'][0], f['ball_predicted_pos'][1]
        elif f['ball_field_pos']:
            target_x, target_y = f['ball_field_pos'][0], f['ball_field_pos'][1]
        else:
            target_x, target_y = -self.attack_dir * 30.0, 0.0

        UnitedPosition.Pos2Pos([target_x, target_y, 0], RECOVER_SPEED_GAIN * 1.2)
        self._sub_state = "defensive"

    def _execute_intercept_emergency(self) -> None:
        self.state = "intercept_emergency"
        self._last_dribble = None
        lockball.LockBallSlip()

    # ---- 边界保护 ----
    def _execute_avoid_wall(self, f: Dict[str, Any]) -> None:
        self.state = "avoid_wall"
        self._set_dribble(False)
        pose = UnitedPosition.FieldRobotPos()
        if pose and len(pose) >= 3:
            try:
                safe_target = UnitedPosition.ProjectSafePose(list(pose))
                if safe_target:
                    UnitedPosition.Pos2Pos(safe_target, 1.2)
            except Exception:
                pass

    # ====================================================================
    # 卡死检测与脱困
    # ====================================================================
    def _check_stuck(self) -> bool:
        # 如果正在脱困，检查是否完成
        if self._escaping_stuck:
            if self._escape_start_time and time.time() - self._escape_start_time > STUCK_ESCAPE_DURATION_S:
                self._escaping_stuck = False
                self._escape_start_time = None
                self._stuck_since = None
                self._position_history.clear()  # 清空历史，重新开始检测
                return False
            return True

        if len(self._position_history) < STUCK_CHECK_FRAMES:
            return False

        recent = self._position_history[-STUCK_CHECK_FRAMES:]
        x0, y0, t0 = recent[0]
        x1, y1, t1 = recent[-1]
        dt = t1 - t0
        if dt < 0.5:
            return False

        displacement = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
        
        # 只有当位移非常小且持续一段时间才判定为卡死
        if displacement < STUCK_POSITION_THRESHOLD:
            self._stuck_since = self._stuck_since or time.time()
            if time.time() - self._stuck_since > 1.0:  # 增加到1秒
                return True
        else:
            self._stuck_since = None
        return False

    def _execute_stuck_escape(self, f: Dict[str, Any]) -> None:
        if not self._escaping_stuck:
            self._escaping_stuck = True
            self._escape_start_time = time.time()
            self._escape_direction *= -1
            logger.info(f"[alpha] STUCK DETECTED! Escaping direction={self._escape_direction}")

        self.state = "stuck_escape"
        self._set_dribble(False)

        speed_x = -self.attack_dir * 20.0
        speed_y = self._escape_direction * 40.0
        goal_yaw = f.get('robot_yaw', 0) if f else 0
        chassis.FieldMoveVector(speed_x, speed_y, goal_yaw)

    def _is_carry_stalled(self, f: Dict[str, Any]) -> bool:
        if not f.get('ball_held', False):
            self._carry_stall_since = None
            return False
        if len(self._position_history) < STUCK_CHECK_FRAMES:
            return False

        recent = self._position_history[-STUCK_CHECK_FRAMES:]
        x0, y0, _ = recent[0]
        x1, y1, _ = recent[-1]
        moved = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
        if moved >= CARRY_STALL_MOVE_CM:
            self._carry_stall_since = None
            return False

        now = time.time()
        self._carry_stall_since = self._carry_stall_since or now
        if now - self._carry_stall_since < CARRY_STALL_S:
            return False

        self._carry_stall_direction = -1 if f['robot_pos'][1] > 0 else 1
        if abs(f['robot_pos'][1]) > LATERAL_CARRY_Y_LIMIT * 0.7:
            self._carry_stall_direction = -1 if f['robot_pos'][1] > 0 else 1
        return True

    # ====================================================================
    # 动态运球功率
    # ====================================================================
    def _calc_dynamic_dribble_power(self, f: Dict[str, Any]) -> float:
        base = CARRY_DRIBBLE_POWER
        if self._possession_age() < POSSESSION_SETTLE_S:
            base = max(base, SECURE_DRIBBLE_POWER)
        if f.get('ball_stability', 1.0) < 0.7:
            base = max(base, SECURE_DRIBBLE_POWER)
        if f.get('alignment_to_goal', 0) > 30.0:
            base = min(0.9, base + 0.06)
        if f.get('boundary_danger_level', -1) >= 0:
            base = min(0.9, base + 0.06)
        return base

    # ====================================================================
    # 射门逻辑
    # ====================================================================
    def _can_shoot(self, f: Dict[str, Any], carry_stalled: bool = False) -> bool:
        if not f['ball_held']: return False
        if f.get('possession_grace', False): return False
        if time.time() - self._last_shot_ts < SHOT_COOLDOWN_S: return False
        if (
            carry_stalled
            and f['progress'] >= STALLED_SHOT_MIN_PROGRESS
            and f['alignment_to_goal'] <= STALLED_SHOT_ALIGNMENT_DEG
        ):
            return True
        if f['alignment_to_goal'] <= DIRECT_SHOT_ALIGNMENT_DEG and f['progress'] >= DIRECT_SHOT_MIN_PROGRESS:
            return True
        if f['progress'] >= COMMITTED_SHOT_PROGRESS and f['alignment_to_goal'] <= SHOOT_ALIGNMENT_THRESHOLD:
            return True
        return False

    def _execute_shot(self, f: Dict[str, Any]) -> bool:
        aim_y = self._best_goal_aim_y(f['robot_pos'])
        goal_yaw = self._calc_goal_yaw(f['robot_pos'], aim_y)
        yaw_error = abs(_angle_error_deg(f['robot_yaw'], goal_yaw))

        if yaw_error <= SHOOT_YAW_TOLERANCE_DEG:
            chassis.stop()
            peripheral.ShootBall()
            self._set_dribble(False)
            self._last_shot_ts = time.time()
            self.state = "shot"
            logger.info(f"[alpha] SHOOT! pos=({f['robot_pos'][0]:.0f},{f['robot_pos'][1]:.0f}) "
                       f"progress={f['progress']:.0f} yaw_err={yaw_error:.1f}°")
            return True
        else:
            turn_gain = _clamp(yaw_error / 45.0, 0.35, 1.0)
            chassis.FieldMoveVector(0, 0, goal_yaw, KpW=turn_gain)
            return False

    def _best_goal_aim_y(self, pose: list) -> float:
        candidates = [-GOAL_HALF_WIDTH * 0.6, 0.0, GOAL_HALF_WIDTH * 0.6]
        if not pose or len(pose) < 2: return 0.0
        return min(candidates, key=lambda gy: abs(gy - pose[1]))

    def _calc_goal_yaw(self, robot_pos: list, aim_y: float) -> float:
        return UnitedPosition.Pos2Angle(robot_pos[:2], [self.goal_x, aim_y]) % 360


# ============================================================================
# 控制器包装
# ============================================================================
class AlphaController:
    def __init__(self, attack_dir: int = 1):
        self.attack_dir = attack_dir
        self.strategy = AlphaStrategy(attack_dir)
        self.step_count = 0

    def step(self) -> None:
        self.step_count += 1
        self.strategy.step()

    @property
    def state(self) -> str:
        return self.strategy.state

    @property
    def current_mode_name(self) -> str:
        return self.strategy.get_mode_name()

    def run_step(self) -> Dict[str, Any]:
        self.step_count += 1
        self.strategy.step()
        features = AlphaFeatures.get_features(self.attack_dir)
        return {'step': self.step_count, 'mode': self.strategy.get_mode_name(), 'features': features}

    def get_stats(self) -> Dict[str, int]:
        return self.strategy.state_stats.copy()

    def stop(self) -> None:
        try:
            peripheral.Dribble(False)
            chassis.stop()
        except Exception:
            pass
