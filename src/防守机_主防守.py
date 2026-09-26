from LunaPre import *
from BasicFuc import lockball, TurnTo_with_ball
from LunaPre.strategy_tools import angle_error_deg, clamp
import math
import time

# ---------- 场地几何与常量（尽量从配置读取） ----------

def _bounds():
    half_long = float(getattr(cfg.Bounds, "Long", 260.0)) * 0.5
    half_short = float(getattr(cfg.Bounds, "Short", 200.0)) * 0.5
    return half_long, half_short

HALF_LONG, HALF_SHORT = _bounds()
OWN_GOAL_X = -HALF_LONG      # 己方球门端线
OPP_GOAL_X = HALF_LONG       # 对方球门端线

DEFEND_DIST = 50.0           # 守位点距球门中心的前向距离（cm）
GOAL_SIDE_MAX = 40.0         # 守位 y 的最大偏移（避免把球门完全让开）
DANGER_X = -50.0             # 己方危险区边界：x < DANGER_X 视为危险区
CARRY_SPEED = 0.30           # 带球出区速度（m/s）
ALIGN_TOL = 8.0              # 出球/射门前航向对准容差（度）
TURN_TOL = 15.0              # 推进前航向对准容差（度）
PRESS_DIST = 90.0            # 逼抢距离阈值（cm）
POST_CLEAR_HOLD = 0.5        # 出球后的回撤守位窗口（秒）

# ---------- 状态记忆 ----------

_last_ball_field = None      # 最近一次有效的场地球坐标 [x, y]
_post_clear_until = 0.0      # 出球后的回撤守位截止时刻（time.time）

# ---------- 基础工具 ----------

def _read_state():
    """读取当前机器人位姿、球状态；顺手记录最后可见的场地球坐标。"""
    global _last_ball_field
    robot_pos = UnitedPosition.FieldRobotPose()
    ball_pos = UnitedPosition.RobotBallPos()
    ball_state = UnitedPosition.BallState(ball_pos)
    if ball_state != "lost":
        fb = UnitedPosition.FieldBallPos()
        if fb is not None and len(fb) >= 2:
            _last_ball_field = [float(fb[0]), float(fb[1])]
    return robot_pos, ball_state

def _goal_guard_point(ball_field=None):
    """球门中心 -> 球 连线上的守位点；球未知时取球门中前。"""
    tx = OWN_GOAL_X + DEFEND_DIST
    ty = 0.0
    if ball_field is not None:
        bx, by = float(ball_field[0]), float(ball_field[1])
        dx, dy = bx - OWN_GOAL_X, by - 0.0
        d = math.hypot(dx, dy)
        if d > 1e-6:
            u = DEFEND_DIST / d
            tx = OWN_GOAL_X + dx * u
            ty = dy * u
    ty = clamp(ty, -GOAL_SIDE_MAX, GOAL_SIDE_MAX)
    return tx, ty

def _shot_window():
    """获取射门窗口，失败时返回安全默认值（空 dict）。"""
    try:
        robot_pos = UnitedPosition.FieldRobotPose()
        return UnitedPosition.ShootWindow(
            target="opponent",
            use_estimated_goal=False,
            current_yaw_deg=robot_pos[2],
        )
    except Exception:
        return {}

def _angle_to_opp_goal(robot_pos):
    """从当前机器人位置指向对方球门中心的角度（场地坐标系，度）。"""
    return math.degrees(math.atan2(0.0 - robot_pos[1], OPP_GOAL_X - robot_pos[0])) % 360.0

# ---------- 持球决策：安全出区 -> 窗口射门 / 前场解围 ----------

def _held_decision(robot_pos):
    """防守机持球：安全第一。返回后由主循环复核球是否还在脚下。"""
    global _post_clear_until

    # 1) 在己方危险区：先把球带离球门方向，朝前场推进
    if robot_pos[0] < DANGER_X:
        aim = _angle_to_opp_goal(robot_pos)
        if abs(angle_error_deg(robot_pos[2], aim)) > TURN_TOL:
            TurnTo_with_ball(aim)                 # 绕球对准前场方向
        else:
            chassis.FieldMoveVector(CARRY_SPEED, 0.0, aim, KpW=0.4)
        return

    # 2) 已离开危险区：窗口开则射（解围式远射），否则朝前场出球
    window = _shot_window()
    can_shoot = window.get("can_shoot_now", False)
    target_yaw = window.get("target_yaw_deg")
    window_open = window.get("window_open", False)
    blockers = window.get("blockers") or []
    open_goal = (
        window.get("goal_key") is not None
        and window_open
        and len(blockers) <= 1
    )

    if can_shoot and open_goal:
        if target_yaw is not None and abs(angle_error_deg(robot_pos[2], target_yaw)) > ALIGN_TOL:
            TurnTo_with_ball(target_yaw)
        else:
            peripheral.ShootBall()
            time.sleep(0.03)
            _post_clear_until = time.time() + POST_CLEAR_HOLD
        return

    # 无窗口：对准前场方向踢出（解围），球飞向前场由进攻机争抢
    aim = _angle_to_opp_goal(robot_pos)
    if abs(angle_error_deg(robot_pos[2], aim)) > ALIGN_TOL:
        TurnTo_with_ball(aim)
    else:
        peripheral.ShootBall()
        time.sleep(0.03)
        _post_clear_until = time.time() + POST_CLEAR_HOLD

# ---------- 无球决策：拦截 / 逼抢 / 回撤守位 ----------

def _free_decision(robot_pos):
    """球可见但未被己方控制（free）。"""
    try:
        fb = UnitedPosition.FieldBallPos()
        if fb is None or len(fb) < 2:
            return
        bx, by = float(fb[0]), float(fb[1])
    except Exception:
        return

    if bx < 0.0:
        # 球在己方半场：优先拦截落点，否则去球位逼抢
        landing = None
        try:
            landing = UnitedPosition.PredictBallLanding(horizon_s=0.45)
        except Exception:
            landing = None

        if landing is not None and landing.get("valid") and landing.get("field_pos"):
            lx = float(landing["field_pos"][0])
            ly = float(landing["field_pos"][1])
            UnitedPosition.Pos2Pos([lx, ly, 0.0])
            return

        dist = math.hypot(robot_pos[0] - bx, robot_pos[1] - by)
        if dist < PRESS_DIST:
            lockball.LockBallSlip()               # 球近直接抢
        else:
            UnitedPosition.Pos2Pos([bx, by, 0.0])
        return

    # 球在对方半场：回撤守位，保持阵型（进攻机负责逼抢）
    tx, ty = _goal_guard_point([bx, by])
    UnitedPosition.Pos2Pos([tx, ty, 0.0])

# ---------- 丢球决策：回守门位 ----------

def _lost_decision(robot_pos):
    """视觉丢失：回球门-球连线守位，守住最后一道线。"""
    tx, ty = _goal_guard_point(_last_ball_field)
    UnitedPosition.Pos2Pos([tx, ty, 0.0])

# ---------- 主循环 ----------

if __name__ == "__main__":
    try:
        while True:
            robot_pos, ball_state = _read_state()

            # 安全区域检查：不在可行动区域时停车等待
            if not UnitedPosition.RestrictedArea():
                chassis.stop()
                time.sleep(0.03)
                continue

            # 出球后的回撤窗口：快速回守位，防止球门空虚
            if time.time() < _post_clear_until:
                tx, ty = _goal_guard_point(_last_ball_field)
                UnitedPosition.Pos2Pos([tx, ty, 0.0])
                time.sleep(0.03)
                continue

            if ball_state == "held":
                peripheral.Dribble(0.7)
                _held_decision(robot_pos)
            elif ball_state == "free":
                _free_decision(robot_pos)
            else:  # lost
                _lost_decision(robot_pos)

            time.sleep(0.03)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print("defend strategy error:", e)
        try:
            UnitedPosition.Pos2Pos([0, 0, 0])
        except Exception:
            pass
    finally:
        chassis.stop()
        peripheral.Dribble(False)
        UnitedPosition.Stop()
