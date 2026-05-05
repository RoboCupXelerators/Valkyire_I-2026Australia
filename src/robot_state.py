"""机器人状态共享模块

这个模块提供了一个全局状态共享机制，让主程序可以设置机器人状态，
control_panel 可以读取这些状态。

使用方法:
    # 在主程序中设置状态
    from src.robot_state import set_robot_state
    set_robot_state(x=100, y=50, yaw=90, has_ball=True)
    
    # 在 control_panel 中读取状态
    from src.robot_state import get_robot_state
    state = get_robot_state()
"""

import threading
import time
from typing import Optional

# 全局状态
_state = {
    "x_cm": 0.0,
    "y_cm": 0.0,
    "yaw_deg": 0.0,
    "has_ball": False,
    "vx": 0.0,
    "vy": 0.0,
    "wz": 0.0,
    "ts": 0.0,
}

_lock = threading.Lock()


def set_robot_state(
    x_cm: float = None,
    y_cm: float = None,
    yaw_deg: float = None,
    has_ball: bool = None,
    vx: float = None,
    vy: float = None,
    wz: float = None,
):
    """设置机器人状态"""
    global _state
    with _lock:
        if x_cm is not None:
            _state["x_cm"] = float(x_cm)
        if y_cm is not None:
            _state["y_cm"] = float(y_cm)
        if yaw_deg is not None:
            _state["yaw_deg"] = float(yaw_deg)
        if has_ball is not None:
            _state["has_ball"] = bool(has_ball)
        if vx is not None:
            _state["vx"] = float(vx)
        if vy is not None:
            _state["vy"] = float(vy)
        if wz is not None:
            _state["wz"] = float(wz)
        _state["ts"] = time.time()


def get_robot_state() -> dict:
    """获取机器人状态"""
    global _state
    with _lock:
        return dict(_state)


def update_from_chassis(chassis, vision=None):
    """从 chassis 模块更新机器人状态
    
    Args:
        chassis: Car 实例，需要有 GetYaw() 方法
        vision: Vision 实例，用于获取球位置
    """
    try:
        yaw = chassis.GetYaw() if chassis and hasattr(chassis, 'GetYaw') else 0.0
    except Exception:
        yaw = 0.0
    
    try:
        has_ball = False
        ball_x, ball_y = 0.0, 0.0
        if vision:
            ball_pos = vision.GetBallPos()
            if ball_pos and len(ball_pos) >= 2:
                ball_x, ball_y = float(ball_pos[0]), float(ball_pos[1])
                # 如果球在机器人前方一定范围内，认为持球
                if 0 < ball_x < 20 and abs(ball_y) < 10:
                    has_ball = True
    except Exception:
        ball_x, ball_y = 0.0, 0.0
        has_ball = False
    
    # 获取速度缓存
    try:
        vx = getattr(chassis, '_last_vx', 0.0)
        vy = getattr(chassis, '_last_vy', 0.0)
        wz = getattr(chassis, '_last_wz', 0.0)
    except Exception:
        vx, vy, wz = 0.0, 0.0, 0.0
    
    set_robot_state(
        yaw_deg=yaw,
        has_ball=has_ball,
        vx=vx,
        vy=vy,
        wz=wz,
    )