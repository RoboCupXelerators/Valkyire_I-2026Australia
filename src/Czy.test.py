from LunaPre import *
from BasicFuc import lockball, TurnTo_with_ball
from LunaPre.strategy_tools import angle_error_deg
import math
import time

def _read_state():
    """Read a fresh robot pose and ball state for the current control cycle."""
    robot_pos = UnitedPosition.FieldRobotPose()
    ball_pos = UnitedPosition.RobotBallPos()
    ball_state = UnitedPosition.BallState(ball_pos)
    side = 1 if robot_pos[1] > 0 else -1
    return robot_pos, ball_state, side

def IsOpenGoal(window):
    return (
        window["goal_key"] is not None
        and window["window_open"]
        and len(window["blockers"]) <=1
    )

def OpenGoalShoot():
    robot_pos = UnitedPosition.FieldRobotPose()
    window = UnitedPosition.ShootWindow(
        target="opponent",
        use_estimated_goal=False,
        current_yaw_deg=robot_pos[2],
    )

    if not IsOpenGoal(window):
        target_yaw = window.get("target_yaw_deg")
        if target_yaw is not None:
            TurnTo_with_ball(target_yaw)
        return False

    if window["can_shoot_now"]:
        peripheral.ShootBall()
        time.sleep(0.03)
        return True

    target_yaw = window.get("target_yaw_deg")
    if target_yaw is not None:
        TurnTo_with_ball(target_yaw)
    return False
if __name__ == "__main__":
    try:
        while True:
            if UnitedPosition.RestrictedArea():
                robot_pos, ball_state, side = _read_state()
                if ball_state == "held":
                    OpenGoalShoot()          
                elif ball_state == "lost":
                    UnitedPosition.Pos2Pos([0, 0, 0])
                else:
                    lockball.LockBallSlip()
                    time.sleep(0.03)
    except Exception:
        try:
            UnitedPosition.Pos2Pos([0, 0, 0])
        except Exception:
            pass
    finally:
        chassis.stop()
        peripheral.Dribble(False)
        UnitedPosition.Stop()