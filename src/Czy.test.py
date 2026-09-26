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

def Macon_kick():
    robot_pos, ball_state, side = _read_state()
    if ball_state == "held":
        peripheral.Dribble(0.2)
        TurnTo_with_ball(100 * side)
        threshold = [[40, 30 * side], [90, 80 * side]]
        if UnitedPosition.PointInShape(robot_pos, threshold):
            if abs(angle_error_deg(robot_pos[2],55*side)) < 10:
                peripheral.ShootBall()
                time.sleep(0.03)
                return True
            else:
                TurnTo_with_ball(55 * side)
            return False
        else:
            UnitedPosition.Pos2Pos([56, 75 * side, 90])
            return False
    else:
        return False
if __name__ == "__main__":
    try:
        while True:
            if UnitedPosition.RestrictedArea():
                robot_pos, ball_state, side = _read_state()
                if ball_state == "held":
                    #    Bits.MorsePlay("99")
                    Macon_kick()
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