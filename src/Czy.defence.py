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

def defence():#防守机
    robot_pos, ball_state, side = _read_state()
    if ball_state == "held":
        if robot_pos[2] < 90 or robot_pos[2] > 270:
            chassis_positions = UnitedPosition.FieldChassisPos()
            if chassis_positions and chassis_positions[0][1] < robot_pos[1]:
                peripheral.Dribble(0.3)
                peripheral.ShootBall()
                time.sleep(0.03)
                return True
            else:
                peripheral.Dribble(0.3)
                TurnTo_with_ball(0)
                peripheral.ShootBall()
                time.sleep(0.03)
                return False          
        else:
            peripheral.Dribble(0.7)
            if UnitedPosition.PointInShape(robot_pos, [[-80, 75], [-90, -75]]):
                TurnTo_with_ball(150 * side)
                peripheral.ShootBall()
                time.sleep(0.03)
                return True           
            else:
                TurnTo_with_ball(0)
                peripheral.ShootBall()
                time.sleep(0.03)
                return True           
    else:
        UnitedPosition.PredictBallLanding()
        return False