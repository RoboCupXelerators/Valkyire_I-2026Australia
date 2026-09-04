from LunaPre import *
from BasicFuc import lockball, TurnTo_with_ball
from LunaPre.strategy_tools import BALL_NOT_FOUND, BALL_HELD
import time

try:
    while True:
        # if UnitedPosition.RestrictedArea():
            ball_pos = UnitedPosition.RobotBallPos()
            ball_state = UnitedPosition.BallState(ball_pos)
            if ball_state == "lost":
                UnitedPosition.Pos2Pos([0,0,0])
                print("ball lost")
            elif ball_state =="held":
                RoboPos = UnitedPosition.FieldRobotPose()
                if RoboPos[1]>0:
                    k=1
                else:
                    k=-1
                peripheral.Dribble(0.7)
                TurnTo_with_ball(90*k)
                thershold = [[40, 30*k], [90, 90*k]]
                if UnitedPosition.PointInShape(RoboPos, thershold):
                    if abs(RoboPos[2]-50) < 10:
                        peripheral.ShootBall()
                    else:
                        chassis.FieldTurn(40*k)
                else:
                    UnitedPosition.Pos2Pos([50,75*k,90])
            else:
                lockball.LockBallSlip()
finally:
    chassis.stop()
    peripheral.Dribble(False)
    UnitedPosition.Stop()