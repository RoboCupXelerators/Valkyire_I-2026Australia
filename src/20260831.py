from LunaPre import *
from BasicFuc import lockball
from LunaPre.strategy_tools import BALL_NOT_FOUND, BALL_HELD
import time

def wall_kick():
    try:
        while True:
            if UnitedPosition.RestrictedArea():
                ball_pos = UnitedPosition.RobotBallPos()
                ball_state = UnitedPosition.BallState(ball_pos)
                if ball_state == "lost":
                    UnitedPosition.Pos2Pos([-15,0,0])
                elif ball_state =="held":
                    RoboPos = UnitedPosition.FieldRobotPose()
                    TargetY = 88
                    x_target = chassis.FieldMoveVector(RoboPos[0],TargetY,90)
                    y = 30
                    if y>0:
                        k=1
                    elif y<0:
                        k=-1
                    peripheral.Dribble(0.7)
                    chassis.FieldTurn(90*k)
                    if x_target:
                        UnitedPosition.Pos2Pos([100,88*k,90*k])
                        chassis.FieldTurn(40)
                        peripheral.ShootBall()
                else:
                    lockball.LockBallSlip()
                    time.sleep(0.03)
    finally:
        chassis.stop()
        peripheral.Dribble(False)
        UnitedPosition.Stop()



