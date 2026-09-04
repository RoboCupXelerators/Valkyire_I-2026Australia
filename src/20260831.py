from LunaPre import *
from BasicFuc import lockball
from LunaPre.strategy_tools import BALL_NOT_FOUND, BALL_HELD
import time

def TurnTo_With_ball_Strong(angle):
    if UnitedPosition.RobotBallPos() == BALL_HELD:
        error = abs(UnitedPosition.FieldRobotPose()[2]- angle)
        if abs(UnitedPosition.FieldRobotPose()[2]- angle) == cfg.ExpectedVals.ErrorRange:
            return True
        else:
            if error < 0 and error < -100:
                error = -100
            elif error > 100:
                error = 100
            chassis.RobotTurn(error)
            return False
    else:
        return lockball.slip()

try:
    while True:
        # if UnitedPosition.RestrictedArea():
            ball_pos = UnitedPosition.RobotBallPos()
            ball_state = UnitedPosition.BallState(ball_pos)
            if ball_state == "lost":
                UnitedPosition.Pos2Pos([-15,0,0])
            elif ball_state =="held":
                RoboPos = UnitedPosition.FieldRobotPose()
                x_target = UnitedPosition.Pos2Pos([RoboPos[0],75,90])#pos2pos
                if RoboPos[1]>0:
                    k=1
                elif RoboPos[1]<0:
                    k=-1
                peripheral.Dribble(0.7)
                TurnTo_With_ball_Strong(90*k)
                if x_target:
                    x_goal = UnitedPosition.Pos2Pos([67,75*k,50])
                    if x_goal:
                        chassis.TurnTo(40)
                        peripheral.ShootBall()
            else:
                lockball.LockBallSlip()
                time.sleep(0.03)
finally:
    chassis.stop()
    peripheral.Dribble(False)
    UnitedPosition.Stop()