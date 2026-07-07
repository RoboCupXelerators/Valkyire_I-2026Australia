from BasicFuc import *
from LunaPre import Settings, UnitedPosition, chassis, logger, peripheral
from LunaPre.strategy_tools import BALL_NOT_FOUND, safe_stop

import numpy as np

def activate_func(input_score):
    x = input_score
    y1 = 1 / (1 + math.e ** (-x))  # sigmoid
    # y11=math.e**(-x)/((1+math.e**(-x))**2)
    y11 = 1 / (2 + math.e ** (-x)+ math.e ** (x))  # sigmoid的导数
    y2 = (math.e ** (x) - math.e ** (-x)) / (math.e ** (x) + math.e ** (-x))  # tanh
    y22 = 1-y2*y2  # tanh函数的导数
    y3 = np.where(x < 0, 0, x)  # relu
    y33 = np.where(x < 0, 0, 1)  # ReLU函数导数
    return y33

def UplaneScore(pose):
    x, y, w = pose
    x = x + 120
    w =  (math.sin(math.radians(w)))
    if y < 0:
        w = - w
    print(w)
    return round((abs(x/240)*0.2 - abs(y/90)*0.5 + w),2)

def BowScore(pose):
    x, y, w = pose
    aim = (UnitedPosition.Pos2Angle([x,y],[100,0])+180)%360
    error_aUw = abs(w - aim)
    return round((activate_func((360-error_aUw)/360-0.5)*0.8 - abs(x/120)*0.1 + abs(y/90)*0.1),2)

def HuckScore(pose):
    x, y, w = pose
    aim = UnitedPosition.Pos2Angle([x,y],[100,0])%360
    error_aUw = abs(w - aim)
    return round((activate_func((360-error_aUw)/360-0.5)*0.8 + abs(x/120)*0.1 - abs(y/90)*0.1),2)

def JumpScore(pose):
    x, y, w = pose
    if y > 0:
        aim_pose = [100,180]
    else:
        aim_pose = [100,-180]
    aim_w = UnitedPosition.Pos2Angle([x,y],aim_pose)%360
    error_aUw = abs(w - aim_w)
    return round((activate_func((360-error_aUw)/360-0.95) - (x/240)*0.5),2)


def logic_step():
    logger.success("UplaneScore: "+str(UplaneScore(UnitedPosition.FieldRobotPose())))
    logger.success("HuckScore: "+str(HuckScore(UnitedPosition.FieldRobotPose())))
    logger.success("JumpScore: "+str(JumpScore(UnitedPosition.FieldRobotPose())))
    logger.warning("BowScore: "+str(BowScore(UnitedPosition.FieldRobotPose())))
    # time.sleep(0.3)
    # return
    logger.info(UnitedPosition.RobotBallPos())
    if not UnitedPosition.RobotBallPos() == BALL_NOT_FOUND:
        if lockball.LockBallSlip():
            pose = UnitedPosition.FieldRobotPose()
            uplane_score = UplaneScore(pose)
            huck_score = HuckScore(pose)
            jump_score = JumpScore(pose)
            scores = [
                uplane_score,
                huck_score,
                jump_score]
            logger.info("LockBallSlip executed successfully.")
            if max(scores) == huck_score:
                if TurnTo_with_ball(UnitedPosition.Pos2Angle(pose,[100,0])%360,3):
                    peripheral.ShootBall()
            elif max(scores) == uplane_score:
                if lockball.slip():
                    UplaneStep()
            elif max(scores) == jump_score:
                peripheral.ShootBall()
    else:
        peripheral.Dribble(False)
        if UnitedPosition.RobotBallPos() == BALL_NOT_FOUND:
            logger.info(f"Ball position is {BALL_NOT_FOUND}, moving to home position.")
            UnitedPosition.Pos2Pos(Settings.CheckPoints.Home)

def UplaneStep(thershold = [[50,30],[100,90]]):
    pose = UnitedPosition.FieldRobotPose()
    x, y, _ = pose
    if y < 0:
        angle_k = -1
        for t in range(len(thershold)):
            thershold[t-1][1] = -thershold[t-1][1]
            try:
                thershold[t-1][2] = -thershold[t-1][2] %360
            except:
                pass
    else:
        angle_k = 1
    if UnitedPosition.PointInShape(pose,thershold):
        if TurnTo_with_ball(UnitedPosition.Pos2Angle([x,y],[100,0])%360):
            peripheral.ShootBall()
    else:
        if TurnTo_with_ball(90*angle_k):
            chassis.FieldMoveVector(0.91,0,90*angle_k)
        

if __name__ == "__main__":
    try:
        while True:
            logic_step()
    except KeyboardInterrupt:
        logger.info("Program interrupted by user.")
    finally:
        safe_stop(chassis, peripheral, UnitedPosition)
