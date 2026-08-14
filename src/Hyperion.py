from BasicFuc import *
from LunaPre import Settings, UnitedPosition, chassis, logger, peripheral
from LunaPre.strategy_tools import BALL_NOT_FOUND, safe_stop

import numpy as np
import math

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

def sigmod(x):
    exp = np.exp(x - np.max(x))
    p = exp / np.sum(exp)
    return list(p)

def UplaneScore(pose):
    x, y, w = pose
    x = x + 120
    w =  (math.sin(math.radians(w)))
    if y < 0:
        w = - w
    if abs(y) > 50:
        return round((abs(x/240)*0.2 - abs(y/90)*0.5 + w),2)
    else:
        return 0

# def BowScore(pose):
#     x, y, w = pose
#     aim = (UnitedPosition.Pos2Angle([x,y],[100,0])+180)%360
#     error_aUw = abs(w - aim)
#     print(((360-error_aUw)/360-0.5)*0.8)
#     return round((activate_func((360-error_aUw)/360-0.5)*0.8 - abs(x/120)*0.1 + abs(y/90)*0.1),2)

def HuckScore(pose):
    x, y, w = pose
    aim = UnitedPosition.Pos2Angle([x,y],[100,0])%360
    error_aUw = abs(w - aim)
    return round((activate_func((360-error_aUw)/360-0.5)*0.8 + abs(x/120)*0.1 - abs(y/90)*0.1),2)

def JumpScore(pose):
    x, y, w = pose
    if y > 0:
        aim_pose = [80,200]
    else:
        aim_pose = [80,-200]
    aim_w = UnitedPosition.Pos2Angle([x,y],aim_pose)%360
    error_aUw = abs(w - aim_w)
    if abs(y) > 50:
        return round((activate_func((360-error_aUw)/360-0.98)+0),2)
    else:
        return 0


def UplaneStep(thershold = [[60,40],[100,80]]):
    pose = UnitedPosition.FieldRobotPose()
    x, y, _ = pose
    if y < 0:
        angle_k = -1
        for t in range(len(thershold)):
            if thershold[t-1][1] >0:
                thershold[t-1][1] = -thershold[t-1][1]
    else:
        angle_k = 1
        for t in range(len(thershold)):
            if thershold[t-1][1] < 0:
                thershold[t-1][1] = -thershold[t-1][1]

    if lockball.slip():
        if UnitedPosition.PointInShape(pose,thershold):
            if abs( _ - UnitedPosition.Pos2Angle([x,y],[110,0])) < cfg.ExpectedVals.ErrorRange:
                peripheral.ShootBall()
                return True
            else:
                chassis.FieldTurn(UnitedPosition.Pos2Angle([x,y],[110,0]))
                return abs( _ - UnitedPosition.Pos2Angle([x,y],[110,0])) < cfg.ExpectedVals.ErrorRange
        elif TurnTo_with_ball(90*angle_k) == True:
            if abs(y) > 50:
                UnitedPosition.Pos2Pos([80,60*angle_k,(90*angle_k)%360])
                return UnitedPosition.Pos2Pos([80,60*angle_k,(90*angle_k)%360])
            else:
                chassis.FieldMoveAngle(90*angle_k,90*angle_k,0.6)
                return abs(y) > 50
        else:
            return TurnTo_with_ball(90*angle_k)
    else:
        return lockball.slip()
         

def logic_step():
    # logger.success("UplaneScore: "+str(UplaneScore(UnitedPosition.FieldRobotPose())))
    # logger.success("HuckScore: "+str(HuckScore(UnitedPosition.FieldRobotPose())))
    # logger.success("JumpScore: "+str(JumpScore(UnitedPosition.FieldRobotPose())))
    # logger.warning("BowScore: "+str(BowScore(UnitedPosition.FieldRobotPose())))
    # logger.info(UnitedPosition.RobotBallPos())

    # if UnitedPosition.RestrictedArea():
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
                # logger.info("LockBallSlip executed successfully.")
                logger.debug(scores)
                # return
                if max(scores) == scores[1]:
                    if UnitedPosition.CanShoot():
                        peripheral.ShootBall()
                    else:
                        try:
                            TurnTo_with_ball((-UnitedPosition.ShootWindow()["target_yaw_deg"]))
                        except:
                            TurnTo_with_ball(UnitedPosition.Pos2Angle(pose,[100,0])%360)
                elif max(scores) == scores[0]:
                    UplaneStep()
                # elif max(scores) == scores[2]:
                #     peripheral.ShootBall()
        else:
            peripheral.Dribble(False)
            if UnitedPosition.RobotBallPos() == BALL_NOT_FOUND:
                logger.info(f"Ball position is {BALL_NOT_FOUND}, moving to home position.")
                UnitedPosition.Pos2Pos(Settings.CheckPoints.Home)
    # else:
        print(UnitedPosition.FieldRobotPose())

if __name__ == "__main__":
    try:
        while True:
            logic_step()
            # time.sleep(0.03)
    except KeyboardInterrupt:
        logger.info("Program interrupted by user.")
    finally:
        safe_stop(chassis, peripheral, UnitedPosition)
