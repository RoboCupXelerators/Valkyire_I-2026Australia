from BasicFuc import *
from LunaPre import Settings, UnitedPosition, chassis, logger, peripheral
from LunaPre.strategy_tools import safe_stop

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

def HuckScore(pose):
    x, y, w = pose
    aim = UnitedPosition.Pos2Angle([x,y],[120,0])%360
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
    logger.critical("UplaneScore: "+str(UplaneScore(UnitedPosition.FieldRobotPose())))
    logger.success("HuckScore: "+str(HuckScore(UnitedPosition.FieldRobotPose())))
    logger.warning("JumpScore: "+str(JumpScore(UnitedPosition.FieldRobotPose())))
    time.sleep(0.3)
    return
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
            if TurnTo_with_ball(UnitedPosition.Pos2Angle(pose,[120,0])%360):
                peripheral.ShootBall()
        elif max(scores) == uplane_score:
            pass
        elif max(scores) == jump_score:
            peripheral.ShootBall()
    else:
        peripheral.Dribble(0.3)
        if UnitedPosition.RobotBallPos() == [0xFFF, 0xFFF]:
            logger.info("Ball position is [0xFFF, 0xFFF], moving to home position.")
            UnitedPosition.Pos2Pos(Settings.CheckPoints.Home)


if __name__ == "__main__":
    try:
        while True:
            logic_step()
    except KeyboardInterrupt:
        logger.info("Program interrupted by user.")
    finally:
        safe_stop(chassis, peripheral, UnitedPosition)