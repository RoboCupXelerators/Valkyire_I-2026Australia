from BasicFuc import *
from LunaPre import Settings, UnitedPosition, chassis, logger, peripheral
from LunaPre.strategy_tools import BALL_NOT_FOUND, safe_stop

import math
import numpy as np


def activate_func(input_score):
    x = input_score
    y1 = 1 / (1 + math.e ** (-x))
    y11 = 1 / (2 + math.e ** (-x) + math.e ** x)
    y2 = (math.e ** x - math.e ** (-x)) / (math.e ** x + math.e ** (-x))
    y22 = 1 - y2 * y2
    y3 = np.where(x < 0, 0, x)
    y33 = np.where(x < 0, 0, 1)
    return y33


def sigmod(x):
    exp = np.exp(x - np.max(x))
    p = exp / np.sum(exp)
    return list(p)


def UplaneScore(pose):
    """Score an upper-plane shot, favoring a wide position."""
    x, y, w = pose
    x = x + 120
    w = math.sin(math.radians(w))
    if y < 0:
        w = -w
    if abs(y) > 50:
        return round(abs(x / 240) * 0.2 - abs(y / 90) * 0.5 + w, 2)
    return 0


def BowScore(pose):
    """Score a curved shot candidate."""
    x, y, w = pose
    aim = (UnitedPosition.Pos2Angle([x, y], [100, 0]) + 180) % 360
    error_aUw = abs(w - aim)
    return round(
        activate_func((360 - error_aUw) / 360 - 0.5) * 0.8
        - abs(x / 120) * 0.1
        + abs(y / 90) * 0.1,
        2,
    )


def HuckScore(pose):
    """Score a direct long shot based on goal alignment."""
    x, y, w = pose
    aim = UnitedPosition.Pos2Angle([x, y], [100, 0]) % 360
    error_aUw = abs(w - aim)
    return round(
        activate_func((360 - error_aUw) / 360 - 0.5) * 0.8
        + abs(x / 120) * 0.1
        - abs(y / 90) * 0.1,
        2,
    )


def JumpScore(pose):
    """Score a side-lane jump or rebound candidate."""
    x, y, w = pose
    aim_pose = [80, 200] if y > 0 else [80, -200]
    aim_w = UnitedPosition.Pos2Angle([x, y], aim_pose) % 360
    error_aUw = abs(w - aim_w)
    if abs(y) > 50:
        return round(activate_func((360 - error_aUw) / 360 - 0.98), 2)
    return 0


def UplaneStep(thershold=[[50, 30], [100, 90]]):
    """Run one upper-plane shot control tick while carrying the ball."""
    pose = UnitedPosition.FieldRobotPose()
    x, y, _ = pose
    if y < 0:
        angle_k = -1
        for threshold in thershold:
            threshold[1] = -abs(threshold[1])
    else:
        angle_k = 1

    if UnitedPosition.PointInShape(pose, thershold):
        if UnitedPosition.CanShoot():
            peripheral.ShootBall()
        else:
            try:
                TurnTo_with_ball_orbit(-UnitedPosition.ShootWindow()["target_yaw_deg"])
            except Exception:
                TurnTo_with_ball_orbit(UnitedPosition.Pos2Angle([x, y], [100, 0]) % 360)
    elif abs(y) <= 50:
        if TurnTo_with_ball(90 * angle_k):
            chassis.FieldMoveVector(0, 1, 90 * angle_k)
    elif TurnTo_with_ball(90 * angle_k):
        chassis.FieldMoveVector(0.91, 0, 90 * angle_k)


def logic_step():
    """Select and execute one shot strategy tick."""
    pose = UnitedPosition.FieldRobotPose()
    logger.success("UplaneScore: %s" % UplaneScore(pose))
    logger.success("HuckScore: %s" % HuckScore(pose))
    logger.success("JumpScore: %s" % JumpScore(pose))
    logger.warning("BowScore: %s" % BowScore(pose))

    if UnitedPosition.RobotBallPos() == BALL_NOT_FOUND:
        peripheral.Dribble(False)
        logger.info("Ball not found, moving to home position.")
        UnitedPosition.Pos2Pos(Settings.CheckPoints.Home)
        return

    if not lockball.LockBallSlip():
        return

    uplane_score = UplaneScore(pose)
    huck_score = HuckScore(pose)
    jump_score = JumpScore(pose)
    scores = [uplane_score, huck_score, jump_score]
    logger.debug(scores)

    if max(scores) == huck_score:
        if UnitedPosition.CanShoot():
            peripheral.ShootBall()
        else:
            try:
                TurnTo_with_ball(-UnitedPosition.ShootWindow()["target_yaw_deg"])
            except Exception:
                TurnTo_with_ball(UnitedPosition.Pos2Angle(pose, [100, 0]) % 360)
    elif max(scores) == uplane_score:
        UplaneStep()
    else:
        logger.debug("Jump/rebound score selected; shot action is disabled.")


if __name__ == "__main__":
    try:
        while True:
            logic_step()
    except KeyboardInterrupt:
        logger.info("Program interrupted by user.")
    finally:
        safe_stop(chassis, peripheral, UnitedPosition)
