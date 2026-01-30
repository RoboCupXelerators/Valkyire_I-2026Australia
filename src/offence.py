from BasicFuc import *

try:
    # peripheral.Dribble(1)
    # time.sleep(3)
    while(1):
    #     # OHMYBACK()
        print(UnitedPosition.AbsRoboPosition())
        BalllPosition = UnitedPosition.Relative_Ball_Position()
        # print("BalllPosition:",BalllPosition)
        # time.sleep(1)
        goal_angle = UnitedPosition.Local2Angle([110,0])
        if BalllPosition == [0xfff,0xfff]:
            peripheral.Dribble(0.7)
            UnitedPosition.Pos2Pos([Settings.CheckPoints.Home[0]+20,Settings.CheckPoints.Home[1],0],4.5)
        elif BalllPosition == [0xddd,0xddd]:
            if TurnTo_with_ball(goal_angle):
                peripheral.Dribble(1)
                chassis.RelMoveAngle(0,200)
                time.sleep(.02)
                peripheral.ShootBall()
                chassis.stop()
                logger.error(1)
            else:
                logger.success(TurnTo_with_ball(goal_angle))
        else:
            logger.warning(1)
            peripheral.Dribble(0.7)
            lockball.LockBallSlip()
except Exception as e:
    raise e
finally:
    peripheral.Dribble(False)
    chassis.stop()
    UnitedPosition.stop()
    chassis.stop()
