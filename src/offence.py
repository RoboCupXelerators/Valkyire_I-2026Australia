from BasicFuc import *

try:
    # peripheral.Dribble(1)
    # time.sleep(3)
    while(1):
    #     # OHMYBACK()
        BalllPosition = UnitedPosition.Relative_Ball_Position()
        # print("BalllPosition:",BalllPosition)
        # time.sleep(1)
        goal_angle = UnitedPosition.Local2Angle([110,0])
        c = 0
        if BalllPosition == [0xfff,0xfff]:
            peripheral.Dribble(0.7)
            UnitedPosition.Pos2Pos([Settings.CheckPoints.Home[0]+20,Settings.CheckPoints.Home[1],0],4.5)
        elif BalllPosition == [0xddd,0xddd]:
            if TurnTo_with_ball((goal_angle)%360):
                if BalllPosition[0] > 80:
                    peripheral.ShootBall()
                else:
                    peripheral.Dribble(0.2)
                    chassis.RelMoveAngle(0,200)
                    peripheral.ShootBall()
                time.sleep(.2)
                chassis.stop()
            else:
                c += 1
                if c >= 50:
                    c = 0
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
