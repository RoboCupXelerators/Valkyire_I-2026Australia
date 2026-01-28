from BasicFuc import *

try:
    peripheral.Dribble(1)
    time.sleep(0.5)
    while(1):
        # OHMYBACK()
        # print(UnitedPosition.AbsRoboPosition())
        BalllPosition = UnitedPosition.Relative_Ball_Position()
        # print("BalllPosition:",BalllPosition)
        # time.sleep(1)
        if BalllPosition == [0xfff,0xfff]:
            peripheral.Dribble(0.3)
            UnitedPosition.Pos2Pos(Settings.CheckPoints.Home)
        elif BalllPosition == [0xddd,0xddd]:
            peripheral.Dribble(True)
            angle = UnitedPosition.Local2Angle([90,0])
            CircleAround(angle)
            chassis.stop()
            peripheral.ShootBall()
            time.sleep(1)
        else:
            peripheral.Dribble(0.5)
            lockball.LockBallSlip()
except Exception as e:
    raise e
finally:
    peripheral.Dribble(False)
    chassis.stop()
    UnitedPosition.stop()
    chassis.stop()
