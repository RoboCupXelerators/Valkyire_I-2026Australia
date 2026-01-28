from BasicFuc import *

try:
    peripheral.Dribble(1)
    time.sleep(1)
    while(1):
        # OHMYBACK()
        print(UnitedPosition.AbsRoboPosition())
        BalllPosition = UnitedPosition.Relative_Ball_Position()
        # print("BalllPosition:",BalllPosition)
        # time.sleep(1)
        if BalllPosition == [0xfff,0xfff]:
            peripheral.Dribble(0.7)
            UnitedPosition.Pos2Pos(Settings.CheckPoints.Home)
        elif BalllPosition == [0xddd,0xddd]:
            peripheral.Dribble(1)
            chassis.stop()
            time.sleep(0.3)
            peripheral.ShootBall()
            time.sleep(1)
        else:
            peripheral.Dribble(0.7)
            lockball.LockBallSlip()
except Exception as e:
    raise e
finally:
    peripheral.Dribble(False)
    chassis.stop()
    UnitedPosition.stop()
    chassis.stop()
