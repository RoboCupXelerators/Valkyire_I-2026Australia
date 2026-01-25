from BasicFuc import *

try:
    while(1):
        BalllPosition = UnitedPosition.Relative_Ball_Position()
        # print("BalllPosition:",BalllPosition)
        # time.sleep(1)
        if BalllPosition == [0xfff,0xfff]:
            UnitedPosition.Pos2Pos(Settings.CheckPoints.Home)
        elif BalllPosition == [0xddd,0xddd]:
            peripheral.Dribble(True)
            chassis.stop()
            peripheral.ShootBall()
            time.sleep(4)
        else:
            lockball.LockBallSlip()
except Exception as e:
    raise e
finally:
    lidar.stop()
    chassis.stop()
