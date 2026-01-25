from BasicFuc import *

try:
    while(1):
        BalllPosition = UnitedPosition.AbsBallPos()
        print("BalllPosition:",BalllPosition)
        # if BalllPosition == [0xfff,0xfff]:
        #     UnitedPosition.Pos2Pos(Settings.CheckPoints.Home)
        # elif BalllPosition == [0xddd,0xddd]:
        #     peripheral.Dribble(True)
        #     chassis.stop()
        #     peripheral.ShootBall()
        #     time.sleep(4)
        # else:
        #     lockball.LockBallSlip()
except Exception as e:
    raise e
finally:
    lidar.stop()
    chassis.stop()
