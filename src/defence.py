from BasicFuc import *

try:
    while(1):
        BalllPosition = UnitedPosition.Relative_Ball_Position()
        Yaw = Bits.GetYaw()
        BallAngle = UnitedPosition.RelBallAngle()
        if BalllPosition == [0xfff,0xfff]:
            UnitedPosition.Pos2Pos(Settings.CheckPoints.Home,3)
        elif BalllPosition == [0xddd,0xddd]:
            peripheral.ShootBall()
            time.sleep(1)
        else:
            if 0 < BalllPosition[0] < 20:
                LockSeries().LockBallSlip()
            else:
                chassis.AbsMoveVetor(0,BalllPosition[1]*10,0)
                # UnitedPosition.Pos2Pos([Settings.CheckPoints.Home[0],BalllPosition[1],0],3)
        time.sleep(.03)
except Exception as e:
    raise e
finally:
    peripheral.Dribble(False)
    chassis.stop()
    UnitedPosition.stop()
    chassis.stop()
