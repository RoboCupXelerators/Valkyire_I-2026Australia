from BasicFuc import *

try:
    while(1):
        # print(TurnTo_with_ball(45))
        # BalllPosition = UnitedPosition.Relative_Ball_Position()
        # Yaw = Bits.GetYaw()
        # BallAngle = UnitedPosition.RelBallAngle()
        # if BalllPosition == [0xfff,0xfff]:
        #     UnitedPosition.Pos2Pos(Settings.CheckPoints.Home)
        # elif BalllPosition == [0xddd,0xddd]:
        #     Angle=UnitedPosition.Calculate().Pos2Angle(BalllPosition,[0,70,0])
        #     chassis.AbsMoveVetor(0,BalllPosition[1]*5,Angle)
        #     peripheral.ShootBall()
        #     time.sleep(1)
        # else:
        #     if BalllPosition[0] < 20:
        #         LockSeries().LockBallSlip()
        #     else:
        #         chassis.AbsMoveVetor(0,BalllPosition[1]*5,0)
        time.sleep(0.001)

except Exception as e:
    raise e
finally:
    peripheral.Dribble(False)
    chassis.stop()
    UnitedPosition.stop()
    chassis.stop()