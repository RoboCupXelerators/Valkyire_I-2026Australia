from BasicFuc import *

try:
    while(1):
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
from LunaPre import *

# chassis.RelMoveVetor(0,0,20)

try:
    # while True:
        # peripheral.Dribble(1.0)
        # peripheral.ShootBall()
        UnitedPosition.Pos2Pos(Settings.CheckPoints.Home,4.5)
        
        # print(Bits.GetYaw())
        time.sleep(.02)

except Exception as e:
    raise e
finally:
    peripheral.Dribble(False)
    # chassis.stop()
    UnitedPosition.stop()
    chassis.stop()