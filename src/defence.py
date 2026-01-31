from BasicFuc import *

try:
    # peripheral.Dribble(True)
    while(1):
        if UnitedPosition.Restricted_Area():
            BalllPosition = UnitedPosition.Relative_Ball_Position()
            Yaw = Bits.GetYaw()
            BallAngle = UnitedPosition.RelBallAngle()
            if BalllPosition == [0xfff,0xfff]:
                UnitedPosition.Pos2Pos(Settings.CheckPoints.Home,4.5)
            elif BalllPosition == [0xddd,0xddd]:
                if TurnTo_with_ball(UnitedPosition.Local2Angle([100,0])):
                    peripheral.Dribble(0.3)
                    peripheral.ShootBall()
                    time.sleep(0.5)
            else:
                peripheral.Dribble(0.5)
                if 0 < BalllPosition[0] < 30:
                    lockball.LockBallSlip()
                else:
                    Distance_cache = UnitedPosition.direct_distance()
                    if Distance_cache[1] > Distance_cache[3]:
                        SpeedCache = Distance_cache[1]
                    else:
                        SpeedCache = Distance_cache[3]

                    chassis.AbsMoveVetor(0,BalllPosition[1]*8-SpeedCache/100,0)
                    # UnitedPosition.Pos2Pos([Settings.CheckPoints.Home[0],BalllPosition[1],0],3)
        time.sleep(.03)
except Exception as e:
    raise e
finally:
    peripheral.Dribble(False)
    chassis.stop()
    UnitedPosition.stop()
    chassis.stop()
