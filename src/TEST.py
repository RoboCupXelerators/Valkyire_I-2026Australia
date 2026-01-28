from LunaPre import *

# chassis.RelMoveVetor(0,0,20)

try:
    while True:
        # peripheral.Dribble(1.0)
        # peripheral.ShootBall()
        UnitedPosition.Pos2Pos(Settings.CheckPoints.Home,4.5)
        
        # print(Bits.GetYaw())
        time.sleep(.03)

except Exception as e:
    raise e
finally:
    peripheral.Dribble(False)
    chassis.stop()
    UnitedPosition.stop()
    chassis.stop()