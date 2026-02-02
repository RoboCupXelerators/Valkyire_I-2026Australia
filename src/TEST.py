from BasicFuc import *

try:
    while True:
        # print(TurnTo_with_ball(UnitedPosition.Local2Angle([110,0]),1))
        print(chassis.GetYaw())
except Exception as e:
    raise e
finally:
    peripheral.Dribble(False)
    chassis.stop()
    UnitedPosition.stop()
    chassis.stop()
