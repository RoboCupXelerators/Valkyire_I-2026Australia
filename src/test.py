from BasicFuc import *

try:
    while True:
        if UnitedPosition.Restricted_Area():
            # print(TurnTo_with_ball(UnitedPosition.Local2Angle([110,0]),1))
            print(UnitedPosition.direct_distance())
        else:
            chassis.stop()
except Exception as e:
    raise e
finally:
    peripheral.Dribble(False)
    chassis.stop()
    UnitedPosition.stop()
    chassis.stop()
