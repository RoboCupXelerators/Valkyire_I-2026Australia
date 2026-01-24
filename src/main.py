from LunaPre import *
from BasicFuc import *

lock = LockSeries()

try:
    pass
    rob = UnitedPosition.AbsRoboPosition()
    print(rob)

    if rob[1] < 0:
        UnitedPosition.Move2Path([[-40,30,0],[0,30,0]],1)
        chassis.TurnTo(340)

    else:
        UnitedPosition.Move2Path([[-40,-30,0],[0,-30,0]],1)
        chassis.TurnTo(20)
        ###
    from BasicFuc import *
    lock = LockSeries()
    while 1:
        if lock.LockBallSlip():
            break
        else:
            continue
        ###
    Peripherals().ShootBall()    

except Exception as e:
    raise e
finally:
    chassis.stop()
    lidar.stop()