from LunaPre import *
from BasicFuc import *

lock = LockSeries()

try:
    Peripherals().ShootBall()    

except Exception as e:
    raise e
finally:
    chassis.stop()
    lidar.stop()