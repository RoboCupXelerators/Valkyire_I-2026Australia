from LunaPre import *

try:
    UnitedPosition.Move2Path([[0,0,90],[-90,50,0],[90,50,0],[90,-50,0],[-90,-50,0]],0.8)
except Exception as e:
    raise e
finally:
    chassis.stop()
    lidar.stop()
