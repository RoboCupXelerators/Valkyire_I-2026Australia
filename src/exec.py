from LunaPre import *
#长260
try:
    
        # #Put ins
        while(1):
            rob = UnitedPosition.AbsRoboPosition()
            # print(rob)
        # chassis.stop()
        # UnitedPosition.Move2Path([[-60,80,0],[50,50,0]],1)
        
        # chassis.TurnTo(315)
        # chassis.stop()
        
        # while(1):
        #     ccc=UnitedPosition.AbsBallPos()
        #     if ccc[0]==0xddd:
        #         break
        #     else:
        #         print(ccc)
        #         continue
        # peripheral.ShootBall()
        

except Exception as e:
    raise e
finally:
    chassis.stop()
    lidar.stop()