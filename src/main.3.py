from LunaPre import *

try:
    while(1):
        # print(Positions().Relative_Ball_Position())
        UnitedPosition.Move2Path([[0,0,90],[-90,50,0],[90,50,0],[90,-50,0],[-90,-50,0]],0.8)
        # chassis.TurnTo(90)
except Exception as e:
    raise e
finally:
    chassis.stop()
    lidar.stop()
# from BasicFuc import *
# while(1):
#     ball = Positions().AbsBallPos()
#     if ball == [0xfff,0xfff]:
#         continue
#     if ball == [0xddd,0xddd]:
#         peripheral.ShootBall()
#         time.sleep(0.5)
#         break
#     else:
#         LockSeries().LockBallSlip()

# chassis.stop()
