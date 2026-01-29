from BasicFuc import *

try:
    peripheral.Dribble(1)
    time.sleep(1)
    while(1):
        # OHMYBACK()
        print(UnitedPosition.AbsRoboPosition())
        BalllPosition = UnitedPosition.Relative_Ball_Position()
        # print("BalllPosition:",BalllPosition)
        # time.sleep(1)
        if BalllPosition == [0xfff,0xfff]:
            peripheral.Dribble(0.3)
            UnitedPosition.Pos2Pos([Settings.CheckPoints.Home[0]+20,Settings.CheckPoints.Home[1],0],4.5)
        elif BalllPosition == [0xddd,0xddd]:
            peripheral.Dribble(1)
            chassis.RelMoveAngle(0,200)
            time.sleep(.02)
            peripheral.ShootBall()
            chassis.stop()
            time.sleep(1)
        else:
            peripheral.Dribble(0.5)
            lockball.LockBallSlip()
        Distance = lidar.GetDists()
        chassis.stop()
        border = 300
        if Distance[1]>650 or Distance[3]>650:
            if Distance[0]<border:
                chassis.AbsMoveAngle(0,180,300)
            elif Distance[2]<border:
                chassis.AbsMoveAngle(0,0,300)

        wedge = 200
        if Distance[0]<wedge:
            chassis.AbsMoveAngle(0,180,300)
        elif Distance[2]<wedge:
            chassis.AbsMoveAngle(0,0,300)
        elif Distance[1]<wedge:
            chassis.AbsMoveAngle(0,270,-300)
        elif Distance[3]<wedge:
            chassis.AbsMoveAngle(0,90,-300)
        time.sleep(.03)
        
except Exception as e:
    raise e
finally:
    peripheral.Dribble(False)
    chassis.stop()
    UnitedPosition.stop()
    chassis.stop()
