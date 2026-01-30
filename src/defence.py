from BasicFuc import *

try:
    while(1):
        BalllPosition = UnitedPosition.Relative_Ball_Position()
        Yaw = Bits.GetYaw()
        BallAngle = UnitedPosition.RelBallAngle()
        if BalllPosition == [0xfff,0xfff]:
            UnitedPosition.Pos2Pos(Settings.CheckPoints.Home,3)
        elif BalllPosition == [0xddd,0xddd]:
            peripheral.ShootBall()
            time.sleep(1)
        else:
            if 0 < BalllPosition[0] < 20:
                LockSeries().LockBallSlip()
            else:
                chassis.AbsMoveVetor(0,BalllPosition[1]*4.5,0)
                # UnitedPosition.Pos2Pos([Settings.CheckPoints.Home[0],BalllPosition[1],0],3)
        Distance = lidar.GetDists()
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
