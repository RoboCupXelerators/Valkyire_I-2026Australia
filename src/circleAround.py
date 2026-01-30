from BasicFuc import *

try:
    peripheral.Dribble(1.0)
    selfPosition = UnitedPosition.AbsRoboPosition()
    c = 0
    while True:
        angle = UnitedPosition.Local2Angle([110,0])%360
        ball = UnitedPosition.AbsBallPos()
        if lockball.LockBallSlip():
            peripheral.Dribble(1.0)
            iDeltaAngle = -int(math.degrees(math.atan2(ball[0], ball[1])) - 90)
            if angle > 180:
                iDirectionFactor = -1
            elif angle <= 180:
                iDirectionFactor = 1
            else:
                chassis.stop()
            if not abs(angle - chassis.GetYaw()) <= Settings.ExpectedVals.ErrorRange:
                chassis.SetMotor([iDirectionFactor * 15,-iDirectionFactor * (150 - iDeltaAngle),-iDirectionFactor * 15,iDirectionFactor * (150 - iDeltaAngle)])
            else:
                chassis.stop()
                c += 1
                time.sleep(.1)
                if selfPosition[0] >= 75:
                    peripheral.Dribble(0.5)
                    peripheral.ShootBall()
                    c = 0
                # elif selfPosition[0] > 100:
                #     peripheral.Dribble(1.0)
                #     chassis.RelMoveAngle(0,-300)
                #     c = 0
                if c >= 3:
                    chassis.RelMoveAngle(0,300)
                    peripheral.Dribble(0.3)
                    time.sleep(0.3)
                    peripheral.ShootBall()
                    chassis.stop()
                    c = 0
        else:
            c = 0
        if ball == [0xfff,0xfff]:
            # peripheral.Dribble(0.8)
            UnitedPosition.Pos2Pos(Settings.CheckPoints.Home,4.5)

        Distance = lidar.GetDists()
        if Distance[0] == 0:
            chassis.stop()


        border = 300
        restricted_area = 700
        if Distance[1]>restricted_area or Distance[3]>restricted_area:
            if Distance[0]<border:
                chassis.AbsMoveAngle(0,180,300)
            elif Distance[2]<border:
                chassis.AbsMoveAngle(0,0,300)
        wedge = 180
        if Distance[0]<wedge:
            chassis.AbsMoveAngle(0,180,300)
        elif Distance[2]<wedge:
            chassis.AbsMoveAngle(0,0,300)
        elif Distance[1]<wedge:
            chassis.AbsMoveAngle(0,270,-300)
        elif Distance[3]<wedge:
            chassis.AbsMoveAngle(0,90,-300)
        time.sleep(0.03)
        # CircleAround(angle)

except Exception as e:
    raise e
finally:
    peripheral.Dribble(False)
    chassis.stop()
    UnitedPosition.stop()
    chassis.stop()
