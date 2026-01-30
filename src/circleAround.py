from BasicFuc import *

try:
    peripheral.Dribble(1.0)
        # p = UnitedPosition.AbsRoboPosition()
    c = 0
    while True:
        angle = UnitedPosition.Local2Angle([110,0])%360
        ball = UnitedPosition.AbsBallPos()
        if lockball.LockBallSlip():
            iDeltaAngle = -int(math.degrees(math.atan2(ball[0], ball[1])) - 90) +180
            if angle > 180:
                iDirectionFactor = -1
            elif angle <= 180:
                iDirectionFactor = 1
            else:
                chassis.stop()
            if not abs(angle - chassis.GetYaw() + 180) <= Settings.ExpectedVals.ErrorRange:
                chassis.SetMotor([iDirectionFactor * 15,-iDirectionFactor * (150 - iDeltaAngle),-iDirectionFactor * 15,iDirectionFactor * (150 - iDeltaAngle)])
            else:
                chassis.stop()
                c += 1
                time.sleep(.1)
                if c >= 3:
                    chassis.RelMoveAngle(0,300)
                    time.sleep(0.5)
                    peripheral.ShootBall()
                    chassis.stop()
                    c = 0
        else:
            c = 0
        if ball == [0xfff,0xfff]:
            peripheral.Dribble(0.6)
            UnitedPosition.Pos2Pos(Settings.CheckPoints.Home,4.5)
        time.sleep(0.03)
        # CircleAround(angle)

except Exception as e:
    raise e
finally:
    peripheral.Dribble(False)
    chassis.stop()
    UnitedPosition.stop()
    chassis.stop()
