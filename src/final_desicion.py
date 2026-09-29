from LunaPre import *
from BasicFuc import *
from Hyperion import *

def main():
    pose = UnitedPosition.FieldRobotPose()
    ball = UnitedPosition.RobotBallPos()
    if not UnitedPosition.RestrictedArea():
        return
    if ball == [0xfff,0xfff]:
        UnitedPosition.Pos2Pos([-20,0,0])
        return
    if not lockball.slip():
        return
    angle2goal = UnitedPosition.Pos2Angle(pose, [80,0])
    if abs(angle2goal) < 90:
        if TurnTo_with_ball(angle2goal):
            peripheral.ShootBall()
    else:
        U_Shoot()
        

while True:
    main()
    time.sleep(0.03)