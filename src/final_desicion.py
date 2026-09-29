from LunaPre import *
from BasicFuc import *
from LunaPre.strategy_tools import *

def U_Shoot(goal=(100, 0), hold_s=3.0, shake_speed=1.5, shake_s=0.20):
    """背对球门时保持带球，再横摆让球借边墙反弹。"""
    pose = UnitedPosition.FieldRobotPose()
    if not lockball.LockBallSlip():
        return False
    back_to_goal = (UnitedPosition.Pos2Angle(pose, goal) + 180) % 360
    if not TurnTo_with_ball(back_to_goal):
        return False
    else:
            dribble_timer = timer()
            if not dribble_timer.countdown(hold_s):
                peripheral.Dribble(0.99)
            else:
                dribble_timer.reset()
                dribble_timer.reset() if dribble_timer.countdown(shake_s*2) else chassis.RobotMoveVector(0, shake_speed, 0) if dribble_timer.countdown(shake_s) else chassis.RobotMoveVector(0, -shake_speed, 0)
                return True

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