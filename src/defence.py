"""defence

同 offence.py：避免 import 即执行死循环。
"""

from __future__ import annotations

from BasicFuc import *

pan_y = simple_pid.PID(4, 0.08, 0.12, setpoint=0)

def main():
    try:
        while True:
            if UnitedPosition.RestrictedArea():
                ball_position = UnitedPosition.RobotBallPos()

                if ball_position == [0xFFF, 0xFFF]:
                    UnitedPosition.Pos2Pos([-30,0,0])
                    pass
                elif ball_position == [0xDDD, 0xDDD]:
                    if TurnTo_with_ball(UnitedPosition.Local2Angle([100, 0])):
                        peripheral.Dribble(0.3)
                        chassis.RobotMoveVector(100,0,0)
                        time.sleep(0.3)
                        peripheral.ShootBall()
                        time.sleep(0.5)
                else:
                    # pass
                    # peripheral.Dribble(0.5)
                    if 0 < ball_position[0] < 30:
                        lockball.LockBallSlip()
                    else:
                        distance_cache = UnitedPosition.DirectDistance()
                        speed_cache = max(distance_cache[1], distance_cache[3])
                        print(lockball.BallScore())
                        if not abs(ball_position[1]) < Settings.ExpectedVals.ErrorRange:
                            chassis.FieldMoveVector(0, -pan_y(ball_position[1]), 0)
                        else:
                            pan_y.reset()
                            chassis.stop()
                # time.sleep(0.03)
    finally:
        peripheral.Dribble(False)
        chassis.stop()
        UnitedPosition.Stop()
        chassis.stop()


if __name__ == "__main__":
    main()
