"""defence

同 offence.py：避免 import 即执行死循环。
"""

from __future__ import annotations

from BasicFuc import *


def main():
    try:
        while True:
            if UnitedPosition.RestrictedArea():
                ball_position = UnitedPosition.RobotBallPos()

                if ball_position == [0xFFF, 0xFFF]:
                    UnitedPosition.Pos2Pos(Settings.CheckPoints.Home, 4.5)
                elif ball_position == [0xDDD, 0xDDD]:
                    if TurnTo_with_ball(UnitedPosition.Local2Angle([100, 0])):
                        peripheral.Dribble(0.3)
                        peripheral.ShootBall()
                        time.sleep(0.5)
                else:
                    peripheral.Dribble(0.5)
                    if 0 < ball_position[0] < 30:
                        lockball.LockBallSlip()
                    else:
                        distance_cache = UnitedPosition.DirectDistance()
                        speed_cache = max(distance_cache[1], distance_cache[3])
                        chassis.AbsMoveVetor(0, ball_position[1] * 8 - speed_cache / 100, 0)
            time.sleep(0.03)
    finally:
        peripheral.Dribble(False)
        chassis.stop()
        UnitedPosition.Stop()
        chassis.stop()


if __name__ == "__main__":
    main()
