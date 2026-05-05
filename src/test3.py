"""template

这里是主逻辑的标准模版
"""

from __future__ import annotations

from BasicFuc import *


def main():
    try:
        while key():
            time.sleep(1)
        peripheral.Dribble(0.35)
        chassis.RobotMoveVector(10, 0, 0)
        time.sleep(1)
        chassis.RobotMoveVector(0, 0, 90)
        time.sleep(1)
        peripheral.ShootBall()
        peripheral.Dribble(0.0)
    finally:
        chassis.stop()
        UnitedPosition.Stop()

if __name__ == "__main__":
    main()
