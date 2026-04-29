"""exec

临时执行脚本（移动路径测试）。
避免 import 即执行。
"""

from __future__ import annotations

from LunaPre import *
from BasicFuc import *


def main():
    try:
        UnitedPosition.Move2Path([[80, 0, 0],[30, 0, 0],[15, -25, 0],[0, -30, 0],[-15, -25, 0],[-30, 0, 0],[-15, 25, 0],[0, 30, 0],[15, 25, 0]], 0.8)
        UnitedPosition.Move2Path([[30, 0, 0]], 0.8)
        while(1):
                if UnitedPosition.RobotBallPos() == [0xDDD, 0xDDD]:
                    break
                else:
                    lockball.LockBallSlip()
                    peripheral.ShootBall()


    finally:
        chassis.stop()
        UnitedPosition.Stop()

if __name__ == "__main__":
    main()
