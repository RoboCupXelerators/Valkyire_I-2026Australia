"""xel_unity_offence_smoke

最小联调用脚本：不跑复杂 offence 策略，直接发送轮速/踢球/带球指令，
用来确认 Python ↔ Unity UDP 通路可用。

运行前：Unity Play，并在机器人根物体上挂 XelUdpBridge (listenPort=20001, robotId=1)
"""

from __future__ import annotations

import os
import time

# force SIM bits
os.environ.setdefault("ROBOCUPX_BITS", "SIM")
os.environ.setdefault("XEL_UNITY_IP", "127.0.0.1")
os.environ.setdefault("XEL_CMD_PORT", "20001")
os.environ.setdefault("XEL_STATUS_PORT", "20002")
os.environ.setdefault("XEL_ROBOT_ID", "1")

from LunaPre import chassis, peripheral, Bits, UnitedPosition


def main():
    print("Bits:", type(Bits))
    t0 = time.time()
    try:
        # forward for 2s
        for _ in range(100):
            # use a larger speed to make motion obvious in Unity
            chassis.RobotMoveAngle(0, 200)
            if _ % 10 == 0:
                print("pose:", UnitedPosition.FieldRobotPos())
                print("ball:", UnitedPosition.RobotBallPos())
            time.sleep(0.02)

        chassis.stop()
        time.sleep(0.3)

        # rotate
        for _ in range(80):
            chassis.RobotTurn(300)
            time.sleep(0.02)

        chassis.stop()
        time.sleep(0.3)

        # dribble on for a while
        peripheral.Dribble(0.7)
        time.sleep(1.0)

        # kick
        peripheral.ShootBall()
        time.sleep(0.5)

        peripheral.Dribble(0.0)
        chassis.stop()
        # ensure simulator receives a few zero commands before exit
        for _ in range(20):
            chassis.stop()
            time.sleep(0.02)

    finally:
        try:
            UnitedPosition.Stop()
        except Exception:
            pass

        # extra safety stop at process exit
        try:
            for _ in range(20):
                chassis.stop()
                time.sleep(0.01)
        except Exception:
            pass

    print("done, dt=", time.time() - t0)


if __name__ == "__main__":
    main()
