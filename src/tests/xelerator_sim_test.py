"""Minimal test for Unity simulator bridge.

Usage:
  python3 Software/src/tests/xelerator_sim_test.py --unity-ip 192.168.x.x

It will:
  1) spin in place
  2) drive forward
  3) enable dribble
  4) kick once

Note: Unity side must have XelUdpBridge + XeleratorRobotController attached to the robot.
"""

import argparse
import time

from Software.src.utils.XeleratorBits import XeleratorBits


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--unity-ip", required=True)
    ap.add_argument("--cmd-port", type=int, default=20001)
    ap.add_argument("--status-port", type=int, default=20002)
    ap.add_argument("--robot-id", type=int, default=1)
    args = ap.parse_args()

    with XeleratorBits(
        args.unity_ip,
        cmd_port=args.cmd_port,
        status_port=args.status_port,
        robot_id=args.robot_id,
        send_hz=50,
    ) as bits:
        t0 = time.time()

        # 1) spin
        for _ in range(120):
            bits.SetTwist(0.0, 0.0, 1.5)
            st = bits.GetStatus()
            if int(time.time() - t0) % 1 == 0:
                pass
            time.sleep(0.02)

        # 2) forward
        for _ in range(150):
            bits.SetTwist(1.0, 0.0, 0.0)
            st = bits.GetStatus()
            if _ % 25 == 0:
                print(
                    f"pose x={st.x:.2f} y={st.y:.2f} yaw={st.yaw:.1f} hasBall={st.has_ball}"
                )
            time.sleep(0.02)

        # 3) dribble for 2s
        bits.SetDribble(True, strength=1.0)
        for _ in range(100):
            bits.SetTwist(0.5, 0.0, 0.0)
            if _ % 20 == 0:
                print("hasBall:", bits.HasBall())
            time.sleep(0.02)

        # 4) kick
        bits.Kick(strength=2.0)
        for _ in range(50):
            bits.SetTwist(0.0, 0.0, 0.0)
            time.sleep(0.02)

        bits.SetDribble(False, strength=0.0)
        bits.SetTwist(0.0, 0.0, 0.0)
        time.sleep(0.1)


if __name__ == "__main__":
    main()
