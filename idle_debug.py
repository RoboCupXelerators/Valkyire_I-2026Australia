"""IDLE-friendly live debug harness.

Run this file from IDLE or a terminal when a real robot or Unity simulator is
available. If neither backend is available, use `python -m py_compile ...`
instead of running this file.
"""

from __future__ import annotations

import os
import sys
import time


BACKEND = os.environ.get("ROBOCUPX_IDLE_BACKEND", "").strip().upper()

if BACKEND == "SIM":
    # Unity Play mode should be running with XelUdpBridge.
    os.environ.setdefault("ROBOCUPX_BITS", "SIM")
    os.environ.setdefault("XEL_UNITY_IP", "127.0.0.1")
    os.environ.setdefault("XEL_CMD_PORT", "20001")
    os.environ.setdefault("XEL_STATUS_PORT", "20002")
    os.environ.setdefault("XEL_ROBOT_ID", "1")
elif BACKEND == "REAL":
    # Use ReasonData config as-is for the real robot.
    pass
else:
    print("Set ROBOCUPX_IDLE_BACKEND=SIM or ROBOCUPX_IDLE_BACKEND=REAL before running idle_debug.py.")
    print("Without Unity/real robot, run: python -m py_compile src/LunaPre.py src/Sensor.py idle_debug.py")
    sys.exit(2)

# LunaPre initializes runtime objects at import time, so backend env must be
# finalized before this import.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))
from LunaPre import UnitedPosition, chassis, peripheral  # noqa: E402


def read_debug_frame():
    """Return one live frame for IDLE/manual checks."""

    robot_pose = UnitedPosition.FieldRobotPose()
    robot_position = UnitedPosition.FieldRobotPosition()
    ball_pos = UnitedPosition.RobotBallPos()
    ball_state = UnitedPosition.BallState(ball_pos)
    raw_distances = UnitedPosition.DirectDistance()

    return {
        "robot_pose": robot_pose,
        "robot_position": robot_position,
        "ball_pos": ball_pos,
        "ball_state": ball_state,
        "raw_distances": raw_distances,
    }


def run_step():
    """Run one safe live strategy step and return the observed frame."""

    frame = read_debug_frame()
    chassis.stop()
    peripheral.Dribble(False)
    return frame


def main(steps=5, dt=0.03):
    print("idle_debug backend:", BACKEND)
    try:
        for i in range(steps):
            frame = run_step()
            print("step", i, frame)
            time.sleep(dt)
    finally:
        UnitedPosition.Stop()


if __name__ == "__main__":
    main()
