"""Repair of the legacy 20260831 sideline playbook.

The routes remain intentionally familiar, but every tick is now nonblocking,
the shooting window is read only once, and heading calculations are made
explicit.  This module has no command-line arguments.
"""

import math
import time

from BasicFuc import TurnTo_with_ball, lockball
from LunaPre import Settings, UnitedPosition, chassis, peripheral
from LunaPre.strategy_tools import angle_error_deg, safe_stop, valid_pose


# 30 Hz keeps this legacy strategy responsive without busy-spinning one CPU core.
LOOP_DT_S = 0.03
# Existing strategy's proven dribbler value; kept in one place for pitch tuning.
DRIBBLER_SPEED = 0.7
# These limits reproduce the original sideline routes in centimetres.
SIDELINE_ENTRY_X_CM = 40.0
SIDELINE_FINISH_X_CM = 90.0
SIDELINE_Y_CM = 40.0
# Begin the central final action before the approximately x=100 cm goal line.
CENTRAL_SHOT_X_CM = 95.0
# Goal target used by the historical back-kick; y is mirrored for each sideline.
BACK_KICK_GOAL_X_CM = 121.5
BACK_KICK_GOAL_Y_CM = 30.0
# Keep the final turn tighter than TurnTo_with_ball's normal completion tolerance.
SHOT_AIM_TOLERANCE_DEG = 8.0


def _read_state():
    """Read a fresh robot pose and semantic ball state for one control cycle."""
    robot_pos = UnitedPosition.FieldRobotPose()
    ball_pos = UnitedPosition.RobotBallPos()
    ball_state = UnitedPosition.BallState(ball_pos)
    # At centre line choose upper route deterministically instead of reacting to tiny pose noise.
    side = 1 if robot_pos[1] >= 0 else -1
    return robot_pos, ball_state, side


def Macon_kick():
    """Carry along a sideline and use the original rebound-kick route."""
    robot_pos, ball_state, side = _read_state()
    if ball_state != "held":
        return False
    peripheral.Dribble(DRIBBLER_SPEED)
    TurnTo_with_ball(90 * side)
    threshold = [[SIDELINE_ENTRY_X_CM, 24 * side], [SIDELINE_FINISH_X_CM, 50 * side]]
    if UnitedPosition.PointInShape(robot_pos, threshold):
        rebound_yaw = (-50.0 * side) % 360.0
        if abs(angle_error_deg(robot_pos[2], rebound_yaw)) < SHOT_AIM_TOLERANCE_DEG:
            peripheral.ShootBall()
            return True
        chassis.FieldTurn(40 * side)  # Legacy short field-frame turn before the rebound release.
        return False
    UnitedPosition.Pos2Pos([56.0, SIDELINE_Y_CM * side, 90.0 * side])
    return False


def back_kick():
    """Aim from the wide lane at the far-side goal point, then release once."""
    robot_pos, ball_state, side = _read_state()
    if ball_state != "held":
        return False
    peripheral.Dribble(DRIBBLER_SPEED)
    if robot_pos[0] <= SIDELINE_FINISH_X_CM:
        UnitedPosition.Pos2Pos([SIDELINE_FINISH_X_CM, robot_pos[1], robot_pos[2]])
        return False

    aim = [BACK_KICK_GOAL_X_CM, -BACK_KICK_GOAL_Y_CM * side]
    # Keep the field-frame aim explicit: atan2 always receives (dy, dx).
    goal = math.degrees(math.atan2(aim[1] - robot_pos[1], aim[0] - robot_pos[0])) % 360.0
    if abs(angle_error_deg(robot_pos[2], goal)) > SHOT_AIM_TOLERANCE_DEG:
        TurnTo_with_ball(goal)
        return False
    peripheral.ShootBall()
    return True


def IsOpenGoal(window=None):
    """Return whether ShootWindow found a usable corridor without rereading sensors."""
    if window is None:
        window = UnitedPosition.ShootWindow(target="opponent", use_estimated_goal=False)
    return window.get("goal_key") is not None and bool(window.get("window_open"))


def OpenGoalShoot():
    """Turn to the one observed shooting corridor, then fire only when aligned."""
    robot_pos = UnitedPosition.FieldRobotPose()
    window = UnitedPosition.ShootWindow(
        target="opponent",
        use_estimated_goal=False,
        current_yaw_deg=robot_pos[2],
    )
    if not IsOpenGoal(window):
        target_yaw = window.get("target_yaw_deg")
        if target_yaw is not None:
            TurnTo_with_ball(target_yaw)
        return False
    if window.get("can_shoot_now"):
        peripheral.ShootBall()
        return True
    target_yaw = window.get("target_yaw_deg")
    if target_yaw is not None:
        TurnTo_with_ball(target_yaw)
    return False


def defence():
    """Legacy fallback: retain ball prediction but never chase a missing coordinate."""
    robot_pos, ball_state, side = _read_state()
    if ball_state == "held":
        peripheral.Dribble(DRIBBLER_SPEED)
        if robot_pos[2] < 90 or robot_pos[2] > 270:
            chassis_positions = UnitedPosition.FieldChassisPos()
            if chassis_positions and chassis_positions[0][1] < robot_pos[1]:
                peripheral.ShootBall()
            else:
                TurnTo_with_ball(0)
                peripheral.ShootBall()
            return True
        if UnitedPosition.PointInShape(robot_pos, [[-80, 75], [-90, -75]]):
            chassis.FieldTurn(150 * side)  # Original wide escape to avoid firing through own corner.
        else:
            chassis.FieldTurn(0)
        peripheral.ShootBall()
        return True

    UnitedPosition.PredictBallLanding()
    UnitedPosition.Pos2Pos(Settings.CheckPoints.Home)
    return False


def step():
    """Run the original playbook for one bounded control cycle."""
    if not UnitedPosition.RestrictedArea():
        # RestrictedArea has sent its nonblocking escape command; do not overwrite it.
        peripheral.Dribble(False)
        return "restricted_recover"

    robot_pos, ball_state, _side = _read_state()
    if not valid_pose(robot_pos):
        chassis.stop()
        peripheral.Dribble(False)
        return "invalid_pose_stop"
    if ball_state == "lost":
        defence()
        return "defence"
    if ball_state == "free":
        if abs(float(robot_pos[1])) >= 45.0:
            chassis.FieldMoveVector(0.0, -math.copysign(0.20, float(robot_pos[1])), robot_pos[2], KpW=0.55)
            return "sideline_recover"
        lockball.LockBallSlip()
        return "lock_ball"
    if robot_pos[0] < 56.0 and abs(robot_pos[1]) > 50.0:
        Macon_kick()
        return "macon_kick"
    if robot_pos[0] >= 56.0 and abs(robot_pos[1]) > 50.0:
        back_kick()
        return "back_kick"
    if 0.0 < robot_pos[0] <= CENTRAL_SHOT_X_CM and abs(robot_pos[1]) < 50.0:
        OpenGoalShoot()
        return "open_goal"
    if robot_pos[0] > CENTRAL_SHOT_X_CM and abs(robot_pos[1]) < 50.0:
        peripheral.ShootBall()
        return "close_shot"
    lockball.LockBallSlip()
    return "reacquire"


def main():
    try:
        while True:
            step()
            time.sleep(LOOP_DT_S)
    finally:
        safe_stop(chassis, peripheral, UnitedPosition)


if __name__ == "__main__":
    main()
