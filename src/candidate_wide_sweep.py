"""Wide-lane visual-search candidate for the local Unity tournament."""

from __future__ import annotations

import math
import os
import time
import json

from BasicFuc import lockball
from LunaPre import UnitedPosition, chassis, peripheral
from LunaPre.strategy_tools import BALL_HELD, BALL_NOT_FOUND, clamp, valid_pose
import rule_candidate_common as helper


PROFILE = "wide_sweep"
LOOP_DT = 0.03
CAPTURE_DISTANCE_CM = 19.0
MOVE_GAIN = 0.014
MOVE_X_LIMIT = 0.58
MOVE_Y_LIMIT = 0.42
SWEEP_STAGE_SECONDS = 2.2
SWEEP_STAGES = ((-42.0, 54.0), (-8.0, -56.0), (30.0, 50.0), (48.0, -46.0))


def robot_id() -> int:
    try:
        return int(os.environ.get("XEL_ROBOT_ID", "1"))
    except ValueError:
        return 1


def teammate_slot(robot: int | None = None) -> int:
    """Return a stable 0/1 slot for either physical two-robot team."""
    value = robot_id() if robot is None else int(robot)
    return (max(1, value) - 1) % 2


def goal_yaw(direction: float) -> float:
    return 0.0 if direction > 0.0 else 180.0


def patrol_target(direction: float, slot: int, now: float) -> list[float]:
    """Visit opposite wide lanes, with a time-varying phase for each teammate."""
    stage = (int(now / SWEEP_STAGE_SECONDS) + slot * 2) % len(SWEEP_STAGES)
    progress, lateral = SWEEP_STAGES[stage]
    return [direction * progress, lateral, goal_yaw(direction)]


def free_ball_owner(field_ball, slot: int) -> bool:
    """Split a free field ball by its visible lateral half between teammates."""
    return slot == (0 if float(field_ball[1]) >= 0.0 else 1)


def _move(pose, target) -> None:
    target = helper._safe_target(target)
    error_x = float(target[0]) - float(pose[0])
    error_y = float(target[1]) - float(pose[1])
    speed_x = 0.0 if abs(error_x) < 2.0 else clamp(error_x * MOVE_GAIN, -MOVE_X_LIMIT, MOVE_X_LIMIT)
    speed_y = 0.0 if abs(error_y) < 2.0 else clamp(error_y * MOVE_GAIN, -MOVE_Y_LIMIT, MOVE_Y_LIMIT)
    chassis.FieldMoveVector(speed_x, speed_y, float(target[2]), KpW=0.72)


def _approach_target(field_ball, direction: float, slot: int) -> list[float]:
    shoulder = 14.0 if slot == 0 else -14.0
    return [
        clamp(float(field_ball[0]) - direction * 18.0, -72.0, 72.0),
        clamp(float(field_ball[1]) + shoulder, -62.0, 62.0),
        goal_yaw(direction),
    ]


def _support_target(field_ball, direction: float, slot: int) -> list[float]:
    lateral = -34.0 if slot == 0 else 34.0
    return [
        clamp(float(field_ball[0]) - direction * 30.0, -62.0, 62.0),
        clamp(float(field_ball[1]) + lateral, -58.0, 58.0),
        goal_yaw(direction),
    ]


def step() -> str:
    """Take one visual-only control tick and return a small testable trace."""
    pose = UnitedPosition.FieldRobotPose()
    if not valid_pose(pose):
        helper.stop()
        return "invalid_pose_stop"

    direction = helper.attack_direction()
    slot = teammate_slot()
    ball = UnitedPosition.RobotBallPos()
    ball_state = helper._ball_state(ball)

    if not UnitedPosition.RestrictedArea():
        peripheral.Dribble(False)
        _move(pose, [-direction * 48.0, 30.0 if slot == 0 else -30.0, goal_yaw(direction)])
        return "restricted_recover"

    if ball_state == "lost":
        peripheral.Dribble(False)
        _move(pose, patrol_target(direction, slot, time.monotonic()))
        return "wide_patrol"

    if ball_state == "free":
        # FieldBallPos is only valid as a local-vision-derived free-ball cue.
        field_ball = UnitedPosition.FieldBallPos()
        if not helper._valid_ball(field_ball):
            peripheral.Dribble(False)
            _move(pose, patrol_target(direction, slot, time.monotonic()))
            return "wide_patrol"
        if not free_ball_owner(field_ball, slot):
            peripheral.Dribble(False)
            _move(pose, _support_target(field_ball, direction, slot))
            return "free_support"
        if helper._valid_ball(ball) and math.hypot(float(ball[0]), float(ball[1])) <= CAPTURE_DISTANCE_CM:
            peripheral.Dribble(0.82)
            lockball.LockBallSlip()
            return "free_lock"
        peripheral.Dribble(0.82)
        _move(pose, _approach_target(field_ball, direction, slot))
        return "free_approach"

    # A held ball must never consult a field-ball coordinate.  Escape wide
    # sidelines/corners before resuming the forward carry toward the own goal.
    progress = direction * float(pose[0])
    if abs(float(pose[1])) >= 44.0 or progress >= 72.0:
        peripheral.Dribble(0.82)
        chassis.FieldMoveVector(
            -direction * 0.20,
            clamp(-float(pose[1]) * 0.016, -0.34, 0.34),
            goal_yaw(direction),
            KpW=0.76,
        )
        return "held_recover"
    if progress >= 57.0 and abs(float(pose[1])) <= 22.0:
        peripheral.Dribble(False)
        peripheral.ShootBall()
        helper.stop()
        return "held_shoot"
    peripheral.Dribble(0.86)
    chassis.FieldMoveVector(
        direction * 0.48,
        clamp(-float(pose[1]) * 0.012, -0.22, 0.22),
        goal_yaw(direction),
        KpW=0.76,
    )
    return "held_carry"


def main() -> None:
    last_trace_state: str | None = None
    last_trace_time = -math.inf
    try:
        while True:
            state = step()
            if os.environ.get("WIDE_SWEEP_TRACE") == "1":
                now = time.monotonic()
                if state != last_trace_state or now - last_trace_time >= 0.5:
                    try:
                        pose = UnitedPosition.FieldRobotPose()
                        pose = pose if valid_pose(pose) else None
                    except Exception:
                        pose = None
                    try:
                        robot_ball = UnitedPosition.RobotBallPos()
                    except Exception:
                        robot_ball = None
                    field_ball = BALL_NOT_FOUND
                    if helper._ball_state(robot_ball) == "free":
                        try:
                            field_ball = UnitedPosition.FieldBallPos()
                        except Exception:
                            field_ball = BALL_NOT_FOUND
                    try:
                        direction = helper.attack_direction()
                    except Exception:
                        direction = None
                    print(
                        json.dumps(
                            {
                                "robot_id": robot_id(),
                                "state": state,
                                "field_robot_pose": pose,
                                "robot_ball_pos": robot_ball,
                                "field_ball_pos": field_ball,
                                "attack_direction": direction,
                                "monotonic_timestamp": now,
                            },
                            separators=(",", ":"),
                            default=str,
                        ),
                        flush=True,
                    )
                    last_trace_state = state
                    last_trace_time = now
            time.sleep(LOOP_DT)
    finally:
        helper.stop()


if __name__ == "__main__":
    main()
