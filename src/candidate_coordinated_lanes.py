"""Nonblocking 2v2 candidate that separates teams into complementary lanes."""

from __future__ import annotations

import math
import os
import time
import json

from BasicFuc import lockball
from LunaPre import UnitedPosition, chassis, peripheral
from LunaPre.strategy_tools import BALL_HELD, BALL_NOT_FOUND, clamp, valid_pose
import rule_candidate_common as helper


PROFILE = "coordinated_lanes"
LOOP_DT = 0.03
SHOULDER_STANDOFF_CM = 20.0
SHOULDER_OFFSET_CM = 30.0
CAPTURE_DISTANCE_CM = 19.0
OPPONENT_CLEARANCE_CM = 42.0
DROP_ZONE_X_CM = 62.0
DROP_ZONE_Y_CM = 42.0
DROP_SHOULDER_REACHED_CM = 12.0
DROP_ENTRY_STANDOFF_CM = 8.0
DROP_ENTRY_LATERAL_CM = 10.0
# The restricted goal box begins near 65 cm.  Start the final action early
# enough that a wide carrier can retreat and centralize without entering it.
FINISH_PROGRESS_CM = 60.0
OUTER_PROGRESS_CM = 78.0
GOAL_MOUTH_HALF_WIDTH_CM = 24.0
MOVE_KP_PER_CM = 0.012
MOVE_X_LIMIT = 0.52
MOVE_Y_LIMIT = 0.34
MOVE_DEADBAND_CM = 2.0
CENTER_OWNERSHIP_DEADBAND_CM = 8.0
POSSESSION_SETTLE_SECONDS = 0.30
CARRY_FORWARD_LIMIT = 0.18
CARRY_LATERAL_LIMIT = 0.12
DEFENDER_CLEAR_LATERAL_SPEED = 0.10
RECOVERY_OUTER_RETREAT_SPEED = 0.18
RECOVERY_RETREAT_SPEED = 0.12
RECOVERY_INWARD_LIMIT = 0.12
RESTRICTED_EXIT_X_CM = 60.0
RESTRICTED_EXIT_Y_CM = 38.0
YIELD_INTERIOR_X_CM = 52.0
YIELD_INTERIOR_Y_CM = 32.0
ORDINARY_YIELD_STANDOFF_CM = 18.0
PRE_SHOT_YAW_TOLERANCE_DEG = 8.0
SEARCH_STAGE_SECONDS = 3.0
SEARCH_PATROL_STAGES = (
    (-44.0, 36.0),
    (-16.0, -36.0),
    (24.0, 36.0),
    (44.0, -36.0),
)
SEARCH_DEFENDER_STAGE_OFFSET = 2

# This candidate runs once per robot process, so a single marker is sufficient
# to require the safe shoulder phase before a close acquisition.
_drop_approach_marker: tuple[float, float, float] | None = None
_free_approach_marker: tuple[float, float, float] | None = None
_held_since: float | None = None
_corner_release_pending = False


def robot_id() -> int:
    try:
        return int(os.environ.get("XEL_ROBOT_ID", "1"))
    except ValueError:
        return 1


def lane_side(robot: int | None = None) -> float:
    """Give physical teams 1/2 and 3/4 reproducible opposite shoulders."""
    value = robot_id() if robot is None else int(robot)
    return 1.0 if ((max(1, value) - 1) // 2) % 2 == 0 else -1.0


def effective_role(requested_role: str) -> str:
    """Assign odd robots to attack and even robots to defensive cover."""
    if os.environ.get("COORDINATED_LANES_TEAM_SPLIT", "1") != "1":
        return requested_role
    return "attacker" if robot_id() % 2 else "defender"


def _goal_yaw(direction: float) -> float:
    return 0.0 if direction > 0.0 else 180.0


def _yaw_error(target_yaw: float, yaw: float) -> float:
    """Return the shortest signed error between two field headings."""
    return (target_yaw - yaw + 180.0) % 360.0 - 180.0


def _enemies() -> list[list[float]]:
    try:
        positions = UnitedPosition.FieldChassisPos()
    except Exception:
        return []
    if not isinstance(positions, list):
        return []
    return [list(point[:2]) for point in positions if point and len(point) >= 2]


def _opponent_distance(field_ball, enemies: list[list[float]]) -> float:
    distances = []
    for enemy in enemies:
        try:
            distances.append(math.hypot(float(enemy[0]) - float(field_ball[0]), float(enemy[1]) - float(field_ball[1])))
        except (TypeError, ValueError):
            continue
    return min(distances, default=math.inf)


def attacker_shoulder(field_ball, direction: float, side: float) -> list[float]:
    """Approach from the own-goal side and a team-specific ball shoulder."""
    return [
        float(field_ball[0]) - direction * SHOULDER_STANDOFF_CM,
        clamp(float(field_ball[1]) + side * SHOULDER_OFFSET_CM, -62.0, 62.0),
        _goal_yaw(direction),
    ]


def search_patrol_target(direction: float, side: float, now: float, role: str = "attacker") -> list[float]:
    """Sweep role-separated patrol stages while waiting for a ball observation."""
    stage_offset = SEARCH_DEFENDER_STAGE_OFFSET if role == "defender" else 0
    stage = (int(now / SEARCH_STAGE_SECONDS) + stage_offset) % len(SEARCH_PATROL_STAGES)
    progress, lateral = SEARCH_PATROL_STAGES[stage]
    return [direction * progress, side * lateral, _goal_yaw(direction)]


def is_safe_drop_zone(field_ball) -> bool:
    """Identify the standard corner and penalty drop-ball areas."""
    return abs(float(field_ball[0])) >= DROP_ZONE_X_CM and abs(float(field_ball[1])) >= DROP_ZONE_Y_CM


def drop_ball_owner(field_ball, direction: float, robot: int | None = None) -> bool:
    """Give the attacking-half team, and only its odd attacker, a drop ball."""
    return is_safe_drop_zone(field_ball) and free_ball_owner(field_ball, direction, robot)


def ball_owner_direction(field_ball) -> float:
    """Choose one field-side owner with a stable kickoff priority at centre."""
    ball_x = float(field_ball[0])
    if ball_x > CENTER_OWNERSHIP_DEADBAND_CM:
        return 1.0
    if ball_x < -CENTER_OWNERSHIP_DEADBAND_CM:
        return -1.0
    return 1.0


def free_ball_owner(field_ball, direction: float, robot: int | None = None) -> bool:
    """Only the designated field-side odd robot may acquire a free ball."""
    value = robot_id() if robot is None else int(robot)
    return value % 2 == 1 and float(direction) == ball_owner_direction(field_ball)


def drop_yield_target(field_ball, direction: float, side: float, robot: int | None = None) -> list[float]:
    """Place each nonowner in a distinct, owner-relative drop-ball staging lane."""
    value = robot_id() if robot is None else int(robot)
    owner_direction = ball_owner_direction(field_ball)
    owner_robot = 1 if owner_direction > 0.0 else 3
    owner_side = lane_side(owner_robot)
    opposing_attacker = 3 if owner_robot == 1 else 1

    if value == owner_robot + 1:
        x_offset, y_offset = 30.0, -30.0
    elif value == opposing_attacker:
        x_offset, y_offset = 7.0, -76.0
    else:
        x_offset, y_offset = 39.0, -60.0
    return [
        clamp(float(field_ball[0]) - owner_direction * x_offset, -YIELD_INTERIOR_X_CM, YIELD_INTERIOR_X_CM),
        clamp(float(field_ball[1]) + owner_side * y_offset, -YIELD_INTERIOR_Y_CM, YIELD_INTERIOR_Y_CM),
        _goal_yaw(owner_direction),
    ]


def ordinary_yield_target(field_ball, direction: float, side: float) -> list[float]:
    """Hold a nonowner in its interior support lane for an ordinary free ball."""
    owner_direction = ball_owner_direction(field_ball)
    return [
        clamp(
            float(field_ball[0]) - owner_direction * ORDINARY_YIELD_STANDOFF_CM,
            -YIELD_INTERIOR_X_CM,
            YIELD_INTERIOR_X_CM,
        ),
        clamp(side * YIELD_INTERIOR_Y_CM, -YIELD_INTERIOR_Y_CM, YIELD_INTERIOR_Y_CM),
        _goal_yaw(owner_direction),
    ]


def drop_entry_target(field_ball, direction: float, side: float) -> list[float]:
    """Make the final approach from an angled own-goal-side pose."""
    return [
        clamp(float(field_ball[0]) - direction * DROP_ENTRY_STANDOFF_CM, -60.0, 60.0),
        clamp(float(field_ball[1]) + side * DROP_ENTRY_LATERAL_CM, -58.0, 58.0),
        _goal_yaw(direction),
    ]


def _same_drop_ball(field_ball, direction: float) -> bool:
    return _same_approach_ball(_drop_approach_marker, field_ball, direction)


def _same_free_ball(field_ball, direction: float) -> bool:
    return _same_approach_ball(_free_approach_marker, field_ball, direction)


def _same_approach_ball(marker, field_ball, direction: float) -> bool:
    if marker is None:
        return False
    marker_x, marker_y, marker_direction = marker
    return marker_direction == direction and math.hypot(float(field_ball[0]) - marker_x, float(field_ball[1]) - marker_y) <= 10.0


def drop_capture_eligible(pose, local_ball, field_ball, direction: float, side: float) -> bool:
    """Permit the designated owner to lock only from its close angled entry."""
    if not helper._valid_ball(local_ball):
        return False
    local_distance = math.hypot(float(local_ball[0]), float(local_ball[1]))
    own_goal_side = direction * (float(field_ball[0]) - float(pose[0])) >= 2.0
    angled = side * (float(pose[1]) - float(field_ball[1])) >= 3.0
    return local_distance <= CAPTURE_DISTANCE_CM and own_goal_side and angled


def capture_eligible(pose, local_ball, field_ball, direction: float, enemies: list[list[float]]) -> bool:
    """Allow the designated owner to lock from close goal-side capture range."""
    if not helper._valid_ball(local_ball) or not helper._valid_ball(field_ball):
        return False
    local_distance = math.hypot(float(local_ball[0]), float(local_ball[1]))
    goal_side = direction * (float(field_ball[0]) - float(pose[0])) >= -3.0
    return local_distance <= CAPTURE_DISTANCE_CM and goal_side


def restricted_exit_target(pose) -> list[float]:
    """Return a conservative interior waypoint from either restricted corner."""
    return [
        clamp(float(pose[0]), -RESTRICTED_EXIT_X_CM, RESTRICTED_EXIT_X_CM),
        clamp(float(pose[1]), -RESTRICTED_EXIT_Y_CM, RESTRICTED_EXIT_Y_CM),
        float(pose[2]),
    ]


def defender_cover_target(field_ball, direction: float, side: float, contested: bool) -> list[float]:
    """Block from the goal side outside the goal box, never on the ball line."""
    progress = direction * float(field_ball[0])
    # The restricted goal box begins around 65 cm from centre; 60 cm is a
    # legal exterior guard line and leaves room for the recovery projection.
    target_progress = clamp(progress - 48.0, -60.0, 38.0)
    lateral = float(field_ball[1]) * 0.55 + side * (32.0 if contested else 22.0)
    return [direction * target_progress, clamp(lateral, -58.0, 58.0), _goal_yaw(direction)]


def _move(target) -> None:
    """Move toward a safe field target with bounded, non-oscillating velocity."""
    safe_target = helper._safe_target(target)
    pose = UnitedPosition.FieldRobotPose()
    if not valid_pose(pose):
        helper.stop()
        return

    error_x = float(safe_target[0]) - float(pose[0])
    error_y = float(safe_target[1]) - float(pose[1])
    speed_x = 0.0 if abs(error_x) <= MOVE_DEADBAND_CM else clamp(error_x * MOVE_KP_PER_CM, -MOVE_X_LIMIT, MOVE_X_LIMIT)
    speed_y = 0.0 if abs(error_y) <= MOVE_DEADBAND_CM else clamp(error_y * MOVE_KP_PER_CM, -MOVE_Y_LIMIT, MOVE_Y_LIMIT)
    chassis.FieldMoveVector(speed_x, speed_y, float(pose[2]), KpW=0.7)


def step(role: str = "attacker") -> str:
    """Perform one bounded tactical decision and return its trace state."""
    global _drop_approach_marker
    global _free_approach_marker
    global _held_since
    global _corner_release_pending

    pose = UnitedPosition.FieldRobotPose()
    if not valid_pose(pose):
        _held_since = None
        _corner_release_pending = False
        helper.stop()
        return "invalid_pose_stop"
    if not UnitedPosition.RestrictedArea():
        _held_since = None
        peripheral.Dribble(False)
        UnitedPosition.Pos2Pos(helper._safe_target(restricted_exit_target(pose)), 1.0)
        return "restricted_recover"

    direction = helper.attack_direction()
    side = lane_side()
    ball = UnitedPosition.RobotBallPos()
    ball_state = helper._ball_state(ball)
    field_ball = UnitedPosition.FieldBallPos() if ball_state == "free" else BALL_NOT_FOUND
    enemies = _enemies() if ball_state == "free" else []

    if ball_state != "held":
        _held_since = None
    elif _held_since is None:
        _held_since = time.monotonic()

    if ball_state == "held" and _corner_release_pending:
        _corner_release_pending = False
        peripheral.Dribble(False)
        peripheral.ShootBall()
        helper.stop()
        return "drop_ball_clearance"

    if ball_state == "held" and time.monotonic() - _held_since < POSSESSION_SETTLE_SECONDS:
        peripheral.Dribble(0.85)
        chassis.FieldMoveVector(0.0, 0.0, float(pose[2]), KpW=0.75)
        return "secure_possession"

    if ball_state == "free" and helper._valid_ball(field_ball) and is_safe_drop_zone(field_ball):
        if not drop_ball_owner(field_ball, direction):
            _drop_approach_marker = None
            _free_approach_marker = None
            peripheral.Dribble(False)
            if role == "defender":
                _move(defender_cover_target(field_ball, direction, side, contested=True))
                return "drop_ball_defend"
            _move(drop_yield_target(field_ball, direction, side))
            return "drop_ball_yield"

        shoulder = attacker_shoulder(field_ball, direction, side)
        if not _same_drop_ball(field_ball, direction):
            if math.hypot(float(pose[0]) - shoulder[0], float(pose[1]) - shoulder[1]) > DROP_SHOULDER_REACHED_CM:
                peripheral.Dribble(False)
                _move(shoulder)
                return "drop_ball_approach"
            _drop_approach_marker = (float(field_ball[0]), float(field_ball[1]), direction)

        if drop_capture_eligible(pose, ball, field_ball, direction, side):
            peripheral.Dribble(0.75)
            lockball.LockBallSlip()
            _corner_release_pending = True
            return "drop_ball_lock"

        peripheral.Dribble(False)
        _move(drop_entry_target(field_ball, direction, side))
        return "drop_ball_enter"

    _drop_approach_marker = None

    if role == "defender":
        _free_approach_marker = None
        if ball_state != "free":
            peripheral.Dribble(False)
            _move(search_patrol_target(direction, side, time.monotonic(), "defender"))
            return "search_lane"
        if ball_state == "free" and helper._valid_ball(field_ball):
            contested = _opponent_distance(field_ball, enemies) <= OPPONENT_CLEARANCE_CM
            peripheral.Dribble(False)
            _move(defender_cover_target(field_ball, direction, side, contested))
            return "defender_contested_block" if contested else "defender_goal_side_cover"
        peripheral.Dribble(False)
        _move([-direction * 58.0, side * 24.0, _goal_yaw(direction)])
        return "defender_home"

    if ball_state == "held":
        _free_approach_marker = None
        progress = direction * float(pose[0])
        if progress >= FINISH_PROGRESS_CM:
            if abs(float(pose[1])) <= GOAL_MOUTH_HALF_WIDTH_CM:
                goal_yaw = _goal_yaw(direction)
                if abs(_yaw_error(goal_yaw, float(pose[2]))) > PRE_SHOT_YAW_TOLERANCE_DEG:
                    peripheral.Dribble(0.85)
                    chassis.FieldMoveVector(0.0, 0.0, goal_yaw, KpW=0.75)
                    return "finish_align_shot"
                peripheral.Dribble(False)
                peripheral.ShootBall()
                helper.stop()
                return "finish_shot"

            retreat_speed = RECOVERY_OUTER_RETREAT_SPEED if progress >= OUTER_PROGRESS_CM else RECOVERY_RETREAT_SPEED
            inward_speed = clamp(-float(pose[1]) * 0.014, -RECOVERY_INWARD_LIMIT, RECOVERY_INWARD_LIMIT)
            peripheral.Dribble(0.85)
            chassis.FieldMoveVector(-direction * retreat_speed, inward_speed, float(pose[2]), KpW=0.75)
            return "finish_recover_lane"

        lane_error = side * 38.0 - float(pose[1])
        peripheral.Dribble(0.85)
        chassis.FieldMoveVector(
            direction * CARRY_FORWARD_LIMIT,
            clamp(lane_error * 0.012, -CARRY_LATERAL_LIMIT, CARRY_LATERAL_LIMIT),
            float(pose[2]),
            KpW=0.75,
        )
        return "carry_lane"
    if ball_state == "free" and helper._valid_ball(field_ball):
        if not free_ball_owner(field_ball, direction):
            _free_approach_marker = None
            peripheral.Dribble(False)
            _move(ordinary_yield_target(field_ball, direction, side))
            return "free_ball_yield"

        if capture_eligible(pose, ball, field_ball, direction, enemies):
            peripheral.Dribble(0.75)
            lockball.LockBallSlip()
            return "free_ball_lock"

        shoulder = attacker_shoulder(field_ball, direction, side)
        if not _same_free_ball(field_ball, direction):
            if math.hypot(float(pose[0]) - shoulder[0], float(pose[1]) - shoulder[1]) > DROP_SHOULDER_REACHED_CM:
                peripheral.Dribble(False)
                _move(shoulder)
                return "contested_shoulder" if _opponent_distance(field_ball, enemies) <= OPPONENT_CLEARANCE_CM else "approach_shoulder"
            _free_approach_marker = (float(field_ball[0]), float(field_ball[1]), direction)
            peripheral.Dribble(False)
            _move(drop_entry_target(field_ball, direction, side))
            return "free_ball_enter"

        if drop_capture_eligible(pose, ball, field_ball, direction, side):
            peripheral.Dribble(0.75)
            lockball.LockBallSlip()
            return "free_ball_lock"
        peripheral.Dribble(False)
        _move(drop_entry_target(field_ball, direction, side))
        return "free_ball_enter"

    _free_approach_marker = None
    peripheral.Dribble(False)
    _move(search_patrol_target(direction, side, time.monotonic(), role))
    return "search_lane"


def main() -> None:
    requested_role = "defender" if os.environ.get("LEAGUE_ROLE") == "defender" else "attacker"
    role = effective_role(requested_role)
    last_trace_state: str | None = None
    last_trace_time = -math.inf
    try:
        while True:
            state = step(role)
            if os.environ.get("COORDINATED_LANES_TRACE") == "1":
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
                            field_ball = None
                    try:
                        direction = helper.attack_direction()
                    except Exception:
                        direction = None
                    print(
                        json.dumps(
                            {
                                "robot_id": robot_id(),
                                "role": role,
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
