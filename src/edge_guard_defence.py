"""Simple edge and fast-break defence.

This strategy follows the lightweight shape of ``Hyperion.py``: one
``logic_step()`` tick reads LunaPre position data, chooses one clear action,
and sends chassis/peripheral commands directly.
"""

from __future__ import annotations

import time

from LunaPre import Settings, UnitedPosition, chassis, logger, peripheral
from LunaPre.strategy_tools import (
    BALL_HELD,
    BALL_NOT_FOUND,
    angle_error_deg,
    clamp,
    distance,
    robot_ball_distance,
    safe_stop,
    valid_pose,
)


LOOP_DT = 0.03

HOME_X_CM = 62.0
HOME_Y_CM = 0.0
SIDELINE_Y_CM = 55.0
FAST_BREAK_X_CM = 25.0
ENEMY_NEAR_BALL_CM = 62.0
CHASE_BALL_CM = 38.0

HOME_SPEED = 1.15
BLOCK_SPEED = 1.95
DRIBBLE_POWER = 0.65
SHOT_COOLDOWN_S = 0.7

_last_state = None
_last_log_ts = 0.0
_last_dribble = None
_last_shot_ts = -999.0


def _field_long() -> float:
    return float(getattr(Settings.Bounds, "Long", 260.0))


def _field_short() -> float:
    return float(getattr(Settings.Bounds, "Short", 200.0))


def _attack_dir() -> float:
    """Return +1 when attacking +X, -1 when attacking -X."""

    try:
        direction = float(getattr(Settings.Strategy, "AttackDirection", 0.0))
        if direction > 0.0:
            return 1.0
        if direction < 0.0:
            return -1.0
    except Exception:
        pass

    try:
        bit_mode = str(getattr(Settings.RoboInfo, "Bit", "")).upper()
        robot_id = int(
            getattr(
                getattr(Settings, "Sim", object()),
                "RobotId",
                getattr(Settings.RoboInfo, "number", 1),
            )
        )
        if bit_mode == "SIM" and robot_id >= 3:
            return -1.0
    except Exception:
        pass

    return 1.0


def _set_state(state: str) -> None:
    global _last_log_ts, _last_state

    now = time.time()
    if state != _last_state or now - _last_log_ts > 0.8:
        logger.info("[edge_guard_defence] state={}", state)
        _last_state = state
        _last_log_ts = now


def _set_dribble(value) -> None:
    global _last_dribble

    if value == _last_dribble:
        return
    peripheral.Dribble(value)
    _last_dribble = value


def _pose_to_ball_yaw(target, ball_field) -> float:
    return UnitedPosition.Pos2Angle(
        [float(target[0]), float(target[1])],
        [float(ball_field[0]), float(ball_field[1])],
    ) % 360.0


def _move_to(target, speed: float) -> None:
    safe = UnitedPosition.ProjectSafePose(target)
    UnitedPosition.Pos2Pos(safe if safe is not None else target, speed)


def _home_target(ball_field=None) -> list[float]:
    attack_dir = _attack_dir()
    x = -attack_dir * HOME_X_CM
    y = HOME_Y_CM
    yaw = 0.0 if attack_dir > 0.0 else 180.0
    if ball_field not in (None, BALL_NOT_FOUND, BALL_HELD):
        yaw = _pose_to_ball_yaw([x, y, 0.0], ball_field)
    return [x, y, yaw]


def _guard_home(ball_field=None) -> str:
    _set_state("hold_home")
    _set_dribble(False)
    _move_to(_home_target(ball_field), HOME_SPEED)
    return "hold_home"


def _sideline_target(ball_field) -> list[float]:
    attack_dir = _attack_dir()
    y_limit = min(_field_short() / 2.0 - 36.0, 58.0)
    side = 1.0 if float(ball_field[1]) >= 0.0 else -1.0
    x = -attack_dir * 60.0
    y = side * clamp(abs(float(ball_field[1])) * 0.72, 36.0, y_limit)
    yaw = _pose_to_ball_yaw([x, y, 0.0], ball_field)
    return [x, y, yaw]


def _fast_break_target(ball_field) -> list[float]:
    attack_dir = _attack_dir()
    ball_progress = attack_dir * float(ball_field[0])
    target_progress = clamp(ball_progress - 44.0, -86.0, -42.0)
    y_limit = min(_field_short() / 2.0 - 38.0, 50.0)
    x = attack_dir * target_progress
    y = clamp(float(ball_field[1]) * 0.48, -y_limit, y_limit)
    yaw = _pose_to_ball_yaw([x, y, 0.0], ball_field)
    return [x, y, yaw]


def _nearest_enemy_distance(ball_field):
    try:
        enemies = UnitedPosition.FieldChassisPos()
    except Exception:
        return None

    best = None
    for enemy in enemies or []:
        if not enemy or len(enemy) < 2:
            continue
        d = distance([float(enemy[0]), float(enemy[1])], ball_field)
        if best is None or d < best:
            best = d
    return best


def _is_sideline_shot(ball_field) -> bool:
    ball_progress = _attack_dir() * float(ball_field[0])
    wide_line = min(_field_short() / 2.0 - 42.0, SIDELINE_Y_CM)
    return abs(float(ball_field[1])) >= wide_line and ball_progress <= FAST_BREAK_X_CM


def _is_fast_break(ball_field) -> bool:
    ball_progress = _attack_dir() * float(ball_field[0])
    if ball_progress <= -FAST_BREAK_X_CM:
        return True

    enemy_distance = _nearest_enemy_distance(ball_field)
    if enemy_distance is None:
        return ball_progress <= 0.0
    return ball_progress <= FAST_BREAK_X_CM and enemy_distance <= ENEMY_NEAR_BALL_CM


def _cover_sideline(ball_field) -> str:
    _set_state("cover_sideline")
    _set_dribble(False)
    _move_to(_sideline_target(ball_field), BLOCK_SPEED)
    return "cover_sideline"


def _block_fast_break(ball_field) -> str:
    _set_state("block_fast_break")
    _set_dribble(False)
    _move_to(_fast_break_target(ball_field), BLOCK_SPEED)
    return "block_fast_break"


def _clear_ball(pose) -> str:
    global _last_shot_ts

    _set_state("clear_ball")
    _set_dribble(DRIBBLE_POWER)

    attack_dir = _attack_dir()
    goal_x = attack_dir * (_field_long() / 2.0 - 14.0)
    target_yaw = UnitedPosition.Pos2Angle(
        [float(pose[0]), float(pose[1])],
        [goal_x, 0.0],
    ) % 360.0
    yaw_error = angle_error_deg(chassis.GetYaw(), target_yaw)

    if abs(yaw_error) <= float(getattr(Settings.Motion, "ErrorRange", 10)):
        chassis.RobotMoveVector(0.42, 0.0, 0.0)
        if time.time() - _last_shot_ts >= SHOT_COOLDOWN_S:
            peripheral.ShootBall()
            _last_shot_ts = time.time()
    else:
        chassis.FieldMoveVector(0.0, 0.0, target_yaw, KpW=0.7)
    return "clear_ball"


def _challenge_ball(ball) -> str:
    _set_state("challenge_ball")
    _set_dribble(0.35)

    ball_dist = robot_ball_distance(ball)
    if ball_dist is None or ball_dist <= 1.0:
        chassis.stop()
        return "challenge_ball"

    speed = clamp(ball_dist * 0.012, 0.18, 0.45)
    speed_x = float(ball[0]) / ball_dist * speed
    speed_y = float(ball[1]) / ball_dist * speed
    chassis.RobotMoveVector(speed_x, speed_y, 0.0)
    return "challenge_ball"


def logic_step() -> str:
    pose = UnitedPosition.FieldRobotPose()
    if not valid_pose(pose):
        _set_state("no_pose")
        _set_dribble(False)
        chassis.stop()
        return "no_pose"

    safe = UnitedPosition.ProjectSafePose(pose)
    if safe is not None:
        _set_state("avoid_wall")
        _set_dribble(False)
        UnitedPosition.Pos2Pos(safe, HOME_SPEED)
        return "avoid_wall"

    ball = UnitedPosition.RobotBallPos()
    if ball == BALL_HELD:
        return _clear_ball(pose)
    if ball == BALL_NOT_FOUND:
        return _guard_home()

    ball_field = UnitedPosition.FieldBallPos()
    if ball_field in (BALL_NOT_FOUND, BALL_HELD) or not ball_field or len(ball_field) < 2:
        return _guard_home()

    if _is_sideline_shot(ball_field):
        return _cover_sideline(ball_field)

    if _is_fast_break(ball_field):
        return _block_fast_break(ball_field)

    ball_dist = robot_ball_distance(ball)
    if ball_dist is not None and ball_dist <= CHASE_BALL_CM:
        return _challenge_ball(ball)

    return _guard_home(ball_field)


def EdgeGuardDefenceStep() -> str:
    return logic_step()


def EdgeGuardDefenceStop() -> None:
    safe_stop(chassis, peripheral, UnitedPosition)


def main() -> None:
    try:
        while True:
            logic_step()
            time.sleep(LOOP_DT)
    finally:
        EdgeGuardDefenceStop()


if __name__ == "__main__":
    main()
