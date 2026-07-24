"""Adapters from the deployed LunaPre state to the canonical policy contract."""

from __future__ import annotations

import time

from .contracts import EntityObservation, PolicyObservation, RefereeState, RoleId


def observation_from_lunapre(role: int = int(RoleId.ATTACKER)) -> PolicyObservation:
    from LunaPre import UnitedPosition, ensure_runtime
    from LunaPre.strategy_tools import BALL_HELD, BALL_NOT_FOUND

    ensure_runtime()
    pose = UnitedPosition.FieldRobotPose()
    ball = UnitedPosition.RobotBallPos()
    has_ball = ball == BALL_HELD
    ball_visible = ball != BALL_NOT_FOUND and not has_ball and isinstance(ball, (list, tuple)) and len(ball) >= 2
    enemies = []
    try:
        raw_enemies = UnitedPosition.RobotChassisPos() or []
        enemies = [EntityObservation(float(item[0]), float(item[1]), True, 0, 1.0) for item in raw_enemies[:4]]
    except Exception:
        pass
    walls = [0.0, 0.0, 0.0, 0.0]
    wall_valid = False
    try:
        raw_walls = UnitedPosition.DirectDistance()
        if isinstance(raw_walls, (list, tuple)) and len(raw_walls) >= 4:
            walls = [float(value) for value in raw_walls[:4]]
            wall_valid = True
    except Exception:
        pass
    velocity = [0.0, 0.0, 0.0, False]
    try:
        velocity = UnitedPosition.RobotVelocity()
    except Exception:
        pass
    control_enabled = False
    penalty_active = False
    penalty_reason = "UNKNOWN"
    penalty_remaining_s = 0.0
    referee_state = int(RefereeState.STOP)
    try:
        referee = UnitedPosition.RefereeState()
        control_enabled = bool(referee.control_enabled)
        penalty_active = bool(referee.penalty_active)
        penalty_reason = str(referee.penalty_reason)
        penalty_remaining_s = float(referee.penalty_remaining_s)
        phase_map = {
            "STOP": RefereeState.STOP,
            "PLAY": RefereeState.PLAY,
            "PENALIZED": RefereeState.PENALIZED,
            "HALF_TIME": RefereeState.HALF_TIME,
            "GAME_OVER": RefereeState.GAME_OVER,
        }
        referee_state = int(phase_map.get(str(referee.phase).upper(), RefereeState.STOP))
    except Exception:
        pass
    return PolicyObservation(
        self_x=float(pose[0]), self_y=float(pose[1]), self_yaw=float(pose[2]),
        self_vx=float(velocity[0]), self_vy=float(velocity[1]), self_wz=float(velocity[2]),
        velocity_valid=bool(velocity[3]),
        ball_rel_x=float(ball[0]) if ball_visible else 0.0,
        ball_rel_y=float(ball[1]) if ball_visible else 0.0,
        ball_visible=ball_visible, ball_age_ms=0 if ball_visible else -1,
        ball_confidence=1.0 if ball_visible else 0.0, has_ball=has_ball,
        enemies=enemies, wall_front=walls[0], wall_left=walls[1],
        wall_back=walls[2], wall_right=walls[3], wall_valid=wall_valid,
        referee_state=referee_state,
        control_enabled=control_enabled, penalty_active=penalty_active,
        penalty_reason=penalty_reason, penalty_remaining_s=penalty_remaining_s, role=role,
        observed_at_ms=int(time.monotonic() * 1000),
    )
