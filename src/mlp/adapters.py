"""Adapters from the deployed LunaPre state to the canonical policy contract."""

from __future__ import annotations

import time

from .contracts import EntityObservation, PolicyObservation, RefereeState, RoleId


def deployable_referee_state(snapshot) -> tuple[int, bool]:
    """Reduce every referee provider to the official PLAY/STOP wire contract."""

    enabled = bool(getattr(snapshot, "control_enabled", False))
    phase = str(getattr(snapshot, "phase", "STOP")).strip().upper()
    is_play = enabled and phase == "PLAY"
    return int(RefereeState.PLAY if is_play else RefereeState.STOP), is_play


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
    referee_state = int(RefereeState.STOP)
    try:
        referee = UnitedPosition.RefereeState()
        referee_state, control_enabled = deployable_referee_state(referee)
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
        control_enabled=control_enabled,
        # These fields are intentionally fixed to deployment defaults. Detailed
        # simulator state is privileged and belongs to rewards and train masks.
        penalty_active=False, penalty_reason="UNKNOWN", penalty_remaining_s=0.0,
        score_diff=0, match_remaining_s=300.0, role=role,
        observed_at_ms=int(time.monotonic() * 1000),
    )
