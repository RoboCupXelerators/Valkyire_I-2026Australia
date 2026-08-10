"""Focused contracts for the wide-sweep tournament candidate."""

from __future__ import annotations

import os
import sys
import json

import pytest

os.environ.setdefault("ROBOCUPX_BITS", "SIM")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import candidate_wide_sweep as candidate  # noqa: E402


def _patch_common(monkeypatch, *, pose=(0.0, 0.0, 0.0), ball_state="lost", local_ball=None):
    moves, dribbles, shots = [], [], []
    local_ball = candidate.BALL_NOT_FOUND if local_ball is None else local_ball
    monkeypatch.setattr(candidate.UnitedPosition, "FieldRobotPose", lambda: list(pose))
    monkeypatch.setattr(candidate.UnitedPosition, "RestrictedArea", lambda: True)
    monkeypatch.setattr(candidate.UnitedPosition, "RobotBallPos", lambda: local_ball)
    monkeypatch.setattr(candidate.UnitedPosition, "BallState", lambda _ball: ball_state)
    monkeypatch.setattr(candidate.UnitedPosition, "ProjectSafePose", lambda target: target)
    monkeypatch.setattr(candidate.chassis, "FieldMoveVector", lambda *args, **_kwargs: moves.append(args))
    monkeypatch.setattr(candidate.peripheral, "Dribble", lambda value: dribbles.append(value))
    monkeypatch.setattr(candidate.peripheral, "ShootBall", lambda: shots.append(True))
    monkeypatch.setattr(candidate.helper, "stop", lambda: None)
    return moves, dribbles, shots


def test_lost_patrol_changes_over_time_and_teammates_do_not_share_target(monkeypatch):
    moves, _dribbles, _shots = _patch_common(monkeypatch)
    now = [0.0]
    monkeypatch.setattr(candidate.time, "monotonic", lambda: now[0])
    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", "1")
    monkeypatch.setenv("XEL_ROBOT_ID", "1")

    assert candidate.step() == "wide_patrol"
    first = moves[-1]
    now[0] = candidate.SWEEP_STAGE_SECONDS
    assert candidate.step() == "wide_patrol"
    assert moves[-1] != first
    assert candidate.patrol_target(1.0, 0, 0.0) != candidate.patrol_target(1.0, 1, 0.0)


def test_lost_ball_never_reads_field_ball(monkeypatch):
    _patch_common(monkeypatch)
    monkeypatch.setattr(candidate.UnitedPosition, "FieldBallPos", lambda: pytest.fail("lost read FieldBallPos"))

    assert candidate.step() == "wide_patrol"


def test_attack_direction_mirrors_patrol_and_held_carry(monkeypatch):
    normal = candidate.patrol_target(1.0, 0, 0.0)
    mirrored = candidate.patrol_target(-1.0, 0, 0.0)
    assert mirrored == [-normal[0], normal[1], 180.0]

    moves, _dribbles, _shots = _patch_common(monkeypatch, ball_state="held", local_ball=candidate.BALL_HELD)
    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", "1")
    assert candidate.step() == "held_carry"
    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", "-1")
    assert candidate.step() == "held_carry"
    assert moves[-1] == (-moves[-2][0], moves[-2][1], 180.0)


def test_free_ball_approaches_and_locks_while_held_carries_or_recovers(monkeypatch):
    moves, dribbles, _shots = _patch_common(monkeypatch, ball_state="free", local_ball=(34.0, 0.0))
    monkeypatch.setenv("XEL_ROBOT_ID", "1")
    monkeypatch.setattr(candidate.UnitedPosition, "FieldBallPos", lambda: (30.0, 12.0))
    assert candidate.step() == "free_approach"
    assert dribbles[-1] == 0.82
    assert moves[-1][0] > 0.0

    locked = []
    monkeypatch.setattr(candidate.UnitedPosition, "RobotBallPos", lambda: (10.0, 0.0))
    monkeypatch.setattr(candidate.lockball, "LockBallSlip", lambda: locked.append(True))
    assert candidate.step() == "free_lock"
    assert locked == [True]

    moves, _dribbles, _shots = _patch_common(
        monkeypatch, pose=(10.0, 52.0, 0.0), ball_state="held", local_ball=candidate.BALL_HELD
    )
    monkeypatch.setattr(candidate.UnitedPosition, "FieldBallPos", lambda: pytest.fail("held read FieldBallPos"))
    assert candidate.step() == "held_recover"
    assert moves[-1][0] < 0.0 and moves[-1][1] < 0.0


def test_held_trace_never_reads_field_ball(monkeypatch, capsys):
    monkeypatch.setenv("WIDE_SWEEP_TRACE", "1")
    monkeypatch.setenv("XEL_ROBOT_ID", "2")
    monkeypatch.setattr(candidate, "step", lambda: "held_carry")
    monkeypatch.setattr(candidate.time, "monotonic", lambda: 10.0)
    monkeypatch.setattr(candidate.time, "sleep", lambda _delay: (_ for _ in ()).throw(KeyboardInterrupt))
    monkeypatch.setattr(candidate.UnitedPosition, "FieldRobotPose", lambda: [1.0, 2.0, 90.0])
    monkeypatch.setattr(candidate.UnitedPosition, "RobotBallPos", lambda: candidate.BALL_HELD)
    monkeypatch.setattr(candidate.UnitedPosition, "BallState", lambda _ball: "held")
    monkeypatch.setattr(candidate.UnitedPosition, "FieldBallPos", lambda: pytest.fail("held trace read FieldBallPos"))
    monkeypatch.setattr(candidate.helper, "attack_direction", lambda: -1.0)
    monkeypatch.setattr(candidate.helper, "stop", lambda: None)

    with pytest.raises(KeyboardInterrupt):
        candidate.main()

    assert json.loads(capsys.readouterr().out) == {
        "robot_id": 2,
        "state": "held_carry",
        "field_robot_pose": [1.0, 2.0, 90.0],
        "robot_ball_pos": candidate.BALL_HELD,
        "field_ball_pos": candidate.BALL_NOT_FOUND,
        "attack_direction": -1.0,
        "monotonic_timestamp": 10.0,
    }
