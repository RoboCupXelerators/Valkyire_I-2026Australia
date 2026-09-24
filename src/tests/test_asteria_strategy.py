"""Focused contracts for the argument-free Asteria simulator strategy."""

from __future__ import annotations

import os
import sys
from types import SimpleNamespace

import pytest

os.environ.setdefault("ROBOCUPX_BITS", "SIM")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import asteria  # noqa: E402


@pytest.fixture(autouse=True)
def isolate_controller(monkeypatch):
    """One fresh controller makes score/cooldown tests independent."""
    monkeypatch.setattr(asteria, "controller", asteria.Asteria())
    monkeypatch.setattr(asteria, "time", SimpleNamespace(**vars(asteria.time)))


def _runtime(monkeypatch, *, pose=(0.0, 0.0, 0.0), ball_state="lost", robot_ball=None):
    moves, dribbles, shots, stops = [], [], [], []
    if robot_ball is None:
        robot_ball = [0xFFF, 0xFFF]
    monkeypatch.setattr(asteria.UnitedPosition, "FieldRobotPose", lambda: list(pose))
    monkeypatch.setattr(asteria.UnitedPosition, "RestrictedArea", lambda: True)
    monkeypatch.setattr(asteria.UnitedPosition, "RobotBallPos", lambda: robot_ball)
    monkeypatch.setattr(asteria.UnitedPosition, "BallState", lambda _ball: ball_state)
    monkeypatch.setattr(asteria.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(asteria.Bits, "GetEnemyPos", lambda: [])
    monkeypatch.setattr(asteria.chassis, "FieldMoveVector", lambda *args, **kwargs: moves.append((args, kwargs)))
    monkeypatch.setattr(asteria.chassis, "stop", lambda: stops.append(True))
    monkeypatch.setattr(asteria.peripheral, "Dribble", lambda value: dribbles.append(value))
    monkeypatch.setattr(asteria.peripheral, "ShootBall", lambda level=None: shots.append(level))
    monkeypatch.setattr(asteria.lockball, "LockBallSlip", lambda: False)
    monkeypatch.setattr(asteria.helper, "_safe_target", lambda target: target)
    monkeypatch.setattr(asteria.helper, "attack_direction", lambda: 1.0)
    return moves, dribbles, shots, stops


def test_lost_ball_search_does_not_read_field_ball(monkeypatch):
    moves, dribbles, shots, _stops = _runtime(monkeypatch)
    monkeypatch.setattr(asteria.UnitedPosition, "FieldBallPos", lambda: pytest.fail("lost ball read FieldBallPos"))

    assert asteria.step() == "search_ball"
    assert moves[-1][0][2] == 0.0
    assert dribbles[-1] is False
    assert shots == []


def test_sim_lost_ball_fallback_reclaims_trusted_ball(monkeypatch):
    monkeypatch.setenv("XEL_SIM_BALL_STATUS_FALLBACK", "1")
    _moves, _dribbles, _shots, _stops = _runtime(monkeypatch, pose=(0.0, 0.0, 0.0))
    monkeypatch.setattr(asteria.Bits, "GetBallPos", lambda: (18.0, 0.0))
    locked = []
    monkeypatch.setattr(asteria.lockball, "LockBallSlip", lambda: locked.append(True))

    assert asteria.step() == "lock_ball"
    assert locked == [True]


def test_free_ball_uses_bounded_approach_then_lockball(monkeypatch):
    moves, dribbles, _shots, _stops = _runtime(
        monkeypatch, ball_state="free", robot_ball=(36.0, 0.0)
    )
    monkeypatch.setattr(asteria.UnitedPosition, "FieldBallPos", lambda: (30.0, 8.0))

    assert asteria.step() == "approach_ball"
    assert dribbles[-1] == 0.82
    assert moves[-1][0][0] > 0.0

    locked = []
    monkeypatch.setattr(asteria.UnitedPosition, "RobotBallPos", lambda: (10.0, 0.0))
    monkeypatch.setattr(asteria.lockball, "LockBallSlip", lambda: locked.append(True))
    assert asteria.step() == "lock_ball"
    assert locked == [True]


def test_held_ball_shoots_only_in_central_finish_corridor(monkeypatch):
    moves, dribbles, shots, stops = _runtime(
        monkeypatch, pose=(45.0, 0.0, 0.0), ball_state="held", robot_ball=[0xDDD, 0xDDD]
    )
    now = [10.0]
    monkeypatch.setattr(asteria.time, "monotonic", lambda: now[0])
    monkeypatch.setattr(asteria.UnitedPosition, "FieldBallPos", lambda: pytest.fail("held ball read FieldBallPos"))

    assert asteria.step() == "shoot"
    assert shots == [asteria.Asteria.KICK_POWER_LEVEL]
    assert stops == [True]
    assert dribbles[-1] == 0.90

    now[0] += 0.1
    assert asteria.step() == "kick_hold"
    assert shots == [asteria.Asteria.KICK_POWER_LEVEL]
    assert moves == []

    now[0] += 0.1
    assert asteria.step() == "kick_release"
    assert dribbles[-1] is False


def test_finish_alignment_keeps_ball_moving_instead_of_turning_in_place(monkeypatch):
    moves, _dribbles, shots, _stops = _runtime(
        monkeypatch, pose=(48.0, 14.0, 46.0), ball_state="held", robot_ball=[0xDDD, 0xDDD]
    )

    assert asteria.step() == "aim_goal_carry"
    assert moves[-1][0][0] > 0.0
    assert shots == []


def test_held_ball_recentres_before_crossing_the_sideline_guard(monkeypatch):
    moves, _dribbles, shots, _stops = _runtime(
        monkeypatch, pose=(20.0, 52.0, 0.0), ball_state="held", robot_ball=[0xDDD, 0xDDD]
    )
    monkeypatch.setattr(asteria.UnitedPosition, "FieldChassisPos", lambda: [[40.0, 52.0, 0.0, 0.0]])

    assert asteria.step() == "recenter_ball"
    assert moves[-1][0][0] < 0.0
    assert moves[-1][0][1] < 0.0
    assert shots == []


def test_shoulder_lane_carries_forward_until_recentering_has_a_real_advantage(monkeypatch):
    moves, _dribbles, _shots, _stops = _runtime(
        monkeypatch, pose=(0.0, 24.0, 0.0), ball_state="held", robot_ball=[0xDDD, 0xDDD]
    )

    assert asteria.step() == "carry_centre"
    assert moves[-1][0][0] > 0.0


def test_restricted_guard_preserves_existing_recovery_command(monkeypatch):
    _moves, dribbles, shots, stops = _runtime(monkeypatch)
    monkeypatch.setattr(asteria.UnitedPosition, "RestrictedArea", lambda: False)
    monkeypatch.setattr(asteria.UnitedPosition, "RobotBallPos", lambda: pytest.fail("recovery read ball"))

    assert asteria.step() == "restricted_recover"
    assert dribbles == [False]
    assert shots == []
    assert stops == []


def test_2v2_free_ball_has_exactly_one_owner_per_team(monkeypatch):
    field_ball = (18.0, 0.0)
    monkeypatch.setenv("XEL_ROBOT_ID", "1")
    assert asteria.Asteria()._free_ball_owner(field_ball, 1.0) is True
    monkeypatch.setenv("XEL_ROBOT_ID", "2")
    assert asteria.Asteria()._free_ball_owner(field_ball, 1.0) is False

    field_ball = (-18.0, 0.0)
    monkeypatch.setenv("XEL_ROBOT_ID", "1")
    assert asteria.Asteria()._free_ball_owner(field_ball, 1.0) is False
    monkeypatch.setenv("XEL_ROBOT_ID", "2")
    assert asteria.Asteria()._free_ball_owner(field_ball, 1.0) is True


def test_centre_ball_handoff_changes_owner_without_creating_two_chasers(monkeypatch):
    monkeypatch.setattr(asteria.time, "monotonic", lambda: 0.0)
    monkeypatch.setenv("XEL_ROBOT_ID", "1")
    attacker_turn = asteria.Asteria()._free_ball_owner((0.0, 0.0), 1.0)
    monkeypatch.setenv("XEL_ROBOT_ID", "2")
    defender_turn = asteria.Asteria()._free_ball_owner((0.0, 0.0), 1.0)
    assert (attacker_turn, defender_turn) == (True, False)

    monkeypatch.setattr(asteria.time, "monotonic", lambda: 4.0)
    monkeypatch.setenv("XEL_ROBOT_ID", "1")
    attacker_turn = asteria.Asteria()._free_ball_owner((0.0, 0.0), 1.0)
    monkeypatch.setenv("XEL_ROBOT_ID", "2")
    defender_turn = asteria.Asteria()._free_ball_owner((0.0, 0.0), 1.0)
    assert (attacker_turn, defender_turn) == (False, True)


def test_non_owner_supports_instead_of_locking_the_same_free_ball(monkeypatch):
    monkeypatch.setenv("XEL_ROBOT_ID", "2")
    moves, dribbles, _shots, _stops = _runtime(
        monkeypatch, ball_state="free", robot_ball=(30.0, 0.0)
    )
    monkeypatch.setattr(asteria.UnitedPosition, "FieldBallPos", lambda: (30.0, 0.0))
    monkeypatch.setattr(asteria.lockball, "LockBallSlip", lambda: pytest.fail("non-owner must not lock"))

    assert asteria.step() == "free_ball_support"
    assert dribbles[-1] is False
    assert moves[-1][0][0] > 0.0


def test_non_owner_in_dribbler_window_takes_over_instead_of_yielding(monkeypatch):
    monkeypatch.setenv("XEL_ROBOT_ID", "2")
    _moves, _dribbles, _shots, _stops = _runtime(
        monkeypatch, ball_state="free", robot_ball=(22.0, 0.0)
    )
    monkeypatch.setattr(asteria.UnitedPosition, "FieldBallPos", lambda: pytest.fail("near ball must not wait for field vision"))
    locked = []
    monkeypatch.setattr(asteria.lockball, "LockBallSlip", lambda: locked.append(True))

    assert asteria.step() == "lock_ball"
    assert locked == [True]


def test_chassis_positions_switch_lost_ball_to_one_goal_guard(monkeypatch):
    monkeypatch.setenv("XEL_ROBOT_ID", "2")
    moves, _dribbles, _shots, _stops = _runtime(monkeypatch)
    monkeypatch.setattr(asteria.UnitedPosition, "FieldChassisPos", lambda: [[-52.0, 14.0, 0.0, 0.0]])

    assert asteria.step() == "lost_goal_guard"
    assert asteria.controller.last_role == "goal_guard"
    assert moves[-1][0][0] < 0.0


def test_medium_depth_chassis_threat_assigns_non_guard_to_press(monkeypatch):
    monkeypatch.setenv("XEL_ROBOT_ID", "2")
    moves, dribbles, _shots, _stops = _runtime(monkeypatch)
    monkeypatch.setattr(asteria.UnitedPosition, "FieldChassisPos", lambda: [[-25.0, 8.0, 0.0, 0.0]])

    assert asteria.step() == "lost_chassis_press"
    assert asteria.controller.last_role == "chassis_press"
    assert dribbles[-1] == 0.75
    assert moves[-1][0][0] < 0.0


def test_sim_chassis_fallback_converts_relative_bits_positions(monkeypatch):
    _moves, _dribbles, _shots, _stops = _runtime(monkeypatch, pose=(20.0, 5.0, 90.0))
    monkeypatch.setattr(asteria.Bits, "GetEnemyPos", lambda: [[10.0, 0.0]])

    assert asteria.controller._opponents((20.0, 5.0, 90.0)) == [[20.0, 15.0]]


def test_chassis_blocker_makes_carrier_dodge_instead_of_ramming(monkeypatch):
    moves, _dribbles, _shots, _stops = _runtime(
        monkeypatch, pose=(20.0, 0.0, 0.0), ball_state="held", robot_ball=[0xDDD, 0xDDD]
    )
    monkeypatch.setattr(asteria.UnitedPosition, "FieldChassisPos", lambda: [[40.0, 0.0, 0.0, 0.0]])

    assert asteria.step() == "carry_avoid_chassis"
    assert moves[-1][0][0] > 0.0
    assert moves[-1][0][1] < 0.0
