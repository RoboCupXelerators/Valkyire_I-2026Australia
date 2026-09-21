"""Regression tests for Hyperion's strategy score helpers."""

import numpy as np

import Hyperion


def test_activate_func_returns_a_python_scalar_for_scalar_input():
    assert Hyperion.activate_func(-0.01) == 0.0
    assert Hyperion.activate_func(0.01) == 1.0
    assert isinstance(Hyperion.activate_func(0.01), float)


def test_activate_func_keeps_vector_inputs_as_arrays():
    result = Hyperion.activate_func(np.array([-0.01, 0.01]))

    assert isinstance(result, np.ndarray)
    assert result.tolist() == [0.0, 1.0]


def test_jump_score_can_round_its_scalar_activation(monkeypatch):
    monkeypatch.setattr(Hyperion.UnitedPosition, "Pos2Angle", lambda _start, _target: 0.0)

    score = Hyperion.JumpScore((0.0, 60.0, 0.0))

    assert score == 1.0
    assert isinstance(score, float)


def _u_shoot_harness(monkeypatch, *, turned=True):
    calls = []
    monkeypatch.setattr(Hyperion.lockball, "LockBallSlip", lambda: True)
    monkeypatch.setattr(Hyperion.UnitedPosition, "FieldRobotPose", lambda: (0.0, 0.0, 0.0))
    monkeypatch.setattr(Hyperion.UnitedPosition, "Pos2Angle", lambda _pose, _goal: 0.0)
    turn_targets = []
    monkeypatch.setattr(
        Hyperion, "TurnTo_with_ball", lambda yaw: turn_targets.append(yaw) or turned
    )
    monkeypatch.setattr(Hyperion.chassis, "stop", lambda: calls.append("stop"))
    monkeypatch.setattr(
        Hyperion.chassis, "RobotMoveVector", lambda *args: calls.append(("move", args))
    )
    monkeypatch.setattr(Hyperion.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(Hyperion.time, "sleep", lambda value: calls.append(("sleep", value)))
    return calls, turn_targets


def test_u_shoot_keeps_chasing_until_the_back_to_goal_turn_is_ready(monkeypatch):
    calls, turns = _u_shoot_harness(monkeypatch, turned=False)

    assert Hyperion.U_Shoot() is False
    assert calls == []
    assert turns == [180.0]


def test_u_shoot_runs_full_dribble_then_four_fast_shakes(monkeypatch):
    calls, turns = _u_shoot_harness(monkeypatch)

    assert Hyperion.U_Shoot() is True
    assert turns == [180.0]
    assert calls == [
        ("dribble", 1.0),
        ("sleep", 3.0),
        ("move", (0, 1.5, 0)),
        ("sleep", 0.20),
        ("move", (0, -1.5, 0)),
        ("sleep", 0.20),
        ("move", (0, 1.5, 0)),
        ("sleep", 0.20),
        ("move", (0, -1.5, 0)),
        ("sleep", 0.20),
        "stop",
        ("dribble", False),
    ]
