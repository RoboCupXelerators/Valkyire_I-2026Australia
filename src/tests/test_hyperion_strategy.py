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
