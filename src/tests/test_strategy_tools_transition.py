"""Scalar interpolation generator tests."""

from __future__ import annotations

import os
import sys

os.environ.setdefault("ROBOCUPX_BITS", "SIM")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from LunaPre.strategy_tools import transition  # noqa: E402


def test_transition_yields_current_immediately():
    generator = transition(0.0, 10.0, 0.2)

    assert next(generator) == 0.0


def test_transition_reaches_exact_aim_and_monotonic():
    values = list(transition(0.0, 10.0, 0.15))

    assert values[0] == 0.0
    assert values[-1] == 10.0
    assert all(0.0 <= value <= 10.0 for value in values)
    assert values == sorted(values)


def test_transition_descending_reaches_aim():
    values = list(transition(5.0, -5.0, 0.1))

    assert values[0] == 5.0
    assert values[-1] == -5.0
    assert all(-5.0 <= value <= 5.0 for value in values)


def test_transition_zero_duration_yields_aim_once():
    assert list(transition(3.0, 9.0, 0.0)) == [9.0]


def test_transition_negative_duration_yields_aim_once():
    assert list(transition(3.0, 9.0, -1.0)) == [9.0]
