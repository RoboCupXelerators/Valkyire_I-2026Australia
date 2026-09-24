"""Regression contracts for repaired legacy strategy 20260831."""

from __future__ import annotations

import importlib
import os
import sys

import pytest

os.environ.setdefault("ROBOCUPX_BITS", "SIM")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

legacy = importlib.import_module("20260831")


def test_back_kick_uses_wrap_safe_heading_error(monkeypatch):
    turn_targets, shots = [], []
    monkeypatch.setattr(legacy, "_read_state", lambda: ([100.0, 60.0, 180.0], "held", 1))
    monkeypatch.setattr(legacy.peripheral, "Dribble", lambda _speed: None)
    monkeypatch.setattr(legacy.peripheral, "ShootBall", lambda: shots.append(True))
    monkeypatch.setattr(legacy, "TurnTo_with_ball", lambda target: turn_targets.append(target))

    assert legacy.back_kick() is False
    assert len(turn_targets) == 1
    assert 0.0 <= turn_targets[0] < 360.0
    assert shots == []


def test_macon_kick_uses_interior_sideline_target(monkeypatch):
    targets = []
    monkeypatch.setattr(legacy, "_read_state", lambda: ([10.0, 20.0, 0.0], "held", 1))
    monkeypatch.setattr(legacy.peripheral, "Dribble", lambda _speed: None)
    monkeypatch.setattr(legacy, "TurnTo_with_ball", lambda _target: None)
    monkeypatch.setattr(legacy.UnitedPosition, "PointInShape", lambda *_args: False)
    monkeypatch.setattr(legacy.UnitedPosition, "Pos2Pos", lambda target: targets.append(target))

    assert legacy.Macon_kick() is False
    assert targets == [[56.0, legacy.SIDELINE_Y_CM, 90.0]]
    assert legacy.SIDELINE_Y_CM < 70.0


def test_open_goal_uses_one_window_snapshot(monkeypatch):
    reads, turns, shots = [], [], []
    monkeypatch.setattr(legacy.UnitedPosition, "FieldRobotPose", lambda: [20.0, 0.0, 5.0])
    monkeypatch.setattr(
        legacy.UnitedPosition,
        "ShootWindow",
        lambda **kwargs: reads.append(kwargs) or {
            "goal_key": "blue_goal", "window_open": True,
            "can_shoot_now": False, "target_yaw_deg": 0.0,
        },
    )
    monkeypatch.setattr(legacy, "TurnTo_with_ball", lambda target: turns.append(target))
    monkeypatch.setattr(legacy.peripheral, "ShootBall", lambda: shots.append(True))

    assert legacy.OpenGoalShoot() is False
    assert len(reads) == 1
    assert turns == [0.0]
    assert shots == []


def test_step_keeps_restricted_area_recovery_command(monkeypatch):
    dribbles = []
    monkeypatch.setattr(legacy.UnitedPosition, "RestrictedArea", lambda: False)
    monkeypatch.setattr(legacy.peripheral, "Dribble", lambda value: dribbles.append(value))
    monkeypatch.setattr(legacy.UnitedPosition, "FieldRobotPose", lambda: pytest.fail("recovery read pose"))

    assert legacy.step() == "restricted_recover"
    assert dribbles == [False]
