"""Offence role branch tests."""

from __future__ import annotations

import os
import sys

os.environ.setdefault("ROBOCUPX_USE_MOCK", "1")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import offence  # noqa: E402
from offence import BALL_HELD, BALL_NOT_FOUND, OffenceConfig, RuleOffenceStrategy  # noqa: E402


def test_offence_searches_from_home_when_ball_not_visible(monkeypatch):
    calls = {}
    strategy = RuleOffenceStrategy(OffenceConfig(loop_dt=0.01))
    home = list(offence.Settings.CheckPoints.Home)

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_NOT_FOUND)
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 0)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.setdefault("dribble", value))

    def fake_pos2pos(target, speed_gain=None):
        calls["target"] = target
        calls["speed_gain"] = speed_gain
        return False

    monkeypatch.setattr(offence.UnitedPosition, "Pos2Pos", fake_pos2pos)

    strategy.step()

    assert strategy.state == "search_ball"
    assert calls["dribble"] == 0.7
    assert calls["target"] == [home[0] + 20, home[1], 0]
    assert calls["speed_gain"] == 4.5


def test_offence_locks_visible_ball(monkeypatch):
    calls = []
    strategy = RuleOffenceStrategy()

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [30, -8])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 0)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.lockball, "LockBallSlip", lambda: calls.append(("lock", None)))
    monkeypatch.setattr(offence.logger, "warning", lambda _value: calls.append(("warning", _value)))

    strategy.step()

    assert strategy.state == "lock_ball"
    assert calls == [("warning", 1), ("dribble", 0.7), ("lock", None)]


def test_offence_turns_to_goal_when_ball_is_held(monkeypatch):
    calls = []
    strategy = RuleOffenceStrategy(OffenceConfig(quick_shot_x_cm=10, ohmyback_y_cm=90))

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [30, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 725)
    monkeypatch.setattr(offence, "TurnTo_with_ball", lambda angle: calls.append(("turn", angle)) or False)

    strategy.step()

    assert strategy.state == "turn_to_goal"
    assert calls == [("turn", 5)]


def test_offence_shoots_after_turning_to_goal(monkeypatch):
    calls = []
    strategy = RuleOffenceStrategy(OffenceConfig(quick_shot_x_cm=10, ohmyback_y_cm=90))

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [30, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 90)
    monkeypatch.setattr(offence, "TurnTo_with_ball", lambda _angle: True)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 90)
    monkeypatch.setattr(offence.peripheral, "ShootBall", lambda: calls.append("shoot"))
    monkeypatch.setattr(offence.chassis, "stop", lambda: calls.append("stop"))

    strategy.step()

    assert strategy.state == "quick_shot"
    assert calls == ["shoot", "stop"]


def test_offence_uses_ohmyback_on_wide_ball(monkeypatch):
    calls = []
    strategy = RuleOffenceStrategy(OffenceConfig(ohmyback_y_cm=30))

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [20, 45, 0])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 0)
    monkeypatch.setattr(offence, "OhMyBackStep", lambda **kwargs: calls.append(kwargs) or False)

    strategy.step()

    assert strategy.state == "oh_my_back"
    assert calls and calls[0]["attack_dir"] == 1


def test_offence_can_use_slip_side(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [20, 35, 0])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 0)
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [[10, -10]])
    monkeypatch.setattr(offence, "Slipsideshot", lambda enemy: calls.append(enemy))

    state = offence._offence_tick(OffenceConfig(ohmyback_y_cm=90, slip_y_cm=20), trick_slot=0)

    assert state == "slip_side"
    assert calls == [[10, -10]]


def test_offence_feint_drive_when_no_trick_slot(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [20, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 15)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args: calls.append(("move", args)))

    state = offence._offence_tick(OffenceConfig(ohmyback_y_cm=90, quick_shot_x_cm=90), trick_slot=1)

    assert state == "feint_drive"
    assert calls == [("dribble", 0.8), ("move", (45, -0.0, 15))]


def test_offence_stop_clears_outputs(monkeypatch):
    calls = []
    offence._offence_strategy = RuleOffenceStrategy()

    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "stop", lambda: calls.append(("stop", None)))
    monkeypatch.setattr(offence.UnitedPosition, "Stop", lambda: calls.append(("united_stop", None)))

    offence.OffenceStop()

    assert calls == [
        ("dribble", False),
        ("stop", None),
        ("united_stop", None),
        ("stop", None),
    ]
    assert offence._offence_strategy is None
