"""Offence role branch tests."""

from __future__ import annotations

import os
import sys

os.environ.setdefault("ROBOCUPX_BITS", "SIM")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import offence  # noqa: E402
from offence import BALL_HELD, BALL_NOT_FOUND  # noqa: E402


def test_offence_searches_from_home_when_ball_not_visible(monkeypatch):
    calls = {}
    home = list(offence.Settings.CheckPoints.Home)

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_NOT_FOUND)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.setdefault("dribble", value))

    def fake_pos2pos(target, speed_gain=None):
        calls["target"] = target
        calls["speed_gain"] = speed_gain
        return False

    monkeypatch.setattr(offence.UnitedPosition, "Pos2Pos", fake_pos2pos)

    assert offence.OffenceStep() == "search_ball"
    assert calls["dribble"] == 0.7
    assert calls["target"] == [home[0] + 20, home[1], 0]
    assert calls["speed_gain"] == 4.5


def test_offence_locks_visible_ball(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [30, -8])
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.lockball, "LockBallSlip", lambda: calls.append(("lock", None)))
    monkeypatch.setattr(offence.logger, "warning", lambda value: calls.append(("warning", value)))

    assert offence.OffenceStep() == "lock_ball"
    assert calls == [("warning", 1), ("dribble", 0.7), ("lock", None)]


def test_offence_turns_to_goal_in_quick_shot_zone(monkeypatch):
    calls = []

    monkeypatch.setattr(offence, "QUICK_SHOT_X", 10)
    monkeypatch.setattr(offence, "OHMYBACK_Y", 90)
    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [30, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 725)
    monkeypatch.setattr(offence, "TurnTo_with_ball", lambda angle: calls.append(("turn", angle)) or False)

    assert offence.OffenceStep() == "turn_to_goal"
    assert calls == [("turn", 5)]


def test_offence_quick_shot(monkeypatch):
    calls = []

    monkeypatch.setattr(offence, "QUICK_SHOT_X", 10)
    monkeypatch.setattr(offence, "OHMYBACK_Y", 90)
    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [30, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 90)
    monkeypatch.setattr(offence, "TurnTo_with_ball", lambda _angle: True)
    monkeypatch.setattr(offence.peripheral, "ShootBall", lambda: calls.append("shoot"))
    monkeypatch.setattr(offence.chassis, "stop", lambda: calls.append("stop"))

    assert offence.OffenceStep() == "quick_shot"
    assert calls == ["shoot", "stop"]


def test_offence_uses_ohmyback_on_wide_ball(monkeypatch):
    calls = []

    monkeypatch.setattr(offence, "OHMYBACK_Y", 30)
    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [20, 45, 0])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 0)
    monkeypatch.setattr(offence, "OhMyBackStep", lambda **kwargs: calls.append(kwargs) or False)

    assert offence.OffenceStep() == "oh_my_back"
    assert calls and calls[0]["attack_dir"] == 1


def test_offence_can_use_slip_side(monkeypatch):
    calls = []

    monkeypatch.setattr(offence, "OHMYBACK_Y", 90)
    monkeypatch.setattr(offence, "SLIP_Y", 20)
    monkeypatch.setattr(offence, "started_at", 0)
    monkeypatch.setattr(offence, "TRICK_CYCLE", 10**9)
    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [20, 35, 0])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 0)
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [[10, -10]])
    monkeypatch.setattr(offence, "Slipsideshot", lambda enemy: calls.append(enemy))

    assert offence.OffenceStep() == "slip_side"
    assert calls == [[10, -10]]


def test_offence_feint_drive_when_no_trick_slot(monkeypatch):
    calls = []

    monkeypatch.setattr(offence, "OHMYBACK_Y", 90)
    monkeypatch.setattr(offence, "QUICK_SHOT_X", 90)
    monkeypatch.setattr(offence, "started_at", 0)
    monkeypatch.setattr(offence, "TRICK_CYCLE", 1)
    monkeypatch.setattr(offence.time, "time", lambda: 1.5)
    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [20, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 15)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args: calls.append(("move", args)))

    assert offence.OffenceStep() == "feint_drive"
    assert calls == [("dribble", 0.8), ("move", (45, -0.0, 15))]


def test_offence_stop_clears_outputs(monkeypatch):
    calls = []

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
    assert offence.state == "stopped"
