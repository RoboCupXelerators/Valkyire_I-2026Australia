"""Small offence strategy smoke tests."""

from __future__ import annotations

import os
import sys

import pytest

os.environ.setdefault("ROBOCUPX_BITS", "SIM")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import offence  # noqa: E402
from offence import BALL_HELD, BALL_NOT_FOUND  # noqa: E402


@pytest.fixture(autouse=True)
def _clean_state(monkeypatch):
    monkeypatch.setattr(offence, "_shot_release_until", 0.0)
    monkeypatch.setattr(offence, "_last_free_ball", None)
    monkeypatch.setattr(offence, "_last_free_ball_ts", 0.0)
    monkeypatch.setattr(offence, "_held_since", None)
    monkeypatch.setattr(offence, "_last_shot_ts", -999999.0)
    monkeypatch.setattr(offence, "HELD_DEBOUNCE_S", 0.0)
    monkeypatch.setattr(offence, "SHOT_COOLDOWN_S", 0.0)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPose", lambda: [9999, 9999, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldBallPos", lambda: BALL_NOT_FOUND)
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])


def test_lost_ball_searches_home(monkeypatch):
    calls = {}
    home = list(offence.Settings.CheckPoints.Home)

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_NOT_FOUND)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.setdefault("dribble", value))
    monkeypatch.setattr(
        offence.UnitedPosition,
        "Pos2Pos",
        lambda target, speed_gain=None: calls.update(target=target, speed_gain=speed_gain) or False,
    )

    assert offence.OffenceStep() == "search_ball"
    assert calls == {"dribble": 0.7, "target": [home[0] + 20, home[1], 0], "speed_gain": offence.SEARCH_SPEED_GAIN}


def test_free_ball_uses_lockball_slip(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [22, -8])
    monkeypatch.setattr(offence.lockball, "LockBallSlip", lambda: calls.append("lock") or False)

    assert offence.OffenceStep() == "lockballslip_chase"
    assert calls == ["lock"]


def test_close_ball_enters_quick_capture(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [3, 6])
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "RobotMoveVector", lambda *args: calls.append(("move", args)))

    assert offence.OffenceStep() == "quick_capture"
    assert calls[0] == ("dribble", 0.95)
    assert calls[1][0] == "move"


def test_held_ball_shoots_when_aligned(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPose", lambda: [40, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 0)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 0)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.peripheral, "ShootBall", lambda: calls.append("shoot"))
    monkeypatch.setattr(offence.chassis, "stop", lambda: calls.append("stop"))

    assert offence.OffenceStep() == "quick_shot"
    assert calls == [("dribble", 0.85), "shoot", ("dribble", False), "stop"]


def test_held_ball_carries_from_own_half(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPose", lambda: [-20, 12, 0])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 40)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 0)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "own_half_carry"
    assert calls == [("dribble", 0.9), ("move", (0.56, -0.06, 40), {"KpW": 0.8})]


def test_offence_stop_clears_outputs(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "stop", lambda: calls.append(("stop", None)))
    monkeypatch.setattr(offence.UnitedPosition, "Stop", lambda: calls.append(("united_stop", None)))

    offence.OffenceStop()

    assert calls == [("dribble", False), ("stop", None), ("united_stop", None)]
    assert offence.state == "stopped"
