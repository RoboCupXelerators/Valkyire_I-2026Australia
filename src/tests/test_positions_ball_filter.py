"""Robot ball position filtering and possession-state tests."""

from __future__ import annotations

import os
import sys
from pathlib import Path

os.environ.setdefault("ROBOCUPX_BITS", "SIM")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from LunaPre import runtime  # noqa: E402
from LunaPre import positions as positions_module  # noqa: E402
from LunaPre.positions import Positions  # noqa: E402
from LunaPre.strategy_tools import BALL_HELD, BALL_NOT_FOUND  # noqa: E402


def test_vision_subprocess_uses_module_entrypoint(monkeypatch):
    captured = {}

    def fake_popen(command, **kwargs):
        captured["command"] = command
        captured["kwargs"] = kwargs
        return object()

    monkeypatch.setattr(positions_module.subprocess, "Popen", fake_popen)

    Positions._start_vision_subprocess()

    assert captured["command"] == [sys.executable, "-m", "vision_utils.Vision"]
    assert captured["kwargs"]["cwd"] == str(Path(positions_module.__file__).resolve().parents[1])


def test_robot_ball_pos_rejects_unphysical_vision_ball(monkeypatch):
    positions = Positions()
    positions.vision_data["ball_pos"] = [-5044.88, -4808.92]

    monkeypatch.setattr(positions, "StartVision", lambda: None)
    monkeypatch.setattr(runtime, "vision", None)

    assert positions.RobotBallPos() == [0xFFF, 0xFFF]


class _FakeVision:
    def __init__(self, ball_pos, has_ball=None):
        self._ball_pos = ball_pos
        self._has_ball = has_ball

    def get_ball_pos(self):
        return list(self._ball_pos)

    def has_ball(self):
        if self._has_ball is None:
            raise AttributeError("has_ball")
        return bool(self._has_ball)


def test_robot_ball_pos_returns_real_held_coordinate(monkeypatch):
    positions = Positions()
    monkeypatch.setattr(runtime, "vision", _FakeVision([8.5, 0.0], has_ball=True))

    assert positions.RobotBallPos() == [8.5, 0.0]
    assert positions.BallStatus() == "held"


def test_robot_ball_pos_never_synthesizes_held_sentinel(monkeypatch):
    positions = Positions()
    # Legacy provider that folds possession into the coordinate without giving
    # a real reading: the Position output is the no-ball sentinel, and the raw
    # sentinel remains resolvable as possession for input compatibility.
    monkeypatch.setattr(runtime, "vision", _FakeVision(BALL_HELD))

    assert positions.RobotBallPos() == BALL_NOT_FOUND
    assert positions.BallStatus() == "held"
    assert positions.BallStatus(BALL_HELD) == "held"


def test_ball_status_classifies_lost_free_and_held(monkeypatch):
    positions = Positions()
    monkeypatch.setattr(positions, "RobotBallPos", lambda: [0xFFF, 0xFFF])
    monkeypatch.setattr(runtime, "vision", None)
    assert positions.BallStatus() == "lost"

    monkeypatch.setattr(positions, "RobotBallPos", lambda: [40.0, 10.0])
    assert positions.BallStatus() == "free"

    monkeypatch.setattr(positions, "RobotBallPos", lambda: [8.5, 0.0])
    assert positions.BallStatus() == "held"


def test_ball_status_prefers_authoritative_provider(monkeypatch):
    positions = Positions()

    monkeypatch.setattr(runtime, "vision", _FakeVision([40.0, 10.0], has_ball=True))
    assert positions.BallStatus([40.0, 10.0]) == "held"

    monkeypatch.setattr(runtime, "vision", _FakeVision([8.5, 0.0], has_ball=False))
    assert positions.BallStatus([8.5, 0.0]) == "free"


def test_ball_state_alias_matches_ball_status(monkeypatch):
    positions = Positions()
    monkeypatch.setattr(positions, "RobotBallPos", lambda: [40.0, 10.0])
    monkeypatch.setattr(runtime, "vision", None)

    assert positions.BallState() == positions.BallStatus() == "free"
    assert positions.BallState(BALL_NOT_FOUND) == "lost"


def test_field_ball_pos_transforms_held_coordinate(monkeypatch):
    positions = Positions()
    monkeypatch.setattr(runtime, "vision", _FakeVision([8.5, 0.0], has_ball=True))
    monkeypatch.setattr(positions, "FieldRobotPose", lambda: [10.0, 20.0, 0.0])

    assert positions.FieldBallPos() == [18.5, 20.0]


def test_field_ball_pos_never_returns_held_sentinel(monkeypatch):
    positions = Positions()
    monkeypatch.setattr(runtime, "vision", _FakeVision(BALL_HELD))
    monkeypatch.setattr(positions, "FieldRobotPose", lambda: [10.0, 20.0, 0.0])

    assert positions.FieldBallPos() == BALL_NOT_FOUND
