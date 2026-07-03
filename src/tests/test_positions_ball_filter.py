"""Robot ball position filtering tests."""

from __future__ import annotations

import os
import sys

os.environ.setdefault("ROBOCUPX_BITS", "SIM")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from LunaPre import runtime  # noqa: E402
from LunaPre.positions import Positions  # noqa: E402


def test_robot_ball_pos_rejects_unphysical_vision_ball(monkeypatch):
    positions = Positions()
    positions.vision_data["ball_pos"] = [-5044.88, -4808.92]

    monkeypatch.setattr(positions, "StartVision", lambda: None)
    monkeypatch.setattr(runtime, "vision", None)
    monkeypatch.setattr(runtime, "get_runtime_ball_pos", lambda: None)
    monkeypatch.setattr(runtime, "runtime_ball_state_is_authoritative", lambda: False)

    assert positions.RobotBallPos() == [0xFFF, 0xFFF]


def test_robot_ball_pos_preserves_authoritative_sim_ball(monkeypatch):
    positions = Positions()
    sim_ball = [-5044.88, -4808.92]

    monkeypatch.setattr(runtime, "get_runtime_ball_pos", lambda: sim_ball)
    monkeypatch.setattr(runtime, "runtime_ball_state_is_authoritative", lambda: True)

    assert positions.RobotBallPos() == sim_ball
