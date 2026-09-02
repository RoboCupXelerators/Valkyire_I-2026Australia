"""Robot ball position filtering tests."""

from __future__ import annotations

import os
import sys
from pathlib import Path

os.environ.setdefault("ROBOCUPX_BITS", "SIM")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from LunaPre import runtime  # noqa: E402
from LunaPre import positions as positions_module  # noqa: E402
from LunaPre.positions import Positions  # noqa: E402


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


def test_robot_ball_pos_preserves_vision_sentinel(monkeypatch):
    positions = Positions()

    class FakeVision:
        def get_ball_pos(self):
            return [0xDDD, 0xDDD]

    monkeypatch.setattr(runtime, "vision", FakeVision())

    assert positions.RobotBallPos() == [0xDDD, 0xDDD]
