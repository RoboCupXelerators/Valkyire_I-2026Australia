"""Defence role tests."""

from __future__ import annotations

import os
import sys

os.environ.setdefault("ROBOCUPX_USE_MOCK", "1")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import defence  # noqa: E402
from defence import BALL_NOT_FOUND, DefenceConfig, RuleDefenceStrategy  # noqa: E402
from LunaPre import UnitedPosition  # noqa: E402


def test_defence_home_is_safe_for_both_sides():
    for direction in (1.0, -1.0):
        strategy = RuleDefenceStrategy(DefenceConfig(attack_direction=direction))

        assert UnitedPosition.ProjectSafePose(strategy.home) is None
        assert strategy.attack_dir == direction
        assert strategy.own_goal_x * direction < 0
        assert strategy.clear_x * direction > 0


def test_defence_intercept_target_is_between_own_goal_and_ball():
    strategy = RuleDefenceStrategy(DefenceConfig(attack_direction=1.0))

    target = strategy._intercept_target([-20.0, 30.0])

    assert target[0] < -20.0
    assert target[0] > strategy.own_goal_x
    assert UnitedPosition.ProjectSafePose(target) is None


def test_defence_hold_home_when_ball_not_visible(monkeypatch):
    captured = {}
    strategy = RuleDefenceStrategy(DefenceConfig(attack_direction=1.0))

    monkeypatch.setattr(defence.UnitedPosition, "FieldRobotPos", lambda: [0.0, 0.0, 0.0])
    monkeypatch.setattr(defence.UnitedPosition, "RobotBallPos", lambda: BALL_NOT_FOUND)
    monkeypatch.setattr(defence.peripheral, "Dribble", lambda _status: None)

    def fake_pos2pos(target, speed_gain=None):
        captured["target"] = target
        captured["speed_gain"] = speed_gain
        return False

    monkeypatch.setattr(defence.UnitedPosition, "Pos2Pos", fake_pos2pos)

    strategy.step()

    assert strategy.state == "hold_home"
    assert captured["target"] == strategy.home


def test_defence_challenges_danger_ball(monkeypatch):
    calls = []
    strategy = RuleDefenceStrategy(DefenceConfig(attack_direction=1.0))

    monkeypatch.setattr(defence.UnitedPosition, "FieldRobotPos", lambda: [-40.0, 0.0, 0.0])
    monkeypatch.setattr(defence.UnitedPosition, "RobotBallPos", lambda: [30.0, 0.0])
    monkeypatch.setattr(defence.UnitedPosition, "FieldBallPos", lambda: [-10.0, 0.0])
    monkeypatch.setattr(defence.lockball, "LockBallSlip", lambda: calls.append("lock"))

    strategy.step()

    assert strategy.state == "challenge_ball"
    assert calls == ["lock"]


def test_defence_clear_ball_has_shot_cooldown(monkeypatch):
    shots = []
    moves = []
    strategy = RuleDefenceStrategy(DefenceConfig(attack_direction=1.0, clear_cooldown_s=10.0))

    monkeypatch.setattr(defence.UnitedPosition, "Pos2Angle", lambda _start, _end: 0.0)
    monkeypatch.setattr(defence.chassis, "GetYaw", lambda: 0.0)
    monkeypatch.setattr(defence.chassis, "RobotMoveVector", lambda *args: moves.append(args))
    monkeypatch.setattr(defence.lockball, "LockBallSlip", lambda: None)
    monkeypatch.setattr(defence.peripheral, "Dribble", lambda _status: None)
    monkeypatch.setattr(defence.peripheral, "ShootBall", lambda: shots.append("shoot"))

    strategy._clear_ball([0.0, 0.0, 0.0])
    strategy._clear_ball([0.0, 0.0, 0.0])

    assert len(moves) == 2
    assert shots == ["shoot"]
