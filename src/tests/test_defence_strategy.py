"""Small defence strategy smoke tests."""

from __future__ import annotations

import os
import sys

os.environ.setdefault("ROBOCUPX_BITS", "SIM")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import defence  # noqa: E402
from defence import BALL_NOT_FOUND, DefenceConfig, RuleDefenceStrategy, choose_defence_cover  # noqa: E402
from LunaPre import UnitedPosition  # noqa: E402


def test_home_pose_is_legal_for_both_attack_directions():
    for direction in [1.0, -1.0]:
        strategy = RuleDefenceStrategy(DefenceConfig(attack_direction=direction))

        assert UnitedPosition.ProjectSafePose(strategy.home) is None
        assert strategy.own_goal_x * direction < 0


def test_cover_plan_keeps_three_basic_lanes():
    assert choose_defence_cover([20.0, 0.0], 1.0)[0] == "cover_direct_shot"
    assert choose_defence_cover([20.0, 30.0], 1.0)[0] == "cover_wall_shot"
    assert choose_defence_cover([20.0, 55.0], 1.0)[0] == "cover_sideline_shot"


def test_lost_ball_holds_home(monkeypatch):
    captured = {}
    strategy = RuleDefenceStrategy(DefenceConfig(attack_direction=1.0))

    monkeypatch.setattr(defence.UnitedPosition, "FieldRobotPose", lambda: [0.0, 0.0, 0.0])
    monkeypatch.setattr(defence.UnitedPosition, "RobotBallPos", lambda: BALL_NOT_FOUND)
    monkeypatch.setattr(defence.peripheral, "Dribble", lambda _status: None)
    monkeypatch.setattr(
        defence.UnitedPosition,
        "Pos2Pos",
        lambda target, speed_gain=None: captured.update(target=target, speed_gain=speed_gain) or False,
    )

    strategy.step()

    assert strategy.state == "hold_home"
    assert captured["target"] == strategy.home


def test_danger_ball_challenges_with_lockball(monkeypatch):
    calls = []
    strategy = RuleDefenceStrategy(DefenceConfig(attack_direction=1.0))

    monkeypatch.setattr(defence.UnitedPosition, "FieldRobotPose", lambda: [-40.0, 0.0, 0.0])
    monkeypatch.setattr(defence.UnitedPosition, "RobotBallPos", lambda: [30.0, 0.0])
    monkeypatch.setattr(defence.UnitedPosition, "FieldBallPos", lambda: [25.0, 0.0])
    monkeypatch.setattr(defence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(defence.lockball, "LockBallSlip", lambda: calls.append("lock"))

    strategy.step()

    assert strategy.state == "challenge_ball"
    assert calls == ["lock"]


def test_intercept_lane_uses_predicted_ball_point(monkeypatch):
    captured = {}
    strategy = RuleDefenceStrategy(DefenceConfig(attack_direction=1.0))

    monkeypatch.setattr(defence.UnitedPosition, "FieldRobotPose", lambda: [0.0, 0.0, 0.0])
    monkeypatch.setattr(defence.UnitedPosition, "RobotBallPos", lambda: [70.0, 0.0])
    monkeypatch.setattr(defence.UnitedPosition, "FieldBallPos", lambda: [-30.0, 0.0])
    monkeypatch.setattr(defence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(
        defence.UnitedPosition,
        "PredictBallLanding",
        lambda **_kwargs: {
            "valid": True,
            "field_pos": [-30.0, 50.0],
            "reason": "ok",
            "velocity_cm_s": [0.0, 100.0],
        },
    )
    monkeypatch.setattr(defence.peripheral, "Dribble", lambda _status: None)
    monkeypatch.setattr(
        defence.UnitedPosition,
        "Pos2Pos",
        lambda target, speed_gain=None: captured.update(target=target, speed_gain=speed_gain) or False,
    )

    strategy.step()

    assert strategy.state == "intercept_lane"
    assert captured["target"][1] > 30.0


def test_wall_recovery_has_priority(monkeypatch):
    captured = {}
    strategy = RuleDefenceStrategy(DefenceConfig(attack_direction=-1.0, wall_margin_cm=30.0))

    monkeypatch.setattr(defence.UnitedPosition, "FieldRobotPose", lambda: [-72.0, 76.0, 120.0])
    monkeypatch.setattr(defence.UnitedPosition, "RobotBallPos", lambda: [20.0, 6.0])
    monkeypatch.setattr(defence.peripheral, "Dribble", lambda _status: None)
    monkeypatch.setattr(
        defence.UnitedPosition,
        "Pos2Pos",
        lambda target, speed_gain=None: captured.update(target=target, speed_gain=speed_gain) or False,
    )

    strategy.step()

    assert strategy.state == "avoid_wall"
    assert captured["target"][1] < 76.0


def test_clear_ball_has_shot_cooldown(monkeypatch):
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
