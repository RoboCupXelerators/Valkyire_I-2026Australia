"""Offence boundary and restricted-area target tests."""

from __future__ import annotations

import os
import sys

os.environ.setdefault("ROBOCUPX_BITS", "SIM")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from LunaPre import UnitedPosition  # noqa: E402
import offence  # noqa: E402
from offence import OffenceConfig, RuleOffenceStrategy  # noqa: E402


def test_carry_target_avoids_goal_box_and_boundary():
    strategy = RuleOffenceStrategy()

    target = strategy._select_carry_target([55.0, 0.0, 0.0])

    assert target[0] <= 60.0
    assert abs(target[1]) >= 55.0
    assert UnitedPosition.ProjectSafePose([target[0], target[1], 0.0]) is None


def test_carry_target_path_does_not_cut_through_goal_box():
    strategy = RuleOffenceStrategy()
    start = [55.0, 0.0, 0.0]
    target = strategy._select_carry_target(start)

    for idx in range(1, 11):
        ratio = idx / 10.0
        pose = [
            start[0] + (target[0] - start[0]) * ratio,
            start[1] + (target[1] - start[1]) * ratio,
            0.0,
        ]
        assert UnitedPosition.ProjectSafePose(pose) is None


def test_carry_target_advances_after_side_lane_is_reached():
    strategy = RuleOffenceStrategy()

    target = strategy._select_carry_target([60.0, 56.0, 0.0])

    assert target[0] > 65.0
    assert abs(target[1]) >= 55.0
    assert UnitedPosition.ProjectSafePose([target[0], target[1], 0.0]) is None


def test_enemy_side_carry_target_attacks_negative_x():
    strategy = RuleOffenceStrategy(OffenceConfig(attack_direction=-1.0))

    target = strategy._select_carry_target([-55.0, 0.0, 180.0])

    assert target[0] >= -60.0
    assert abs(target[1]) >= 55.0
    assert UnitedPosition.ProjectSafePose([target[0], target[1], 180.0]) is None


def test_enemy_side_advances_after_side_lane_is_reached():
    strategy = RuleOffenceStrategy(OffenceConfig(attack_direction=-1.0))

    target = strategy._select_carry_target([-60.0, -56.0, 180.0])

    assert target[0] < -65.0
    assert abs(target[1]) >= 55.0
    assert UnitedPosition.ProjectSafePose([target[0], target[1], 180.0]) is None


def test_enemy_side_shoot_window_uses_negative_x():
    strategy = RuleOffenceStrategy(OffenceConfig(attack_direction=-1.0))

    assert strategy.goal_x < 0
    assert strategy._can_shoot([-58.0, 0.0, 180.0], [strategy.goal_x, 0.0])
    assert not strategy._can_shoot([58.0, 0.0, 0.0], [strategy.goal_x, 0.0])


def test_search_points_stay_outside_restricted_area():
    for direction in (1.0, -1.0):
        strategy = RuleOffenceStrategy(OffenceConfig(attack_direction=direction))
        for target in strategy.search_points:
            assert UnitedPosition.ProjectSafePose(target) is None


def test_held_ball_step_does_not_command_goal_box_cut(monkeypatch):
    captured = {}
    strategy = RuleOffenceStrategy()
    strategy._last_shot_ts = 10**12

    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [55.0, 0.0, 0.0])
    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: offence.BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "Pos2Angle", lambda _start, _end: 0.0)
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda _status: None)

    def fake_pos2pos(target, speed_gain=None):
        captured["target"] = target
        captured["speed_gain"] = speed_gain
        return False

    monkeypatch.setattr(offence.UnitedPosition, "Pos2Pos", fake_pos2pos)

    strategy.step()

    assert strategy.state == "carry_to_lane"
    assert captured["target"][0] <= 60.0
    assert abs(captured["target"][1]) >= 55.0


def test_offence_boundary_recovery_does_not_pick_arrival_edge():
    strategy = RuleOffenceStrategy()

    target = strategy._project_recovery_pose([0.0, 68.0, 0.0])

    assert target is not None
    assert target[1] < 60.0
    assert abs(68.0 - target[1]) > 8.0


def test_unsafe_pose_turns_dribbler_off_before_recovery(monkeypatch):
    captured = {}
    strategy = RuleOffenceStrategy()

    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [69.0, -19.0, 0.0])
    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: offence.BALL_HELD)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda status: captured.setdefault("dribble", status))

    def fake_pos2pos(target, speed_gain=None):
        captured["target"] = target
        captured["speed_gain"] = speed_gain
        return False

    monkeypatch.setattr(offence.UnitedPosition, "Pos2Pos", fake_pos2pos)

    strategy.step()

    assert strategy.state == "avoid_wall"
    assert captured["dribble"] is False
    assert captured["target"][0] < 60.0
