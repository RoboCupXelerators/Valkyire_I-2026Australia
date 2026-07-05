from __future__ import annotations

import time

from LunaPre import UnitedPosition, chassis
from LunaPre.strategy_tools import angle_error_deg


def test_angle_error_uses_shortest_wraparound():
    assert angle_error_deg(330, -30) == 0
    assert angle_error_deg(359, 1) == 2
    assert angle_error_deg(1, 359) == -2


def test_pos2pos_arrives_when_negative_target_yaw_matches_wrapped_pose(monkeypatch):
    monkeypatch.setattr(UnitedPosition, "FieldRobotPose", lambda: [10.0, 20.0, 330.0])
    monkeypatch.setattr(chassis, "stop", lambda: None)

    assert UnitedPosition.Pos2Pos([10.0, 20.0, -30.0], speed_gain=1.0) is True


def test_navigate2_uses_target_when_path_is_clear(monkeypatch):
    targets_seen = []

    monkeypatch.setattr(UnitedPosition, "FieldRobotPose", lambda: [0.0, 0.0, 0.0])
    monkeypatch.setattr(UnitedPosition, "Pos2Pos", lambda target, speed_gain=None: targets_seen.append(target) or "ok")

    assert UnitedPosition.Navigate2([20.0, 20.0, 90.0], speed_gain=1.0) == "ok"
    assert targets_seen == [[20.0, 20.0, 90.0]]


def test_navigate2_inserts_waypoint_when_straight_path_crosses_goal_box(monkeypatch):
    targets_seen = []

    monkeypatch.setattr(UnitedPosition, "FieldRobotPose", lambda: [80.0, 60.0, 0.0])
    monkeypatch.setattr(UnitedPosition, "Pos2Pos", lambda target, speed_gain=None: targets_seen.append(target) or False)

    assert UnitedPosition.Navigate2([80.0, -60.0, 0.0], speed_gain=1.0) is False
    assert targets_seen
    assert targets_seen[0] != [80.0, -60.0, 0.0]
    assert abs(targets_seen[0][1]) > 50.0
    assert UnitedPosition.ProjectSafePose(targets_seen[0]) is None


def test_move2path_advances_to_next_point(monkeypatch):
    targets_seen = []
    calls_by_target = {}

    def fake_pos2pos(target):
        key = tuple(target)
        calls_by_target[key] = calls_by_target.get(key, 0) + 1
        targets_seen.append(key)
        return calls_by_target[key] >= 2

    monkeypatch.setattr(UnitedPosition, "Pos2Pos", fake_pos2pos)
    monkeypatch.setattr(time, "sleep", lambda _: None)

    path = [[1, 0, 0], [2, 0, -30], [3, 0, 330]]
    assert UnitedPosition.Move2Path(path, 0, timeout_s=1.0, tick_s=0.01) is True
    assert targets_seen.count((1, 0, 0)) == 2
    assert targets_seen.count((2, 0, -30)) == 2
    assert targets_seen.count((3, 0, 330)) == 2
