from __future__ import annotations

import time

from LunaPre import UnitedPosition, chassis


def test_angle_error_uses_shortest_wraparound():
    assert UnitedPosition._angle_error_deg(330, -30) == 0
    assert UnitedPosition._angle_error_deg(359, 1) == 2
    assert UnitedPosition._angle_error_deg(1, 359) == -2


def test_pos2pos_arrives_when_negative_target_yaw_matches_wrapped_pose(monkeypatch):
    monkeypatch.setattr(UnitedPosition, "FieldRobotPos", lambda: [10.0, 20.0, 330.0])
    monkeypatch.setattr(chassis, "stop", lambda: None)

    assert UnitedPosition.Pos2Pos([10.0, 20.0, -30.0], speed_gain=1.0) is True


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
