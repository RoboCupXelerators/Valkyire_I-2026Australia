"""Coordinate-space restricted-area guard tests."""

from __future__ import annotations

import os
import sys

import pytest

os.environ.setdefault("ROBOCUPX_BITS", "SIM")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from LunaPre import UnitedPosition, runtime
from LunaPre.strategy_tools import POSE_NOT_FOUND
from ReasonData import Settings


@pytest.fixture(autouse=True)
def _standard_restricted_area(monkeypatch):
    restricted = Settings.Motion.RestrictedArea
    for name, value in {
        "XLimit": 100.0,
        "YLimit": 70.0,
        "GoalBoxWidth": 25.0,
        "GoalBoxYHalf": 40.0,
        "GoalBoxCornerRadius": 20.0,
        "ProtectionLevel": 1,
        "Clearance": 5.0,
        "RecoveryClearance": 16.0,
        "RecoveryExtra": 3.0,
    }.items():
        monkeypatch.setattr(restricted, name, value, raising=False)


def test_safe_pose_needs_no_projection():
    assert UnitedPosition.ProjectSafePose([0, 0, 0]) is None
    assert not UnitedPosition.IsPoseInRestrictedArea([0, 0, 0])


def test_point_in_shape_accepts_rectangle_diagonal_points():
    rect = [[70, 30], [120, 90]]

    assert UnitedPosition.PointInShape([80, 40], rect)
    assert UnitedPosition.PointInShape([70, 60], rect)
    assert not UnitedPosition.PointInShape([60, 40], rect)
    assert UnitedPosition.PointInShape([80, 40], list(reversed(rect)))


def test_boundary_pose_projects_inside_coordinate_limits():
    target = UnitedPosition.ProjectSafePose([110, 80, 30])

    assert target is not None
    assert target[0] < 90
    assert target[1] < 70
    assert target[2] == 30


def test_boundary_recovery_target_is_beyond_pos2pos_arrival_slop():
    target = UnitedPosition.ProjectSafePose([0, 68, 0])

    assert target is not None
    assert target[1] < 60
    assert abs(68 - target[1]) > 8


def test_goal_box_pose_retreats_by_coordinate_geometry():
    target = UnitedPosition.ProjectSafePose([80, 0, 0])

    assert target is not None
    assert target[0] < 65
    assert target[1] == 0


def test_rounded_goal_box_corner_is_handled_by_strict_body_envelope():
    # The strict envelope includes the robot radius, so a point that is close
    # to the marked rounded corner is projected instead of cutting the corner.
    target = UnitedPosition.ProjectSafePose([80, 48, 0])

    assert target is not None
    assert not UnitedPosition.IsPoseInRestrictedArea(target)


def test_competition_level_allows_partial_entry_but_not_full_entry(monkeypatch):
    restricted = Settings.Motion.RestrictedArea
    strict_target = UnitedPosition.ProjectSafePose([80, 0, 0])

    monkeypatch.setattr(restricted, "ProtectionLevel", 2, raising=False)
    competition_target = UnitedPosition.ProjectSafePose([80, 0, 0])

    assert strict_target is not None
    assert competition_target is not None
    assert strict_target[0] < 65
    # x=80 cm is allowed to cross the 75 cm marked line partially, but the
    # projected point stays just outside the full-body forbidden envelope.
    assert 75 < competition_target[0] < 85
    assert competition_target[0] > strict_target[0]
    assert UnitedPosition.ProjectSafePose([78, 0, 0]) is None


def test_rounded_goal_box_body_still_projects_outside():
    target = UnitedPosition.ProjectSafePose([80, 30, 0])

    assert target is not None
    assert target[0] < 80
    assert not UnitedPosition.IsPoseInRestrictedArea(target)


def test_restricted_area_clearance_positive_when_safe():
    # Center is far from every forbidden edge.
    assert UnitedPosition.RestrictedAreaClearance([0.0, 0.0, 0.0]) > 40.0
    # Approaching the +x boundary reduces the clearance monotonically.
    near = UnitedPosition.RestrictedAreaClearance([85.0, 0.0, 0.0])
    far = UnitedPosition.RestrictedAreaClearance([0.0, 0.0, 0.0])
    assert near < far


def test_restricted_area_clearance_none_without_pose():
    assert UnitedPosition.RestrictedAreaClearance(list(POSE_NOT_FOUND)) is None


def _speed_scale_env(monkeypatch, slow_zone=40.0, min_scale=0.3):
    restricted = Settings.Motion.RestrictedArea
    monkeypatch.setattr(restricted, "SpeedSlowZoneCm", slow_zone, raising=False)
    monkeypatch.setattr(restricted, "SpeedMinScale", min_scale, raising=False)


def test_speed_scale_full_when_far_and_floor_inside(monkeypatch):
    _speed_scale_env(monkeypatch)

    assert UnitedPosition.RestrictedAreaSpeedScale([0.0, 0.0, 0.0]) == pytest.approx(1.0)
    # x=85 is inside the strict body envelope (guard x=86) -> floor.
    assert UnitedPosition.RestrictedAreaSpeedScale([85.0, 0.0, 0.0]) == pytest.approx(0.3)


def test_speed_scale_ramps_inside_the_slow_zone(monkeypatch):
    _speed_scale_env(monkeypatch)

    # 26cm to the y-guard (56cm) is inside the 40cm slow zone.
    mid = UnitedPosition.RestrictedAreaSpeedScale([0.0, 30.0, 0.0])
    assert 0.3 < mid < 1.0
    # Closer to the line is slower.
    assert UnitedPosition.RestrictedAreaSpeedScale([0.0, 50.0, 0.0]) < mid


def test_speed_scale_slow_zone_is_derived_from_field(monkeypatch):
    # SpeedSlowZoneCm=0 -> derived as 0.5 * min(XLimit=100, YLimit=70) = 35cm.
    _speed_scale_env(monkeypatch, slow_zone=0.0)

    # 26cm from the y-guard (56cm) is inside the derived 35cm slow zone.
    assert 0.3 < UnitedPosition.RestrictedAreaSpeedScale([0.0, 30.0, 0.0]) < 1.0
    # The field center is outside the derived slow zone -> full speed.
    assert UnitedPosition.RestrictedAreaSpeedScale([0.0, 0.0, 0.0]) == pytest.approx(1.0)


def test_speed_scale_disabled_when_min_scale_is_one(monkeypatch):
    _speed_scale_env(monkeypatch, slow_zone=0.0, min_scale=1.0)

    assert UnitedPosition.RestrictedAreaSpeedScale([85.0, 0.0, 0.0]) == pytest.approx(1.0)


def test_pos2pos_applies_restricted_speed_scale(monkeypatch):
    commands = []

    class FakeChassis:
        def FieldMoveVector(self, x, y, yaw, **_kwargs):
            commands.append((x, y, yaw))

        def stop(self):
            pass

    monkeypatch.setattr(runtime, "chassis", FakeChassis())
    monkeypatch.setattr(UnitedPosition, "FieldRobotPose", lambda: [60.0, 0.0, 0.0])
    monkeypatch.setattr(
        UnitedPosition, "RestrictedAreaSpeedScale", lambda pose=None: 0.5
    )
    UnitedPosition._reset_pos2pos_pid()

    assert UnitedPosition.Pos2Pos([0.0, 0.0, 0.0], speed_gain=6.0) is False
    command_norm = (commands[0][0] ** 2 + commands[0][1] ** 2) ** 0.5
    assert command_norm <= 0.5 * UnitedPosition._pos2pos_command_limit(60.0) + 1e-9
    assert command_norm > 0.0

