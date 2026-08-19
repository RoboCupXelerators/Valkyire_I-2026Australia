"""Coordinate-space restricted-area guard tests."""

from __future__ import annotations

import os
import sys

import pytest

os.environ.setdefault("ROBOCUPX_BITS", "SIM")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from LunaPre import UnitedPosition
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
