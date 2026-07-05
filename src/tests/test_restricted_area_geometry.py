"""Coordinate-space restricted-area guard tests."""

from __future__ import annotations

import os
import sys

os.environ.setdefault("ROBOCUPX_BITS", "SIM")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from LunaPre import UnitedPosition


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


def test_goal_box_edge_pose_can_sidestep_when_nearest():
    target = UnitedPosition.ProjectSafePose([80, 48, 0])

    assert target is not None
    assert target[0] == 80
    assert target[1] > 50
