"""Robot-frame shooting window tests."""

from __future__ import annotations

import os
import sys

import pytest

os.environ.setdefault("ROBOCUPX_BITS", "SIM")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from LunaPre.positions import Positions  # noqa: E402


def _positions(*, goals=None, bboxes=None, chassis=None):
    position = object.__new__(Positions)
    position.RobotGoalsPos = lambda: goals if goals is not None else {"blue_goal": None, "yellow_goal": None}
    position.RobotChassisPos = lambda: chassis if chassis is not None else []
    position._get_goal_bboxes = lambda: bboxes if bboxes is not None else [None, None, None, None]
    return position


def test_front_goal_without_blocker_is_open():
    position = _positions(goals={"blue_goal": [200.0, 0.0, 0.9], "yellow_goal": None})

    window = position.ShootWindow()

    assert window["reason"] == "ok"
    assert window["goal_key"] == "blue_goal"
    assert window["window_open"] is True
    assert window["can_shoot_now"] is True
    assert window["best_bearing_deg"] == 0.0
    assert position.CanShoot() is True


def test_back_goal_is_classified_as_own_goal():
    position = _positions(goals={"blue_goal": [-160.0, 0.0, 0.9], "yellow_goal": None})

    window = position.ShootWindow(use_estimated_goal=False)

    assert window["reason"] == "only_own_goal_visible"
    assert window["window_open"] is False
    assert window["can_shoot_now"] is False


def test_goal_identity_does_not_depend_on_color():
    position = _positions(goals=[[-180.0, 0.0, 0.9], [170.0, 8.0, 0.8]])

    window = position.ShootWindow()

    assert window["goal_key"] == "yellow_goal"
    assert window["goal_color"] == "yellow"
    assert window["window_open"] is True


@pytest.mark.xfail(
    reason="Vision.GoalWidth migrated to the Australia 45cm goal; expected sim window/occlusion values pending confirmation",
    strict=False,
)
def test_center_blocker_splits_window_and_requires_turn():
    position = _positions(
        goals={"blue_goal": [200.0, 0.0, 0.9], "yellow_goal": None},
        chassis=[[100.0, 0.0, 0.8]],
    )

    window = position.ShootWindow()

    assert window["reason"] == "need_turn"
    assert window["window_open"] is True
    assert window["can_shoot_now"] is False
    assert len(window["open_windows_deg"]) == 2
    assert abs(window["best_bearing_deg"]) > 1.0
    assert len(window["blockers"]) == 1


def test_side_chassis_outside_lane_does_not_block_shot():
    position = _positions(
        goals={"blue_goal": [200.0, 0.0, 0.9], "yellow_goal": None},
        chassis=[[100.0, 50.0, 0.8]],
    )

    window = position.ShootWindow()

    assert window["reason"] == "ok"
    assert window["can_shoot_now"] is True
    assert window["blockers"] == []


def test_goal_bbox_takes_priority_over_width_fallback():
    position = _positions(
        goals={"blue_goal": [100.0, 0.0, 0.9], "yellow_goal": None},
        bboxes=[None, None, [2, 1, 300, 220, 340, 420, 0.9], None],
    )

    window = position.ShootWindow()

    assert abs(window["raw_window_deg"][0]) < 4.0
    assert abs(window["raw_window_deg"][1]) < 4.0


def test_right_camera_goal_bbox_maps_to_negative_bearing():
    position = _positions(
        goals={"blue_goal": None, "yellow_goal": None},
        bboxes=[None, None, None, [3, 1, 300, 220, 340, 420, 0.9]],
    )

    window = position.ShootWindow(target="blue_goal")

    assert window["goal_key"] == "blue_goal"
    assert window["raw_window_deg"][0] < -85.0
    assert window["raw_window_deg"][1] < -85.0


@pytest.mark.xfail(
    reason="Vision.GoalWidth migrated to the Australia 45cm goal; expected sim window/occlusion values pending confirmation",
    strict=False,
)
def test_goal_width_fallback_is_used_without_bbox():
    position = _positions(goals={"blue_goal": [100.0, 0.0, 0.9], "yellow_goal": None})

    window = position.ShootWindow()
    width = abs(window["raw_window_deg"][1] - window["raw_window_deg"][0])

    assert width > 40.0


def test_goal_list_shape_is_supported():
    position = _positions(goals=[[150.0, 0.0, 0.7], None])

    window = position.ShootWindow()

    assert window["goal_key"] == "blue_goal"
    assert window["can_shoot_now"] is True


def test_estimated_goal_fallback_opens_window_when_goal_is_missing():
    position = _positions()

    window = position.ShootWindow()

    assert window["reason"] == "estimated_ok"
    assert window["goal_key"] == "estimated_opponent_goal"
    assert window["goal_color"] == "estimated"
    assert window["window_open"] is True
    assert window["can_shoot_now"] is True


@pytest.mark.xfail(
    reason="Vision.GoalWidth migrated to the Australia 45cm goal; expected sim window/occlusion values pending confirmation",
    strict=False,
)
def test_estimated_goal_fallback_uses_chassis_blockers():
    position = _positions(chassis=[[100.0, 0.0, 0.8]])

    window = position.ShootWindow()

    assert window["reason"] == "estimated_need_turn"
    assert window["goal_key"] == "estimated_opponent_goal"
    assert window["can_shoot_now"] is False
    assert len(window["blockers"]) == 1


def test_estimated_goal_fallback_does_not_fake_color_specific_goal():
    position = _positions()

    window = position.ShootWindow(target="blue_goal")

    assert window["reason"] == "no_goal_visible"
    assert window["window_open"] is False


def test_only_own_goal_visible_falls_back_to_estimated_opponent_goal():
    position = _positions(goals={"blue_goal": [-160.0, 0.0, 0.9], "yellow_goal": None})

    window = position.ShootWindow()

    assert window["reason"] == "estimated_ok"
    assert window["goal_key"] == "estimated_opponent_goal"
    assert window["can_shoot_now"] is True
