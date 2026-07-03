"""Offence role branch tests."""

from __future__ import annotations

import os
import sys

import pytest

os.environ.setdefault("ROBOCUPX_BITS", "SIM")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import offence  # noqa: E402
from LunaPre.simple_strategy import choose_attack_plan  # noqa: E402
from offence import BALL_HELD, BALL_NOT_FOUND  # noqa: E402


def _stub_lockballslip(monkeypatch, calls, *, locked=False):
    monkeypatch.setattr(offence.lockball, "LockBallSlip", lambda: calls.append("lockballslip") or locked)


def _assert_lockballslip_chase(expected, calls, *, locked=False):
    assert offence.OffenceStep() == expected
    assert offence.last_decision["action"] == expected
    assert offence.last_decision["mode"] == "lockballslip_chase"
    assert offence.last_decision["locked"] is locked
    assert calls == ["lockballslip"]


@pytest.fixture(autouse=True)
def _isolate_live_field_state(monkeypatch):
    """Keep unit tests independent from any running simulator frame."""
    monkeypatch.setattr(offence, "_drive_intent", {})
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [9999, 9999, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldBallPos", lambda: BALL_NOT_FOUND)
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])


def test_offence_searches_from_home_when_ball_not_visible(monkeypatch):
    calls = {}
    home = list(offence.Settings.CheckPoints.Home)

    monkeypatch.setattr(offence, "_last_free_ball", None)
    monkeypatch.setattr(offence, "_last_free_ball_ts", 0.0)
    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_NOT_FOUND)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.setdefault("dribble", value))

    def fake_pos2pos(target, speed_gain=None):
        calls["target"] = target
        calls["speed_gain"] = speed_gain
        return False

    monkeypatch.setattr(offence.UnitedPosition, "Pos2Pos", fake_pos2pos)

    assert offence.OffenceStep() == "search_ball"
    assert calls["dribble"] == 0.7
    assert calls["target"] == [home[0] + 20, home[1], 0]
    assert calls["speed_gain"] == 4.5


def test_offence_acquires_visible_near_ball(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [22, -8])
    _stub_lockballslip(monkeypatch, calls)

    _assert_lockballslip_chase("lockballslip_chase", calls)


def test_offence_captures_ball_gently_in_dribble_window(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [3, 6])
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "RobotMoveVector", lambda *args: calls.append(("move", args)))

    assert offence.OffenceStep() == "quick_capture"
    assert offence.last_decision["action"] == "quick_capture"
    assert offence.last_decision["mode"] == "dribbler_window"
    assert calls[0] == ("dribble", 0.95)
    assert calls[1][0] == "move"


def test_offence_treats_visible_dribbler_contact_as_controlled_carry(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [8, -1])
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [-36, 46, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldBallPos", lambda: [-28, 45])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 358)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 0)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "own_half_carry"
    assert offence.last_decision["action"] == "own_half_carry"
    assert offence.last_decision["control_source"] == "quick_capture"
    assert offence.last_decision["contact_ball"] == [8.0, -1.0]
    assert calls[0] == ("dribble", 0.95)
    assert calls[1] == ("dribble", 0.9)


def test_offence_uses_sideline_plan_for_visible_dribbler_contact(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [7, 2])
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [30, 52, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldBallPos", lambda: [37, 54])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence, "_best_goal_target", lambda _pose, _enemies: ([110, 0], -0.4))
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 14)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 90)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "sideline_hide"
    assert offence.last_decision["action"] == "sideline_hide"
    assert offence.last_decision["control_source"] == "quick_capture"
    assert offence.last_decision["contact_ball"] == [7.0, 2.0]
    assert calls[0] == ("dribble", 0.95)
    assert calls[1] == ("dribble", 0.92)


def test_offence_releases_free_ball_when_defender_closes_contact_window(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [3, 9])
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [48, 2, 60])
    monkeypatch.setattr(offence.UnitedPosition, "FieldBallPos", lambda: [42, 8])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [[70, 19]])
    _stub_lockballslip(monkeypatch, calls)

    _assert_lockballslip_chase("lockballslip_chase", calls)


def test_offence_aligns_sideways_before_pushing_close_ball(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [17, 12])
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [0, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldBallPos", lambda: [17, 12])
    _stub_lockballslip(monkeypatch, calls)

    _assert_lockballslip_chase("lockballslip_chase", calls)


def test_offence_uses_lockballslip_for_wide_free_ball_from_inside(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [70, -47])
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [30, -25, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldBallPos", lambda: [100, -72])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    _stub_lockballslip(monkeypatch, calls)

    _assert_lockballslip_chase("lockballslip_chase", calls)


def test_offence_holds_center_when_ball_is_in_enemy_goal_net(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [41, 65])
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [72, 20, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldBallPos", lambda: [113, 85])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [[67, 45]])
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "post_goal_reset_hold"
    assert offence.last_decision["phase"] == "recover_lane"
    assert offence.last_decision["action"] == "post_goal_reset_hold"
    assert offence.last_decision["mode"] == "net_ball"
    assert offence.last_decision["target"] == [46.0, 0.0, 0.0]
    assert calls[0] == ("dribble", False)
    assert calls[1][0] == "move"
    assert calls[1][1][0] < 0.0
    assert calls[1][1][1] < 0.0
    assert calls[1][2] == {"KpW": 0.65}


def test_offence_uses_lockballslip_for_wide_goal_side_ball_that_did_not_score(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [36.5, 46.8])
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [67, 26, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldBallPos", lambda: [103.5, 72.8])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [[66, 50]])
    _stub_lockballslip(monkeypatch, calls)

    _assert_lockballslip_chase("lockballslip_chase", calls)


def test_offence_directly_chases_visible_far_ball(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [65, 18])
    _stub_lockballslip(monkeypatch, calls)

    _assert_lockballslip_chase("lockballslip_chase", calls)


def test_offence_directly_chases_midrange_ball(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [34, -5])
    _stub_lockballslip(monkeypatch, calls)

    _assert_lockballslip_chase("lockballslip_chase", calls)


def test_offence_turns_in_place_when_ball_is_behind(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [-25, 3])
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [0, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldBallPos", lambda: [0, 0])
    _stub_lockballslip(monkeypatch, calls)

    _assert_lockballslip_chase("lockballslip_chase", calls)


def test_offence_escapes_own_backline_before_chasing(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [10, 14])
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [-82, 12, 118])
    monkeypatch.setattr(offence.UnitedPosition, "FieldBallPos", lambda: [-99, 16])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 20)
    _stub_lockballslip(monkeypatch, calls)

    _assert_lockballslip_chase("lockballslip_chase", calls)


def test_offence_escapes_centered_ball_on_own_backline(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [18, 3])
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [-86, -8, 132])
    monkeypatch.setattr(offence.UnitedPosition, "FieldBallPos", lambda: [-103, -6])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 28)
    _stub_lockballslip(monkeypatch, calls)

    _assert_lockballslip_chase("lockballslip_chase", calls)


def test_offence_clears_own_half_wide_ball_before_local_alignment(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [12, 11])
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [-82, 47, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldBallPos", lambda: [-70, 58])
    _stub_lockballslip(monkeypatch, calls)

    _assert_lockballslip_chase("lockballslip_chase", calls)


def test_offence_clears_own_corner_ball_from_farther_approach(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [36, 6])
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [-30, 56, 180])
    monkeypatch.setattr(offence.UnitedPosition, "FieldBallPos", lambda: [-67, 50])
    _stub_lockballslip(monkeypatch, calls)

    _assert_lockballslip_chase("lockballslip_chase", calls)


def test_offence_uses_lockballslip_for_sideline_free_ball(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [22, 9])
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [-55, 62, 140])
    monkeypatch.setattr(offence.UnitedPosition, "FieldBallPos", lambda: [-78, 69])
    _stub_lockballslip(monkeypatch, calls)

    _assert_lockballslip_chase("lockballslip_chase", calls)


def test_offence_chases_last_seen_ball_on_short_dropout(monkeypatch):
    calls = []
    now = {"value": 100.0}

    monkeypatch.setattr(offence.time, "monotonic", lambda: now["value"])
    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [60, -12])
    _stub_lockballslip(monkeypatch, calls)

    _assert_lockballslip_chase("lockballslip_chase", calls)

    calls.clear()
    now["value"] = 100.1
    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_NOT_FOUND)

    _assert_lockballslip_chase("lockballslip_last_ball", calls)


def test_offence_shoots_when_open_goal_is_deep_enough(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [40, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 725)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 5)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.peripheral, "ShootBall", lambda: calls.append("shoot"))
    monkeypatch.setattr(offence.chassis, "stop", lambda: calls.append("stop"))

    assert offence.OffenceStep() == "quick_shot"
    assert calls == [("dribble", 0.85), "shoot", ("dribble", False), "stop"]
    assert offence.last_decision["shot_family"] == "direct_shot"


def test_attack_plan_direct_shot_for_1v0_open_lane():
    plan = choose_attack_plan([36, 0, 0], [], 1, 1.0)

    assert plan[0] == "direct_shot"
    assert plan[1:3] == (110.0, 0.0)


def test_attack_plan_wall_setup_when_front_is_crowded():
    plan = choose_attack_plan([30, 0, 0], [[54, 0], [70, 12]], 1, -0.4)

    assert plan[0] == "wall_setup"
    assert plan[2] > 0


def test_attack_plan_2v1_prefers_wall_shot_over_waiting():
    plan = choose_attack_plan([66, 0, 0], [[92, -5], [112, 10]], 1, -0.5)

    assert plan[0] == "wall_shot"
    assert plan[1] > 44
    assert abs(plan[2]) >= 80


def test_attack_plan_2v2_still_selects_a_simple_exit_lane():
    plan = choose_attack_plan([66, -6, 0], [[90, -8], [108, 10], [35, 58], [100, -50]], 1, -0.5)

    assert plan[0] == "wall_shot"
    assert plan[1] > 46
    assert abs(plan[2]) >= 80


def test_attack_plan_sideline_drive_from_wide_lane():
    plan = choose_attack_plan([30, 56, 0], [], 1, -0.4)

    assert plan[0] == "sideline_hide"
    assert plan[2] > 60
    assert plan[4] > 0

    mid_wide_plan = choose_attack_plan([30, -40, 0], [], 1, 1.0)
    assert mid_wide_plan[0] == "sideline_hide"
    assert mid_wide_plan[2] < -60
    assert mid_wide_plan[4] < 0


def test_offence_does_not_shoot_quick_zone_when_badly_misaligned(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [75, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 100)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 0)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "advance_lane"
    assert calls == [("dribble", 0.85), ("move", (65, -0.0, 100), {"KpW": 0.9})]


def test_offence_quick_shot(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [40, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 90)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 90)
    monkeypatch.setattr(offence, "TurnTo_with_ball", lambda _angle: True)
    monkeypatch.setattr(offence.peripheral, "ShootBall", lambda: calls.append("shoot"))
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "stop", lambda: calls.append("stop"))

    assert offence.OffenceStep() == "quick_shot"
    assert calls == [("dribble", 0.85), "shoot", ("dribble", False), "stop"]


def test_fire_now_locks_short_shot_release_intent(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.peripheral, "ShootBall", lambda: calls.append("shoot"))
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "stop", lambda: calls.append("stop"))

    assert offence._fire_now() == "quick_shot"
    assert calls == [("dribble", 0.85), "shoot", ("dribble", False), "stop"]
    assert offence._active_drive_intent("shot_release") is not None


def test_offence_shot_release_peels_away_from_nearest_defender(monkeypatch):
    calls = []

    offence._remember_drive_intent("shot_release", 1.0)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [60.0, 10.0, 0.0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [[70.0, 15.0, 0.0]])
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "shot_release"
    assert offence.last_decision["action"] == "shot_release"
    assert offence.last_decision["mode"] == "post_shot_peel"
    assert calls == [("dribble", False), ("move", (-18.0, -32.0, 0.0), {"KpW": 0.7})]


def test_offence_shoots_open_goal_when_decisive_lane_ready(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [40, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 40)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 0)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.peripheral, "ShootBall", lambda: calls.append("shoot"))
    monkeypatch.setattr(offence.chassis, "stop", lambda: calls.append("stop"))

    assert offence.OffenceStep() == "quick_shot"
    assert calls == [("dribble", 0.85), "shoot", ("dribble", False), "stop"]


def test_offence_does_not_fire_midfield_shot_while_sliding_wide(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [20, -15, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence, "_best_goal_target", lambda _pose, _enemies: ([110, -38], 1.0))
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 346)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 346)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.peripheral, "ShootBall", lambda: calls.append("shoot"))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "advance_lane"
    assert "shoot" not in calls
    assert offence.last_decision["force_gate"] == "no_close_blocker"


def test_offence_fires_decisive_midfield_lane_before_losing_ball(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [31.0, -15.0, 0.0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence, "_best_goal_target", lambda _pose, _enemies: ([110, 0], 1.0))
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 0)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 0)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.peripheral, "ShootBall", lambda: calls.append("shoot"))
    monkeypatch.setattr(offence.chassis, "stop", lambda: calls.append("stop"))

    assert offence.OffenceStep() == "quick_shot"
    assert offence.last_decision["action"] == "direct_shot"
    assert offence.last_decision["shot_gate"] == "decisive_midfield_lane"
    assert calls == [("dribble", 0.85), "shoot", ("dribble", False), "stop"]


def test_offence_commits_midwide_ball_to_sideline_hide(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [30, -40, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence, "_best_goal_target", lambda _pose, _enemies: ([110, 0], 1.0))
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 350)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 0)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "sideline_hide"
    assert offence.last_decision["action"] == "sideline_hide"
    assert offence.last_decision["shot_gate"] == "corner_entry"
    assert offence.last_decision["target"][1] < -60
    assert calls == [("dribble", 0.92), ("move", (38.0, -22.0, 270.0), {"KpW": 0.85})]


def test_offence_fires_decisive_sideline_window(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [72, 82, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence, "_best_goal_target", lambda _pose, _enemies: ([110, 0], -0.2))
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 291)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 291)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.peripheral, "ShootBall", lambda: calls.append("shoot"))
    monkeypatch.setattr(offence.chassis, "stop", lambda: calls.append("stop"))

    assert offence.OffenceStep() == "quick_shot"
    assert offence.last_decision["action"] == "sideline_shot"
    assert offence.last_decision["shot_gate"] == "decisive_sideline_window"
    assert abs(offence.last_decision["shot_y_at_goal"]) <= offence.SIDELINE_GOAL_HALF_WIDTH_CM
    assert calls == [("dribble", 0.85), "shoot", ("dribble", False), "stop"]


def test_offence_stays_in_sideline_drive_when_goal_line_is_not_ready(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [72, 82, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence, "_best_goal_target", lambda _pose, _enemies: ([110, 0], 1.0))
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 140)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 0)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.peripheral, "ShootBall", lambda: calls.append("shoot"))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "sideline_drive"
    assert offence.last_decision["action"] == "sideline_drive"
    assert offence.last_decision["shot_gate"] == "sideline_goal_line_not_ready"
    assert "shoot" not in calls
    assert calls == [("dribble", 0.85), ("move", (28.0, 0.0, 140), {"KpW": 0.9})]


def test_offence_does_not_shoot_sideline_ball_while_still_facing_outward(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [72, 82, 54])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence, "_best_goal_target", lambda _pose, _enemies: ([110, 0], 1.0))
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 332)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 54)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.peripheral, "ShootBall", lambda: calls.append("shoot"))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "sideline_drive"
    assert offence.last_decision["action"] == "sideline_drive"
    assert offence.last_decision["shot_gate"] == "sideline_goal_line_not_ready"
    assert "shoot" not in calls
    assert calls == [("dribble", 0.85), ("move", (28.0, 0.0, 332), {"KpW": 0.9})]


def test_offence_shoots_sideline_contact_ball_after_corner_alignment(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [3.0, 8.0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [80.0, 59.0, 296.0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldBallPos", lambda: [88.5, 59.8])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 298)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 300)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "sideline_drive"
    assert offence.last_decision["action"] == "sideline_drive"
    assert offence.last_decision["control_source"] == "quick_capture"
    assert offence.last_decision["contact_ball"] == [3.0, 8.0]
    assert calls[0] == ("dribble", 0.95)
    assert calls[1] == ("dribble", 0.85)


def test_sideline_restores_width_without_flipping_side(monkeypatch):
    calls = []

    monkeypatch.setattr(
        offence,
        "_drive_intent",
        {"action": "sideline_drive", "until": 9999999999.0, "target_y": -18.0, "side": 1.0},
    )
    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [86.0, 15.0, 303.0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence, "_best_goal_target", lambda _pose, _enemies: ([110, 0], 1.0))
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 303.0)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "sideline_hide"
    assert offence.last_decision["action"] == "sideline_hide"
    assert offence.last_decision["shot_gate"] == "restore_width"
    assert offence.last_decision["target"][1] == 82.0
    assert calls == [("dribble", 0.92), ("move", (0.0, 14.0, 90.0), {"KpW": 0.85})]


def test_offence_carries_from_own_half_when_line_is_clean(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [-20, 12, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 40)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 0)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "own_half_carry"
    assert calls == [("dribble", 0.9), ("move", (62.0, -6.6000000000000005, 40), {"KpW": 0.8})]


def test_offence_does_not_shoot_open_goal_from_own_half(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [-8, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 20)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 0)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "own_half_carry"
    assert calls == [("dribble", 0.9), ("move", (62.0, -0.0, 20), {"KpW": 0.8})]


def test_offence_does_not_shoot_open_goal_when_yaw_is_too_wide(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [25, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 55)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 0)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "advance_lane"
    assert calls == [("dribble", 0.85), ("move", (52, -0.0, 55), {"KpW": 0.9})]


def test_offence_advances_when_front_lane_clear(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [40, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 15)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 90)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "advance_lane"
    assert calls == [("dribble", 0.85), ("move", (65, -0.0, 15), {"KpW": 0.9})]


def test_offence_fast_shoots_when_front_lane_clear_and_aligned(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [40, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 15)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 10)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.peripheral, "ShootBall", lambda: calls.append("shoot"))
    monkeypatch.setattr(offence.chassis, "stop", lambda: calls.append("stop"))

    assert offence.OffenceStep() == "quick_shot"
    assert calls == [("dribble", 0.85), "shoot", ("dribble", False), "stop"]


def test_offence_back_shot_when_front_goal_line_clean(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [76, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 0)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 180)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.peripheral, "ShootBall", lambda: calls.append("shoot"))
    monkeypatch.setattr(offence.chassis, "stop", lambda: calls.append("stop"))

    assert offence.OffenceStep() == "quick_shot"
    assert offence.last_decision["action"] == "back_shot"
    assert calls == [("dribble", 0.85), "shoot", ("dribble", False), "stop"]


def test_offence_shoots_around_single_blocker_when_side_lane_open(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [48, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [[72, 0]])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda target: 8 if abs(target[1]) > 1 else 0)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 0)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.peripheral, "ShootBall", lambda: calls.append("shoot"))
    monkeypatch.setattr(offence.chassis, "stop", lambda: calls.append("stop"))

    assert offence.OffenceStep() == "quick_shot"
    assert calls == [("dribble", 0.85), "shoot", ("dribble", False), "stop"]


def test_offence_uses_ohmyback_on_wide_ball(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [20, 45, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 0)
    monkeypatch.setattr(offence, "OhMyBackStep", lambda **kwargs: calls.append(kwargs) or False)

    assert offence.OffenceStep() == "oh_my_back"
    assert calls and calls[0]["attack_dir"] == 1


def test_offence_recovers_corner_ball_without_ohmyback(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [-76, 68, 150])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 20)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "corner_recover"
    assert calls == [("dribble", 0.9), ("move", (46, -44.0, 20), {"KpW": 0.9})]


def test_offence_keeps_deep_enemy_corner_in_sideline_flow(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [92.0, -50.0, 176.0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [[67.0, -48.0]])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 160)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "sideline_drive"
    assert offence.last_decision["action"] == "sideline_drive"
    assert offence.last_decision["shot_gate"] == "sideline_width_not_ready"
    assert calls == [("dribble", 0.85), ("move", (-8.0, -12.0, 160), {"KpW": 0.9})]


def test_offence_dodges_blocker_instead_of_waiting(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [20, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 0)
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [[45, 0]])
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 80)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "attack_dodge"
    assert calls == [("dribble", 0.85), ("move", (34, 34, 0), {"KpW": 0.95})]


def test_offence_releases_shot_before_double_pressure_push(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [16.0, -10.0, 3.0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [[35.0, 8.0], [40.0, -16.0]])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 8)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 3.0)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.peripheral, "ShootBall", lambda: calls.append("shoot"))
    monkeypatch.setattr(offence.chassis, "stop", lambda: calls.append("stop"))

    assert offence.OffenceStep() == "quick_shot"
    assert offence.last_decision["action"] == "pressure_release_shot"
    assert calls == [("dribble", 0.85), "shoot", ("dribble", False), "stop"]


def test_offence_wall_shoots_when_deep_direct_lane_is_blocked(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [66, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [[90, 0]])
    monkeypatch.setattr(offence, "_best_goal_target", lambda _pose, _enemies: ([110, 0], -0.2))
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 0)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 0)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.peripheral, "ShootBall", lambda: calls.append("shoot"))
    monkeypatch.setattr(offence.chassis, "stop", lambda: calls.append("stop"))

    assert offence.OffenceStep() == "quick_shot"
    assert offence.last_decision["action"] == "wall_shot"
    assert calls == [("dribble", 0.85), "shoot", ("dribble", False), "stop"]


def test_offence_sets_up_wall_in_midfield_instead_of_random_wall_shot(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [44, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [[68, 0]])
    monkeypatch.setattr(offence, "_best_goal_target", lambda _pose, _enemies: ([110, 0], -0.2))
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 18)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 0)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "wall_setup"
    assert offence.last_decision["action"] == "wall_setup"
    assert calls == [("dribble", 0.85), ("move", (42.0, 22.0, 18), {"KpW": 0.9})]


def test_offence_sets_up_wall_shot_before_deep_zone(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [30, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [[54, 0], [70, 12]])
    monkeypatch.setattr(offence, "_best_goal_target", lambda _pose, _enemies: ([110, 0], -0.4))
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 22)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 85)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "wall_setup"
    assert offence.last_decision["action"] == "wall_setup"
    assert calls == [("dribble", 0.85), ("move", (44.0, 26.0, 22), {"KpW": 0.9})]


def test_offence_sideline_shoots_from_wide_deep_lane(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [72, 82, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence, "_best_goal_target", lambda _pose, _enemies: ([110, 0], -0.4))
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 291)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 291)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.peripheral, "ShootBall", lambda: calls.append("shoot"))
    monkeypatch.setattr(offence.chassis, "stop", lambda: calls.append("stop"))

    assert offence.OffenceStep() == "quick_shot"
    assert offence.last_decision["action"] == "sideline_shot"
    assert abs(offence.last_decision["shot_y_at_goal"]) <= offence.SIDELINE_GOAL_HALF_WIDTH_CM
    assert calls == [("dribble", 0.85), "shoot", ("dribble", False), "stop"]


def test_offence_sideline_drives_before_shot_window(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [30, 56, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence, "_best_goal_target", lambda _pose, _enemies: ([110, 0], -0.4))
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 14)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 90)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "sideline_hide"
    assert offence.last_decision["action"] == "sideline_hide"
    assert offence.last_decision["shot_gate"] == "corner_entry"
    assert calls == [("dribble", 0.92), ("move", (38.0, 22.0, 90.0), {"KpW": 0.85})]


def test_offence_filters_side_and_rear_chassis_before_attacking(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [40, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 0)
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [[-10, 0], [45, 90], [140, 0]])
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 80)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "advance_lane"
    assert calls == [("dribble", 0.85), ("move", (65, -0.0, 0), {"KpW": 0.9})]


def test_offence_forces_shot_under_pressure_near_blocker(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [50, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 0)
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [[70, 0]])
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 55)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.peripheral, "ShootBall", lambda: calls.append("shoot"))
    monkeypatch.setattr(offence.chassis, "stop", lambda: calls.append("stop"))

    assert offence.OffenceStep() == "quick_shot"
    assert calls == [("dribble", 0.85), "shoot", ("dribble", False), "stop"]


def test_offence_forces_contested_shot_before_pressure_zone(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [34, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 0)
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [[58, 0]])
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 55)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.peripheral, "ShootBall", lambda: calls.append("shoot"))
    monkeypatch.setattr(offence.chassis, "stop", lambda: calls.append("stop"))

    assert offence.OffenceStep() == "quick_shot"
    assert calls == [("dribble", 0.85), "shoot", ("dribble", False), "stop"]


def test_offence_does_not_force_contested_shot_before_threat_zone(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [-4, -10, 12])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 18)
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [[23, 0], [59, 0]])
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 12)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "own_half_carry"
    assert calls == [("dribble", 0.9), ("move", (62.0, 5.5, 18), {"KpW": 0.8})]
    assert offence.last_decision["action"] == "own_half_carry"


def test_offence_advances_when_no_trick_slot(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [20, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 15)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 90)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "advance_lane"
    assert calls == [("dribble", 0.85), ("move", (52, -0.0, 15), {"KpW": 0.9})]


def test_offence_stop_clears_outputs(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "stop", lambda: calls.append(("stop", None)))
    monkeypatch.setattr(offence.UnitedPosition, "Stop", lambda: calls.append(("united_stop", None)))

    offence.OffenceStop()

    assert calls == [
        ("dribble", False),
        ("stop", None),
        ("united_stop", None),
    ]
    assert offence.state == "stopped"
