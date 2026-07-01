"""Offence role branch tests."""

from __future__ import annotations

import os
import sys

os.environ.setdefault("ROBOCUPX_BITS", "SIM")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import offence  # noqa: E402
from LunaPre.simple_strategy import choose_attack_plan  # noqa: E402
from offence import BALL_HELD, BALL_NOT_FOUND  # noqa: E402


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
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "RobotMoveVector", lambda *args: calls.append(("move", args)))

    assert offence.OffenceStep() == "near_ball_acquire"
    assert calls[0] == ("dribble", 0.9)
    assert calls[1][0] == "move"
    assert calls[1][1][0] > 0
    assert calls[1][1][1] < 0


def test_offence_captures_ball_gently_in_dribble_window(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [3, 6])
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "RobotMoveVector", lambda *args: calls.append(("move", args)))

    assert offence.OffenceStep() == "near_ball_capture"
    assert calls[0] == ("dribble", 0.95)
    assert calls[1][0] == "move"
    assert calls[1][1][0] == 8.0
    assert 0 < calls[1][1][1] <= 10.0
    assert abs(calls[1][1][2]) <= 24.0


def test_offence_aligns_sideways_before_pushing_close_ball(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [17, 12])
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [0, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldBallPos", lambda: [17, 12])
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "RobotMoveVector", lambda *args: calls.append(("move", args)))

    assert offence.OffenceStep() == "near_ball_align"
    assert calls[0] == ("dribble", 0.95)
    assert calls[1][0] == "move"
    assert 0.0 <= calls[1][1][0] <= 12.0
    assert calls[1][1][1] > 20.0
    assert abs(calls[1][1][2]) <= 18.0


def test_offence_directly_chases_visible_far_ball(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [65, 18])
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "RobotMoveVector", lambda *args: calls.append(("move", args)))

    assert offence.OffenceStep() == "chase_ball"
    assert calls[0] == ("dribble", 0.75)
    assert calls[1][0] == "move"
    assert calls[1][1][0] > 0


def test_offence_directly_chases_midrange_ball(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [34, -5])
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "RobotMoveVector", lambda *args: calls.append(("move", args)))

    assert offence.OffenceStep() == "chase_ball"
    assert calls[0] == ("dribble", 0.75)
    assert calls[1][0] == "move"


def test_offence_turns_in_place_when_ball_is_behind(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [-25, 3])
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [0, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldBallPos", lambda: [0, 0])
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "RobotMoveVector", lambda *args: calls.append(("move", args)))

    assert offence.OffenceStep() == "turn_to_ball"
    assert calls[0] == ("dribble", 0.75)
    assert calls[1][0] == "move"
    move = calls[1][1]
    assert move[0] == 0.0
    assert 0.0 < move[1] <= 10.0
    assert 22.0 <= move[2] <= 48.0


def test_offence_escapes_own_backline_before_chasing(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [10, 14])
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [-82, 12, 118])
    monkeypatch.setattr(offence.UnitedPosition, "FieldBallPos", lambda: [-99, 16])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 20)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "own_backline_escape"
    assert calls == [("dribble", 0.75), ("move", (36, -7.2, 20), {"KpW": 0.9})]


def test_offence_escapes_centered_ball_on_own_backline(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [18, 3])
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [-86, -8, 132])
    monkeypatch.setattr(offence.UnitedPosition, "FieldBallPos", lambda: [-103, -6])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 28)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "own_backline_escape"
    assert calls == [("dribble", 0.75), ("move", (36, 2.7, 28), {"KpW": 0.9})]


def test_offence_clears_own_half_wide_ball_before_local_alignment(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [12, 11])
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [-82, 47, 60])
    monkeypatch.setattr(offence.UnitedPosition, "FieldBallPos", lambda: [-70, 50])
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "RobotMoveVector", lambda *args: calls.append(("move", args)))

    assert offence.OffenceStep() == "own_half_wide_clear"
    assert offence.last_decision["action"] == "own_half_wide_clear"
    assert calls[0] == ("dribble", 0.9)
    assert calls[1][0] == "move"
    assert calls[1][1][0] == 22.0
    assert calls[1][1][1] == 25.299999999999997
    assert 6.0 <= calls[1][1][2] <= 7.0


def test_offence_chases_last_seen_ball_on_short_dropout(monkeypatch):
    calls = []
    now = {"value": 100.0}

    monkeypatch.setattr(offence.time, "monotonic", lambda: now["value"])
    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [60, -12])
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "RobotMoveVector", lambda *args: calls.append(("move", args)))

    assert offence.OffenceStep() == "chase_ball"

    calls.clear()
    now["value"] = 100.1
    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_NOT_FOUND)

    assert offence.OffenceStep() == "chase_last_ball"
    assert calls[0] == ("dribble", 0.75)
    assert calls[1][0] == "move"


def test_offence_shoots_immediately_in_quick_shot_zone(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [30, 0, 0])
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

    assert plan[0] == "sideline_drive"
    assert plan[2] > 0


def test_offence_does_not_shoot_quick_zone_when_badly_misaligned(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [75, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 100)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 0)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "fast_attack"
    assert calls == [("dribble", 0.85), ("move", (65, -0.0, 100), {"KpW": 0.9})]


def test_offence_quick_shot(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [30, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 90)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 90)
    monkeypatch.setattr(offence, "TurnTo_with_ball", lambda _angle: True)
    monkeypatch.setattr(offence.peripheral, "ShootBall", lambda: calls.append("shoot"))
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "stop", lambda: calls.append("stop"))

    assert offence.OffenceStep() == "quick_shot"
    assert calls == [("dribble", 0.85), "shoot", ("dribble", False), "stop"]


def test_offence_shoots_open_goal_before_old_opportunity_zone(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [24, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 40)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 0)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.peripheral, "ShootBall", lambda: calls.append("shoot"))
    monkeypatch.setattr(offence.chassis, "stop", lambda: calls.append("stop"))

    assert offence.OffenceStep() == "quick_shot"
    assert calls == [("dribble", 0.85), "shoot", ("dribble", False), "stop"]


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

    assert offence.OffenceStep() == "feint_drive"
    assert calls == [("dribble", 0.85), ("move", (52, -0.0, 55), {"KpW": 0.9})]


def test_offence_fast_attacks_when_front_lane_clear(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [40, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 15)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 90)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "fast_attack"
    assert calls == [("dribble", 0.85), ("move", (65, -0.0, 15), {"KpW": 0.9})]


def test_offence_fast_shoots_when_front_lane_clear_and_aligned(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [30, 0, 0])
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
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [60, 56, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence, "_best_goal_target", lambda _pose, _enemies: ([110, 0], -0.4))
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 0)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 0)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.peripheral, "ShootBall", lambda: calls.append("shoot"))
    monkeypatch.setattr(offence.chassis, "stop", lambda: calls.append("stop"))

    assert offence.OffenceStep() == "quick_shot"
    assert offence.last_decision["action"] == "sideline_shot"
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

    assert offence.OffenceStep() == "sideline_drive"
    assert offence.last_decision["action"] == "sideline_drive"
    assert calls == [("dribble", 0.85), ("move", (46.0, -18.0, 14), {"KpW": 0.9})]


def test_offence_filters_side_and_rear_chassis_before_attacking(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [40, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 0)
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [[-10, 0], [45, 90], [140, 0]])
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 80)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "fast_attack"
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


def test_offence_feint_drive_when_no_trick_slot(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [20, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 15)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 90)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "feint_drive"
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
