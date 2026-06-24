"""Offence role branch tests."""

from __future__ import annotations

import os
import sys

os.environ.setdefault("ROBOCUPX_BITS", "SIM")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import offence  # noqa: E402
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


def test_offence_locks_visible_ball(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [22, -8])
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.lockball, "LockBallSlip", lambda: calls.append(("lock", None)))

    assert offence.OffenceStep() == "lock_ball"
    assert calls == [("dribble", 0.7), ("lock", None)]


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


def test_offence_shoots_open_goal_from_own_half_when_line_is_clean(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [20, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 40)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 0)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.peripheral, "ShootBall", lambda: calls.append("shoot"))
    monkeypatch.setattr(offence.chassis, "stop", lambda: calls.append("stop"))

    assert offence.OffenceStep() == "quick_shot"
    assert calls == [("dribble", 0.85), "shoot", ("dribble", False), "stop"]


def test_offence_does_not_shoot_open_goal_from_own_half(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [-8, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 20)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 0)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "feint_drive"
    assert calls == [("dribble", 0.85), ("move", (52, -0.0, 20), {"KpW": 0.9})]


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
    assert calls == [("dribble", 0.85), ("move", (42, 42, 0), {"KpW": 0.95})]


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

    assert offence.OffenceStep() == "attack_dodge"
    assert calls == [("dribble", 0.85), ("move", (42, -36.25, 18), {"KpW": 0.95})]
    assert offence.last_decision["force_gate"] == "forced_shot_before_threat_zone"


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
