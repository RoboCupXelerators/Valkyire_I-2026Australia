"""Offence role branch tests for the pruned 1v0 striker flow."""

from __future__ import annotations

import os
import sys

import pytest

os.environ.setdefault("ROBOCUPX_BITS", "SIM")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import offence  # noqa: E402
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
    monkeypatch.setattr(offence, "_drive_intent", {})
    monkeypatch.setattr(offence, "_last_free_ball", None)
    monkeypatch.setattr(offence, "_last_free_ball_ts", 0.0)
    monkeypatch.setattr(offence, "_held_since", None)
    monkeypatch.setattr(offence, "_last_shot_ts", -999999.0)
    monkeypatch.setattr(offence, "HELD_DEBOUNCE_S", 0.0)
    monkeypatch.setattr(offence, "SHOT_COOLDOWN_S", 0.0)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [9999, 9999, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldBallPos", lambda: BALL_NOT_FOUND)
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [])


def test_offence_searches_from_home_when_ball_not_visible(monkeypatch):
    calls = {}
    home = list(offence.Settings.CheckPoints.Home)

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
    assert calls["speed_gain"] == offence.SEARCH_SPEED_GAIN


def test_offence_rejects_unphysical_robot_ball_as_not_visible(monkeypatch):
    calls = {}
    home = list(offence.Settings.CheckPoints.Home)

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [-5044.88, -4808.92])
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.setdefault("dribble", value))

    def fake_pos2pos(target, speed_gain=None):
        calls["target"] = target
        calls["speed_gain"] = speed_gain
        return False

    monkeypatch.setattr(offence.UnitedPosition, "Pos2Pos", fake_pos2pos)

    assert offence.OffenceStep() == "search_ball"
    assert offence.last_decision["action"] == "search_ball"
    assert calls["target"] == [home[0] + 20, home[1], 0]
    assert calls["speed_gain"] == offence.SEARCH_SPEED_GAIN


def test_offence_uses_lockballslip_for_visible_free_ball(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [22, -8])
    _stub_lockballslip(monkeypatch, calls)

    _assert_lockballslip_chase("lockballslip_chase", calls)


def test_offence_captures_ball_in_dribbler_window(monkeypatch):
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


def test_offence_holds_center_when_ball_is_in_enemy_goal_net(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: [41, 65])
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [72, 20, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldBallPos", lambda: [113, 85])
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "post_goal_reset_hold"
    assert offence.last_decision["phase"] == "recover_lane"
    assert offence.last_decision["action"] == "post_goal_reset_hold"
    assert offence.last_decision["target"] == [46.0, 0.0, 0.0]
    assert calls[0] == ("dribble", False)
    assert calls[1][0] == "move"


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
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 725)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 5)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.peripheral, "ShootBall", lambda: calls.append("shoot"))
    monkeypatch.setattr(offence.chassis, "stop", lambda: calls.append("stop"))

    assert offence.OffenceStep() == "quick_shot"
    assert offence.last_decision["action"] == "direct_shot"
    assert calls == [("dribble", 0.85), "shoot", ("dribble", False), "stop"]


def test_offence_debounces_new_possession_before_shoot(monkeypatch):
    calls = []
    now = {"value": 100.0}

    monkeypatch.setattr(offence, "HELD_DEBOUNCE_S", 0.18)
    monkeypatch.setattr(offence.time, "monotonic", lambda: now["value"])
    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [40, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 0)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 0)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.peripheral, "ShootBall", lambda: calls.append("shoot"))
    monkeypatch.setattr(offence.chassis, "stop", lambda: calls.append("stop"))

    assert offence.OffenceStep() == "shot_guard_hold"
    assert offence.last_decision["mode"] == "held_debounce"
    assert "shoot" not in calls

    calls.clear()
    now["value"] = 100.2

    assert offence.OffenceStep() == "quick_shot"
    assert "shoot" in calls


def test_offence_blocks_shot_during_cooldown(monkeypatch):
    calls = []
    now = {"value": 50.0}

    monkeypatch.setattr(offence, "SHOT_COOLDOWN_S", 1.2)
    monkeypatch.setattr(offence, "_last_shot_ts", 49.4)
    monkeypatch.setattr(offence.time, "monotonic", lambda: now["value"])
    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [76, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 0)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 180)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.peripheral, "ShootBall", lambda: calls.append("shoot"))
    monkeypatch.setattr(offence.chassis, "stop", lambda: calls.append("stop"))

    assert offence.OffenceStep() == "shot_guard_hold"
    assert offence.last_decision["mode"] == "shot_cooldown"
    assert offence.last_decision["blocked_action"] == "back_shot"
    assert "shoot" not in calls


def test_offence_advances_when_badly_misaligned(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [75, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 100)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 0)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "advance_lane"
    assert calls == [("dribble", 0.85), ("move", (65.0, -0.0, 100), {"KpW": 0.9})]


def test_fire_now_locks_short_shot_release_intent(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.peripheral, "ShootBall", lambda: calls.append("shoot"))
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "stop", lambda: calls.append("stop"))

    assert offence._fire_now() == "quick_shot"
    assert calls == [("dribble", 0.85), "shoot", ("dribble", False), "stop"]
    assert offence._active_drive_intent("shot_release") is not None


def test_offence_shot_release_peels_away_after_shot(monkeypatch):
    calls = []

    monkeypatch.setattr(offence, "_drive_intent", {"action": "shot_release", "until": 9999999999.0})
    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [50, 10, 0])
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "shot_release"
    assert calls == [("dribble", False), ("move", (-18.0, -32.0, 0.0), {"KpW": 0.7})]
    assert offence.last_decision["action"] == "shot_release"


def test_offence_carries_from_own_half(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [-20, 12, 0])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 40)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 0)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "own_half_carry"
    assert calls == [("dribble", 0.9), ("move", (62.0, -6.6000000000000005, 40), {"KpW": 0.8})]


def test_offence_recovers_deep_own_corner(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [-76, 68, 150])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 20)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "corner_recover"
    assert calls == [("dribble", 0.9), ("move", (46.0, -44.0, 20), {"KpW": 0.9})]


def test_offence_back_shot_when_front_goal_line_clean(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [76, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 0)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 180)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.peripheral, "ShootBall", lambda: calls.append("shoot"))
    monkeypatch.setattr(offence.chassis, "stop", lambda: calls.append("stop"))

    assert offence.OffenceStep() == "quick_shot"
    assert offence.last_decision["action"] == "back_shot"
    assert calls == [("dribble", 0.85), "shoot", ("dribble", False), "stop"]


def test_offence_ignores_blocker_branches_and_keeps_main_lane(monkeypatch):
    calls = []

    monkeypatch.setattr(offence.UnitedPosition, "RobotBallPos", lambda: BALL_HELD)
    monkeypatch.setattr(offence.UnitedPosition, "FieldRobotPos", lambda: [20, 0, 0])
    monkeypatch.setattr(offence.UnitedPosition, "FieldChassisPos", lambda: [[45, 0]])
    monkeypatch.setattr(offence.UnitedPosition, "Local2Angle", lambda _target: 0)
    monkeypatch.setattr(offence.chassis, "GetYaw", lambda: 80)
    monkeypatch.setattr(offence.peripheral, "Dribble", lambda value: calls.append(("dribble", value)))
    monkeypatch.setattr(offence.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))

    assert offence.OffenceStep() == "advance_lane"
    assert offence.last_decision["action"] == "advance_lane"
    assert calls == [("dribble", 0.85), ("move", (52.0, -0.0, 0), {"KpW": 0.9})]


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
