"""LockBallSlip target-cache regression tests."""

from __future__ import annotations

import math
import os
import sys
from types import SimpleNamespace

import pytest

os.environ.setdefault("ROBOCUPX_BITS", "SIM")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import BasicFuc as basic  # noqa: E402


def test_lockball_loads_all_pid_terms_and_output_limits(monkeypatch):
    tuning = SimpleNamespace(
        ForwardKp=0.02,
        ForwardKi=0.006,
        ForwardKd=0.005,
        LateralKp=0.016,
        LateralKi=0.0005,
        LateralKd=0.005,
        AngularKp=1.3,
        AngularKi=0.06,
        AngularKd=0.15,
        ForwardMaxMps=0.8,
        LateralMaxMps=0.65,
        AngularMaxDegS=85.0,
    )
    monkeypatch.setattr(basic.Settings.Motion, "LockBallSlip", tuning)

    lock = basic.LockSeries()

    assert lock.pan_x.tunings == (0.02, 0.006, 0.005)
    assert lock.pan_y.tunings == (0.016, 0.0005, 0.005)
    assert lock.pan_w.tunings == (1.3, 0.06, 0.15)
    assert lock.pan_x.output_limits == (-0.8, 0)
    assert lock.pan_y.output_limits == (-0.65, 0.65)
    assert lock.pan_w.output_limits == (-85.0, 85.0)


def test_lockball_loads_near_dock_envelope(monkeypatch):
    tuning = SimpleNamespace(
        NearDockDecelMps2=0.6,
    )
    monkeypatch.setattr(basic.Settings.Motion, "LockBallSlip", tuning)

    lock = basic.LockSeries()

    assert lock.near_dock_decel == 0.6


def _lock_with_observations(monkeypatch, observations):
    values = iter(observations)
    monkeypatch.setattr(basic.UnitedPosition, "RobotBallPos", lambda: next(values))

    lock = basic.LockSeries()
    lock.ball_jump_reject_cm = 45.0
    lock.ball_jump_confirm_frames = 2
    resets = []
    monkeypatch.setattr(lock, "_reset_lockball_pid", lambda: resets.append(True))
    return lock, resets


def test_lockball_accepts_a_persistent_jump_and_resets_pid(monkeypatch):
    lock, resets = _lock_with_observations(
        monkeypatch,
        ([20.0, 0.0], [100.0, 0.0], [90.0, 10.0]),
    )

    accepted = [lock._current_or_recent_ball()[0] for _ in range(3)]

    assert accepted == [[20.0, 0.0], [20.0, 0.0], [90.0, 10.0]]
    assert lock._lockball_last_seen == [90.0, 10.0]
    assert len(resets) == 1


def test_lockball_discards_a_single_jump_when_observation_recovers(monkeypatch):
    lock, resets = _lock_with_observations(
        monkeypatch,
        ([20.0, 0.0], [100.0, 0.0], [21.0, 1.0]),
    )

    accepted = [lock._current_or_recent_ball()[0] for _ in range(3)]

    assert accepted == [[20.0, 0.0], [20.0, 0.0], [21.0, 1.0]]
    assert lock._lockball_last_seen == [21.0, 1.0]
    assert resets == []


def _capture_move(monkeypatch, ball_xy):
    lock, _ = _lock_with_observations(monkeypatch, (ball_xy,))
    monkeypatch.setattr(lock, "_reset_on_pose_change", lambda: None)
    moves = []
    monkeypatch.setattr(basic.chassis, "RobotMoveVector", lambda *args: moves.append(args))
    monkeypatch.setattr(basic.peripheral, "Dribble", lambda *_args: None)
    return lock, moves


def test_lockball_brakes_on_curve_close_to_ball(monkeypatch):
    lock, moves = _capture_move(monkeypatch, (12.0, 0.0))
    lock.pan_x.setpoint = 6.0
    lock.pan_x.tunings = (0.1, 0.0, 0.0)
    lock.pan_x.output_limits = (-0.8, 0.8)
    lock.near_dock_decel = 0.2

    assert lock.LockBallSlip() is False
    # 0.1 * (12 - 6) = 0.6 would be commanded without the braking curve;
    # the curve caps it at sqrt(2*0.2*0.06) ~= 0.155.
    assert moves[-1][0] <= math.sqrt(2.0 * 0.2 * 0.06) + 1e-9


def test_lockball_braking_curve_fades_with_distance(monkeypatch):
    ball = [20.0, 0.0]
    monkeypatch.setattr(basic.UnitedPosition, "RobotBallPos", lambda: list(ball))
    lock = basic.LockSeries()
    lock.pan_x.setpoint = 6.0
    lock.pan_x.tunings = (0.025, 0.0, 0.0)
    lock.pan_x.output_limits = (-0.8, 0.8)
    lock.pan_x.sample_time = None
    lock.near_dock_decel = 0.5
    lock.ball_jump_reject_cm = 1000.0
    monkeypatch.setattr(lock, "_reset_on_pose_change", lambda: None)
    moves = []
    monkeypatch.setattr(basic.chassis, "RobotMoveVector", lambda *args: moves.append(args))
    monkeypatch.setattr(basic.peripheral, "Dribble", lambda *_args: None)

    # 12cm: inside the brake zone (starts at 0.8^2/(2*0.5) = 64cm).
    ball[0] = 12.0
    assert lock.LockBallSlip() is False
    near_speed = moves[-1][0]

    # 60cm: near the edge of the brake zone, so it runs much faster.
    ball[0] = 60.0
    lock.pan_x.reset()
    assert lock.LockBallSlip() is False
    far_speed = moves[-1][0]

    assert far_speed > near_speed


def test_lockball_turns_in_place_for_large_front_angle(monkeypatch):
    lock, _ = _lock_with_observations(monkeypatch, ([15.0, 60.0],))
    lock.turn_only_angle = 70.0
    lock.turn_only_kp = 0.75
    lock.turn_only_max = 55.0
    monkeypatch.setattr(lock, "_reset_on_pose_change", lambda: None)
    moves = []
    monkeypatch.setattr(basic.chassis, "RobotMoveVector", lambda *args: moves.append(args))
    monkeypatch.setattr(basic.peripheral, "Dribble", lambda *_args: None)

    assert lock.LockBallSlip() is False
    assert moves == [(0.0, 0.0, 55.0)]


@pytest.mark.parametrize("confirm_frames", [1, 2, 3])
def test_lockball_confirm_frame_setting_is_respected(monkeypatch, confirm_frames):
    lock, resets = _lock_with_observations(
        monkeypatch,
        ([20.0, 0.0], [100.0, 0.0], [90.0, 0.0], [80.0, 0.0]),
    )
    lock.ball_jump_confirm_frames = confirm_frames

    accepted = [lock._current_or_recent_ball()[0] for _ in range(4)]

    assert accepted[0] == [20.0, 0.0]
    expected_target = {
        1: [100.0, 0.0],
        2: [90.0, 0.0],
        3: [80.0, 0.0],
    }[confirm_frames]
    assert accepted[confirm_frames] == expected_target
    assert len(resets) == 1


class _CarryLog:
    def __init__(self):
        self.twists = []
        self.stops = 0
        self.chase = 0

    @property
    def last(self):
        return self.twists[-1]


def _carry_harness(monkeypatch, yaw, raw_ball, lock_result=True):
    """Harness for the raw-coordinate carry flow.

    ``TurnTo_with_ball`` refreshes UnitedPosition.ballPos via RobotBallPos()
    before reading it, so the harness stubs both. ``lock_result`` is what
    LockBallSlip returns when the carry falls back to chasing.
    """
    log = _CarryLog()
    monkeypatch.setattr(basic.UnitedPosition, "RobotBallPos", lambda: list(raw_ball))
    monkeypatch.setattr(basic.UnitedPosition, "ballPos", list(raw_ball), raising=False)

    def lock():
        log.chase += 1
        return lock_result

    monkeypatch.setattr(basic.lockball, "LockBallSlip", lock)
    monkeypatch.setattr(basic.chassis, "GetYaw", lambda: yaw)
    monkeypatch.setattr(
        basic.chassis,
        "RobotMoveTwist",
        lambda vx, vy, wz: log.twists.append((vx, vy, wz)),
    )
    monkeypatch.setattr(basic.chassis, "stop", lambda *a, **k: setattr(log, "stops", log.stops + 1))
    monkeypatch.setattr(basic.peripheral, "Dribble", lambda *_a: None)
    return log


def test_raw_held_ball_xy_rejects_sentinels(monkeypatch):
    monkeypatch.setattr(basic.UnitedPosition, "ballPos", [3549, 3549])
    assert basic._raw_held_ball_xy() is None
    monkeypatch.setattr(basic.UnitedPosition, "ballPos", [4095, 4095])
    assert basic._raw_held_ball_xy() is None
    monkeypatch.setattr(basic.UnitedPosition, "ballPos", [0, 0])
    assert basic._raw_held_ball_xy() is None
    monkeypatch.setattr(basic.UnitedPosition, "ballPos", [6.9, 1.2])
    assert basic._raw_held_ball_xy() == (6.9, 1.2)


def test_turn_with_ball_keeps_chasing_when_not_holding(monkeypatch):
    # Raw coordinate is the no-ball sentinel: fall back to chasing.
    calls = _carry_harness(monkeypatch, 0.0, [4095, 4095], lock_result=False)

    assert basic.TurnTo_with_ball(90) is False
    assert calls.chase == 1
    assert calls.twists == []


def test_turn_with_ball_aborts_carry_when_ball_leaves_envelope(monkeypatch):
    # Ball far to the side: carry hands back to LockBallSlip instead of orbiting.
    calls = _carry_harness(monkeypatch, 0.0, [7.0, 40.0], lock_result=False)

    assert basic.TurnTo_with_ball(90) is False
    assert calls.chase == 1
    assert calls.twists == []


def test_turn_with_ball_orbits_when_ball_seated(monkeypatch):
    calls = _carry_harness(monkeypatch, 0.0, [7.0, 1.2])

    assert basic.TurnTo_with_ball(90) is False
    vx, vy, wz = calls.last
    # Historical wheel mix for a -90 degree error.
    assert (round(vx, 3), round(vy, 3), round(wz, 3)) == (0.03, -0.27, 2.5)


def test_turn_with_ball_stops_when_aligned(monkeypatch):
    calls = _carry_harness(monkeypatch, 90.0, [7.0, 1.2])

    assert basic.TurnTo_with_ball(90) is True
    assert calls.stops == 1
    assert calls.twists == []
    assert calls.chase == 0


def test_turn_with_ball_stop_uses_normalized_angle(monkeypatch):
    # Raw abs(355 - 2) = 353 looks far, but the normalized error is 7 degrees.
    calls = _carry_harness(monkeypatch, 2.0, [7.0, 1.2])

    assert basic.TurnTo_with_ball(355) is True
    assert calls.stops == 1
    assert calls.twists == []


def test_turn_with_ball_orbits_the_other_way(monkeypatch):
    calls = _carry_harness(monkeypatch, 180.0, [7.0, 1.2])

    assert basic.TurnTo_with_ball(90) is False
    _vx, _vy, wz = calls.last
    assert wz < 0


def test_turn_with_ball_speedk_scales_orbit(monkeypatch):
    base = _carry_harness(monkeypatch, 0.0, [7.0, 1.2])
    assert basic.TurnTo_with_ball(90, 1.0) is False
    base_wz = base.last[2]

    fast = _carry_harness(monkeypatch, 0.0, [7.0, 1.2])
    assert basic.TurnTo_with_ball(90, 2.0) is False
    fast_wz = fast.last[2]

    assert fast_wz > base_wz


def test_turn_with_ball_x_hold_pushes_ball_back(monkeypatch):
    monkeypatch.setattr(basic, "TURN_HOLD_X_MPS_PER_CM", 0.05)
    near = _carry_harness(monkeypatch, 0.0, [10.0, 1.0])
    assert basic.TurnTo_with_ball(90) is False
    held_vx = near.last[0]

    monkeypatch.setattr(basic, "TURN_HOLD_X_MPS_PER_CM", 0.0)
    plain = _carry_harness(monkeypatch, 0.0, [10.0, 1.0])
    assert basic.TurnTo_with_ball(90) is False
    plain_vx = plain.last[0]

    # Ball ahead of catch_x=6 must add forward (positive) vx.
    assert held_vx > plain_vx


def test_turn_with_ball_y_hold_pushes_ball_back(monkeypatch):
    monkeypatch.setattr(basic, "TURN_HOLD_Y_MPS_PER_CM", 0.05)
    left = _carry_harness(monkeypatch, 0.0, [7.0, 3.0])
    assert basic.TurnTo_with_ball(90) is False
    left_vy = left.last[1]

    monkeypatch.setattr(basic, "TURN_HOLD_Y_MPS_PER_CM", 0.0)
    plain = _carry_harness(monkeypatch, 0.0, [7.0, 3.0])
    assert basic.TurnTo_with_ball(90) is False
    plain_vy = plain.last[1]

    assert left_vy > plain_vy
