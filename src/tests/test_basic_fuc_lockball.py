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
