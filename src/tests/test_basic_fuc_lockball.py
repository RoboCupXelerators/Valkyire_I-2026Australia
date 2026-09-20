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


def test_lockball_uses_source_owned_pid_terms_and_output_limits(monkeypatch):
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

    # Old deployments may retain a LockBallSlip section in config, but motion
    # PID calibration is intentionally source-owned so it cannot drift at
    # runtime between robots.
    assert lock.pan_x.tunings == (
        basic.LOCKBALL_FORWARD_KP,
        basic.LOCKBALL_FORWARD_KI,
        basic.LOCKBALL_FORWARD_KD,
    )
    assert lock.pan_y.tunings == (
        basic.LOCKBALL_LATERAL_KP,
        basic.LOCKBALL_LATERAL_KI,
        basic.LOCKBALL_LATERAL_KD,
    )
    assert lock.pan_w.tunings == (
        basic.LOCKBALL_ANGULAR_KP,
        basic.LOCKBALL_ANGULAR_KI,
        basic.LOCKBALL_ANGULAR_KD,
    )
    assert lock.pan_x.output_limits == (-basic.LOCKBALL_FORWARD_MAX_MPS, 0)
    assert lock.pan_y.output_limits == (
        -basic.LOCKBALL_LATERAL_MAX_MPS,
        basic.LOCKBALL_LATERAL_MAX_MPS,
    )
    assert lock.pan_w.output_limits == (
        -basic.LOCKBALL_ANGULAR_MAX_DEGPS,
        basic.LOCKBALL_ANGULAR_MAX_DEGPS,
    )


def test_lockball_uses_source_owned_near_dock_envelope(monkeypatch):
    tuning = SimpleNamespace(
        NearDockDecelMps2=0.6,
    )
    monkeypatch.setattr(basic.Settings.Motion, "LockBallSlip", tuning)

    lock = basic.LockSeries()

    assert lock.near_dock_decel == basic.LOCKBALL_NEAR_DOCK_DECEL_MPS2


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
    monkeypatch.setattr(
        basic.chassis, "RobotMoveVector", lambda *args, **_kwargs: moves.append(args)
    )
    monkeypatch.setattr(basic.peripheral, "Dribble", lambda *_args: None)
    return lock, moves


def test_lockball_passes_semantic_vision_state_to_adaptive_dribbler(monkeypatch):
    lock, _ = _lock_with_observations(monkeypatch, (basic.BALL_HELD,))
    monkeypatch.setattr(lock, "_reset_on_pose_change", lambda: None)
    calls = []
    monkeypatch.setattr(
        basic.peripheral,
        "DribbleAdaptive",
        lambda power, vision_state=None, lateral_error_cm=None: calls.append(
            (power, vision_state, lateral_error_cm)
        ),
        raising=False,
    )
    monkeypatch.setattr(basic.chassis, "stop", lambda: None)

    assert lock.LockBallSlip() is True
    assert calls == [(0.3, "held", None)]


def test_lockball_uses_near_state_only_inside_capture_envelope(monkeypatch):
    lock, _ = _lock_with_observations(monkeypatch, ([15.0, 0.0],))
    monkeypatch.setattr(lock, "_reset_on_pose_change", lambda: None)
    lock.catch_x = 6.0
    lock.catch_y = 0.0
    calls = []
    monkeypatch.setattr(
        basic.peripheral,
        "DribbleAdaptive",
        lambda power, vision_state=None, lateral_error_cm=None: calls.append(
            (power, vision_state, lateral_error_cm)
        ),
        raising=False,
    )
    monkeypatch.setattr(basic.chassis, "RobotMoveVector", lambda *_args, **_kwargs: None)

    assert lock.LockBallSlip() is False
    assert calls == [(basic.LOCKBALL_DRIBBLE_FAR_POWER, "near", 0.0)]


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
    monkeypatch.setattr(
        basic.chassis, "RobotMoveVector", lambda *args, **_kwargs: moves.append(args)
    )
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


def test_lockball_uses_wheel_bypass_only_for_aligned_far_chase(monkeypatch):
    # A 94cm straight, front-side error saturates the new pursuit cap.  This
    # is the only LockBallSlip geometry permitted to exercise the temporary
    # wheel-limit debug route.
    lock, _ = _lock_with_observations(monkeypatch, ([100.0, 1.0],))
    lock.catch_x = 6.0
    lock.catch_y = 1.0
    lock.pan_x.setpoint = lock.catch_x
    lock.pan_y.setpoint = lock.catch_y
    lock.pan_x.sample_time = None
    monkeypatch.setattr(lock, "_reset_on_pose_change", lambda: None)
    monkeypatch.setattr(
        basic.peripheral, "DribbleAdaptive", lambda *_args, **_kwargs: None,
        raising=False,
    )
    commands = []
    monkeypatch.setattr(
        basic.chassis,
        "RobotMoveVector",
        lambda *args, **kwargs: commands.append((args, kwargs)),
    )

    assert lock.LockBallSlip() is False
    args, kwargs = commands[-1]
    assert args[0] == pytest.approx(basic.LOCKBALL_FORWARD_MAX_MPS)
    assert kwargs == {"bypass_wheel_limits": True}


def test_lockball_turns_in_place_for_large_front_angle(monkeypatch):
    lock, _ = _lock_with_observations(monkeypatch, ([15.0, 60.0],))
    lock.turn_only_angle = 70.0
    lock.turn_only_kp = 0.75
    lock.turn_only_max = 55.0
    monkeypatch.setattr(lock, "_reset_on_pose_change", lambda: None)
    moves = []
    monkeypatch.setattr(
        basic.chassis, "RobotMoveVector", lambda *args, **_kwargs: moves.append(args)
    )
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


def _turn_harness(monkeypatch, yaw, holding=True, raw_ball=None):
    """Stub the chassis for one TurnTo_with_ball tick.

    ``raw_ball`` feeds the raw-coordinate verification merged from feat
    23f2bd5: a seated reading by default when holding, and the no-ball
    sentinel when not, so upstream expectations keep holding. Pass an
    explicit raw_ball to pin the raw-vs-semantic interaction.
    """
    if raw_ball is None:
        raw_ball = [7.0, 1.2] if holding else [4095, 4095]
    calls = []
    monkeypatch.setattr(basic.lockball, "LockBallSlip", lambda: holding)
    monkeypatch.setattr(basic.UnitedPosition, "RobotBallPos", lambda: list(raw_ball))
    monkeypatch.setattr(basic.UnitedPosition, "ballPos", list(raw_ball), raising=False)
    monkeypatch.setattr(basic.chassis, "GetYaw", lambda: yaw)
    monkeypatch.setattr(
        basic.chassis,
        "RobotMoveTwist",
        lambda vx, vy, wz: calls.append(("twist", vx, vy, wz)),
    )
    monkeypatch.setattr(
        basic.chassis,
        "RobotMoveVector",
        lambda vx, vy, wz: calls.append(("vector", vx, vy, wz)),
    )
    monkeypatch.setattr(basic.chassis, "stop", lambda *a, **k: calls.append(("stop",)))
    monkeypatch.setattr(basic.peripheral, "Dribble", lambda *_a: None)
    # The aligned-stop latch is module state; every test starts a fresh skill.
    basic._turnto_stop_latched = False
    return calls


def test_turn_with_ball_keeps_chasing_when_not_holding(monkeypatch):
    calls = _turn_harness(monkeypatch, 0.0, holding=False)

    # Must not report success or command an orbit before the ball is held.
    assert basic.TurnTo_with_ball(90) is False
    assert calls == []


def test_turn_with_ball_stops_when_aligned(monkeypatch):
    calls = _turn_harness(monkeypatch, 90.0, holding=True)

    assert basic.TurnTo_with_ball(90) is True
    assert calls == [("stop",)]


def test_turn_with_ball_stop_is_edge_triggered(monkeypatch):
    # The stop command latches on the backend: consecutive aligned ticks must
    # not resend it (it would reset the command filter and spam the bus).
    calls = _turn_harness(monkeypatch, 90.0, holding=True)

    assert basic.TurnTo_with_ball(90) is True
    assert basic.TurnTo_with_ball(90) is True
    assert calls == [("stop",)]


def test_turn_with_ball_stops_again_after_orbit(monkeypatch):
    calls = _turn_harness(monkeypatch, 90.0, holding=True)

    assert basic.TurnTo_with_ball(90) is True
    assert calls == [("stop",)]

    monkeypatch.setattr(basic.chassis, "GetYaw", lambda: 40.0)
    assert basic.TurnTo_with_ball(90) is False
    assert calls[-1][0] == "twist"

    monkeypatch.setattr(basic.chassis, "GetYaw", lambda: 89.0)
    assert basic.TurnTo_with_ball(90) is True
    assert calls[-1] == ("stop",)


def test_turn_with_ball_stop_uses_normalized_angle(monkeypatch):
    # Raw abs(355 - 2) = 353 looks far, but the normalized error is 7 degrees.
    calls = _turn_harness(monkeypatch, 2.0, holding=True)

    assert basic.TurnTo_with_ball(355) is True
    assert calls == [("stop",)]


def test_turn_with_ball_orbits_toward_aim(monkeypatch):
    calls = _turn_harness(monkeypatch, 0.0, holding=True)

    assert basic.TurnTo_with_ball(90) is False
    kind, vx, vy, wz = calls[-1]
    assert kind == "twist"
    assert wz > 0  # counter-clockwise toward +90
    # Historical wheel mix for a -90 degree error.
    assert (round(vx, 3), round(vy, 3), round(wz, 3)) == (0.03, -0.27, 2.5)


def test_turn_with_ball_orbits_the_other_way(monkeypatch):
    calls = _turn_harness(monkeypatch, 180.0, holding=True)

    assert basic.TurnTo_with_ball(90) is False
    _, _vx, _vy, wz = calls[-1]
    assert wz < 0


def test_turn_with_ball_speedk_scales_orbit(monkeypatch):
    base = _turn_harness(monkeypatch, 0.0, holding=True)
    assert basic.TurnTo_with_ball(90, 1.0) is False
    base_wz = base[-1][3]

    fast = _turn_harness(monkeypatch, 0.0, holding=True)
    assert basic.TurnTo_with_ball(90, 2.0) is False
    fast_wz = fast[-1][3]

    assert fast_wz > base_wz


def test_lockball_held_stop_is_edge_triggered(monkeypatch):
    # While the ball is held, the capture-edge stop must fire exactly once;
    # callers such as TurnTo_with_ball command their own motion afterwards.
    stops = []
    monkeypatch.setattr(basic.UnitedPosition, "RobotBallPos", lambda: basic.BALL_HELD)
    monkeypatch.setattr(basic.chassis, "stop", lambda *a, **k: stops.append(True))
    monkeypatch.setattr(basic.peripheral, "Dribble", lambda *_a: None)
    monkeypatch.setattr(basic.chassis, "CanMove", lambda: True)

    lock = basic.LockSeries()
    assert lock.LockBallSlip() is True
    assert len(stops) == 1

    assert lock.LockBallSlip() is True
    assert len(stops) == 1


def test_lockball_recatch_stops_again(monkeypatch):
    # Losing and re-catching the ball must re-arm the capture-edge stop.
    stops = []
    observations = iter([basic.BALL_HELD, [30.0, 0.0], basic.BALL_HELD])
    monkeypatch.setattr(
        basic.UnitedPosition, "RobotBallPos", lambda: next(observations)
    )
    monkeypatch.setattr(basic.chassis, "stop", lambda *a, **k: stops.append(True))
    monkeypatch.setattr(basic.chassis, "RobotMoveVector", lambda *a, **k: None)
    monkeypatch.setattr(basic.peripheral, "Dribble", lambda *_a: None)
    monkeypatch.setattr(basic.chassis, "CanMove", lambda: True)

    lock = basic.LockSeries()
    assert lock.LockBallSlip() is True
    assert len(stops) == 1

    assert lock.LockBallSlip() is False  # free ball -> chase, no capture stop
    assert len(stops) == 1

    assert lock.LockBallSlip() is True  # re-caught -> one more edge stop
    assert len(stops) == 2


# ---- raw-coordinate carry loop (merged back from feat 23f2bd5) ----
# Upstream's TurnTo_with_ball suite above runs on _turn_harness; the
# tests below pin the merged raw-reading verification and the (currently
# zero-gain) orbit position hold on the local _carry_harness.

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
