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


def test_adaptive_dribbler_layer_is_removed():
    import importlib

    with pytest.raises(ModuleNotFoundError):
        importlib.import_module("LunaPre.motion.dribble_adaptive")
    assert not hasattr(basic.peripheral, "DribbleAdaptive")
    assert not hasattr(basic.peripheral, "GetDribbleAdaptiveTelemetry")


def _lock_with_observations(monkeypatch, observations, loaded=False):
    values = iter(observations)
    monkeypatch.setattr(basic.UnitedPosition, "RobotBallPos", lambda: next(values))
    # loaded=False -> 盘球未受载，按“追球”处理；True -> 电流确认咬住，直接判捕获。
    monkeypatch.setattr(basic.UnitedPosition, "BallLoadInfo", lambda: {"loaded": loaded})

    lock = basic.LockSeries()
    lock.ball_jump_reject_cm = 45.0
    lock.ball_jump_confirm_frames = 2
    return lock


def _clamp(value, limit):
    return max(-limit, min(limit, value))


def _capture_move(monkeypatch, ball_xy):
    lock = _lock_with_observations(monkeypatch, (ball_xy,))
    monkeypatch.setattr(lock, "_reset_on_pose_change", lambda: None)
    moves = []
    monkeypatch.setattr(
        basic.chassis, "RobotMoveVector", lambda *args, **_kwargs: moves.append(args)
    )
    monkeypatch.setattr(basic.peripheral, "Dribble", lambda *_args: None)
    return lock, moves


def test_lockball_uses_piecewise_gain_table(monkeypatch):
    # 远 / 中 / 近三档，按“最远轴距离”选档；前进分量额外乘 FORWARD_BOOST。
    for ball_x, (gain, _yaw_gain) in (
        (100.0, basic.LOCKBALL_GAIN_TABLE[0][1:]),
        (40.0, basic.LOCKBALL_GAIN_TABLE[1][1:]),
        (10.0, basic.LOCKBALL_GAIN_TABLE[2][1:]),
    ):
        lock, moves = _capture_move(monkeypatch, [ball_x, 0.0])
        assert lock.LockBallSlip() is False
        vx, _vy, wz = moves[-1]
        cap = (
            basic.LOCKBALL_DOCK_MAX_MPS
            if ball_x <= basic.LOCKBALL_DOCK_CM
            else basic.LOCKBALL_MAX_FORWARD_MPS
        )
        expected = _clamp(
            ball_x * gain * basic.LOCKBALL_FORWARD_BOOST,
            cap,
        )
        assert vx == pytest.approx(expected)
        assert wz == pytest.approx(0.0)


def test_lockball_clamps_commands(monkeypatch):
    lock, moves = _capture_move(monkeypatch, [500.0, 500.0])
    assert lock.LockBallSlip() is False
    vx, vy, wz = moves[-1]
    assert abs(vx) <= basic.LOCKBALL_MAX_FORWARD_MPS + 1e-9
    assert abs(vy) <= basic.LOCKBALL_MAX_LATERAL_MPS + 1e-9
    assert abs(wz) <= basic.LOCKBALL_MAX_YAW_DEGPS + 1e-9


def test_lockball_turns_toward_rear_ball_before_charging(monkeypatch):
    # 球在身后：先原地转对准，不盲目倒车冲。
    lock, moves = _capture_move(monkeypatch, [-20.0, 0.0])
    assert lock.LockBallSlip() is False
    vx, _vy, wz = moves[-1]
    assert vx == pytest.approx(0.0)
    assert abs(wz) > 0.0


def test_lockball_accepts_a_persistent_jump(monkeypatch):
    lock = _lock_with_observations(
        monkeypatch,
        ([20.0, 0.0], [100.0, 0.0], [90.0, 10.0]),
    )

    accepted = [lock._current_or_recent_ball()[0] for _ in range(3)]

    assert accepted == [[20.0, 0.0], [20.0, 0.0], [90.0, 10.0]]
    assert lock._lockball_last_seen == [90.0, 10.0]


def test_lockball_discards_a_single_jump_when_observation_recovers(monkeypatch):
    lock = _lock_with_observations(
        monkeypatch,
        ([20.0, 0.0], [100.0, 0.0], [21.0, 1.0]),
    )

    accepted = [lock._current_or_recent_ball()[0] for _ in range(3)]

    assert accepted == [[20.0, 0.0], [20.0, 0.0], [21.0, 1.0]]
    assert lock._lockball_last_seen == [21.0, 1.0]


def test_lockball_uses_fixed_dribble_powers(monkeypatch):
    dribbles = []
    monkeypatch.setattr(basic.peripheral, "Dribble", lambda value: dribbles.append(value))
    monkeypatch.setattr(basic.chassis, "CanMove", lambda: True)
    monkeypatch.setattr(basic.chassis, "stop", lambda *args, **kwargs: None)

    lock = _lock_with_observations(monkeypatch, (basic.BALL_HELD,), loaded=True)
    monkeypatch.setattr(lock, "_reset_on_pose_change", lambda: None)
    assert lock.LockBallSlip() is True
    assert dribbles[-1] == basic.LOCKBALL_DRIBBLE_FAR_POWER

    lock_lost = _lock_with_observations(monkeypatch, (basic.BALL_NOT_FOUND,))
    monkeypatch.setattr(lock_lost, "_reset_on_pose_change", lambda: None)
    assert lock_lost.LockBallSlip() is False
    assert dribbles[-1] == basic.LOCKBALL_DRIBBLE_LOST_POWER


@pytest.mark.parametrize("confirm_frames", [1, 2, 3])
def test_lockball_confirm_frame_setting_is_respected(monkeypatch, confirm_frames):
    lock = _lock_with_observations(
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
    # 抓住后必须停车（只在捕获沿停一次），否则车会一直往前蹭把球推走。
    stops = []
    monkeypatch.setattr(basic.UnitedPosition, "RobotBallPos", lambda: basic.BALL_HELD)
    monkeypatch.setattr(basic.UnitedPosition, "BallLoadInfo", lambda: {"loaded": True})
    monkeypatch.setattr(basic.chassis, "stop", lambda *a, **k: stops.append(True))
    monkeypatch.setattr(basic.peripheral, "Dribble", lambda *_a: None)
    monkeypatch.setattr(basic.chassis, "CanMove", lambda: True)

    lock = basic.LockSeries()
    assert lock.LockBallSlip() is True
    assert len(stops) == 1

    assert lock.LockBallSlip() is True
    assert len(stops) == 1


def test_lockball_recatch_stops_again(monkeypatch):
    # 丢球后再次抓住，会重新触发一次捕获沿停车。
    stops = []
    observations = iter([basic.BALL_HELD, [30.0, 0.0], basic.BALL_HELD])
    state = {"ball": None}

    def next_ball():
        state["ball"] = next(observations)
        return state["ball"]

    monkeypatch.setattr(basic.UnitedPosition, "RobotBallPos", next_ball)
    # 只有 [0xddd,0xddd] 哨兵才算“电流咬住”，自由球坐标不算。
    monkeypatch.setattr(
        basic.UnitedPosition, "BallLoadInfo",
        lambda: {"loaded": state["ball"] == basic.BALL_HELD},
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
