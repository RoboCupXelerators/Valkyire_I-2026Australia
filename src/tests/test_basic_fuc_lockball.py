"""LockBallSlip (7月最简版) 与 TurnTo_with_ball/carry 回归测试。"""

from __future__ import annotations

import os
import sys

import pytest

os.environ.setdefault("ROBOCUPX_BITS", "SIM")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import BasicFuc as basic  # noqa: E402


def _lock_with_ball(monkeypatch, ball, status):
    monkeypatch.setattr(basic.UnitedPosition, "RobotBallPos", lambda: ball)
    monkeypatch.setattr(basic.UnitedPosition, "BallStatus", lambda *_a, **_k: status)
    lock = basic.LockSeries()
    monkeypatch.setattr(lock, "_reset_lockball_pid", lambda: None)
    return lock


def test_held_ball_stops_and_returns_true(monkeypatch):
    stops = []
    monkeypatch.setattr(basic.chassis, "stop", lambda *a, **k: stops.append(True))
    monkeypatch.setattr(basic.peripheral, "Dribble", lambda *_a: None)
    lock = _lock_with_ball(monkeypatch, [6.9, 0.0], "held")
    assert lock.LockBallSlip() is True
    assert stops == [True]


def test_lost_ball_stops_and_returns_false(monkeypatch):
    stops = []
    monkeypatch.setattr(basic.chassis, "stop", lambda *a, **k: stops.append(True))
    monkeypatch.setattr(basic.peripheral, "Dribble", lambda *_a: None)
    lock = _lock_with_ball(monkeypatch, basic.BALL_NOT_FOUND, "lost")
    assert lock.LockBallSlip() is False
    assert stops == [True]


def test_free_ball_drives_forward(monkeypatch):
    moves = []
    monkeypatch.setattr(basic.peripheral, "Dribble", lambda *_a: None)
    monkeypatch.setattr(
        basic.chassis, "RobotMoveVector",
        lambda vx, vy, wz, **k: moves.append((vx, vy, wz)),
    )
    lock = _lock_with_ball(monkeypatch, [30.0, 0.0], "free")
    assert lock.LockBallSlip() is False
    vx, _vy, _wz = moves[-1]
    assert vx > 0.0


def test_rear_ball_does_not_charge(monkeypatch):
    moves = []
    monkeypatch.setattr(basic.peripheral, "Dribble", lambda *_a: None)
    monkeypatch.setattr(
        basic.chassis, "RobotMoveVector",
        lambda vx, vy, wz, **k: moves.append((vx, vy, wz)),
    )
    lock = _lock_with_ball(monkeypatch, [-20.0, 0.0], "free")
    assert lock.LockBallSlip() is False
    vx, _vy, _wz = moves[-1]
    assert vx == 0.0


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


def test_lockball_holds_stop_every_tick(monkeypatch):
    # 7 月最简版：只要判为持球就停车并返回 True（无沿触发语义）。
    stops = []
    monkeypatch.setattr(basic.UnitedPosition, "RobotBallPos", lambda: basic.BALL_HELD)
    monkeypatch.setattr(basic.UnitedPosition, "BallStatus", lambda *_a, **_k: "held")
    monkeypatch.setattr(basic.chassis, "stop", lambda *a, **k: stops.append(True))
    monkeypatch.setattr(basic.peripheral, "Dribble", lambda *_a: None)

    lock = basic.LockSeries()
    assert lock.LockBallSlip() is True
    assert lock.LockBallSlip() is True
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
