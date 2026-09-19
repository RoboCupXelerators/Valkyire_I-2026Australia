"""Shared-layer update-cadence regression tests (hardware independent).

Derived from the 2026-09-19 bench diagnosis on robot1 (IceLoong + TunaIntelligence)
and the fix that followed it:

- Instrumented runs measured ``RestrictedArea()`` preempting
  ``LockBallSlip()`` in 26308/29129 cycles (LockBallSlip: 0 calls).
- ``LockBallSlip`` is pure-P: identical (pose, ball) input yields an identical
  command; that is expected and NOT a stall.
- The IceLoongBits command loop originally stacked a fixed 20 ms sleep on top
  of four blocking serial transactions (~11-20 ms ack latency each, ~10 %
  50 ms timeouts), throttling the designed 50 Hz cycle to ~9 Hz. The fix
  batches wheel+dribble into one multi-command frame (firmware verified:
  acks ``0x80 0x84 0x86``), polls slow variables at 1 Hz, LINK_GET at 0.5 Hz,
  paces the loop adaptively to ``command_period_s`` and parses short
  (timeout-truncated) responses instead of discarding them.

These tests pin the shared-layer semantics so regressions surface without
hardware. They deliberately never resolve the LunaPre runtime: ``BasicFuc``
module globals ``chassis``/``peripheral`` are replaced with stubs, and ball /
pose feeds are patched on the module-level ``UnitedPosition`` instance.
"""

from __future__ import annotations

import os
import struct
import sys
import threading
import time

import pytest

os.environ.setdefault("ROBOCUPX_BITS", "SIM")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import BasicFuc as basic  # noqa: E402


# ---------------------------------------------------------------- fixtures
@pytest.fixture()
def motion_harness(monkeypatch):
    """Replace BasicFuc module globals with recording stubs (no runtime)."""

    commands: list[tuple[float, float, float]] = []

    class StubChassis:
        def CanMove(self):
            return True

        def RobotMoveVector(self, vx, vy, wz):
            commands.append((float(vx), float(vy), float(wz)))

        def stop(self):
            commands.append((0.0, 0.0, 0.0))

        def FieldTurn(self, angle):
            commands.append((0.0, 0.0, float(angle)))

    class StubPeripheral:
        def Dribble(self, status):
            pass

        def ShootBall(self):
            pass

    monkeypatch.setattr(basic, "chassis", StubChassis())
    monkeypatch.setattr(basic, "peripheral", StubPeripheral())
    return commands


def _feed_ball(monkeypatch, values):
    stream = iter(values)
    monkeypatch.setattr(basic.UnitedPosition, "RobotBallPos", lambda: next(stream))


# ---------------------------------------------------------------- 1. cadence
def test_lockball_command_stream_tracks_a_moving_ball(motion_harness, monkeypatch):
    """A ball approaching at fixed 50 Hz must keep the command stream alive.

    Guards against the reported multi-second output stagnation: while the ball
    coordinate changes every frame, no identical-command run may exceed 0.5 s
    of control time (25 frames at 50 Hz).

    The loop paces itself at the real 50 Hz strategy tick: calling faster than
    simple_pid's sample_time legitimately returns the previous output, which
    must not be confused with a stall.
    """

    lock = basic.LockSeries()
    lock.ball_jump_reject_cm = 1000.0
    monkeypatch.setattr(lock, "_reset_on_pose_change", lambda: None)

    frames = 100  # 2 s at 50 Hz
    # Start inside the proportional band (no actuator saturation) so output
    # changes are attributable to input changes, not to saturation windows.
    start = [20.0, 4.0]
    catch = list(basic.Settings.Motion.CatchVal)
    step = [(catch[0] - start[0]) / frames, (catch[1] - start[1]) / frames]
    observations = [
        [round(start[0] + step[0] * i, 3), round(start[1] + step[1] * i, 3)]
        for i in range(frames)
    ]
    _feed_ball(monkeypatch, observations)

    for _ in range(frames):
        lock.LockBallSlip()
        time.sleep(0.02)  # real strategy tick: 50 Hz

    assert motion_harness, "LockBallSlip must command the chassis every call"
    assert len(motion_harness) == frames

    # Longest run of identical commands (frame units).
    longest = 1
    current = 1
    for prev, now in zip(motion_harness, motion_harness[1:]):
        current = current + 1 if prev == now else 1
        longest = max(longest, current)

    assert longest * 0.02 <= 0.5, (
        f"command stream stagnated: identical output held for "
        f"{longest} frames ({longest * 0.02:.2f}s) while the ball moved"
    )

    # The stream must converge: near the catch point the dock/brake logic
    # fades the forward speed toward zero (creep/brake caps are documented
    # LockBallSlip behaviour), so |vx| must shrink relative to the start.
    assert abs(motion_harness[-1][0]) < abs(motion_harness[0][0]), (
        "forward command must fade as the ball reaches CatchVal"
    )


def test_lockball_constant_scene_yields_constant_output(motion_harness, monkeypatch):
    """Identical ball input producing identical output is P-control, not a stall.

    Pins the semantics so a constant output on a static scene is not mistaken
    for the multi-second update bug.
    """

    lock = basic.LockSeries()
    monkeypatch.setattr(lock, "_reset_on_pose_change", lambda: None)
    _feed_ball(monkeypatch, [[40.0, 15.0]] * 50)

    for _ in range(50):
        lock.LockBallSlip()

    assert len(set(motion_harness)) == 1, (
        "pure-P controller must be deterministic for identical inputs"
    )


# ---------------------------------------------------------------- 2. RA gate
def test_restricted_area_preempts_lockball_while_pose_unsafe(monkeypatch):
    """Document current RestrictedArea semantics observed on the bench.

    With the robot pose inside the restricted area, ``RestrictedArea()``
    drives ``Pos2Pos`` toward the projected safe pose and returns False, so
    the strategy main loop never reaches ``LockBallSlip()``. Ball state is
    irrelevant to this decision. Recorded 2026-09-19: 26308/29129 cycles
    preempted, 0 LockBallSlip calls.
    """

    from LunaPre import UnitedPosition as up

    p2p_calls: list[list] = []

    # Pose firmly inside the goal box (observed bench pose ~[-67, -38]).
    monkeypatch.setattr(up, "FieldRobotPose", lambda: [-67.0, -38.0, 30.0])

    def fake_pos2pos(aim, *args, **kwargs):
        p2p_calls.append(list(aim))
        return False

    monkeypatch.setattr(up, "Pos2Pos", fake_pos2pos)

    results = [up.RestrictedArea() for _ in range(20)]

    assert not any(results), "unsafe pose must make RestrictedArea() return False"
    assert p2p_calls, "RestrictedArea() must drive the escape Pos2Pos itself"
    assert all(
        call[0] > -67.0 or call[1] > -38.0 for call in p2p_calls
    ), "escape target must be projected outside the restricted area"


def test_restricted_area_releases_control_when_pose_safe(monkeypatch):
    """When the projected pose is safe, RestrictedArea() must report True so
    the strategy main loop proceeds to ball chasing."""

    from LunaPre import UnitedPosition as up

    monkeypatch.setattr(up, "FieldRobotPose", lambda: [0.0, 0.0, 0.0])
    p2p_calls: list[list] = []
    monkeypatch.setattr(
        up, "Pos2Pos", lambda aim, *args, **kwargs: p2p_calls.append(list(aim)) or False
    )

    assert up.RestrictedArea() is True
    assert not p2p_calls, "safe pose must not trigger the escape Pos2Pos"


# ---------------------------------------------------------------- 3. serial cadence
def _instrument_bits(monkeypatch):
    """IceLoongBits with a recording serial layer (ack bytes per expected len)."""

    from utils.IceLoongBits import IceLoongBits

    bits = IceLoongBits(testing=True)

    calls: list[tuple[bytes, bool]] = []

    def fake_send_hex(data, read_response=False, response_length=0, response_timeout=None):
        calls.append((bytes(data), bool(read_response)))
        return b"\x80" * max(0, response_length - 1)

    def fake_send_variable_hex(data, response_timeout=0.05):
        calls.append((bytes(data), False))  # LINK_GET variable-length read
        return b""

    monkeypatch.setattr(bits, "SendHex", fake_send_hex)
    monkeypatch.setattr(bits, "_SendVariableHex", fake_send_variable_hex)
    return bits, calls


def test_iceloong_sends_one_wheel_frame_per_cycle(monkeypatch):
    """Wheel + dribble share one multi-command frame; acks are still read.

    Bench probe 2026-09-19: firmware answers ``0x80 0x84 0x86`` for the
    batched frame (20/20 stable, ~11 ms), so the batch keeps full ack
    semantics while halving the blocking transactions per cycle.
    """

    from utils.IceLoongBits import Commands

    bits, calls = _instrument_bits(monkeypatch)
    bits.SetWheelSpeed(10.0, -20.0, 30.0, -40.0)

    cycles = 5
    for _ in range(cycles):
        bits._SendUnitedCommandOnce()

    wheel_frames = [
        data for data, _ in calls
        if len(data) == 24 and data[0] == 0x00 and data[1] == int(Commands.SET_WHEELS_SPEED)
    ]
    assert len(wheel_frames) == cycles, (
        f"expected exactly one batched SET frame per cycle, got {len(wheel_frames)}"
    )

    k = bits.motor_speed_k
    payload = struct.unpack("<ffff", wheel_frames[-1][2:18])
    assert payload == pytest.approx((-10.0 * k, 20.0 * k, 30.0 * k, -40.0 * k), rel=1e-3)
    dribble = struct.unpack("<f", wheel_frames[-1][19:23])
    assert dribble == pytest.approx((0.0,))

    # The batched frame still waits for its acks (fire-and-forget was rejected:
    # ack latency ~ cycle period would interleave stale acks into later reads).
    batched = [rr for data, rr in calls if len(data) == 24]
    assert batched and all(batched), "batched SET frame must read its acks"


def test_iceloong_polls_slow_variables_and_link_at_low_rate(monkeypatch):
    """Dribble current / battery ride the GET group at 1 Hz; LINK_GET at 0.5 Hz."""

    from utils.IceLoongBits import Commands

    bits, calls = _instrument_bits(monkeypatch)

    cycles = 5
    for _ in range(cycles):
        bits._SendUnitedCommandOnce()

    get_frames = [data for data, _ in calls if len(data) >= 4 and data[0] == 0x00 and data[1] == int(Commands.HAND_SHAKE)]
    assert len(get_frames) == cycles, "GET group must be sent every cycle"

    with_slow = [data for data in get_frames if 0x0B in data or 0x0C in data]
    assert len(with_slow) == 1, (
        "dribble current / battery must poll at slow_poll_interval_s (1 Hz), "
        f"seen in {len(with_slow)}/{cycles} cycles"
    )

    link_frames = [data for data, _ in calls if data[:1] == bytes([int(Commands.LINK_GET)])]
    assert len(link_frames) == 1, (
        "LINK_GET must be throttled to link_poll_interval_s (0.5 Hz)"
    )


def test_iceloong_command_loop_holds_design_period(monkeypatch):
    """Adaptive pacing keeps ~20 ms between cycles when transactions are fast.

    The old loop stacked a fixed 20 ms sleep on top of transaction time,
    which measured 72-230 ms per cycle on the bench.
    """

    from utils.IceLoongBits import IceLoongBits

    bits = IceLoongBits(testing=True)
    monkeypatch.setattr(bits, "_ExecuteLinkGet", lambda: [])
    monkeypatch.setattr(bits, "SendHex", lambda *a, **k: b"")

    done = threading.Event()
    state = {"n": 0}

    def fake_once():
        state["n"] += 1
        if state["n"] >= 5:
            done.set()
            bits._stop_thread.set()

    monkeypatch.setattr(bits, "_SendUnitedCommandOnce", fake_once)

    thread = threading.Thread(target=bits._SendUnitedCommandLoop, daemon=True)
    start = time.perf_counter()
    thread.start()
    thread.join(timeout=3.0)

    elapsed = time.perf_counter() - start
    assert done.is_set(), "command loop did not stop after _stop_thread"
    assert elapsed >= 4 * bits.command_period_s, (
        "loop must not busy-spin faster than command_period_s"
    )
    assert elapsed <= 4 * bits.command_period_s + 0.5, (
        f"loop pacing took {elapsed:.2f}s for 5 fast cycles; transactions or "
        "sleep no longer track command_period_s"
    )


def test_execute_commands_parses_truncated_response(monkeypatch):
    """Short reads (response timeouts) keep complete records, drop the tail.

    Bench: ~10 % of multi-command reads timed out and were discarded whole,
    losing GET_POSE for that cycle. The parser now walks records by id and
    stops cleanly at a truncated tail.
    """

    from utils.IceLoongBits import Commands, IceLoongBits

    bits = IceLoongBits(testing=True)

    def fake_send_hex(data, read_response=False, response_length=0, response_timeout=None):
        # Complete response would be 0x80 + 0x81 + 0x83 + 24B + 0x87 0x00.
        return bytes([0x80, 0x81, 0x83]) + b"\x00" * 7  # truncated GET_POSE

    monkeypatch.setattr(bits, "SendHex", fake_send_hex)

    rx = bits.ExecuteCommands([(Commands.HAND_SHAKE,), (Commands.GET_POSE,)])
    assert rx == [(0x81,)], "complete HAND_SHAKE record must survive a truncated tail"

    def garbage_send_hex(*a, **k):
        return b"\xab"

    monkeypatch.setattr(bits, "SendHex", garbage_send_hex)
    assert bits.ExecuteCommands([(Commands.HAND_SHAKE,)]) == [], (
        "unknown record ids must stop parsing without raising"
    )
