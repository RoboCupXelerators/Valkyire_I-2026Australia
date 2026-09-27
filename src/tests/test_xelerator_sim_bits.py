"""XeleratorSimBits simulator semantics tests."""

from __future__ import annotations

import os
import sys
import threading
import math
import queue
import socket
import struct
import time

import pytest

os.environ.setdefault("ROBOCUPX_BITS", "SIM")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from utils.XeleratorSimBits import SimStatus, XeleratorSimBits  # noqa: E402
from vision_utils.XeleratorVision import XeleratorVision  # noqa: E402
from LunaPre.positions import Positions, cfg as positions_cfg  # noqa: E402
from LunaPre import runtime  # noqa: E402


def _catch_val():
    catch_val = positions_cfg.Motion.CatchVal
    return [float(catch_val[0]), float(catch_val[1])]


def _make_bits_for_possession(
    ball_x_m: float | None,
    ball_y_m: float | None,
    *,
    dribble: float,
    has_ball: bool = False,
) -> XeleratorSimBits:
    """构造不启动 UDP 线程的最小 SIM Bits 对象。"""
    bits = object.__new__(XeleratorSimBits)
    bits._status = SimStatus(ball_x_m=ball_x_m, ball_y_m=ball_y_m, has_ball=has_ball)
    bits._status_lock = threading.Lock()
    bits._unity_to_cm = 100.0
    bits._pose_scale = 1.0
    bits._dribble_strength = dribble
    bits._ball_status_fallback = False
    bits._ball_observation_min_cm = 0.25
    bits._ball_observation_max_cm = 350.0
    bits._infer_dribbler_possession = True
    bits._infer_dribbler_min_strength = 0.2
    bits._infer_dribbler_x_min_cm = -2.0
    bits._infer_dribbler_x_max_cm = 22.0
    bits._infer_dribbler_y_abs_cm = 10.0
    bits._vision_noise_level = 0.0
    bits._vision_rng = None
    bits._held_ball_pos_cm = _catch_val()
    bits._wheel_delivery_order = ("LF", "LB", "RF", "RB")
    return bits


def _make_bits_for_kick():
    bits = object.__new__(XeleratorSimBits)
    bits._status_lock = threading.Lock()
    bits._flags = 0
    bits._kick_max_strength = 8.0
    bits._kick_strength = 8.0
    bits.kick_power_level = 4
    return bits


def _make_bits_for_pose(status: SimStatus) -> XeleratorSimBits:
    bits = object.__new__(XeleratorSimBits)
    bits._status = status
    bits._status_lock = threading.Lock()
    bits._unity_to_cm = 100.0
    bits._origin_x_cm = 0.0
    bits._origin_y_cm = 0.0
    bits._pose_scale = 1.0
    bits._yaw_zero_at_start = False
    bits._yaw_zero_deg = None
    bits._yaw_offset_deg = 0.0
    bits._side_yaw_offset_deg = 0.0
    bits._pose_max_age_s = 1.0
    return bits


def test_sim_pose_calibrates_origin_from_running_scene():
    bits = _make_bits_for_pose(SimStatus(ts=time.time(), x_m=4.2, y_m=-1.7, yaw_deg=33.0))
    bits.robot_id = 1
    bits._origin_x_cm = 1000.0
    bits._origin_y_cm = -800.0
    bits._origin_is_explicit = False
    bits._scene_origin_calibrated = False
    bits._scene_origin_next_attempt_ts = 0.0
    bits._scene_status_snapshot = lambda: {
        "ok": True,
        "robots": [{"id": 1, "active": True, "x_cm": 20.0, "y_cm": -30.0, "pose_revision": 7}],
    }

    bits._calibrate_origin_from_scene(420.0, -170.0)

    assert bits.GetPose() == (20.0, -30.0, 33.0)
    assert (bits._origin_x_cm, bits._origin_y_cm) == (400.0, -140.0)


def test_sim_pose_keeps_explicit_origin_offset():
    bits = _make_bits_for_pose(SimStatus(ts=time.time(), x_m=4.2, y_m=-1.7, yaw_deg=33.0))
    bits.robot_id = 1
    bits._origin_x_cm = 100.0
    bits._origin_y_cm = -80.0
    bits._origin_is_explicit = True
    bits._scene_status_snapshot = lambda: {
        "ok": True,
        "robots": [{"id": 1, "active": True, "x_cm": 20.0, "y_cm": -30.0, "pose_revision": 7}],
    }

    assert bits.GetPose() == (320.0, -90.0, 33.0)
    assert (bits._origin_x_cm, bits._origin_y_cm) == (100.0, -80.0)


@pytest.mark.parametrize("power_level, multiplier", [(1, 0.5), (2, 2 / 3), (3, 5 / 6), (4, 1.0)])
def test_sim_kick_uses_selected_fraction_of_configured_maximum(power_level, multiplier):
    bits = _make_bits_for_kick()

    assert bits.SetKickPowerLevel(power_level) == power_level
    assert bits.Kick() is True
    # A legacy sender multiplier must not multiply Unity's configured maximum.
    assert bits._kick_strength == pytest.approx(multiplier)
    assert bits._kick_strength * 6.0 == pytest.approx(power_level + 2.0)
    assert bits._flags & bits.FLAG_KICK


@pytest.mark.parametrize("power_level", [0, 5, 1.5, True, None, math.inf])
def test_sim_kick_rejects_invalid_power_level(power_level):
    bits = _make_bits_for_kick()

    with pytest.raises(ValueError, match="integer from 1 to 4"):
        bits.SetKickPowerLevel(power_level)


def test_sim_virtual_kick_port_uses_selected_default_level():
    bits = _make_bits_for_kick()
    bits.SetKickPowerLevel(2)

    bits.VirtualPorts(3, 0)

    assert bits._kick_strength == pytest.approx(2 / 3)
    assert bits._flags & bits.FLAG_KICK


def _robot_ball_pos_for_observation(observation):
    position = object.__new__(Positions)
    position._get_ball_pos = lambda: observation
    position._robot_ball_out_of_bounds = lambda ball_pos: False
    position.ballPos = None
    position.ballPosOut = None
    vision = runtime.vision
    try:
        runtime.vision = None
        ball = Positions.RobotBallPos(position)
        return ball, Positions.BallStatus(position, ball)
    finally:
        runtime.vision = vision


@pytest.mark.parametrize("ball,dribble,has_ball", [
    pytest.param(tuple(v / 100 for v in _catch_val()), 0.7, False, id="inferred-contact"),
    pytest.param((None, None), 0.0, True, id="status-without-ball-extension"),
    pytest.param((0.0987, 0.004), 0.7, True, id="shifted-contact"),
])
def test_sim_possession_reaches_lunapre_as_held(ball, dribble, has_ball):
    bits = _make_bits_for_possession(*ball, dribble=dribble, has_ball=has_ball)

    assert bits.HasBall() is True
    observation = bits.GetBallPos()
    assert observation == _catch_val()
    assert _robot_ball_pos_for_observation(observation) == (_catch_val(), "held")


@pytest.mark.parametrize("observation,has_ball,expected,state", [
    ([9.5, 0.0], False, [9.5, 0.0], "free"),
    ([12.0, 1.0], True, [12.0, 1.0], "held"),
])
def test_sim_provider_possession_controls_lunapre_ball_state(
    monkeypatch, observation, has_ball, expected, state,
):
    class FakeVision:
        def get_ball_pos(self):
            return observation

        def has_ball(self):
            return has_ball

    position = object.__new__(Positions)
    position._robot_ball_out_of_bounds = lambda ball_pos: False
    position.ballPos = None
    position.ballPosOut = None
    monkeypatch.setattr(runtime, "vision", FakeVision())

    assert Positions.RobotBallPos(position) == expected
    assert Positions.BallState(position) == state


def test_sim_bits_reconstructs_pure_strafe_from_default_wheel_order():
    bits = object.__new__(XeleratorSimBits)
    bits._twist_from_wheels = True
    bits._wheel_to_v = 1.0
    bits._wheel_to_wz = 1.0
    bits._planar_move_scale = 1.0
    bits._flags = 0
    bits._wheel_delivery_order = ("LF", "LB", "RF", "RB")

    bits.SetWheelSpeed(-1.0, 1.0, -1.0, 1.0)

    assert bits._vx == 0.0
    assert bits._vy == 1.0
    assert bits._wz == 0.0


@pytest.mark.parametrize("dribble", [0.7, 0.0], ids=["recovery-window", "dribbler-off"])
def test_sim_bits_keeps_free_ball_outside_catch_window(dribble):
    bits = _make_bits_for_possession(0.12, 0.01, dribble=dribble)

    assert bits.HasBall() is False
    assert bits.GetBallPos() == [12.0, 1.0]


def test_sim_bits_trusts_fresh_explicit_free_possession():
    bits = _make_bits_for_possession(0.095, 0.0, dribble=0.7)
    bits._status.possession_authoritative = True

    assert bits.HasBall() is False
    assert bits.GetBallPos() == [9.5, 0.0]


def test_sim_bits_uninitialized_fallback_flags_default_to_disabled():
    bits = _make_bits_for_possession(0.12, 0.01, dribble=0.0)
    del bits._ball_status_fallback
    bits._get_scene_ball_pos_fallback = lambda: pytest.fail("scene fallback must remain disabled")

    assert not hasattr(bits, "_prefer_scene_ball_fallback")
    assert bits.GetBallPos() == [12.0, 1.0]


def test_xelerator_vision_exposes_cam_ports_without_frame_thread(monkeypatch):
    monkeypatch.setenv("XEL_CAM_DISABLE_FRAMES", "1")
    monkeypatch.setenv("XEL_CAM_PORTS", "10,11,12,13")

    vision = XeleratorVision()
    try:
        assert vision.CamPorts == [10, 11, 12, 13]
        assert len(vision.Frames) == 4
        assert vision._frame_thread is None
    finally:
        vision.close(join_timeout=0.1)


@pytest.mark.camera
def test_united_vision_xv_constructs_without_camera_stream(monkeypatch):
    monkeypatch.setenv("XEL_CAM_DISABLE_FRAMES", "1")

    from vision_utils.Vision import UnitedVision  # noqa: E402

    vision = UnitedVision()
    try:
        assert vision.CamPorts == [0, 2, 4, 6]
        assert len(vision.Frames) == 4
        assert vision._frame_thread is None
    finally:
        vision.close(join_timeout=0.1)


def test_xelerator_vision_normalizes_upstream_sentinels_to_empty_observation(monkeypatch):
    monkeypatch.setenv("XEL_CAM_DISABLE_FRAMES", "1")

    class FakeBits:
        def __init__(self):
            self.values = iter([
                (0xFFF, 0xFFF),
                ["4095", "4095"],
                [0xFFF, 12.0],
                (0xDDD, 0xDDD),
                ["12.5", "-3"],
            ])

        def GetBallPos(self):
            return next(self.values)

    vision = XeleratorVision(FakeBits())
    try:
        assert vision.get_ball_pos() == [0, 0]
        assert vision.get_ball_pos() == [0, 0]
        assert vision.get_ball_pos() == [0, 0]
        assert vision.get_ball_pos() == [0, 0]
        assert vision.get_ball_pos() == [12.5, -3.0]
    finally:
        vision.close(join_timeout=0.1)


def _make_message_bits(robot_id, command_port):
    bits = object.__new__(XeleratorSimBits)
    bits.robot_id = robot_id
    bits.unity_ip = "127.0.0.1"
    bits.cmd_port = command_port
    bits._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    bits._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    bits._rx.bind(("127.0.0.1", 0))
    bits._rx.settimeout(0.05)
    bits.status_port = bits._rx.getsockname()[1]
    bits._running = True
    bits._messages = queue.Queue(maxsize=256)
    bits._rx_thread = threading.Thread(target=bits._rx_loop, daemon=True)
    bits._rx_thread.start()
    return bits


def test_bidirectional_udp_envelope_and_opaque_payload():
    # A socket stands in for Unity to inspect the wire envelope and forward it unchanged.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as relay:
        relay.bind(("127.0.0.1", 0))
        relay.settimeout(1)
        a = _make_message_bits(1, relay.getsockname()[1])
        b = _make_message_bits(2, relay.getsockname()[1])
        try:
            for sender, target, payload in [(a, b, bytes(range(256))),
                                            (b, a, "你好\x00#B\n"),
                                            (a, b, b""), (b, a, b"x" * 4096)]:
                raw = payload.encode("utf-8") if isinstance(payload, str) else payload
                assert sender.SendMessage(target.robot_id, payload) == len(raw)
                packet, _ = relay.recvfrom(65535)
                assert packet[:8] == struct.pack("<BBBBHH",
                    1, 3, sender.robot_id, target.robot_id, sender.status_port, len(raw))
                assert packet[8:] == raw
                relay.sendto(packet, ("127.0.0.1", target.status_port))
                assert target.ReceiveMessage(timeout=1) == (sender.robot_id, raw)
            # Exercise LunaPre framing over the same real sockets, without a fake transport.
            from LunaPre.communication import Communication
            sending = Communication(bits=a, peer_id=2)
            receiving = Communication(bits=b, peer_id=1)
            payload = sending.BuildFrame(identity_id=1, status=1, x_pos=7, y_neg=9)
            assert sending.Send(payload)
            packet, _ = relay.recvfrom(65535)
            relay.sendto(packet, ("127.0.0.1", b.status_port))
            parsed = None
            deadline = time.monotonic() + 1
            while parsed is None and time.monotonic() < deadline:
                parsed = receiving.Receive()
                if parsed is None:
                    time.sleep(0.005)
            assert parsed is not None
            assert parsed["signed_xy"] == [7, -9]
            receiving.Close()
            assert b._running  # Communication does not own the shared Bits lifecycle.
            assert a.ReceiveMessage() is None
            with pytest.raises(ValueError):
                a.SendMessage(2, b"x" * 4097)
            with pytest.raises(ValueError):
                a.SendMessage(0, b"x")
        finally:
            for bits in (a, b):
                bits.Close()
                bits._rx_thread.join(1)
                assert not bits._rx_thread.is_alive()
        with pytest.raises(OSError):
            a.SendMessage(2, b"closed")


def test_invalid_packets_and_bounded_queue():
    bits = object.__new__(XeleratorSimBits)
    bits.robot_id = 2
    bits._messages = queue.Queue(maxsize=256)
    header = bits.MESSAGE_HEADER
    for data in [b"", b"\x01\x03", header.pack(1, 3, 1, 3, 123, 1) + b"x",
                 header.pack(1, 3, 1, 2, 123, 2) + b"x",
                 header.pack(2, 3, 1, 2, 123, 1) + b"x",
                 header.pack(1, 3, 0, 2, 123, 1) + b"x"]:
        bits._receive_message_packet(data)
    assert bits.ReceiveMessage() is None
    for i in range(257):
        bits._receive_message_packet(header.pack(1, 3, 1, 2, 123, 2) + i.to_bytes(2, "little"))
    assert bits._messages.qsize() == 256
    assert bits.ReceiveMessage() == (1, b"\x01\x00")
