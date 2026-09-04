"""XeleratorSimBits simulator semantics tests."""

from __future__ import annotations

import os
import sys
import threading
import math

import pytest

os.environ.setdefault("ROBOCUPX_BITS", "SIM")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from utils.XeleratorSimBits import SimStatus, XeleratorSimBits  # noqa: E402
from vision_utils.XeleratorVision import XeleratorVision  # noqa: E402
from LunaPre.positions import Positions, cfg as positions_cfg  # noqa: E402
from LunaPre import runtime  # noqa: E402
from LunaPre.strategy_tools import BALL_HELD  # noqa: E402


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
    bits._kick_strength = 4.0
    bits.kick_power_level = 4
    return bits


@pytest.mark.parametrize("power_level", [1, 2, 3, 4])
def test_sim_kick_accepts_power_level_and_emits_selected_strength(power_level):
    bits = _make_bits_for_kick()

    assert bits.SetKickPowerLevel(power_level) == power_level
    assert bits.Kick() is True
    assert bits._kick_strength == float(power_level)
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

    assert bits._kick_strength == 2.0
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
        return Positions.RobotBallPos(position), Positions.BallState(position)
    finally:
        runtime.vision = vision


def test_sim_bits_returns_catch_window_coords_when_dribbler_covers_ball():
    catch_x, catch_y = _catch_val()
    bits = _make_bits_for_possession(catch_x / 100.0, catch_y / 100.0, dribble=0.7)

    assert bits.HasBall() is True
    observation = bits.GetBallPos()
    assert observation == [catch_x, catch_y]
    robot_ball_pos, ball_state = _robot_ball_pos_for_observation(observation)
    assert robot_ball_pos == BALL_HELD
    assert ball_state == "held"


def test_sim_bits_synthesizes_catch_window_coords_when_status_has_ball_without_extension():
    bits = _make_bits_for_possession(None, None, dribble=0.0, has_ball=True)

    assert bits.HasBall() is True
    observation = bits.GetBallPos()
    assert observation == _catch_val()
    robot_ball_pos, ball_state = _robot_ball_pos_for_observation(observation)
    assert robot_ball_pos == BALL_HELD
    assert ball_state == "held"


def test_sim_bits_canonicalizes_shifted_contact_coords_when_status_has_ball():
    bits = _make_bits_for_possession(0.0987, 0.004, dribble=0.7, has_ball=True)

    observation = bits.GetBallPos()
    assert observation == _catch_val()
    robot_ball_pos, ball_state = _robot_ball_pos_for_observation(observation)
    assert robot_ball_pos == BALL_HELD
    assert ball_state == "held"


def test_robot_ball_pos_classifies_catch_val_coords_as_held():
    robot_ball_pos, ball_state = _robot_ball_pos_for_observation(_catch_val())

    assert robot_ball_pos == BALL_HELD
    assert ball_state == "held"


def test_sim_provider_keeps_nearby_free_ball_out_of_held_state(monkeypatch):
    class FakeVision:
        def get_ball_pos(self):
            return [9.5, 0.0]

        def has_ball(self):
            return False

    position = object.__new__(Positions)
    position._robot_ball_out_of_bounds = lambda ball_pos: False
    position.ballPos = None
    position.ballPosOut = None
    monkeypatch.setattr(runtime, "vision", FakeVision())

    assert Positions.RobotBallPos(position) == [9.5, 0.0]
    assert Positions.BallState(position) == "free"


def test_sim_provider_explicit_possession_is_held(monkeypatch):
    class FakeVision:
        def get_ball_pos(self):
            return [12.0, 1.0]

        def has_ball(self):
            return True

    position = object.__new__(Positions)
    position._robot_ball_out_of_bounds = lambda ball_pos: False
    position.ballPos = None
    position.ballPosOut = None
    monkeypatch.setattr(runtime, "vision", FakeVision())

    assert Positions.RobotBallPos(position) == BALL_HELD
    assert Positions.BallState(position) == "held"


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


def test_sim_bits_keeps_free_ball_when_only_in_recovery_window():
    bits = _make_bits_for_possession(0.12, 0.01, dribble=0.7)

    assert bits.HasBall() is False
    assert bits.GetBallPos() == [12.0, 1.0]


def test_sim_bits_keeps_free_ball_when_dribbler_is_off():
    bits = _make_bits_for_possession(0.12, 0.01, dribble=0.0)

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
