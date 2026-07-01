"""XeleratorSimBits simulator semantics tests."""

from __future__ import annotations

import os
import sys
import threading

os.environ.setdefault("ROBOCUPX_BITS", "SIM")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from utils.XeleratorSimBits import SimStatus, XeleratorSimBits  # noqa: E402


def _make_bits_for_possession(ball_x_m: float | None, ball_y_m: float | None, *, dribble: float) -> XeleratorSimBits:
    """构造不启动 UDP 线程的最小 SIM Bits 对象。"""
    bits = object.__new__(XeleratorSimBits)
    bits._status = SimStatus(ball_x_m=ball_x_m, ball_y_m=ball_y_m, has_ball=False)
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
    return bits


def test_sim_bits_infers_possession_when_dribbler_covers_ball():
    bits = _make_bits_for_possession(0.09, 0.005, dribble=0.7)

    assert bits.HasBall() is True
    assert bits.GetBallPos() == [0xDDD, 0xDDD]


def test_sim_bits_keeps_free_ball_when_only_in_recovery_window():
    bits = _make_bits_for_possession(0.12, 0.01, dribble=0.7)

    assert bits.HasBall() is False
    assert bits.GetBallPos() == [12.0, 1.0]


def test_sim_bits_keeps_free_ball_when_dribbler_is_off():
    bits = _make_bits_for_possession(0.12, 0.01, dribble=0.0)

    assert bits.HasBall() is False
    assert bits.GetBallPos() == [12.0, 1.0]

