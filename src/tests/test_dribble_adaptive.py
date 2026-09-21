"""Regression coverage for current-and-vision dribbler adaptation."""

from __future__ import annotations

import os
import sys

import pytest

os.environ.setdefault("ROBOCUPX_BITS", "SIM")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from LunaPre.motion.dribble_adaptive import (  # noqa: E402
    DribbleAdaptiveController,
    VISION_APPROACHING,
    VISION_HELD,
    VISION_LOST,
    VISION_NEAR,
)


def controller(**overrides):
    return DribbleAdaptiveController(
        enabled=True,
        search_power=0.20,
        capture_power_start=0.30,
        capture_power=0.90,
        hold_power=0.75,
        relief_power=0.65,
        loaded_current_min_a=1.65,
        excess_current_a=2.45,
        capture_power_step=0.10,
        capture_power_step_interval_s=0.25,
        min_update_interval_s=0.0,
        **overrides,
    )


def test_disabled_controller_preserves_existing_dribble_command():
    decision = DribbleAdaptiveController(enabled=False).decide(
        0.3, vision_state=VISION_HELD, current_a=2.0, now_s=1.0
    )
    assert decision.power == pytest.approx(0.3)
    assert decision.mode == "disabled"


def test_missing_current_never_raises_power_from_vision_alone():
    decision = controller().decide(
        0.3, vision_state=VISION_APPROACHING, current_a=None, now_s=1.0
    )
    assert decision.power == pytest.approx(0.3)
    assert decision.mode == "current_unavailable"


def test_vision_and_current_choose_search_near_ramp_hold_and_relief():
    adaptive = controller()

    assert adaptive.decide(0.3, vision_state=VISION_LOST, current_a=0.9, now_s=1.0).power == pytest.approx(0.2)
    assert adaptive.decide(0.3, vision_state=VISION_APPROACHING, current_a=1.4, now_s=2.0).power == pytest.approx(0.3)

    near_start = adaptive.decide(0.3, vision_state=VISION_NEAR, current_a=1.4, now_s=3.0)
    assert near_start.power == pytest.approx(0.3)
    assert near_start.mode == "near_ramping"
    assert adaptive.decide(0.3, vision_state=VISION_NEAR, current_a=1.4, now_s=3.25).power == pytest.approx(0.4)
    assert adaptive.decide(0.3, vision_state=VISION_NEAR, current_a=1.4, now_s=4.50).power == pytest.approx(0.9)

    low_load = adaptive.decide(0.3, vision_state=VISION_HELD, current_a=1.4, now_s=4.75)
    assert low_load.power == pytest.approx(0.9)
    assert low_load.mode == "held_ramping"

    held = adaptive.decide(0.3, vision_state=VISION_HELD, current_a=2.0, now_s=5.0)
    assert held.power == pytest.approx(0.75)
    assert held.mode == "held"

    relief = adaptive.decide(0.3, vision_state=VISION_HELD, current_a=2.5, now_s=5.25)
    assert relief.power == pytest.approx(0.65)
    assert relief.mode == "excess_load_relief"


def test_unknown_vision_state_preserves_requested_power():
    decision = controller().decide(0.3, vision_state="maybe", current_a=2.0, now_s=1.0)
    assert decision.power == pytest.approx(0.3)
    assert decision.mode == "vision_unavailable"


def test_default_capture_ramp_waits_for_the_slow_current_poll():
    adaptive = DribbleAdaptiveController(enabled=True)

    assert adaptive.decide(0.3, vision_state=VISION_NEAR, current_a=1.0, now_s=0.0).power == pytest.approx(0.5)
    assert adaptive.decide(0.3, vision_state=VISION_NEAR, current_a=1.0, now_s=1.09).power == pytest.approx(0.5)
    assert adaptive.decide(0.3, vision_state=VISION_NEAR, current_a=1.0, now_s=1.10).power == pytest.approx(0.55)


def test_default_near_profile_boosts_only_for_lateral_error():
    centred = DribbleAdaptiveController(enabled=True).decide(
        0.3, vision_state=VISION_NEAR, current_a=1.0, lateral_error_cm=0.0, now_s=0.0
    )
    lateral = DribbleAdaptiveController(enabled=True).decide(
        0.3, vision_state=VISION_NEAR, current_a=1.0, lateral_error_cm=12.0, now_s=0.0
    )

    assert centred.power == pytest.approx(0.50)
    assert lateral.power == pytest.approx(0.60)
