"""Guarded LockBallSlip wheel-limit debug path tests."""

from types import SimpleNamespace

import pytest

from LunaPre.motion.motion_backend import MotionBackend


def _backend():
    cfg = SimpleNamespace(
        Motion=SimpleNamespace(
            NormalizeWheelOutput=True,
            EnableCoupledWheelAllocator=True,
            EnableTurnTranslationDerate=False,
            WheelAllocatorSaturationMode="preserve_twist",
        ),
        Chassis=SimpleNamespace(MaxWheelRPM=500),
    )
    return MotionBackend(cfg)


def test_wheel_speed_bypass_preserves_explicit_debug_vector():
    backend = _backend()
    values = [600.0, -300.0, 150.0, -450.0]

    assert backend.clamp_motor_vector(values) == pytest.approx(
        [500.0, -250.0, 125.0, -375.0]
    )
    assert backend.clamp_motor_vector(values, bypass_speed_limit=True) == values


def test_wheel_mix_bypass_skips_allocator_scaling_only_when_requested():
    backend = _backend()
    planar = [600.0, 600.0, 600.0, 600.0]
    turn = [100.0, 100.0, 100.0, 100.0]

    assert backend.allocate_wheel_mix(planar, turn, 0.0) == pytest.approx(
        [500.0, 500.0, 500.0, 500.0]
    )
    assert backend.allocate_wheel_mix(
        planar, turn, 0.0, bypass_speed_limit=True
    ) == [700.0, 700.0, 700.0, 700.0]
