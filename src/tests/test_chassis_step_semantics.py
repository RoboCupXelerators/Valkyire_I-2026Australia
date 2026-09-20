"""RobotMoveVector step-semantics regression tests.

Field report 2026-09-19 (robot1): ``RobotMoveVector(0, 0, 0)`` decelerated
instead of braking and ``RobotMoveVector(0, 0, 360)`` crept up instead of
commanding 360 deg/s. Root cause: two chained ramps —
``MotionBackend.filter_chassis_command`` (traction control, planar 3 m/s^2 /
angular 450 deg/s^2) and ``apply_wheel_slew_limit`` (1800 RPM/s) — treating
every velocity request as a ramp target.

Fix: velocity requests are step commands. The ramps are disabled in config
(``Motion.EnableTractionControl: false``, ``Motion.EnableWheelSlewLimit:
false``); the code stays available for future mechanical protection needs.
"""

from __future__ import annotations

import math
import os
import sys

import pytest

os.environ.setdefault("ROBOCUPX_BITS", "SIM")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


def _make_car(monkeypatch, *, traction=None, slew=None):
    """Car with a recording SetMotorFunc and controlled ramp flags."""
    from LunaPre.motion.chassis import Car

    wheel_commands: list[tuple[int, int, int, int]] = []
    car = Car(SetMotorFunc=lambda a, b, c, d: wheel_commands.append((a, b, c, d)),
              GetYaw=lambda: 0.0)
    monkeypatch.setattr(car, "CanMove", lambda: True)
    if traction is not None:
        monkeypatch.setattr(car.cfg.Motion, "EnableTractionControl", traction)
    if slew is not None:
        monkeypatch.setattr(car.cfg.Motion, "EnableWheelSlewLimit", slew)
    return car, wheel_commands


def test_zero_command_brakes_immediately_when_ramps_disabled(monkeypatch):
    car, wheels = _make_car(monkeypatch, traction=False, slew=False)

    # Cruise forward, then order a full stop: the very next wheel command
    # must already be the zero/brake state.
    car.RobotMoveVector(0.8, 0.0, 0.0)
    assert any(any(w != 0 for w in cmd) for cmd in wheels), "cruise must command wheels"

    car.RobotMoveVector(0.0, 0.0, 0.0)
    assert wheels[-1] == (0, 0, 0, 0), (
        f"RobotMoveVector(0,0,0) must brake in one step, got {wheels[-1]}"
    )


def test_rotation_command_reaches_target_in_one_step(monkeypatch):
    car, wheels = _make_car(monkeypatch, traction=False, slew=False)

    car.RobotMoveVector(0.0, 0.0, 360.0)

    # Pure in-place spin at 360 deg/s: |w1|=|w2| opposite pairs, magnitude
    # = 360 * rpm_per_degps (clamped only by MaxWheelRPM).
    rpm_per_degps = car._rpm_per_degps()
    expected = 360.0 * rpm_per_degps
    vals = [abs(w) for w in wheels[-1]]
    assert all(v == pytest.approx(expected, rel=0.02) for v in vals), (
        f"single call must command the full 360 deg/s mix, got {wheels[-1]}"
    )
    w1, w2, w3, w4 = wheels[-1]
    assert (w1 > 0) == (w2 > 0) and (w3 > 0) == (w4 > 0) and (w1 > 0) != (w3 > 0)


def test_legacy_ramp_behavior_is_opt_in(monkeypatch):
    """With the ramps explicitly enabled the old creep behavior applies.

    Pins the contrast so the step semantics cannot silently regress back.
    """
    car, wheels = _make_car(monkeypatch, traction=True, slew=False)

    car.RobotMoveVector(0.0, 0.0, 360.0)
    first = wheels[-1]
    assert max(abs(w) for w in first) < 0.2 * 360.0 * car._rpm_per_degps(), (
        "traction ramp enabled: first tick must be a small fraction of target"
    )


def test_deployed_config_disables_command_ramps():
    """The machine config must ship with both ramps off (step semantics)."""
    from ReasonData import Settings

    assert Settings.Motion.EnableTractionControl is False, (
        "Motion.EnableTractionControl must be false in the deployed config"
    )
    assert Settings.Motion.EnableWheelSlewLimit is False, (
        "Motion.EnableWheelSlewLimit must be false in the deployed config"
    )
