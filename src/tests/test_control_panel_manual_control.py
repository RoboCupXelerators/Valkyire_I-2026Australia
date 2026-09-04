import math
import threading
from types import SimpleNamespace
from unittest.mock import Mock

from control_panel import CONTROL_COMMAND_TIMEOUT_S, CONTROL_MAX_LINEAR_MPS, ControlPanelApp


def _app(chassis, *, bits=None):
    app = ControlPanelApp.__new__(ControlPanelApp)
    app.bits = bits
    app._control_lock = threading.Lock()
    app._control_enabled = False
    app._control_speed = 0.5
    app._control_turn_speed = 120.0
    app._dribble_active = False
    app._control_runtime_error = None
    app._get_chassis_and_peripheral = lambda initialize=False: (chassis, object())
    app._is_control_runtime_initialized = lambda: True
    return app


def test_enable_and_disable_use_generic_chassis_surface_without_arm_or_profile():
    calls = []
    chassis = SimpleNamespace(stop=lambda: calls.append("stop"))
    bits = SimpleNamespace(
        Arm=lambda: (_ for _ in ()).throw(AssertionError("legacy Arm called")),
        SetChassisProfile=lambda _: (_ for _ in ()).throw(AssertionError("legacy profile called")),
    )
    app = _app(chassis, bits=bits)

    assert app.control_enable(True) == {"success": True, "enabled": True}
    assert app.control_enable(False) == {"success": True, "enabled": False}
    assert calls == ["stop"]


def test_manual_vector_uses_common_twist_interface_and_clamps_speed():
    calls = []
    chassis = SimpleNamespace(RobotMoveTwist=lambda *values: calls.append(values))
    app = _app(chassis)
    app._control_enabled = True

    result = app.control_move_vector(3.0, 0.0, 90.0)

    assert result == {
        "success": True,
        "vx": CONTROL_MAX_LINEAR_MPS,
        "vy": 0.0,
        "wz": 90.0,
    }
    assert calls == [(CONTROL_MAX_LINEAR_MPS, 0.0, math.pi / 2)]


def test_zero_manual_vector_uses_immediate_chassis_stop():
    calls = []
    chassis = SimpleNamespace(
        RobotMoveTwist=lambda *values: calls.append(("move", values)),
        stop=lambda: calls.append(("stop",)),
    )
    app = _app(chassis)
    app._control_enabled = True

    app.control_move_vector(0.5, 0.0, 0.0)
    result = app.control_move_vector(0.0, 0.0, 0.0)

    assert result == {"success": True, "vx": 0.0, "vy": 0.0, "wz": 0.0}
    assert calls == [("move", (0.5, 0.0, 0.0)), ("stop",)]
    assert app._control_last_motion_ts is None


def test_stale_manual_motion_is_stopped_by_watchdog():
    calls = []
    chassis = SimpleNamespace(stop=lambda: calls.append("stop"))
    app = _app(chassis)
    app._control_enabled = True
    app._control_last_motion_ts = 10.0

    assert app._control_stop_if_stale(10.0 + CONTROL_COMMAND_TIMEOUT_S + 0.01) is True
    assert calls == ["stop"]
    assert app._control_last_motion_ts is None


def test_control_status_contains_no_archived_backend_contract_fields():
    app = _app(SimpleNamespace())

    status = app.control_get_status()

    assert "profile" not in status
    assert "lease_age_ms" not in status
    assert "lease_timeout_ms" not in status


def test_kick_power_setting_uses_new_bits_default_level():
    calls = []

    def set_kick_power(level):
        calls.append(level)
        bits.kick_power_level = level

    bits = SimpleNamespace(kick_power_level=4, SetKickPowerLevel=set_kick_power)
    app = _app(SimpleNamespace(), bits=bits)

    result = app.control_set_kick_power(2)

    assert result == {
        "success": True,
        "kick_power_level": 2,
        "kick_power_min": 1,
        "kick_power_max": 4,
    }
    assert calls == [2]
    assert app.control_get_status()["kick_power_level"] == 2
    assert app.control_get_status()["kick_power_supported"] is True


def test_kick_power_setting_rejects_non_integer_or_out_of_range_levels():
    setter = Mock()
    app = _app(SimpleNamespace(), bits=SimpleNamespace(SetKickPowerLevel=setter))

    for value in (0, 5, 1.5, True, None):
        result = app.control_set_kick_power(value)
        assert result == {
            "success": False,
            "error": "kick power level must be an integer from 1 to 4",
        }

    setter.assert_not_called()
