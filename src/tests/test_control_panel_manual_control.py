import math
import threading
from types import SimpleNamespace

from control_panel import CONTROL_MAX_LINEAR_MPS, ControlPanelApp


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


def test_control_status_contains_no_archived_backend_contract_fields():
    app = _app(SimpleNamespace())

    status = app.control_get_status()

    assert "profile" not in status
    assert "lease_age_ms" not in status
    assert "lease_timeout_ms" not in status
