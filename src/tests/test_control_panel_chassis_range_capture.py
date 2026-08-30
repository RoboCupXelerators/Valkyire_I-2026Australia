from types import SimpleNamespace

from control_panel import ControlPanelApp


def test_chassis_range_capture_api_delegates_only_to_vision_provider():
    calls = []
    vision = SimpleNamespace(
        StartChassisRangeCapture=lambda known_range_cm: calls.append(("start", known_range_cm)) or {"success": True},
        GetChassisRangeCaptureStatus=lambda: {"state": "capturing"},
    )
    app = ControlPanelApp.__new__(ControlPanelApp)
    app.vision = vision

    assert app.chassis_range_capture_start(80) == {"success": True}
    assert app.chassis_range_capture_status() == {"success": True, "capture": {"state": "capturing"}}
    assert calls == [("start", 80.0)]


def test_chassis_range_capture_api_does_not_initialize_or_command_runtime():
    app = ControlPanelApp.__new__(ControlPanelApp)
    app.vision = None

    assert app.chassis_range_capture_start(20) == {"success": False, "error": "vision_not_available"}
    assert app.chassis_range_capture_status() == {"success": False, "error": "vision_not_available"}
