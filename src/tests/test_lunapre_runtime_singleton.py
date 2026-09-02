"""Concurrency contracts for LunaPre's lazy runtime initialization."""

from __future__ import annotations

import threading
import time
from types import SimpleNamespace

from LunaPre import runtime


def test_debug_motion_bypasses_only_the_runtime_motion_permission(monkeypatch):
    bits = SimpleNamespace(GetRefereeUnlockPin=lambda: False)

    monkeypatch.delenv("ROBOCUPX_DEBUG", raising=False)
    assert runtime._motion_permission(bits) is False

    monkeypatch.setenv("ROBOCUPX_DEBUG", "true")
    assert runtime._motion_permission(bits) is True


def test_concurrent_runtime_initialization_builds_one_backend(monkeypatch):
    names = (
        "Bits",
        "lidar",
        "sensor",
        "chassis",
        "compass",
        "peripheral",
        "key",
        "Odometer",
        "Morse",
        "referee",
        "vision",
        "_field_pose_provider",
        "_velocity_provider",
        "_referee_status_provider",
        "_referee_state_provider",
        "_referee_serial_provider",
        "_runtime_initialized",
    )
    previous = {name: getattr(runtime, name) for name in names}
    factory_started = threading.Event()
    release_factory = threading.Event()
    factory_calls = []
    chassis = object()

    def factory(*, auto_start_lidar):
        factory_calls.append(auto_start_lidar)
        factory_started.set()
        assert release_factory.wait(timeout=1.0)
        return {
            "Bits": object(),
            "raw_lidar": None,
            "vision": None,
            "chassis": chassis,
            "compass": lambda: 0.0,
            "peripheral": object(),
            "key": None,
            "Odometer": None,
            "Morse": None,
            "referee": None,
            "field_pose_provider": lambda: None,
            "velocity_provider": lambda: (0.0, 0.0, 0.0),
            "referee_status_provider": lambda: True,
        }

    try:
        for name in names:
            setattr(runtime, name, None)
        runtime._runtime_initialized = False
        monkeypatch.setattr(runtime, "_selected_backend_name", lambda: "TEST")
        monkeypatch.setattr(runtime, "_BACKEND_FACTORIES", {"TEST": factory})
        monkeypatch.setenv("ROBOCUPX_REFEREE_SOURCE", "io")
        monkeypatch.delenv("ROBOCUPX_REFEREE_PORT", raising=False)

        errors = []

        def initialize():
            try:
                runtime._init_runtime(auto_start_lidar=False)
            except Exception as exc:  # pragma: no cover - assertion reports below
                errors.append(exc)

        first = threading.Thread(target=initialize)
        second = threading.Thread(target=initialize)
        first.start()
        second.start()

        assert factory_started.wait(timeout=1.0)
        time.sleep(0.05)
        release_factory.set()
        first.join(timeout=1.0)
        second.join(timeout=1.0)

        assert not first.is_alive()
        assert not second.is_alive()
        assert not errors
        assert factory_calls == [False]
        assert runtime.chassis is chassis
    finally:
        release_factory.set()
        runtime.shutdown_runtime()
        for name, value in previous.items():
            setattr(runtime, name, value)
