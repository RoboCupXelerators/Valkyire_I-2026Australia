import importlib
import math
import struct
import threading
import time
from types import SimpleNamespace

import pytest

pytestmark = pytest.mark.dm2325

from control_panel import ControlPanelApp
from control_panel import DM_CONTROL_LEASE_TIMEOUT_S
from utils.DM2325Bits import (
    CHASSIS_DIAGNOSTICS_PAYLOAD_LEN,
    Frame,
    FrameParser,
    MsgType,
    SerialTransport,
    decode_chassis_diagnostics,
)


def _app(chassis):
    app = ControlPanelApp.__new__(ControlPanelApp)
    app._control_lock = threading.Lock()
    app._control_enabled = True
    app._control_runtime_error = None
    app._control_speed = 0.5
    app._control_turn_speed = 120.0
    app._get_chassis_and_peripheral = lambda initialize=False: (chassis, object())
    return app


def test_move_vector_uses_si_dm_interface_and_converts_legacy_values():
    calls = []
    chassis = SimpleNamespace(
        RobotMoveTwist=lambda *values: calls.append(values),
        RobotMoveVector=lambda *_: pytest.fail("legacy interface used on DM backend"),
    )
    app = _app(chassis)

    result = app.control_move_vector(100, -200, 90)

    assert result == {"success": True, "vx": 0.1, "vy": -0.2, "wz": 90.0}
    assert calls == [(0.1, -0.2, pytest.approx(math.pi / 2))]


def test_move_vector_limits_diagonal_magnitude_without_changing_direction():
    calls = []
    chassis = SimpleNamespace(RobotMoveTwist=lambda *values: calls.append(values))
    app = _app(chassis)

    result = app.control_move_vector(2.29, 2.29, 0)

    assert math.hypot(result["vx"], result["vy"]) == pytest.approx(2.29)
    assert result["vx"] == pytest.approx(result["vy"])
    assert calls[0][2] == 0.0


def test_speed_endpoint_accepts_si_and_cached_legacy_slider_values():
    app = _app(SimpleNamespace())
    assert app.control_set_speed(0.75)["speed"] == pytest.approx(0.75)
    assert app.control_set_speed(300)["speed"] == pytest.approx(0.3)
    assert app.control_set_speed(999999)["speed"] == pytest.approx(2.29)


def test_enable_dm_backend_arms_without_peripheral():
    calls = []
    chassis = SimpleNamespace(stop=lambda: calls.append("chassis.stop"))
    bits = SimpleNamespace(Arm=lambda: calls.append("arm"))
    app = _app(chassis)
    app._control_enabled = False
    app.bits = bits
    app._get_chassis_and_peripheral = lambda initialize=False: (chassis, None)
    app._ensure_control_worker = lambda: calls.append("worker")

    result = app.control_enable(True)

    assert result == {"success": True, "enabled": True}
    assert calls == ["arm", "worker"]


def test_disable_dm_backend_stops_and_disables_all_motors():
    calls = []
    chassis = SimpleNamespace(stop=lambda: calls.append("chassis.stop"))
    bits = SimpleNamespace(
        stop=lambda: calls.append("bits.stop"),
        disable=lambda: calls.append("bits.disable"),
    )
    app = _app(chassis)
    app.bits = bits
    app._control_target = (0.3, 0.0, 0.0)

    result = app.control_enable(False)

    assert result == {"success": True, "enabled": False}
    assert app._control_target == (0.0, 0.0, 0.0)
    assert calls == ["bits.stop", "bits.disable"]


def test_repeated_disable_is_an_idempotent_no_op():
    bits = SimpleNamespace(
        stop=lambda: pytest.fail("already Disabled chassis was stopped again"),
        disable=lambda: pytest.fail("already Disabled chassis was disabled again"),
    )
    app = _app(SimpleNamespace())
    app.bits = bits
    app._control_enabled = False
    app._control_target = (0.0, 0.0, 0.0)

    assert app.control_enable(False) == {"success": True, "enabled": False}


def test_lightweight_lease_renewal_does_not_send_hardware_command():
    app = _app(SimpleNamespace())
    app._control_last_lease = 0.0
    app._control_disable_reason = None

    result = app.control_renew_lease()

    assert result == {"success": True, "enabled": True}
    assert 0.0 <= time.monotonic() - app._control_last_lease < 0.1
    assert DM_CONTROL_LEASE_TIMEOUT_S >= 0.75


def test_lease_renewal_reports_prior_expiry_when_disabled():
    app = _app(SimpleNamespace())
    app._control_enabled = False
    app._control_disable_reason = "lease_expired"

    assert app.control_renew_lease() == {
        "success": False,
        "enabled": False,
        "error": "Control not enabled",
        "disable_reason": "lease_expired",
    }


def test_cdc_exchanges_are_serialized_across_web_threads():
    transport = SerialTransport("unused")
    parser = FrameParser()
    written = []
    active = 0
    max_active = 0

    class FakeSerial:
        is_open = True
        timeout = 0.005
        in_waiting = 0

        def write(self, data):
            parser.feed(data)
            written.append(parser.next_frame())
            return len(data)

    transport._ser = FakeSerial()

    def poll(_timeout):
        nonlocal active, max_active
        active += 1
        max_active = max(max_active, active)
        time.sleep(0.02)
        request = written[-1]
        transport._pending.append(
            Frame(msg_type=MsgType.PONG, seq=request.seq, payload=b"")
        )
        active -= 1

    transport._poll_frames = poll
    barrier = threading.Barrier(3)
    responses = []

    def exchange(seq):
        barrier.wait()
        responses.append(
            transport.send_and_wait(
                Frame(msg_type=MsgType.PING, seq=seq, payload=b""), MsgType.PONG
            )
        )

    threads = [threading.Thread(target=exchange, args=(seq,)) for seq in (1, 2)]
    for thread in threads:
        thread.start()
    barrier.wait()
    for thread in threads:
        thread.join()

    assert max_active == 1
    assert len(responses) == 2
    assert all(response is not None for response in responses)


def test_profile_is_applied_before_arm_and_cannot_change_while_enabled():
    calls = []
    bits = SimpleNamespace(
        SetChassisProfile=lambda profile: calls.append(("profile", profile)),
        Arm=lambda: calls.append(("arm", None)),
    )
    app = _app(SimpleNamespace())
    app._control_enabled = False
    app._control_profile = "competition_candidate"
    app.bits = bits
    app._ensure_control_worker = lambda: None

    assert app.control_enable(True) == {"success": True, "enabled": True}
    assert calls == [("profile", "competition_candidate"), ("arm", None)]
    assert app.control_set_profile("safe") == {
        "success": False,
        "error": "Disable chassis before changing profile",
    }


def test_profile_selection_rejects_unknown_value_without_hardware_io():
    app = _app(SimpleNamespace())
    app._control_enabled = False
    app._control_profile = "normal"
    assert app.control_set_profile("race_final") == {
        "success": False,
        "error": "Unknown control profile: race_final",
    }


def test_reverse_diagnostics_decode_real_and_unsupported_fields():
    payload = bytearray(CHASSIS_DIAGNOSTICS_PAYLOAD_LEN)
    struct.pack_into("<BBBBIHH8f", payload, 0, 1, 1, 1, 15, 5000, 10, 10,
                     16.7, 25.0, 40.0, 66.7, 240.0, 0.2, 0.5, 0.05)
    for index in range(4):
        struct.pack_into("<BBBB5fHBB", payload, 44 + index * 28,
                         index + 1, 1, 2, 7, -10.0, -20.0, 5.0, -15.0,
                         0.5, 2, 1, 1)
    struct.pack_into("<5I4f4B3I", payload, 156,
                     9, 100, 110, 200, 260, 20.0, -20.0, 18.0, 1.1,
                     4, 1, 1, 1, 10, 90, 60)

    decoded = decode_chassis_diagnostics(bytes(payload))

    assert decoded["profile_name"] == "normal"
    assert decoded["wheels"][0]["measured_torque_nm"] == pytest.approx(0.5)
    assert decoded["wheels"][0]["measured_current_a"] is None
    assert decoded["wheels"][0]["bus_voltage_v"] is None
    assert decoded["latest_reverse_event"]["command_to_brake_latency_ms"] == 10


def test_control_panel_formats_imu_state_without_web_specific_driver_code():
    state = SimpleNamespace(
        healthy=True,
        source="GYRO_RELATIVE",
        age_ms=4.5,
        received_monotonic=time.monotonic(),
        flags=0x4000,
        accel_mps2=(0.1, 0.2, 9.8),
        gyro_rad_s=(0.01, 0.02, 0.03),
        roll_rad=math.radians(10),
        pitch_rad=math.radians(-5),
        yaw_rad=math.radians(370),
        temperature_c=33.25,
        calibration_state=2,
        sample_count=123,
        missed_sample_count=1,
        spi_error_count=0,
    )

    telemetry = ControlPanelApp._control_imu_telemetry(state)

    assert telemetry["available"] is True
    assert telemetry["attitude_deg"] == pytest.approx({
        "roll": 10.0,
        "pitch": -5.0,
        "yaw": 10.0,
    })
    assert telemetry["accel_mps2"] == [0.1, 0.2, 9.8]
    assert telemetry["flags"] == 0x4000


def test_control_panel_replaces_nonfinite_imu_values_with_null():
    state = SimpleNamespace(
        healthy=False,
        source="GYRO_RELATIVE",
        age_ms=float("inf"),
        received_monotonic=time.monotonic(),
        flags=0,
        accel_mps2=(float("nan"), 0.0, 9.8),
        gyro_rad_s=(0.0, 0.0, 0.0),
        roll_rad=float("nan"),
        pitch_rad=0.0,
        yaw_rad=0.0,
        temperature_c=float("nan"),
        calibration_state=0,
        sample_count=0,
        missed_sample_count=0,
        spi_error_count=0,
    )

    telemetry = ControlPanelApp._control_imu_telemetry(state)

    assert telemetry["age_ms"] is None
    assert telemetry["accel_mps2"][0] is None
    assert telemetry["attitude_deg"]["roll"] is None
    assert telemetry["temperature_c"] is None


def test_enabled_control_panel_reads_cached_imu_without_extra_cdc_exchange():
    calls = []
    imu_state = SimpleNamespace(
        healthy=True,
        source="GYRO_RELATIVE",
        age_ms=1.0,
        received_monotonic=time.monotonic(),
        flags=0,
        accel_mps2=(0.0, 0.0, 9.8),
        gyro_rad_s=(0.0, 0.0, 0.0),
        roll_rad=0.0,
        pitch_rad=0.0,
        yaw_rad=0.0,
        temperature_c=32.0,
        calibration_state=2,
        sample_count=1,
        missed_sample_count=0,
        spi_error_count=0,
    )
    bits = SimpleNamespace(
        GetChassisDiagnostics=lambda: {"wheels": []},
        GetChassisTelemetry=lambda: SimpleNamespace(
            motors=(), runtime_state=3, runtime_fault=0, can_esr=0
        ),
        GetCachedIMUState=lambda: (calls.append("cached") or imu_state),
        GetIMUState=lambda: pytest.fail("enabled telemetry started an IMU CDC exchange"),
    )
    app = _app(SimpleNamespace())
    app.bits = bits
    app._ensure_control_runtime = lambda: None

    telemetry = app.control_get_telemetry()

    assert telemetry["success"] is True
    assert telemetry["imu"]["available"] is True
    assert calls == ["cached"]


def test_no_runtime_mode_blocks_lazy_lunapre_initialization(monkeypatch):
    app = ControlPanelApp.__new__(ControlPanelApp)
    app._allow_runtime_init = False
    app._control_runtime_error = None

    monkeypatch.setattr(
        importlib,
        "import_module",
        lambda _name: pytest.fail("--no-runtime imported LunaPre lazily"),
    )

    assert app._ensure_control_runtime() is None
    assert app._control_runtime_error == "Runtime initialization disabled"
