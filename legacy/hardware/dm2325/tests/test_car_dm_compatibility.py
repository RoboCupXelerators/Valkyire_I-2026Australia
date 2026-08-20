import math

import pytest

from LunaPre.motion.chassis import Car

pytestmark = pytest.mark.dm2325


def test_80_100_encoder_speedometer_recovers_body_velocity():
    from utils.DM2325Bits import estimate_chassis_velocity

    forward = estimate_chassis_velocity((-6.427876, 6.427876, -7.660444, 7.660444))
    assert forward.vx_mps == pytest.approx(0.30, abs=1e-5)
    assert forward.vy_mps == pytest.approx(0.0, abs=1e-5)
    assert forward.wz_rad_s == pytest.approx(0.0, abs=1e-5)
    assert forward.speed_mps == pytest.approx(0.30, abs=1e-5)

    left = estimate_chassis_velocity((5.106963, 5.106963, -4.285251, -4.285251))
    assert left.vx_mps == pytest.approx(0.0, abs=1e-5)
    assert left.vy_mps == pytest.approx(0.20, abs=1e-5)
    assert left.wz_rad_s == pytest.approx(0.0, abs=1e-5)


def test_encoder_speedometer_rejects_bad_motor_data():
    from utils.DM2325Bits import estimate_chassis_velocity

    with pytest.raises(ValueError, match="four"):
        estimate_chassis_velocity((1.0, 2.0, 3.0))
    with pytest.raises(ValueError, match="finite"):
        estimate_chassis_velocity((0.0, 0.0, float("nan"), 0.0))


def test_robot_move_vector_uses_twist_backend_with_si_units():
    calls = []
    car = Car(SetMotorFunc=None, GetYaw=lambda: 0.0, SetChassisTwistFunc=lambda *v: calls.append(v))
    car.RobotMoveVector(0.1, 0, 0)
    car.RobotMoveVector(0, 0.1, 0)
    car.RobotTurn(50)
    car.stop()
    assert calls[0] == (0.1, 0.0, 0.0)
    assert calls[1] == (0.0, 0.1, 0.0)
    assert calls[2] == (0.0, 0.0, pytest.approx(math.radians(50)))
    assert calls[3] == (0.0, 0.0, 0.0)


def test_si_twist_and_lifecycle_callbacks():
    calls = []
    car = Car(
        SetChassisTwistFunc=lambda *v: calls.append(("twist", v)),
        StopFunc=lambda: calls.append(("stop",)),
        DisableFunc=lambda: calls.append(("disable",)),
        ShutdownFunc=lambda: calls.append(("shutdown",)),
        EmergencyStopFunc=lambda: calls.append(("emergency",)),
    )
    car.RobotMoveTwist(0.1, -0.2, 0.3)
    car.stop(); car.disable(); car.shutdown(); car.emergency_stop()
    assert ("twist", (0.1, -0.2, 0.3)) in calls
    assert ("stop",) in calls
    assert ("disable",) in calls and ("shutdown",) in calls and ("emergency",) in calls


def test_dm_disable_waits_for_confirmed_disabled(monkeypatch):
    from utils.DM2325Bits import DM2325Bits
    from types import SimpleNamespace

    bits = DM2325Bits()
    calls = []
    monkeypatch.setattr(bits, "_exchange", lambda *args, **kwargs: calls.append(args))
    status = SimpleNamespace(enabled_mask=0, all_disabled=True)
    monkeypatch.setattr(bits, "_wait_for_state", lambda state: status if state == 1 else None)
    assert bits.disable() is True
    assert calls


def test_refresh_disabled_telemetry_uses_safe_refresh_message(monkeypatch):
    from utils.DM2325Bits import DM2325Bits, MsgType
    from types import SimpleNamespace

    bits = DM2325Bits()
    payload = bytes(248)
    response = SimpleNamespace(payload=payload)
    calls = []
    monkeypatch.setattr(bits, "_exchange", lambda *args, **kwargs: calls.append(args) or response)
    status = bits.RefreshDisabledTelemetry()
    assert calls[0][0] == MsgType.CHASSIS_REFRESH_DISABLED
    assert status.runtime_state == 0


def test_dm_emergency_stop_latches_fault_and_waits_for_confirmation(monkeypatch):
    from utils.DM2325Bits import DM2325Bits, MsgType
    from types import SimpleNamespace

    bits = DM2325Bits()
    calls = []
    monkeypatch.setattr(bits, "_exchange", lambda *args, **kwargs: calls.append(args))
    status = SimpleNamespace(all_disabled=True, enabled_mask=0, runtime_fault=8)
    monkeypatch.setattr(bits, "_wait_for_state", lambda state: status if state == 6 else None)
    assert bits.emergency_stop() is True
    assert calls[0][0] == MsgType.CHASSIS_EMERGENCY_STOP


def test_dm_clear_fault_requires_fresh_disabled_confirmation(monkeypatch):
    from utils.DM2325Bits import DM2325Bits, MsgType
    from types import SimpleNamespace

    bits = DM2325Bits()
    calls = []
    refreshed = SimpleNamespace(all_disabled=True)
    monkeypatch.setattr(bits, "RefreshDisabledTelemetry", lambda: calls.append(("refresh",)) or refreshed)
    monkeypatch.setattr(bits, "_exchange", lambda *args, **kwargs: calls.append(args))
    status = SimpleNamespace(all_disabled=True, enabled_mask=0, runtime_fault=0)
    monkeypatch.setattr(bits, "_wait_for_state", lambda state: status if state == 1 else None)
    assert bits.ClearFault() is True
    assert calls[0] == ("refresh",)
    assert calls[1][0] == MsgType.CHASSIS_CLEAR_FAULT


def test_dm_shutdown_attempts_disable_after_stop_failure_and_always_closes(monkeypatch):
    from utils.DM2325Bits import DM2325Bits

    bits = DM2325Bits()
    calls = []
    stop_error = RuntimeError("stop failed")
    monkeypatch.setattr(bits, "stop", lambda: (_ for _ in ()).throw(stop_error))
    monkeypatch.setattr(bits, "disable", lambda: calls.append("disable") or True)
    monkeypatch.setattr(bits, "ClosePort", lambda: calls.append("close"))
    with pytest.raises(RuntimeError, match="stop failed"):
        bits.shutdown()
    assert calls == ["disable", "close"]
    assert bits.shutdown() is True
    assert calls == ["disable", "close", "close"]


@pytest.mark.parametrize("stop_fails", [False, True])
def test_dm_shutdown_retries_when_disabled_confirmation_fails(monkeypatch, stop_fails):
    from utils.DM2325Bits import DM2325Bits

    bits = DM2325Bits()
    calls = []

    def stop():
        calls.append("stop")
        if stop_fails:
            raise RuntimeError("stop failed")
        return True

    def disable_failed():
        calls.append("disable")
        raise RuntimeError("disable failed")

    monkeypatch.setattr(bits, "stop", stop)
    monkeypatch.setattr(bits, "disable", disable_failed)
    monkeypatch.setattr(bits, "ClosePort", lambda: calls.append("close"))

    with pytest.raises(RuntimeError, match="disable failed"):
        bits.shutdown()
    assert bits._shutdown_complete is False
    assert calls == ["stop", "disable", "close"]

    monkeypatch.setattr(bits, "stop", lambda: calls.append("retry-stop") or True)
    monkeypatch.setattr(bits, "disable", lambda: calls.append("retry-disable") or True)
    assert bits.shutdown() is True
    assert bits._shutdown_complete is True
    assert calls[-3:] == ["retry-stop", "retry-disable", "close"]


def test_dm_stop_and_disable_are_idempotent_when_already_safe(monkeypatch):
    from types import SimpleNamespace
    from utils.DM2325Bits import DM2325Bits

    bits = DM2325Bits()
    safe = SimpleNamespace(runtime_state=1, enabled_mask=0, all_disabled=True)
    monkeypatch.setattr(bits, "GetChassisTelemetry", lambda: safe)
    monkeypatch.setattr(
        bits, "SetChassisTwist",
        lambda *args: (_ for _ in ()).throw(RuntimeError("NOT_ENABLED")),
    )
    monkeypatch.setattr(
        bits, "_exchange",
        lambda *args, **kwargs: (_ for _ in ()).throw(RuntimeError("BUSY")),
    )
    assert bits.stop() is True
    assert bits.disable() is True


def test_dm_offline_zero_flags_are_not_confirmed_disabled():
    from utils.DM2325Bits import decode_chassis_status, status_from_protocol

    payload = bytes(248)
    status = status_from_protocol(decode_chassis_status(payload), payload, 1.0)
    assert status.enabled_mask == 0
    assert status.all_online is False
    assert status.any_stale is True
    assert status.all_disabled is False


def test_four_motor_enabled_feedback_code_one_is_healthy():
    from utils.DM2325Bits import FourMotorStatus, WheelMotorStatus

    motors = tuple(
        WheelMotorStatus(
            wheel=name, esc_id=index + 1, master_id=0x11 + index,
            online=True, enabled=True, fault=False, fault_code=1,
            position_rad=0.0, speed_rad_s=0.0, torque_nm=0.0,
            temperature_c=30.0, feedback_sequence=1, feedback_age_ms=1.0,
        )
        for index, name in enumerate(("FL", "FR", "RL", "RR"))
    )
    status = FourMotorStatus(
        motors=motors, snapshot_sequence=1, received_monotonic=1.0,
        online_mask=0x0F, enabled_mask=0x0F, fault_mask=0, stale_mask=0,
        can_esr=0, runtime_state=3, runtime_fault=0, raw_payload=b"",
    )
    assert status.all_enabled is True


def test_car_shutdown_delegates_to_atomic_backend_without_pre_stop():
    calls = []
    car = Car(
        StopFunc=lambda: calls.append(("stop",)),
        ShutdownFunc=lambda: calls.append(("shutdown",)),
    )
    car.shutdown()
    assert calls == [("shutdown",)]


def test_dm_arm_rejects_unconfirmed_disabled_snapshot(monkeypatch):
    from types import SimpleNamespace
    from utils.DM2325Bits import DM2325Bits

    bits = DM2325Bits()
    offline = SimpleNamespace(runtime_state=1, all_disabled=False)
    monkeypatch.setattr(bits, "RefreshDisabledTelemetry", lambda: offline)
    monkeypatch.setattr(bits, "_exchange", lambda *args, **kwargs: pytest.fail("unexpected ARM"))
    with pytest.raises(RuntimeError, match="four fresh Disabled"):
        bits.Arm()


def test_dm_arm_uses_zero_twist_as_ready_watchdog_handshake(monkeypatch):
    from types import SimpleNamespace
    from utils.DM2325Bits import DM2325Bits

    bits = DM2325Bits()
    disabled = SimpleNamespace(runtime_state=1, all_disabled=True)
    calls = []
    twist_attempts = iter((RuntimeError("NOT_ENABLED"), None))
    monkeypatch.setattr(bits, "RefreshDisabledTelemetry", lambda: disabled)
    monkeypatch.setattr(bits, "_exchange", lambda *args, **kwargs: calls.append(("arm", args)))
    monkeypatch.setattr(bits, "GetChassisTelemetry", lambda: pytest.fail("unexpected STATUS"))
    def set_twist(*twist):
        calls.append(("twist", twist))
        error = next(twist_attempts)
        if error is not None:
            raise error

    monkeypatch.setattr(bits, "SetChassisTwist", set_twist)

    assert bits.Arm() is True
    assert calls[1] == ("twist", (0.0, 0.0, 0.0))
    assert calls[2] == ("twist", (0.0, 0.0, 0.0))


def test_dm_stop_uses_zero_and_ack_without_status_roundtrip(monkeypatch):
    from utils.DM2325Bits import DM2325Bits

    bits = DM2325Bits()
    calls = []
    monkeypatch.setattr(bits, "GetChassisTelemetry", lambda: pytest.fail("unexpected STATUS"))
    monkeypatch.setattr(bits, "SetChassisTwist", lambda *args: calls.append(("twist", args)))
    monkeypatch.setattr(bits, "_exchange", lambda *args, **kwargs: calls.append(("exchange", args)))
    assert bits.stop() is True
    assert calls[0] == ("twist", (0.0, 0.0, 0.0))
    assert calls[1][0] == "exchange"
