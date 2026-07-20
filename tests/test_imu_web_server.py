import json
import math
import threading
import time
import urllib.request
from pathlib import Path

import pytest

from tools.imu_web import server


def sample(sequence=1, source="hardware"):
    return {
        "sequence": sequence,
        "timestamp_us": sequence * 1000,
        "source": source,
        "healthy": True,
        "age_ms": 1.0,
        "flags": 1 << 14,
        "accel_mps2": [0.1, 0.2, 9.8],
        "gyro_rad_s": [0.01, 0.02, 0.03],
        "roll_rad": 0.1,
        "pitch_rad": 0.2,
        "yaw_rad": 0.3,
        "temperature_c": 33.0,
        "spi_error_count": 0,
        "missed_sample_count": 0,
    }


def test_parser_rejects_nonfinite_missing_and_wrong_vector_data():
    assert server.parse_json_line(json.dumps(sample()))["source"] == "hardware"
    with pytest.raises(ValueError):
        server.parse_json_line(json.dumps({**sample(), "yaw_rad": math.nan}))
    with pytest.raises(ValueError):
        server.parse_json_line(json.dumps({**sample(), "gyro_rad_s": [1, 2]}))
    missing = sample()
    missing.pop("sequence")
    with pytest.raises(ValueError):
        server.parse_json_line(json.dumps(missing))


def _start_http_server():
    httpd = server.ThreadingHTTPServer(("127.0.0.1", 0), server.Handler)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    return httpd, thread


def test_status_endpoint_returns_latest_hardware_state_and_timeout(monkeypatch):
    server.reset_runtime_state()
    now = [100.0]
    monkeypatch.setattr(server.time, "monotonic", lambda: now[0])
    server.publish(sample(7))
    httpd, thread = _start_http_server()
    try:
        url = f"http://127.0.0.1:{httpd.server_port}/api/status"
        fresh = json.load(urllib.request.urlopen(url, timeout=2))
        assert fresh["source"] == "hardware"
        assert fresh["sequence"] == 7
        assert fresh["stream_connected"] is True
        now[0] += server.STREAM_TIMEOUT_S + 0.1
        stale = json.load(urllib.request.urlopen(url, timeout=2))
        assert stale["healthy"] is False
        assert stale["stream_connected"] is False
    finally:
        httpd.shutdown()
        httpd.server_close()
        thread.join(timeout=2)
        server.STOP.set()


def test_stream_endpoint_pushes_multiple_sse_samples():
    server.reset_runtime_state()
    server.publish(sample(10))
    httpd, thread = _start_http_server()
    try:
        url = f"http://127.0.0.1:{httpd.server_port}/api/stream"
        response = urllib.request.urlopen(url, timeout=2)
        first = response.readline().decode().strip()
        assert json.loads(first.removeprefix("data: "))["sequence"] == 10
        assert response.readline() == b"\n"
        server.publish(sample(11))
        second = response.readline().decode().strip()
        assert json.loads(second.removeprefix("data: "))["sequence"] == 11
        response.close()
    finally:
        server.STOP.set()
        with server.COND:
            server.COND.notify_all()
        httpd.shutdown()
        httpd.server_close()
        thread.join(timeout=2)


def test_capture_subprocess_is_killed_with_its_process_group_on_stop():
    server.reset_runtime_state()
    worker = threading.Thread(
        target=server.command_source,
        args=("exec python3 -c 'import time; time.sleep(60)'", 0, 2.0),
    )
    worker.start()
    deadline = time.monotonic() + 3
    proc = None
    while time.monotonic() < deadline:
        with server.PROCESS_LOCK:
            proc = server.ACTIVE_PROCESS
        if proc is not None:
            break
        time.sleep(0.01)
    assert proc is not None and proc.poll() is None
    server.STOP.set()
    server.stop_active_process()
    worker.join(timeout=3)
    assert not worker.is_alive()
    assert proc.poll() is not None
    with server.PROCESS_LOCK:
        assert server.ACTIVE_PROCESS is None


def test_page_has_deadman_control_and_bounded_speed_without_extra_web_dependency():
    html = (Path(server.__file__).parent / "index.html").read_text()
    compact_html = " ".join(html.split())
    assert "数据超时" in html
    assert "停止并导出 CSV" in html
    assert "中断 200 ms 后自动归零并 Disable" in compact_html
    assert 'max="2.29"' in html
    assert 'id="driveMeasuredSpeed"' in html
    assert 'id="driveMeasuredTwist"' in html
    assert 'id="drivePeakSpeed"' in html
    assert 'id="driveMotorTorque"' in html
    assert 'id="drivePeakTorque"' in html
    assert 'id="joystick"' in html
    assert 'id="joystickKnob"' in html
    assert "updateJoystick" in html
    assert "pressedMoveKeys" in html
    assert "Math.hypot(vx, vy)" in html
    assert html.index('id="driveHeading"') < html.index('id="level"')
    assert 'setPointerCapture' in html
    assert "pointerleave" not in html
    assert 'aria-label="前进" disabled' in compact_html
    assert "drive.transitioning = true" in html
    assert "drive.armed = false" in html
    assert "if (drive.pending) await drive.pending" in html
    assert "if (!drive.transitioning) drive.armed" in html
    assert "drive.ownsControl" in html
    assert "其他页面控制" in html
    assert 'if (drive.ownsControl) drivePost("disable")' in html
    assert 'state(true, "控制中")' in html
    assert "/api/control/arm" not in html  # Endpoints are assembled from fixed actions.
    source = Path(server.__file__).read_text()
    assert "serial.Serial" not in source
    assert "MAX_LINEAR_MPS = 2.29" in source
    assert "CONTROL_HEARTBEAT_S = 0.02" in source
    assert "CONTROL_DEADMAN_S = 0.20" in source
    assert "bits.SetChassisTwist(*target_twist)" in source
    assert "A zero heartbeat may already be in flight" in source
    assert 'was_armed = control_snapshot()["armed"]' in source
    assert "if was_armed:" in source
    assert 'owner_client_id != client_id' in source
    assert 'if any(abs(value) > 1e-6 for value in (vx, vy, wz))' in source
    assert "from utils.DM2325Bits import" in source


def test_control_telemetry_exposes_encoder_speedometer_values():
    from types import SimpleNamespace

    estimate = SimpleNamespace(vx_mps=0.28, vy_mps=0.03, wz_rad_s=-0.04,
                               speed_mps=0.2816,
                               motor_rad_s=(-6.0, 6.1, -7.0, 7.1))
    status = SimpleNamespace(
        online_mask=15, enabled_mask=15, fault_mask=0,
        runtime_state=4, runtime_fault=0,
        motors=[SimpleNamespace(feedback_age_ms=age, torque_nm=torque)
                for age, torque in zip((2, 3, 4, 5), (-0.2, 0.3, -0.4, 0.5))],
        velocity_estimate=estimate,
    )
    values = server._telemetry_dict(status)
    assert values["measured_speed_mps"] == pytest.approx(0.2816)
    assert values["measured_vx_mps"] == pytest.approx(0.28)
    assert values["measured_vy_mps"] == pytest.approx(0.03)
    assert values["measured_wz_rad_s"] == pytest.approx(-0.04)
    assert values["motor_speed_rad_s"] == [-6.0, 6.1, -7.0, 7.1]
    assert values["feedback_age_ms"] == 5
    assert values["motor_torque_nm"] == [-0.2, 0.3, -0.4, 0.5]
    assert values["max_abs_torque_nm"] == pytest.approx(0.5)


def test_control_rejects_missing_hardware_and_out_of_range_values():
    server.reset_runtime_state()
    assert server.submit_control("arm")["success"] is False
    assert server._finite_limited(0.15, 0.15, "vx") == 0.15
    with pytest.raises(ValueError):
        server._finite_limited(0.151, 0.15, "vx")
    with pytest.raises(ValueError):
        server._finite_limited(math.nan, 0.15, "vx")


def test_control_status_endpoint_starts_disabled():
    server.reset_runtime_state()
    httpd, thread = _start_http_server()
    try:
        url = f"http://127.0.0.1:{httpd.server_port}/api/control/status"
        status = json.load(urllib.request.urlopen(url, timeout=2))
        assert status["armed"] is False
        assert status["hardware"] is False
        assert status["enabled_mask"] == 0
    finally:
        httpd.shutdown()
        httpd.server_close()
        thread.join(timeout=2)
