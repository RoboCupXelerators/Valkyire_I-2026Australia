import ast
import io
import json
import math
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest

from utils import DM2325Bits as bits_module


pytestmark = pytest.mark.imu

def imu_state(*, sequence=10, timestamp=100_000, sample_count=80,
              accel=(0.1, -0.2, 9.8), gyro=(0.01, -0.02, 0.03)):
    return SimpleNamespace(
        sequence=sequence,
        cboard_timestamp=timestamp,
        received_monotonic=10.0 + sequence / 100.0,
        accel_mps2=accel,
        gyro_rad_s=gyro,
        roll_rad=0.1,
        pitch_rad=-0.2,
        yaw_rad=1.3,
        temperature_c=33.5,
        age_ms=1.2,
        flags=1 << 14,
        imu_online=True,
        imu_ready=True,
        imu_calibrated=True,
        imu_stale=False,
        imu_fault=False,
        healthy=False,
        calibration_state=2,
        state=3,
        sample_count=sample_count,
        missed_sample_count=0,
        spi_error_count=0,
    )


def test_json_line_has_hardware_fields_increasing_sequence_and_strict_numbers():
    tracker = bits_module.IMUJsonRateTracker()
    first = bits_module.build_imu_web_record(imu_state(), tracker)
    second = bits_module.build_imu_web_record(
        imu_state(sequence=20, timestamp=112_500, sample_count=90), tracker)
    decoded = json.loads(bits_module.encode_imu_web_record(second))
    assert first["sequence"] < decoded["sequence"]
    assert decoded["source"] == "hardware"
    assert decoded["healthy"] is True
    assert len(decoded["accel_mps2"]) == len(decoded["gyro_rad_s"]) == 3
    assert decoded["age_ms"] >= 0
    assert decoded["sample_hz"] == 800.0
    assert decoded["axis_mapping_verified"] is False
    assert decoded["robot_direction_defined"] is False


def test_nan_and_inf_become_null_and_cannot_be_healthy_or_invalid_json():
    state = imu_state(accel=(math.nan, 0.0, 9.8), gyro=(0.0, math.inf, 0.0))
    record = bits_module.build_imu_web_record(state)
    rendered = bits_module.encode_imu_web_record(record)
    assert "NaN" not in rendered and "Infinity" not in rendered
    decoded = json.loads(rendered)
    assert decoded["accel_mps2"][0] is None
    assert decoded["gyro_rad_s"][1] is None
    assert decoded["healthy"] is False
    assert decoded["flags"] & bits_module.HOST_FLAG_NONFINITE


def test_stream_stdout_is_json_only_errors_use_stderr_and_port_always_closes():
    calls = []

    class FakeBits:
        def OpenPort(self):
            calls.append("OpenPort")

        def GetIMUState(self):
            calls.append("GetIMUState")
            if calls.count("GetIMUState") == 1:
                raise RuntimeError("discard me")
            return imu_state()

        def ClosePort(self):
            calls.append("ClosePort")

    stdout = io.StringIO()
    stderr = io.StringIO()
    emitted = bits_module.run_imu_json_stream(
        FakeBits(), stdout, stderr, threading.Event(), 0.0001, max_frames=1)
    lines = stdout.getvalue().splitlines()
    assert emitted == 1 and len(lines) == 1
    assert json.loads(lines[0])["source"] == "hardware"
    assert "discard me" not in stdout.getvalue()
    assert "discard me" in stderr.getvalue()
    assert calls[0] == "OpenPort" and calls[-1] == "ClosePort"


def test_pre_stopped_stream_still_opens_and_closes_without_output():
    calls = []
    bits = SimpleNamespace(
        OpenPort=lambda: calls.append("open"),
        ClosePort=lambda: calls.append("close"),
    )
    stopped = threading.Event()
    stopped.set()
    assert bits_module.run_imu_json_stream(
        bits, io.StringIO(), io.StringIO(), stopped, 0.01) == 0
    assert calls == ["open", "close"]


def test_stream_source_can_only_call_read_only_bits_methods():
    path = Path(bits_module.__file__)
    tree = ast.parse(path.read_text())
    stream_function = next(
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "run_imu_json_stream"
    )
    calls = {
        node.func.attr
        for node in ast.walk(stream_function)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and isinstance(node.func.value, ast.Name)
        and node.func.value.id == "bits"
    }
    assert calls == {"OpenPort", "GetIMUState", "ClosePort"}
    source = path.read_text()
    stream_source = ast.get_source_segment(source, stream_function)
    assert "PASS_AXIS_MAPPING" not in stream_source
    assert "ZeroYaw" not in stream_source
    assert "SetYawReference" not in stream_source
