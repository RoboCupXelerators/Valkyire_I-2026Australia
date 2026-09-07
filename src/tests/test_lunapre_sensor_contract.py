"""Focused contracts for LunaPre's semantic sensor and Bits pose boundary."""

from __future__ import annotations

import math
import threading
import time

import pytest

from LunaPre import runtime
from LunaPre.positions import Positions
from LunaPre.sensor import LIDAR_DISABLED_DISTS, Sensor
from LunaPre.strategy_tools import POSE_NOT_FOUND
import utils.IceLoongBits as iceloong_bits_module
from utils.IceLoongBits import IceLoongBits
from utils.XeleratorSimBits import SimStatus, XeleratorSimBits


@pytest.mark.parametrize("pose", [None, (1, 2), (1, 2, math.nan), (math.inf, 0, 0)])
def test_unavailable_or_invalid_pose_has_no_fabricated_walls(pose):
    sensor = Sensor(pose_provider=lambda: pose)
    assert sensor.GetPose() is None
    assert sensor.DirectDistance() == LIDAR_DISABLED_DISTS


@pytest.mark.parametrize("yaw, expected", [
    (0, [1200, 1200, 1400, 800]),
    (90, [1200, 1400, 800, 1200]),
    (180, [1400, 800, 1200, 1200]),
    (270, [800, 1200, 1200, 1400]),
])
def test_bits_pose_derives_robot_frame_wall_ranges(yaw, expected):
    calls = []
    def pose():
        calls.append(True)
        return (10, -20, yaw)
    sensor = Sensor(pose_provider=pose, bounds_long_cm=260, bounds_short_cm=200)
    assert sensor.DirectDistance() == pytest.approx(expected)
    assert len(calls) == 1


def test_failed_provider_keeps_pose_unavailable():
    def unavailable():
        raise OSError("disconnected")
    sensor = Sensor(pose_provider=unavailable)
    assert sensor.GetPose() is None
    assert sensor.DirectDistanceSnapshot() == LIDAR_DISABLED_DISTS


def test_field_robot_pose_reads_only_runtime_pose_without_sensor_or_motion(monkeypatch):
    position = object.__new__(Positions)
    position.LidarPos = list(POSE_NOT_FOUND)
    monkeypatch.setattr(runtime, "get_field_pose", lambda: None)
    monkeypatch.setattr(position, "DirectDistance", lambda: pytest.fail("pose must not read wall distances"))

    assert position.FieldRobotPose() == POSE_NOT_FOUND

    monkeypatch.setattr(runtime, "get_field_pose", lambda: [12.5, -8.0, 90.0])
    assert position.FieldRobotPose() == [12.5, -8.0, 90.0]


def _pose_bits(status: SimStatus) -> XeleratorSimBits:
    bits = object.__new__(XeleratorSimBits)
    bits._status = status
    bits._status_lock = threading.Lock()
    bits._unity_to_cm = 100.0
    bits._origin_x_cm = 0.0
    bits._origin_y_cm = 0.0
    bits._pose_scale = 1.0
    bits._yaw_zero_at_start = False
    bits._yaw_zero_deg = None
    bits._yaw_offset_deg = 0.0
    bits._side_yaw_offset_deg = 0.0
    bits._pose_max_age_s = 1.0
    return bits


def test_sim_bits_pose_is_none_until_a_fresh_finite_status_arrives():
    bits = _pose_bits(SimStatus())
    assert bits.GetPose() is None

    bits._status = SimStatus(ts=time.time(), x_m=0.12, y_m=-0.34, yaw_deg=45.0)
    assert bits.GetPose() == (12.0, -34.0, 45.0)

    bits._status = SimStatus(ts=time.time() - 2.0, x_m=0.12, y_m=-0.34, yaw_deg=45.0)
    assert bits.GetPose() is None

    bits._status = SimStatus(ts=time.time(), x_m=math.nan, y_m=0.0, yaw_deg=0.0)
    assert bits.GetPose() is None


def test_iceloong_pose_contract_uses_motion_controller_pose():
    bits = IceLoongBits(testing=True)
    bits.ExecuteCommands = lambda commands: [
        (0x83, 45.0, 1.0, -2.0, 120.0, -340.0, 0.0)
    ]

    assert bits.GetPose() == (12.0, -34.0, 45.0)


def test_iceloong_runtime_wires_bits_pose_to_field_robot_pose(monkeypatch):
    class FakeIceLoongBits:
        def __init__(self, port):
            self.port = port
            self.io_calls = []

        def GetPose(self):
            return (12.5, -34.0, 270.0)

        def GetYaw(self):
            return 270.0

        def SetWheelSpeed(self, *_speeds):
            pass

        def VirtualPorts(self, port, value):
            self.io_calls.append((port, value))

        def GetRefereeUnlockPin(self):
            return True

        def MorsePlay(self, _message):
            pass

    monkeypatch.setattr(iceloong_bits_module, "IceLoongBits", FakeIceLoongBits)
    assembly = runtime._build_iceloong_backend(auto_start_lidar=False)
    sensor = Sensor(
        pose_provider=assembly["field_pose_provider"],
    )
    monkeypatch.setattr(runtime, "_field_pose_provider", sensor.GetPose)

    assert sensor.GetPose() == (12.5, -34.0, 270.0)
    assert runtime.get_field_pose() == [12.5, -34.0, 270.0]

    positions = object.__new__(Positions)
    positions.LidarPos = list(POSE_NOT_FOUND)
    assert positions.FieldRobotPose() == [12.5, -34.0, 270.0]


def test_optional_morse_output_is_safe_when_backend_has_none(monkeypatch):
    monkeypatch.setattr(runtime, "Morse", None)
    runtime.play_morse("No Position Data!")


def test_optional_morse_output_delegates_to_hardware_callback(monkeypatch):
    messages = []
    monkeypatch.setattr(runtime, "Morse", messages.append)
    runtime.play_morse("No Position Data!")
    assert messages == ["No Position Data!"]


def test_bits_thread_monitor_owns_its_stop_event(monkeypatch):
    class FakeBits:
        def IsThreadAlive(self):
            return True

        def RestartThreadIfNeeded(self):
            raise AssertionError("a live fake thread should not be restarted")

    runtime._stop_bits_thread_monitor()
    runtime._start_bits_thread_monitor(FakeBits())
    first_thread = runtime._bits_monitor_thread

    try:
        runtime._start_bits_thread_monitor(FakeBits())
        assert first_thread is not runtime._bits_monitor_thread
        assert not first_thread.is_alive()
    finally:
        runtime._stop_bits_thread_monitor()

    assert runtime._bits_monitor_thread is None
    assert runtime._bits_monitor_stop is None
