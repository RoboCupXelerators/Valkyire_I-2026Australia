"""Focused contracts for LunaPre's semantic sensor and Bits pose boundary."""

from __future__ import annotations

import math
import threading
import time

import pytest

from LunaPre import runtime
from LunaPre.positions import Positions
from LunaPre.lidar_driver_ld07 import LD07Driver
from LunaPre.sensor import LIDAR_DISABLED_DISTS, Lidar, Sensor, _load_driver_class
from LunaPre.strategy_tools import POSE_NOT_FOUND
import utils.IceLoongBits as iceloong_bits_module
from utils.IceLoongBits import IceLoongBits
from utils.XeleratorSimBits import SimStatus, XeleratorSimBits


class _RawLidar:
    def __init__(self, backend: str, distances):
        self.backend = backend
        self.distances = distances
        self.last_point_cloud = ((1.0, 2.0),)
        self.running = True
        self.disabled = False
        self.driver = object()
        self.last_line_count = 12
        self.last_scan_point_count = 34
        self.last_raw_scan_count = 56

    def GetDists(self):
        return self.distances

    def GetNearbyObjects(self):
        return [{"distance_cm": 20.0}]


def test_auto_sensor_prefers_bits_pose_and_derives_four_walls():
    sensor = Sensor(
        pose_provider=lambda: (10.0, -20.0, 0.0),
        lidar=_RawLidar("ros2", [1.0, 2.0, 3.0, 4.0]),
        source="auto",
        bounds_long_cm=260.0,
        bounds_short_cm=200.0,
    )

    assert sensor.DirectDistance() == [1200.0, 1200.0, 1400.0, 800.0]
    assert sensor.last_source == "bits"


def test_auto_sensor_falls_back_from_unavailable_bits_to_active_ros2():
    sensor = Sensor(
        pose_provider=lambda: None,
        lidar=_RawLidar("ros2", [111.0, 222.0, 333.0, 444.0]),
        source="auto",
    )

    assert sensor.DirectDistance() == [111.0, 222.0, 333.0, 444.0]
    assert sensor.last_source == "ros2"


def test_sensor_pose_honors_explicit_source_and_auto_fallback_order():
    raw_lidar = _RawLidar("ros2", [1200.0, 1200.0, 1400.0, 800.0])
    auto = Sensor(
        pose_provider=lambda: None,
        yaw_provider=lambda: 15.0,
        lidar=raw_lidar,
        source="auto",
        bounds_long_cm=260.0,
        bounds_short_cm=200.0,
    )
    ros2 = Sensor(
        pose_provider=lambda: (50.0, -25.0, 90.0),
        yaw_provider=lambda: 15.0,
        lidar=raw_lidar,
        source="ros2",
        bounds_long_cm=260.0,
        bounds_short_cm=200.0,
    )

    assert auto.GetPose() == (10.0, 20.0, 15.0)
    assert auto.last_source == "ros2"
    assert ros2.GetPose() == (10.0, 20.0, 15.0)
    assert ros2.last_source == "ros2"


def test_sensor_source_requires_matching_native_backend_and_reports_disabled():
    sensor = Sensor(
        pose_provider=lambda: None,
        lidar=_RawLidar("ros2", [111.0, 222.0, 333.0, 444.0]),
        source="ld07",
    )

    assert sensor.DirectDistance() == LIDAR_DISABLED_DISTS
    assert sensor.last_source is None
    with pytest.raises(ValueError, match="Sensor.Source"):
        sensor.Source = "rplidar"


def test_sensor_delegates_raw_lidar_diagnostics_for_control_panel_compatibility():
    raw_lidar = _RawLidar("ld07", [111.0, 222.0, 333.0, 444.0])
    sensor = Sensor(pose_provider=lambda: None, lidar=raw_lidar, source="ld07")

    assert sensor.running is True
    assert sensor.disabled is False
    assert sensor.driver is raw_lidar.driver
    assert sensor.last_point_cloud == ((1.0, 2.0),)
    assert sensor.last_line_count == 12
    assert sensor.last_scan_point_count == 34
    assert sensor.last_raw_scan_count == 56


def test_ld07_source_loads_the_tracked_direct_driver():
    lidar = Lidar(backend="ld07", auto_start=False)

    assert lidar.backend == "ld07"
    assert _load_driver_class("ld07") is LD07Driver


def test_sensor_retains_lidar_geometric_pose_for_diagnostics_only():
    sensor = Sensor(
        pose_provider=lambda: None,
        yaw_provider=lambda: 15.0,
        lidar=_RawLidar("ros2", [1200.0, 1200.0, 1400.0, 800.0]),
        source="ros2",
        bounds_long_cm=260.0,
        bounds_short_cm=200.0,
    )

    assert sensor.GetGeometricPose() == [10.0, 20.0, 15.0]


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
        yaw_provider=assembly["compass"],
        lidar=assembly["raw_lidar"],
        source="auto",
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


def test_sim_runtime_uses_bits_sensor_source_for_stale_hardware_config(monkeypatch):
    monkeypatch.setattr(runtime.cfg.Sensor, "Source", "ros2")
    monkeypatch.delenv("XEL_SIM_SENSOR_SOURCE", raising=False)
    assert runtime._resolve_runtime_sensor_source("SIM") == "bits"


def test_sim_runtime_sensor_source_can_be_explicitly_overridden(monkeypatch):
    monkeypatch.setenv("XEL_SIM_SENSOR_SOURCE", "auto")
    assert runtime._resolve_runtime_sensor_source("SIM") == "auto"
