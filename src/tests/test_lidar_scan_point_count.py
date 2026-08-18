import math
import queue

from ReasonData import Settings
from LunaPre.lidar_driver_ros2 import _ROSLidarParser
from LunaPre.lidar_driver_rplidar import RPLidarDriver, _resolve_serial_port
from LunaPre.sensor import _is_valid_scan_distance, _resolve_backend


class _FakeLaserScan:
    def __init__(self, ranges, angle_min=0.0, angle_increment=math.radians(1.0)):
        self.ranges = ranges
        self.angle_min = angle_min
        self.angle_increment = angle_increment


class _FakeOwner:
    def __init__(self):
        self.last_publish_ts = 0.0
        self.last_publish_interval_s = 0.0
        self.last_scan_point_count = 0
        self.published_scan_count = 0
        self.dropped_scan_count = 0
        self.callback_error_count = 0


def test_valid_scan_distance_rejects_nan_inf_and_zero():
    assert _is_valid_scan_distance(1.25)
    assert not _is_valid_scan_distance(0)
    assert not _is_valid_scan_distance(float("nan"))
    assert not _is_valid_scan_distance(float("inf"))
    assert not _is_valid_scan_distance("bad")


def test_ros2_scan_count_uses_valid_ranges_not_dense_laser_array():
    owner = _FakeOwner()
    parser = object.__new__(_ROSLidarParser)
    parser.dirDistance = []
    parser.Queue = queue.Queue()
    parser.owner = owner

    # Dense S1-style frame: fixed length with many invalid slots.
    ranges = [float("inf")] * 1080
    for index in (10, 20, 30, 40, 50):
        ranges[index] = 1.0 + index / 100.0

    parser._handle_scan(_FakeLaserScan(ranges))

    published = parser.Queue.get_nowait()
    assert len(published) == 5
    assert owner.last_scan_point_count == 5
    assert all(0 < distance < float("inf") for _, distance in published)


def test_rplidar_driver_records_valid_points_per_rotation():
    driver = RPLidarDriver(port="/dev/null", use_scan_iterator=False)
    scan_queue = queue.Queue()
    scan = [(index * 0.5, 0.8) for index in range(271)]

    driver._publish_scan(scan_queue, scan)

    assert driver.last_scan_point_count == 271
    assert len(scan_queue.get_nowait()) == 271


def test_rplidar_s3_uses_one_megabaud_defaults():
    driver = RPLidarDriver(lidar_type="s3", port="/dev/null", use_scan_iterator=False)

    assert driver.baudrate == 1000000
    assert driver.max_buf_meas == 131072


def test_rplidar_legacy_driver_normalizes_inverted_angles():
    driver = RPLidarDriver(
        lidar_type="s1",
        port="/dev/null",
        inverted=True,
        angle_offset_deg=10.0,
        use_scan_iterator=False,
    )

    assert driver._normalize_angle(20.0) == 350.0


def test_rplidar_s3_legacy_driver_is_rejected_explicitly():
    driver = RPLidarDriver(lidar_type="s3", port="/dev/null")

    assert driver.start(queue.Queue()) is False
    assert "official ROS2" in driver.last_error


def test_s3_legacy_backend_routes_to_official_ros2_driver(monkeypatch):
    monkeypatch.setattr(Settings.RoboInfo, "LidarType", "s3")
    assert _resolve_backend("rplidar") == "ros2"


def test_rplidar_missing_default_port_uses_unique_serial_alias(monkeypatch):
    monkeypatch.setattr("LunaPre.lidar_driver_rplidar.os.path.exists", lambda path: path != "/dev/sllidar")
    monkeypatch.setattr(
        "LunaPre.lidar_driver_rplidar.glob.glob",
        lambda pattern: ["/dev/serial/by-id/usb-test"] if "by-id" in pattern else [],
    )
    monkeypatch.setattr("LunaPre.lidar_driver_rplidar.os.path.realpath", lambda path: "/dev/ttyACM0")

    assert _resolve_serial_port("/dev/sllidar") == "/dev/serial/by-id/usb-test"


def test_lidar_facade_raw_count_is_valid_points_not_dense_length():
    ranges = [(0.0, float("inf")), (1.0, 0.0), (2.0, 1.2), (3.0, 0.8), (4.0, float("nan"))]

    raw_point_count = 0
    near_point_count = 0
    for angle, distance in ranges:
        if not _is_valid_scan_distance(distance):
            continue
        raw_point_count += 1
        if distance < 2.5:
            near_point_count += 1

    assert raw_point_count == 2
    assert near_point_count == 2
