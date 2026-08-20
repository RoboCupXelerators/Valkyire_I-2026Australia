"""Historical direct-RPLidar tests, excluded from active pytest collection."""

import queue

from .lidar_driver_rplidar import RPLidarDriver, _resolve_serial_port


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


def test_rplidar_missing_default_port_uses_unique_serial_alias(monkeypatch):
    monkeypatch.setattr("LunaPre.lidar_driver_rplidar.os.path.exists", lambda path: path != "/dev/sllidar")
    monkeypatch.setattr(
        "LunaPre.lidar_driver_rplidar.glob.glob",
        lambda pattern: ["/dev/serial/by-id/usb-test"] if "by-id" in pattern else [],
    )
    monkeypatch.setattr("LunaPre.lidar_driver_rplidar.os.path.realpath", lambda path: "/dev/ttyACM0")

    assert _resolve_serial_port("/dev/sllidar") == "/dev/serial/by-id/usb-test"
