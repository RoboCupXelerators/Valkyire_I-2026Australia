import struct
import queue

from LunaPre.lidar_driver_ld07 import (
    EXPECTED_DATA_LENGTH as LD07_DATA_LENGTH,
    GET_DISTANCE_COMMAND,
    LD07Driver,
    SYNC_VALUE,
)
from LunaPre.lidar_driver_ldlidar import (
    LDLidarDriver,
    PACKET_LENGTH,
    PACKET_VERSION_LENGTH,
)


def _make_ld06_frame(
    points=None,
    *,
    speed=2152,
    start_angle=1000,
    end_angle=2100,
    timestamp=1234,
):
    if points is None:
        points = [(321 + index, 20 + index) for index in range(12)]
    packet = bytearray(struct.pack("<BBHH", 0x54, PACKET_VERSION_LENGTH, speed, start_angle))
    for distance_mm, intensity in points:
        packet.extend(struct.pack("<HB", distance_mm, intensity))
    packet.extend(struct.pack("<HH", end_angle, timestamp))
    assert len(packet) == PACKET_LENGTH - 1
    packet.append(LDLidarDriver.crc8(packet))
    return bytes(packet)


def _make_ld07_frame(values, timestamp=1234, device=1, command=GET_DISTANCE_COMMAND):
    payload = struct.pack("<L", timestamp) + b"".join(
        struct.pack("<H", value) for value in values
    )
    payload += b"\x00" * (LD07_DATA_LENGTH - len(payload))
    header = struct.pack("<LBBHH", SYNC_VALUE, device, command, 0, len(payload))
    checksum = (sum(header[4:]) + sum(payload)) & 0xFF
    return header + payload + bytes([checksum])


def test_ld06_defaults_use_protocol_baudrate_and_configurable_port():
    driver = LDLidarDriver(port="/dev/test-ld06")
    assert driver.baudrate == 230400
    assert driver.port == "/dev/test-ld06"


def test_decode_ld06_frame_extracts_distance_intensity_and_interpolated_angles():
    driver = LDLidarDriver(angle_offset_deg=1.0)
    frame = _make_ld06_frame(
        [(321, 200), (123, 37)] + [(500, 10)] * 10,
        start_angle=1000,
        end_angle=2100,
    )

    decoded = driver.decode_packet(frame)

    assert decoded is not None
    assert decoded["speed_deg_s"] == 2152
    assert decoded["timestamp"] == 1234
    assert decoded["sample_count"] == 12
    assert decoded["samples"][0]["distance_mm"] == 321
    assert decoded["samples"][0]["intensity"] == 200
    assert decoded["samples"][1]["distance_mm"] == 123
    assert decoded["samples"][1]["confidence"] == 37
    assert decoded["samples"][0]["angle_deg"] == 11.0
    assert decoded["samples"][-1]["angle_deg"] == 22.0


def test_decode_ld06_wraps_angles_and_rejects_bad_crc():
    driver = LDLidarDriver()
    frame = bytearray(_make_ld06_frame(start_angle=35000, end_angle=1000))
    decoded = driver.decode_packet(frame)
    assert decoded is not None
    assert decoded["samples"][0]["angle_deg"] == 350.0
    assert decoded["samples"][-1]["angle_deg"] == 10.0

    frame[-1] ^= 0x01
    assert driver.decode_packet(frame) is None


def test_take_ld06_frames_handles_noise_and_partial_input():
    frame = _make_ld06_frame()
    buffer = bytearray(b"noise" + frame[:23])
    assert list(LDLidarDriver._take_frames(buffer)) == []
    buffer.extend(frame[23:])
    assert list(LDLidarDriver._take_frames(buffer)) == [frame]
    assert buffer == bytearray()


def test_ld06_scan_count_is_valid_points_per_rotation():
    driver = LDLidarDriver(scan_min_points=1)
    scan_queue = queue.Queue()
    scan = [(index * 0.5, 0.9) for index in range(347)]

    driver._publish_rotation(scan_queue, scan)

    assert driver.last_packet_point_count == 0
    assert driver.last_scan_point_count == 347
    assert len(scan_queue.get_nowait()) == 347


def test_detect_ld06_objects_groups_near_field_points():
    driver = LDLidarDriver()
    decoded = driver.decode_packet(
        _make_ld06_frame(
            [(0, 0), (0, 0), (200, 10), (205, 11), (210, 12)] + [(0, 0)] * 7,
            start_angle=0,
            end_angle=1100,
        )
    )
    assert decoded is not None
    driver.last_packet = decoded

    objects = driver.detect_objects()

    assert len(objects) == 1
    assert objects[0]["point_count"] == 3
    assert objects[0]["distance_cm"] <= 21.0


def test_ld07_protocol_remains_available():
    driver = LD07Driver()
    frame = _make_ld07_frame([321, (37 << 9) | 123])

    decoded = driver.decode_packet(frame)

    assert driver.baudrate == 921600
    assert decoded is not None
    assert decoded["sample_count"] == 160
    assert decoded["samples"][0]["distance_mm"] == 321
    assert decoded["samples"][1]["confidence"] == 37
