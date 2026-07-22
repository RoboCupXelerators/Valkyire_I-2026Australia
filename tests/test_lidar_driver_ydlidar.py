import struct

from LunaPre.lidar_driver_ydlidar import (
    EXPECTED_DATA_LENGTH,
    GET_DISTANCE_COMMAND,
    LD07Driver,
    SYNC_VALUE,
)


def _make_frame(values, timestamp=1234, device=1, command=GET_DISTANCE_COMMAND):
    payload = struct.pack("<L", timestamp) + b"".join(struct.pack("<H", value) for value in values)
    payload += b"\x00" * (EXPECTED_DATA_LENGTH - len(payload))
    header = struct.pack("<LBBHH", SYNC_VALUE, device, command, 0, len(payload))
    checksum = (sum(header[4:]) + sum(payload)) & 0xFF
    return header + payload + bytes([checksum])


def test_ld07_defaults_use_protocol_baudrate():
    assert LD07Driver().baudrate == 921600


def test_decode_ld07_frame_extracts_distance_confidence_and_angle():
    driver = LD07Driver(angle_start_deg=-45, fov_deg=90)
    frame = _make_frame([321, (37 << 9) | 123])

    decoded = driver.decode_packet(frame)

    assert decoded is not None
    assert decoded["timestamp"] == 1234
    assert decoded["sample_count"] == 160
    assert decoded["samples"][0]["distance_mm"] == 321
    assert decoded["samples"][0]["confidence"] == 0
    assert decoded["samples"][1]["distance_mm"] == 123
    assert decoded["samples"][1]["confidence"] == 37
    assert decoded["samples"][0]["angle_deg"] == -45
    assert decoded["samples"][-1]["angle_deg"] == -45 + 90 * 159 / 160


def test_decode_ld07_rejects_bad_checksum_and_wrong_command():
    driver = LD07Driver()
    frame = bytearray(_make_frame([100]))
    frame[-1] ^= 0x01
    assert driver.decode_packet(frame) is None

    assert driver.decode_packet(_make_frame([100], command=0x0F)) is None


def test_take_frames_handles_noise_and_partial_input():
    frame = _make_frame([100])
    buffer = bytearray(b"noise" + frame[:23])
    assert list(LD07Driver._take_frames(buffer)) == []
    buffer.extend(frame[23:])

    frames = list(LD07Driver._take_frames(buffer))

    assert frames == [frame]
    assert buffer == bytearray()


def test_detect_objects_groups_near_field_points_in_robot_coordinates():
    driver = LD07Driver()
    frame = _make_frame([0] * 10)
    decoded = driver.decode_packet(frame)
    assert decoded is not None
    decoded["samples"][20]["distance_mm"] = 200
    decoded["samples"][21]["distance_mm"] = 205
    decoded["samples"][22]["distance_mm"] = 210
    decoded["samples"][20]["distance_m"] = 0.2
    decoded["samples"][21]["distance_m"] = 0.205
    decoded["samples"][22]["distance_m"] = 0.21
    driver.last_packet = decoded

    objects = driver.detect_objects()

    assert len(objects) == 1
    assert objects[0]["point_count"] == 3
    assert 16.0 <= objects[0]["x_forward_cm"] <= 18.0
    assert -12.0 <= objects[0]["y_left_cm"] <= -10.0
    assert objects[0]["distance_cm"] <= 21.0
