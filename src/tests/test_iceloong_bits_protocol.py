import math
import struct

import pytest

from utils.IceLoongBits import COMMAND_RESPONSE_LENGTHS, Commands, IceLoongBits


def test_command_table_matches_motion_controller_firmware():
    assert [(command.name, command.value) for command in Commands] == [
        ("HAND_SHAKE", 0x01),
        ("SET_ROOM_SIZE", 0x02),
        ("GET_POSE", 0x03),
        ("SET_WHEELS_SPEED", 0x04),
        ("KICK", 0x05),
        ("SET_DRIBBLE_SPEED", 0x06),
        ("GET_REFEREE_UNLOCK_PIN", 0x07),
        ("MORSE_PLAY", 0x08),
        ("LINK_SEND", 0x09),
        ("LINK_GET", 0x0A),
        ("GET_DRIBBLE_CURRENT", 0x0B),
        ("GET_BATTERY_VOLTAGE", 0x0C),
        ("LINK_STATUS", 0x0D),
    ]
    assert COMMAND_RESPONSE_LENGTHS == {
        Commands.HAND_SHAKE: 1,
        Commands.SET_ROOM_SIZE: 1,
        Commands.GET_POSE: 25,
        Commands.SET_WHEELS_SPEED: 1,
        Commands.KICK: 1,
        Commands.SET_DRIBBLE_SPEED: 1,
        Commands.GET_REFEREE_UNLOCK_PIN: 2,
        Commands.MORSE_PLAY: 1,
        Commands.LINK_SEND: 1,
        Commands.LINK_GET: None,
        Commands.GET_DRIBBLE_CURRENT: 5,
        Commands.GET_BATTERY_VOLTAGE: 5,
        Commands.LINK_STATUS: 5,
    }


def test_link_send_encodes_flags_u16_length_and_payload():
    bits = IceLoongBits(testing=True)
    sent = []

    def send_hex(data, read_response, response_length, response_timeout):
        sent.append((data, read_response, response_length, response_timeout))
        return b"\x89"

    bits.SendHex = send_hex

    payload = bytes(range(32))
    assert bits.ExecuteCommands([(Commands.LINK_SEND, payload)]) == [(0x89,)]
    assert sent == [(
        b"\x09\x00" + bytes([len(payload)]) + payload,
        True,
        1,
        0.05,
    )]


def test_link_send_with_pose_marks_the_flag_byte_and_keeps_one_byte_payloads():
    bits = IceLoongBits(testing=True)
    sent = []

    def send_hex(data, read_response, response_length, response_timeout):
        sent.append(data)
        return b"\x89"

    bits.SendHex = send_hex

    assert bits.ExecuteCommands([(Commands.LINK_SEND, b"\x2a", True)]) == [(0x89,)]
    assert sent == [b"\x09\x01\x01\x2a"]


def test_link_send_payload_limits_follow_the_pose_flag():
    bits = IceLoongBits(testing=True)

    def send_hex(data, read_response, response_length, response_timeout):
        return b"\x89"

    bits.SendHex = send_hex

    assert bits.ExecuteCommands([(Commands.LINK_SEND, bytes(221))]) == [(0x89,)]
    assert bits.ExecuteCommands([(Commands.LINK_SEND, bytes(199), True)]) == [(0x89,)]

    with pytest.raises(ValueError, match="from 1 to 221 bytes"):
        bits.ExecuteCommands([(Commands.LINK_SEND, bytes(222))])

    with pytest.raises(ValueError, match="from 0 to 199 bytes"):
        bits.ExecuteCommands([(Commands.LINK_SEND, bytes(200), True)])

    with pytest.raises(ValueError, match="from 1 to 221 bytes"):
        bits.ExecuteCommands([(Commands.LINK_SEND, b"")])

    with pytest.raises(ValueError, match="must be bytes-like"):
        bits.ExecuteCommands([(Commands.LINK_SEND, "strategy", True)])


def test_link_send_accepts_a_pose_only_message():
    bits = IceLoongBits(testing=True)
    sent = []

    def send_hex(data, read_response, response_length, response_timeout):
        sent.append(data)
        return b"\x89"

    bits.SendHex = send_hex

    assert bits.ExecuteCommands([(Commands.LINK_SEND, b"", True)]) == [(0x89,)]
    assert sent == [b"\x09\x01\x00"]
    assert bits.SendLinkData(b"", include_pose=True) is True
    assert bits.CommandLine["LINK_SEND"] == (b"", True)


def test_link_send_requires_a_payload_and_an_optional_pose_flag():
    bits = IceLoongBits(testing=True)

    with pytest.raises(ValueError, match="optional include_pose flag"):
        bits.ExecuteCommands([(Commands.LINK_SEND,)])

    with pytest.raises(ValueError, match="optional include_pose flag"):
        bits.ExecuteCommands([(Commands.LINK_SEND, b"\x01", True, 0)])


def test_send_link_data_queues_one_message_with_the_pose_flag():
    bits = IceLoongBits(testing=True)
    sent = []

    def send_hex(data, read_response, response_length, response_timeout):
        sent.append(data)
        return b"\x80" * response_length

    bits.SendHex = send_hex
    bits._ExecuteLinkGet = lambda: []

    assert bits.SendLinkData(b"strategy", include_pose=True) is True
    bits._SendUnitedCommandOnce()

    assert b"\x09\x01\x08" + b"strategy" in sent
    assert bits.CommandLine["LINK_SEND"] is None


def test_link_get_reads_variable_length_response():
    bits = IceLoongBits(testing=True)
    payload = b"\x00" + bytes(range(37))
    sent = []

    def send_variable(data, response_timeout):
        sent.append((data, response_timeout))
        return b"\x8a" + bytes([len(payload)]) + payload

    bits._SendVariableHex = send_variable

    response = bits.ExecuteCommands([(Commands.LINK_GET,)])
    bits._ApplyResponses(response)

    assert sent == [(b"\x0a", 0.05)]
    assert response == [(0x8A, payload)]
    assert bits.GetLinkData() == bytes(range(37))


def test_link_get_exposes_the_peer_pose_and_strips_the_payload_header():
    bits = IceLoongBits(testing=True)
    payload = (
        b"\x01"
        + struct.pack("<ffffff", 30.0, 1.0, -2.0, 100.0, -200.0, 0.0)
        + b"strategy"
    )

    bits._ApplyResponses([(0x8A, payload)])

    assert bits.GetLinkData() == b"strategy"
    assert bits.peer_position_mm == (100.0, -200.0, 0.0)
    assert bits.GetPeerPose() == pytest.approx((10.0, -20.0, 30.0))
    assert bits.GetPeerYaw() == pytest.approx(30.0)


def test_link_get_without_a_pose_flag_only_exposes_the_payload():
    bits = IceLoongBits(testing=True)

    bits._ApplyResponses([(0x8A, b"\x00strategy")])

    assert bits.GetLinkData() == b"strategy"
    assert bits.GetPeerPose() is None
    assert bits.GetPeerYaw() is None
    assert bits.peer_position_mm is None


def test_link_get_without_application_data_reports_no_link_data():
    bits = IceLoongBits(testing=True)
    payload = b"\x01" + struct.pack("<ffffff", 30.0, 1.0, -2.0, 100.0, -200.0, 0.0)

    bits._ApplyResponses([(0x8A, payload)])
    assert bits.GetLinkData() is None
    assert bits.GetPeerPose() == pytest.approx((10.0, -20.0, 30.0))

    bits._ApplyResponses([(0x8A, b"\x00")])
    assert bits.GetLinkData() is None
    assert bits.GetPeerPose() is None


def test_link_get_without_a_message_exposes_nothing():
    bits = IceLoongBits(testing=True)

    bits._ApplyResponses([(0x8A, b"")])

    assert bits.GetLinkData() is None
    assert bits.GetPeerPose() is None


def test_link_payload_with_a_broken_pose_is_ignored():
    bits = IceLoongBits(testing=True)

    bits._ApplyResponses([(0x8A, b"\x01" + bytes(10))])
    assert bits.GetLinkData() is None
    assert bits.GetPeerPose() is None

    bits._ApplyResponses([(0x8A, b"\x00strategy")])
    bits._ApplyResponses([
        (0x8A, b"\x01" + struct.pack("<ffffff", math.inf, 0.0, 0.0, 1.0, 2.0, 0.0) + b"x")
    ])
    assert bits.GetLinkData() == b"x"
    assert bits.GetPeerPose() is None


def test_get_pose_command_decodes_all_six_floats_and_updates_public_pose():
    bits = IceLoongBits(testing=True)
    response = bytes([0x83]) + struct.pack(
        "<ffffff", -10.0, 2.5, -3.5, 123.0, -456.0, 0.0
    )
    sent = []

    def send_hex(data, read_response, response_length, response_timeout):
        sent.append((data, read_response, response_length, response_timeout))
        return response

    bits.SendHex = send_hex

    decoded = bits.ExecuteCommands([(Commands.GET_POSE,)])
    bits._ApplyResponses(decoded)

    assert sent == [(b"\x03", True, 25, 0.05)]
    assert decoded == [(0x83, -10.0, 2.5, -3.5, 123.0, -456.0, 0.0)]
    assert bits.yaw == 350.0
    assert bits.pitch == 2.5
    assert bits.roll == -3.5
    assert bits.position_mm == (123.0, -456.0, 0.0)
    assert bits.pose == pytest.approx((12.3, -45.6, 350.0))


def test_composite_command_uses_protocol_delimiters_and_response_lengths():
    bits = IceLoongBits(testing=True)
    response = (
        b"\x80\x81\x83"
        + struct.pack("<ffffff", 45.0, 1.0, 2.0, 100.0, -200.0, 0.0)
        + b"\x87\x01\x80"
    )
    sent = []

    def send_hex(data, read_response, response_length, response_timeout):
        sent.append((data, response_length))
        return response

    bits.SendHex = send_hex

    decoded = bits.ExecuteCommands([
        (Commands.HAND_SHAKE,),
        (Commands.GET_POSE,),
        (Commands.GET_REFEREE_UNLOCK_PIN,),
    ])
    bits._ApplyResponses(decoded)

    assert sent == [(b"\x00\x01\x03\x07\x00", 30)]
    assert decoded == [
        (0x81,),
        (0x83, 45.0, 1.0, 2.0, 100.0, -200.0, 0.0),
        (0x87, 1),
    ]
    assert bits.pose == (10.0, -20.0, 45.0)
    assert bits.GetRefereeUnlockPin() is True


def test_invalid_firmware_position_is_not_exposed_as_a_pose():
    bits = IceLoongBits(testing=True)

    bits._ApplyResponses([(0x83, 30.0, 0.0, 0.0, math.inf, math.inf, 0.0)])

    assert bits.yaw == 30.0
    assert bits.position_mm is None
    assert bits.pose is None


def test_dribble_request_uses_protocol_range_without_legacy_scaling():
    bits = IceLoongBits(testing=True)
    sent = []

    def send_hex(data, read_response, response_length, response_timeout):
        sent.append(data)
        return b"\x86"

    bits.SendHex = send_hex

    assert bits.ExecuteCommands([(Commands.SET_DRIBBLE_SPEED, 1.5)]) == [(0x86,)]
    assert sent == [b"\x06" + struct.pack("<f", 1.0)]


def test_room_size_request_encodes_horizontal_then_vertical_mm():
    bits = IceLoongBits(testing=True)
    sent = []

    def send_hex(data, read_response, response_length, response_timeout):
        sent.append((data, response_length))
        return b"\x82"

    bits.SendHex = send_hex

    assert bits.SetRoomSize(horizontal_mm=2430, vertical_mm=1820) == [(0x82,)]
    assert sent == [(b"\x02" + struct.pack("<ff", 2430.0, 1820.0), 1)]
    assert struct.unpack("<ff", sent[0][0][1:]) == (2430.0, 1820.0)


@pytest.mark.parametrize("horizontal_mm, vertical_mm", [
    (0, 1820),
    (2430, -1),
    (math.nan, 1820),
    (2430, math.inf),
    (True, 1820),
])
def test_room_size_request_rejects_invalid_dimensions(horizontal_mm, vertical_mm):
    bits = IceLoongBits(testing=True)

    with pytest.raises(ValueError, match="horizontal, vertical order"):
        bits.SetRoomSize(horizontal_mm, vertical_mm)


def test_room_size_command_requires_horizontal_and_vertical_only():
    bits = IceLoongBits(testing=True)

    with pytest.raises(ValueError, match="horizontal, vertical order"):
        bits.ExecuteCommands([(Commands.SET_ROOM_SIZE, 2430)])

    with pytest.raises(ValueError, match="horizontal, vertical order"):
        bits.ExecuteCommands([(Commands.SET_ROOM_SIZE, 2430, 1820, 100)])


@pytest.mark.parametrize("power_level", [1, 2, 3, 4])
def test_kick_request_sends_selected_power_level(power_level):
    bits = IceLoongBits(testing=True)
    sent = []

    def send_hex(data, read_response, response_length, response_timeout):
        sent.append(data)
        return b"\x85"

    bits.SendHex = send_hex

    assert bits.ExecuteCommands([(Commands.KICK, power_level)]) == [(0x85,)]
    assert sent == [bytes([Commands.KICK, power_level])]


@pytest.mark.parametrize("power_level", [0, 5, 1.5, True, None, math.inf])
def test_kick_request_rejects_invalid_power_level(power_level):
    bits = IceLoongBits(testing=True)

    with pytest.raises(ValueError, match="integer from 1 to 4"):
        bits.ExecuteCommands([(Commands.KICK, power_level)])


def test_kick_request_requires_exactly_one_power_level():
    bits = IceLoongBits(testing=True)

    with pytest.raises(ValueError, match="integer from 1 to 4"):
        bits.ExecuteCommands([(Commands.KICK,)])

    with pytest.raises(ValueError, match="integer from 1 to 4"):
        bits.ExecuteCommands([(Commands.KICK, 1, 2)])


def test_kick_uses_configured_default_level_and_current_dribble_speed():
    bits = IceLoongBits(testing=True)
    sent = []

    def send_hex(data, read_response, response_length, response_timeout):
        sent.append((data, response_length))
        return b"\x80\x85\x86\x80"

    bits.SendHex = send_hex
    bits.CommandLine["SET_DRIBBLE_SPEED"] = 0.5

    assert bits.SetKickPowerLevel(2) == 2
    assert bits.Kick() == [(0x85,), (0x86,)]
    assert sent == [(
        b"\x00\x05\x02\x06" + struct.pack("<f", 0.5) + b"\x00",
        4,
    )]


def test_morse_request_uses_command_0x08_and_accepts_23_bytes():
    bits = IceLoongBits(testing=True)
    sent = []

    def send_hex(data, read_response, response_length, response_timeout):
        sent.append(data)
        return b"\x88"

    bits.SendHex = send_hex
    message = "ABCDEFGHIJKLMNOPQRSTUVW"

    assert bits.ExecuteCommands([(Commands.MORSE_PLAY, message)]) == [(0x88,)]
    assert sent == [b"\x08\x17" + message.encode("ascii")]


def test_referee_unlock_state_tracks_both_pin_levels():
    bits = IceLoongBits(testing=True)

    bits._ApplyResponses([(0x87, 1)])
    assert bits.GetRefereeUnlockPin() is True

    bits._ApplyResponses([(0x87, 0)])
    assert bits.GetRefereeUnlockPin() is False


def test_get_dribble_current_decodes_float_amperes():
    bits = IceLoongBits(testing=True)
    sent = []

    def send_hex(data, read_response, response_length, response_timeout):
        sent.append((data, read_response, response_length, response_timeout))
        return bytes([0x8B]) + struct.pack("<f", 6.25)

    bits.SendHex = send_hex

    decoded = bits.ExecuteCommands([(Commands.GET_DRIBBLE_CURRENT,)])
    bits._ApplyResponses(decoded)

    assert sent == [(b"\x0b", True, 5, 0.05)]
    assert decoded == [(0x8B, 6.25)]
    assert bits.GetDribbleCurrent() == pytest.approx(6.25)


def test_get_battery_voltage_decodes_float_volts():
    bits = IceLoongBits(testing=True)
    sent = []

    def send_hex(data, read_response, response_length, response_timeout):
        sent.append((data, read_response, response_length, response_timeout))
        return bytes([0x8C]) + struct.pack("<f", 11.875)

    bits.SendHex = send_hex

    decoded = bits.ExecuteCommands([(Commands.GET_BATTERY_VOLTAGE,)])
    bits._ApplyResponses(decoded)

    assert sent == [(b"\x0c", True, 5, 0.05)]
    assert decoded == [(0x8C, 11.875)]
    assert bits.GetBatteryVoltage() == pytest.approx(11.875)


def test_composite_command_reads_dribble_current_and_battery_voltage():
    bits = IceLoongBits(testing=True)
    response = (
        b"\x80\x8B"
        + struct.pack("<f", 1.5)
        + b"\x8C"
        + struct.pack("<f", 12.125)
        + b"\x80"
    )
    sent = []

    def send_hex(data, read_response, response_length, response_timeout):
        sent.append((data, response_length))
        return response

    bits.SendHex = send_hex

    decoded = bits.ExecuteCommands([
        (Commands.GET_DRIBBLE_CURRENT,),
        (Commands.GET_BATTERY_VOLTAGE,),
    ])
    bits._ApplyResponses(decoded)

    assert sent == [(b"\x00\x0b\x0c\x00", 12)]
    assert decoded == [(0x8B, 1.5), (0x8C, 12.125)]
    assert bits.dribble_current == pytest.approx(1.5)
    assert bits.battery_voltage == pytest.approx(12.125)
