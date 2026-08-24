import math
import struct

import pytest

from utils.IceLoongBits import COMMAND_RESPONSE_LENGTHS, Commands, IceLoongBits


def test_command_table_matches_motion_controller_firmware():
    assert [(command.name, command.value) for command in Commands] == [
        ("HAND_SHAKE", 0x01),
        ("GET_POSE", 0x02),
        ("SET_WHEELS_SPEED", 0x03),
        ("KICK", 0x04),
        ("SET_DRIBBLE_SPEED", 0x05),
        ("GET_REFEREE_UNLOCK_PIN", 0x06),
        ("MORSE_PLAY", 0x07),
    ]
    assert COMMAND_RESPONSE_LENGTHS == {
        Commands.HAND_SHAKE: 1,
        Commands.GET_POSE: 25,
        Commands.SET_WHEELS_SPEED: 1,
        Commands.KICK: 1,
        Commands.SET_DRIBBLE_SPEED: 1,
        Commands.GET_REFEREE_UNLOCK_PIN: 2,
        Commands.MORSE_PLAY: 1,
    }


def test_get_pose_command_decodes_all_six_floats_and_updates_public_pose():
    bits = IceLoongBits(testing=True)
    response = bytes([0x82]) + struct.pack(
        "<ffffff", -10.0, 2.5, -3.5, 123.0, -456.0, 0.0
    )
    sent = []

    def send_hex(data, read_response, response_length, response_timeout):
        sent.append((data, read_response, response_length, response_timeout))
        return response

    bits.SendHex = send_hex

    decoded = bits.ExecuteCommands([(Commands.GET_POSE,)])
    bits._ApplyResponses(decoded)

    assert sent == [(b"\x02", True, 25, 0.05)]
    assert decoded == [(0x82, -10.0, 2.5, -3.5, 123.0, -456.0, 0.0)]
    assert bits.yaw == 350.0
    assert bits.pitch == 2.5
    assert bits.roll == -3.5
    assert bits.position_mm == (123.0, -456.0, 0.0)
    assert bits.pose == pytest.approx((12.3, -45.6, 350.0))


def test_composite_command_uses_protocol_delimiters_and_response_lengths():
    bits = IceLoongBits(testing=True)
    response = (
        b"\x80\x81\x82"
        + struct.pack("<ffffff", 45.0, 1.0, 2.0, 100.0, -200.0, 0.0)
        + b"\x86\x01\x80"
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

    assert sent == [(b"\x00\x01\x02\x06\x00", 30)]
    assert decoded == [
        (0x81,),
        (0x82, 45.0, 1.0, 2.0, 100.0, -200.0, 0.0),
        (0x86, 1),
    ]
    assert bits.pose == (10.0, -20.0, 45.0)
    assert bits.GetRefereeUnlockPin() is True


def test_invalid_firmware_position_is_not_exposed_as_a_pose():
    bits = IceLoongBits(testing=True)

    bits._ApplyResponses([(0x82, 30.0, 0.0, 0.0, math.inf, math.inf, 0.0)])

    assert bits.yaw == 30.0
    assert bits.position_mm is None
    assert bits.pose is None


def test_dribble_request_uses_protocol_range_without_legacy_scaling():
    bits = IceLoongBits(testing=True)
    sent = []

    def send_hex(data, read_response, response_length, response_timeout):
        sent.append(data)
        return b"\x85"

    bits.SendHex = send_hex

    assert bits.ExecuteCommands([(Commands.SET_DRIBBLE_SPEED, 1.5)]) == [(0x85,)]
    assert sent == [b"\x05" + struct.pack("<f", 1.0)]


@pytest.mark.parametrize("power_level", [1, 2, 3, 4])
def test_kick_request_sends_selected_power_level(power_level):
    bits = IceLoongBits(testing=True)
    sent = []

    def send_hex(data, read_response, response_length, response_timeout):
        sent.append(data)
        return b"\x84"

    bits.SendHex = send_hex

    assert bits.ExecuteCommands([(Commands.KICK, power_level)]) == [(0x84,)]
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
        return b"\x80\x84\x85\x80"

    bits.SendHex = send_hex
    bits.CommandLine["SET_DRIBBLE_SPEED"] = 0.5

    assert bits.SetKickPowerLevel(2) == 2
    assert bits.Kick() == [(0x84,), (0x85,)]
    assert sent == [(
        b"\x00\x04\x02\x05" + struct.pack("<f", 0.5) + b"\x00",
        4,
    )]


def test_morse_request_uses_command_0x07_and_accepts_23_bytes():
    bits = IceLoongBits(testing=True)
    sent = []

    def send_hex(data, read_response, response_length, response_timeout):
        sent.append(data)
        return b"\x87"

    bits.SendHex = send_hex
    message = "ABCDEFGHIJKLMNOPQRSTUVW"

    assert bits.ExecuteCommands([(Commands.MORSE_PLAY, message)]) == [(0x87,)]
    assert sent == [b"\x07\x17" + message.encode("ascii")]


def test_referee_unlock_state_tracks_both_pin_levels():
    bits = IceLoongBits(testing=True)

    bits._ApplyResponses([(0x86, 1)])
    assert bits.GetRefereeUnlockPin() is True

    bits._ApplyResponses([(0x86, 0)])
    assert bits.GetRefereeUnlockPin() is False
