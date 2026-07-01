import struct

from utils.IceLoongBits import Commands, IceLoongBits, MAX_DRIBBLE_SPEED


class CapturingIceLoongBits(IceLoongBits):
    def __init__(self):
        super().__init__(testing=True)
        self.sent = []

    def SendHex(self, data, read_response=False, response_length=0, response_timeout=None):
        self.sent.append(
            {
                "data": bytes(data),
                "read_response": read_response,
                "response_length": response_length,
                "response_timeout": response_timeout,
            }
        )
        return b""


def test_virtual_ports_kick_sends_immediate_test_bot_frame():
    bits = CapturingIceLoongBits()

    bits.VirtualPorts(3, 0)

    assert len(bits.sent) == 1
    assert bits.sent[0]["data"] == (
        bytes([0x00, Commands.KICK, Commands.SET_DRIBBLE_SPEED])
        + struct.pack("<ff", 0.0, 0.0)
        + bytes([0x00])
    )
    assert bits.sent[0]["read_response"] is True
    assert bits.sent[0]["response_length"] == 4
    assert bits.CommandLine["KICK"] is False


def test_virtual_ports_dribble_sends_immediate_speed_command():
    bits = CapturingIceLoongBits()

    bits.VirtualPorts(4, 0.95)

    mapped_speed = 0.95 * MAX_DRIBBLE_SPEED
    assert len(bits.sent) == 1
    assert bits.sent[0]["data"] == (
        bytes([Commands.SET_DRIBBLE_SPEED])
        + struct.pack("<ff", mapped_speed, 0.0)
    )
    assert bits.sent[0]["read_response"] is True
    assert bits.sent[0]["response_length"] == 1
    assert bits.CommandLine["SET_DRIBBLE_SPEED"] == [0.95, 0]


def test_execute_commands_maps_dribble_speed_before_serial_pack():
    bits = CapturingIceLoongBits()

    bits.ExecuteCommands([(Commands.SET_DRIBBLE_SPEED, 0.95, -1.0)])

    mapped_speed = 0.95 * MAX_DRIBBLE_SPEED
    assert len(bits.sent) == 1
    assert bits.sent[0]["data"] == (
        bytes([Commands.SET_DRIBBLE_SPEED])
        + struct.pack("<ff", mapped_speed, 0.0)
    )


def test_kick_frame_maps_cached_dribble_speed():
    bits = CapturingIceLoongBits()

    bits.CommandLine["SET_DRIBBLE_SPEED"] = [0.95, 0.7]
    bits.VirtualPorts(3, 0)

    assert len(bits.sent) == 1
    assert bits.sent[0]["data"] == (
        bytes([0x00, Commands.KICK, Commands.SET_DRIBBLE_SPEED])
        + struct.pack("<ff", 0.95 * MAX_DRIBBLE_SPEED, 0.7 * MAX_DRIBBLE_SPEED)
        + bytes([0x00])
    )


def test_get_yaw_refreshes_bmi_data_without_background_thread():
    class YawIceLoongBits(IceLoongBits):
        def SendHex(self, data, read_response=False, response_length=0, response_timeout=None):
            assert bytes(data) == bytes([Commands.GET_BMI088_DATA])
            assert response_length == 13
            return bytes([Commands.GET_BMI088_DATA ^ 0x80]) + struct.pack("<fff", 725.0, 1.0, 2.0)

    bits = YawIceLoongBits(testing=True)

    assert bits.GetYaw() == 5.0
