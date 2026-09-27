"""The team transport uses owned Bits without opening a second device."""
import pytest
from LunaPre.communication import Communication
from LunaPre import runtime
from LunaPre.positions import Positions
from LunaPre.referee import RefereeSnapshot
from LunaPre.strategy_tools import POSE_NOT_FOUND


class Link:
    def __init__(self):
        self.payload = None
    def SendLinkData(self, payload):
        self.payload = payload
        return True
    def GetLinkData(self):
        return self.payload


def test_runtime_link_roundtrip(monkeypatch):
    bits = Link()
    monkeypatch.setattr(runtime, "Bits", bits)
    comm = Communication()
    payload = comm.PackSelfStatus(position=[12, -34])
    assert comm.Send(payload)
    assert bits.payload == payload
    assert comm.Receive() == comm.ParseFrame(payload)
    comm.Close()
    assert comm.Send(payload)


def test_requires_initialized_bits(monkeypatch):
    monkeypatch.setattr(runtime, "Bits", None)
    with pytest.raises(RuntimeError, match="Initialize runtime Bits"):
        Communication()


def test_sim_requires_peer():
    with pytest.raises(ValueError, match="peer_id"):
        Communication(bits=object())


def test_link_rejection_propagates():
    bits = Link()
    bits.SendLinkData = lambda payload: False
    assert Communication(bits=bits).Send(b"test") is False


def test_role_is_forced_to_ban_when_the_referee_is_locked(monkeypatch):
    monkeypatch.setattr(
        runtime, "get_referee_state",
        lambda: RefereeSnapshot(source="uart", control_enabled=False),
    )
    comm = Communication(bits=Link(), peer_id=2)

    assert comm.PackSelfStatus(position=[0, 0])[2] == Communication.ROLE_BAN
    assert comm.PackSelfStatus(role=Communication.ROLE_OFFENCE_BLUE)[2] == Communication.ROLE_BAN


@pytest.mark.parametrize("state", [
    RefereeSnapshot(source="uart", control_enabled=True),
    RefereeSnapshot(),
    RefereeSnapshot(source="provider-error"),
])
def test_role_is_left_alone_when_the_referee_allows_or_is_unknown(monkeypatch, state):
    monkeypatch.setattr(runtime, "get_referee_state", lambda: state)
    comm = Communication(bits=Link(), peer_id=2)

    assert comm.PackSelfStatus(position=[0, 0])[2] == comm.Role
    assert comm.PackSelfStatus(role=Communication.ROLE_DEFENCE_BLUE)[2] == Communication.ROLE_DEFENCE_BLUE


def _communication_with_peer():
    """Build a Communication whose peer id never collides with the local id."""
    local_id = int(getattr(runtime.cfg.RoboInfo, "number", 1))
    peer_id = 2 if local_id != 2 else 3
    return Communication(bits=Link(), peer_id=peer_id), peer_id


def test_parse_frame_publishes_peer_pose_for_united_position():
    runtime.clear_peer_state()
    comm, peer_id = _communication_with_peer()
    # reserve 32 -> 32/128*360 = 90 degrees of encoded yaw.
    frame = comm.BuildFrame(
        identity_id=peer_id,
        role=Communication.ROLE_OFFENCE_YELLOW,
        status=Communication.STATUS_CHASING,
        x_pos=12,
        y_pos=0,
        x_neg=0,
        y_neg=34,
        reserve=32,
    )

    parsed = comm.ParseFrame(frame)

    assert parsed["signed_xy"] == [12, -34]
    positions = Positions()
    assert positions.PeerAvailable() is True
    assert positions.PeerRole() == Communication.ROLE_OFFENCE_YELLOW
    assert positions.PeerStatus() == Communication.STATUS_CHASING
    assert positions.FieldPeerYaw() == pytest.approx(90.0)
    assert positions.FieldPeerPose() == pytest.approx([12.0, -34.0, 90.0])


def test_peer_pose_is_hidden_while_the_peer_declares_ban():
    runtime.clear_peer_state()
    comm, peer_id = _communication_with_peer()
    frame = comm.BuildFrame(
        identity_id=peer_id,
        role=Communication.ROLE_BAN,
        status=Communication.STATUS_NO_BALL,
        x_pos=12,
        y_pos=0,
        x_neg=0,
        y_neg=34,
    )

    comm.ParseFrame(frame)

    positions = Positions()
    assert positions.PeerAvailable() is False
    assert positions.FieldPeerPose() == POSE_NOT_FOUND


def test_parse_frame_ignores_frames_from_the_local_identity():
    runtime.clear_peer_state()
    comm, _peer_id = _communication_with_peer()
    frame = comm.BuildFrame(identity_id=comm.RobotID, role=Communication.ROLE_OFFENCE_YELLOW)

    comm.ParseFrame(frame)

    assert runtime.get_peer_state() is None
