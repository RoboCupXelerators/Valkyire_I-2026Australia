"""The team transport uses owned Bits without opening a second device."""
import pytest
from LunaPre.communication import Communication
from LunaPre import runtime


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
