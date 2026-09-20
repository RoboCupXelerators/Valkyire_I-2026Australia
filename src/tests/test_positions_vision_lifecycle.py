from __future__ import annotations

from LunaPre import runtime
from LunaPre.positions import Positions


class FakeSocket:
    def __init__(self, *_args, **_kwargs):
        self.closed = False
        self.bound = None

    def bind(self, address):
        self.bound = address

    def settimeout(self, _timeout):
        pass

    def shutdown(self, _how):
        pass

    def close(self):
        self.closed = True


def test_positions_vision_can_restart_after_stop(monkeypatch):
    sockets = []

    def socket_factory(*args, **kwargs):
        sock = FakeSocket(*args, **kwargs)
        sockets.append(sock)
        return sock

    positions = Positions()
    positions._vision_enabled = True
    monkeypatch.setattr(runtime, "vision", None)
    monkeypatch.setattr(runtime, "chassis", None)
    monkeypatch.setattr(runtime, "peripheral", None)
    monkeypatch.setattr("LunaPre.positions.socket.socket", socket_factory)
    monkeypatch.setattr(positions, "_get_vision_data", lambda: None)

    assert positions.StartVision()
    first_event = positions.vision_data_stop_event
    positions.Stop()
    assert first_event.is_set()
    assert positions.sock is None
    assert not positions._vision_started
    assert sockets[0].closed

    assert positions.StartVision()
    assert positions.vision_data_stop_event is not first_event
    assert not positions.vision_data_stop_event.is_set()
    positions.Stop()
    assert sockets[1].closed
