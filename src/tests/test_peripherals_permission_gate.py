"""Referee permission transitions must zero peripheral IO."""

import os
import sys
from types import SimpleNamespace

os.environ.setdefault("ROBOCUPX_BITS", "SIM")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from LunaPre.motion.chassis import Car, Peripherals  # noqa: E402


def _make_peripheral(io_log):
    cfg = SimpleNamespace(Ports=SimpleNamespace(Dribble=4, ElecMagnet=3))
    p = Peripherals(lambda port, value: io_log.append((port, value)), lambda: True)
    p.cfg = cfg
    return p


def test_permission_loss_forces_dribble_off_and_resets_cache():
    io_log = []
    p = _make_peripheral(io_log)
    car = Car(permission=lambda: True)
    car.attach_peripherals(p)
    io_log.clear()

    assert car.CanMove() is True
    p.Dribble(0.3)
    assert io_log[-1] == (4, 0.3)
    assert p.DribbleStatus == 0.3

    # Referee shuts down: the next permission poll must force IO off even
    # though strategy code stops calling Dribble() once motion is denied.
    car.Permission = lambda: False
    assert car.CanMove() is False
    assert io_log[-2] == (4, 0.0)
    assert io_log[-1] == (3, 1)
    assert p.DribbleStatus is False

    # Permission returns: the next Dribble call must re-send (cache miss).
    car.Permission = lambda: True
    assert car.CanMove() is True
    p.Dribble(0.3)
    assert io_log[-1] == (4, 0.3)


def test_permission_loss_edge_fires_once():
    io_log = []
    p = _make_peripheral(io_log)
    car = Car(permission=lambda: True)
    car.attach_peripherals(p)
    car.CanMove()
    io_log.clear()

    car.Permission = lambda: False
    car.CanMove()
    car.CanMove()  # still denied: no repeated force-off spam
    assert io_log == [(4, 0.0), (3, 1)]


def test_dribble_denied_updates_status_cache():
    io_log = []
    p = _make_peripheral(io_log)
    p.Permission = lambda: False
    p.Dribble(0.3)
    assert io_log[-1] == (4, 0.0)
    assert p.DribbleStatus is False
