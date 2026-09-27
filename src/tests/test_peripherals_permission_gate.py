"""Referee permission transitions must zero peripheral IO."""

import os
import sys
from types import SimpleNamespace

os.environ.setdefault("ROBOCUPX_BITS", "SIM")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from LunaPre.motion.chassis import Car, Peripherals  # noqa: E402
from LunaPre.strategy_tools import register_permission_pid  # noqa: E402


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


def test_permission_loss_resets_registered_pids():
    # A controller that is simply never called while the gate is closed must
    # still drop its integral on the referee off edge, not on re-enable.
    class FakePID:
        def __init__(self):
            self.resets = 0

        def reset(self):
            self.resets += 1

    io_log = []
    p = _make_peripheral(io_log)
    car = Car(permission=lambda: True)
    car.attach_peripherals(p)
    pid = FakePID()
    register_permission_pid(pid)

    assert car.CanMove() is True
    assert pid.resets == 0

    car.Permission = lambda: False
    assert car.CanMove() is False
    assert pid.resets >= 1

    # Still denied: no repeated reset spam.
    resets_after_edge = pid.resets
    car.CanMove()
    assert pid.resets == resets_after_edge
    p.stop_watchdog()


def test_dribble_denied_updates_status_cache():
    io_log = []
    p = _make_peripheral(io_log)
    p.Permission = lambda: False
    p.Dribble(0.3)
    assert io_log[-1] == (4, 0.0)
    assert p.DribbleStatus is False
    p.stop_watchdog()


def test_watchdog_forces_off_without_any_caller():
    # The watchdog must catch the referee on->off edge even when nothing
    # polls CanMove() (IDLE/REPL or control-panel debugging).
    import time

    io_log = []
    p = _make_peripheral(io_log)
    try:
        p.Dribble(0.3)
        assert p.DribbleStatus == 0.3
        io_log.clear()

        p.Permission = lambda: False
        deadline = time.time() + 2.0
        while p.DribbleStatus is not False and time.time() < deadline:
            time.sleep(0.02)
        assert p.DribbleStatus is False
        assert (4, 0.0) in io_log
        assert (3, 1) in io_log

        # Permission returns: cache is re-armed, next Dribble re-sends.
        p.Permission = lambda: True
        deadline = time.time() + 2.0
        while p.DribbleStatus is not False and time.time() < deadline:
            time.sleep(0.02)
        p.Dribble(0.3)
        assert io_log[-1] == (4, 0.3)
    finally:
        p.stop_watchdog()
