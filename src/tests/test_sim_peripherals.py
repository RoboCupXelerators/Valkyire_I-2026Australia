import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from LunaPre.motion.chassis import Peripherals


class SimulatorPeripheralTests(unittest.TestCase):
    @staticmethod
    def peripheral():
        peripheral = Peripherals.__new__(Peripherals)
        peripheral.Permission = lambda: True
        peripheral.SetIO = Mock()
        peripheral.cfg = SimpleNamespace(Ports=SimpleNamespace(ElecMagnet=3))
        peripheral.logger = Mock()
        return peripheral

    def test_simulator_kick_is_non_blocking(self):
        peripheral = self.peripheral()
        with patch.dict("os.environ", {"ROBOCUPX_BITS": "SIM"}), patch(
            "LunaPre.motion.chassis.time.sleep"
        ) as sleep:
            peripheral.ShootBall()
        peripheral.SetIO.assert_called_once_with(3, 0)
        sleep.assert_not_called()

    def test_hardware_kick_keeps_electromagnet_pulse(self):
        peripheral = self.peripheral()
        with patch.dict("os.environ", {"ROBOCUPX_BITS": "AB"}), patch(
            "LunaPre.motion.chassis.time.sleep"
        ) as sleep:
            peripheral.ShootBall()
        sleep.assert_called_once_with(0.3)

    def test_level_aware_kick_uses_direct_bits_interface_without_legacy_pulse(self):
        peripheral = self.peripheral()
        peripheral.KickFunc = Mock(return_value=True)

        with patch("LunaPre.motion.chassis.time.sleep") as sleep:
            assert peripheral.ShootBall(2) is True

        peripheral.KickFunc.assert_called_once_with(2)
        peripheral.SetIO.assert_not_called()
        sleep.assert_not_called()

    def test_level_aware_kick_respects_permission_gate(self):
        peripheral = self.peripheral()
        peripheral.Permission = lambda: False
        peripheral.KickFunc = Mock(return_value=True)

        assert peripheral.ShootBall(4) is False
        peripheral.KickFunc.assert_not_called()
        peripheral.SetIO.assert_not_called()


if __name__ == "__main__":
    unittest.main()
