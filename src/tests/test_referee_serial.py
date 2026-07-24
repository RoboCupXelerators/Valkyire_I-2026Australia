import ast
import os
import sys
import time
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from LunaPre.referee import IoRefereeProvider, RefereeLineDecoder, SerialRefereeProvider, VirtualRefereeSerial


class RefereeSerialTests(unittest.TestCase):
    def test_decoder_accepts_only_official_lines_and_fails_closed_initially(self):
        decoder = RefereeLineDecoder()
        self.assertFalse(decoder.snapshot().control_enabled)
        self.assertFalse(decoder.feed("PENALIZED 60"))
        self.assertFalse(decoder.snapshot().control_enabled)
        self.assertTrue(decoder.feed(b"PLAY\r\n", now_ms=123))
        self.assertTrue(decoder.snapshot().control_enabled)
        self.assertTrue(decoder.feed("stop"))
        self.assertFalse(decoder.snapshot().control_enabled)

    def test_disconnect_revokes_play(self):
        decoder = RefereeLineDecoder()
        decoder.feed("PLAY")
        decoder.disconnect()
        snapshot = decoder.snapshot()
        self.assertFalse(snapshot.connected)
        self.assertFalse(snapshot.control_enabled)
        self.assertEqual(snapshot.phase, "STOP")

    def test_io_provider_keeps_existing_bits_unlock_contract(self):
        state = {"enabled": False}
        provider = IoRefereeProvider(lambda: state["enabled"])
        self.assertFalse(provider.snapshot().control_enabled)
        state["enabled"] = True
        self.assertTrue(provider.snapshot().control_enabled)

    @unittest.skipUnless(os.name == "posix", "PTY is POSIX-only")
    def test_virtual_serial_emits_strict_stop_and_play_lines(self):
        with VirtualRefereeSerial(initial_state="STOP") as referee:
            flags = os.O_RDONLY | os.O_NONBLOCK | getattr(os, "O_NOCTTY", 0)
            fd = os.open(referee.port, flags)
            try:
                self.assertEqual(os.read(fd, 32), b"STOP\n")
                referee.play()
                self.assertEqual(os.read(fd, 32), b"PLAY\n")
                referee.penalty()
                self.assertEqual(os.read(fd, 32), b"STOP\n")
            finally:
                os.close(fd)

    @unittest.skipUnless(os.name == "posix", "PTY is POSIX-only")
    def test_serial_provider_reads_virtual_referee(self):
        with VirtualRefereeSerial(initial_state="STOP") as referee:
            provider = SerialRefereeProvider(referee.port, timeout_s=0.02, reconnect_s=0.02).start()
            try:
                deadline = time.monotonic() + 1.0
                while not provider.snapshot().connected and time.monotonic() < deadline:
                    referee.stop()
                    time.sleep(0.02)
                self.assertTrue(provider.snapshot().connected)
                self.assertFalse(provider.snapshot().control_enabled)
                referee.play()
                self.assertTrue(self._wait_until(lambda: provider.snapshot().control_enabled))
                referee.penalty()
                self.assertTrue(self._wait_until(lambda: not provider.snapshot().control_enabled))
            finally:
                provider.stop()

    def test_mlp_adapter_has_no_bits_or_unity_transport_dependency(self):
        path = ROOT / "src" / "mlp" / "adapters.py"
        tree = ast.parse(path.read_text(encoding="utf-8"))
        imported = set()
        names = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom):
                imported.add(node.module or "")
                names.update(alias.name for alias in node.names)
            elif isinstance(node, ast.Name):
                names.add(node.id)
        self.assertNotIn("Bits", names)
        self.assertNotIn("socket", imported)
        self.assertFalse(any("Xelerator" in name for name in imported))

    @staticmethod
    def _wait_until(predicate, timeout_s=1.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.01)
        return False


if __name__ == "__main__":
    unittest.main()
