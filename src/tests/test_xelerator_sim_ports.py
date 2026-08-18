"""Parallel simulator instance port contract tests."""

from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path
from unittest.mock import patch


SOFTWARE_SRC = Path(__file__).resolve().parents[1]
if str(SOFTWARE_SRC) not in sys.path:
    sys.path.insert(0, str(SOFTWARE_SRC))

from utils.XeleratorSimPorts import resolve_port  # noqa: E402
from utils.XeleratorSceneControl import XeleratorSceneControl  # noqa: E402


class TestXeleratorSimPorts(unittest.TestCase):
    def test_instance_offset_is_applied(self) -> None:
        with patch.dict(os.environ, {"XEL_INSTANCE_ID": "2"}, clear=False):
            os.environ.pop("XEL_PORT_OFFSET", None)
            os.environ.pop("XEL_SCENE_CONTROL_PORT", None)
            self.assertEqual(resolve_port(20001), 20201)
            self.assertEqual(resolve_port(20012), 20212)
            self.assertEqual(XeleratorSceneControl().port, 20300)


    def test_explicit_offset_wins(self) -> None:
        with patch.dict(os.environ, {"XEL_INSTANCE_ID": "2", "XEL_PORT_OFFSET": "7"}, clear=False):
            self.assertEqual(resolve_port(20001), 20008)


    def test_explicit_scene_control_port_is_final(self) -> None:
        with patch.dict(
            os.environ,
            {"XEL_INSTANCE_ID": "2", "XEL_SCENE_CONTROL_PORT": "25000"},
            clear=False,
        ):
            self.assertEqual(XeleratorSceneControl().port, 25000)


if __name__ == "__main__":
    unittest.main()
