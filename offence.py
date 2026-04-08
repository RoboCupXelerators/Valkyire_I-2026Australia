"""Convenience launcher for the SIM/offence strategy.

This wrapper lets you run the strategy from the repository root with:

    python Software/offence.py

It adds `Software/src` to `sys.path` and then delegates to
`src/offence.py`'s `main()` entry point.
"""

from __future__ import annotations

import sys
from pathlib import Path


_ROOT = Path(__file__).resolve().parent
_SRC_DIR = _ROOT / "src"

if str(_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(_SRC_DIR))

from offence import main as _main  # noqa: E402


if __name__ == "__main__":
    _main()