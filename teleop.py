"""Repository-root launcher for ``python -m teleop``."""

from __future__ import annotations

import runpy
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
SRC = ROOT / "src"


def main():
    sys.path.insert(0, str(SRC))
    runpy.run_path(str(SRC / "teleop.py"), run_name="__main__")


if __name__ == "__main__":
    main()
