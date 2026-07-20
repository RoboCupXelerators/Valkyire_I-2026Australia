"""Repository-root compatibility loader for the src-layout LunaPre package."""

from __future__ import annotations

import sys
from pathlib import Path


_ROOT = Path(__file__).resolve().parents[1]
_SRC = _ROOT / "src"
_PACKAGE = _SRC / "LunaPre"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))
__path__ = [str(_PACKAGE)]
__file__ = str(_PACKAGE / "__init__.py")
exec(compile((_PACKAGE / "__init__.py").read_bytes(), __file__, "exec"), globals())
