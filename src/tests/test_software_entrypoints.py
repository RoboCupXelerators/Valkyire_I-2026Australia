import os
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"


def run_from_src(*args):
    environment = os.environ.copy()
    environment.pop("PYTHONPATH", None)
    return subprocess.run(
        [sys.executable, *args],
        cwd=SRC,
        env=environment,
        text=True,
        capture_output=True,
        timeout=10,
        check=False,
    )


def test_lunapre_import_works_from_src():
    result = run_from_src(
        "-c",
        "import LunaPre; assert not LunaPre._runtime._runtime_initialized; "
        "from LunaPre import *; "
        "assert not LunaPre._runtime._runtime_initialized; print(Car.__name__)",
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip().splitlines()[-1] == "Car"


def test_mlp_strategy_help_is_hardware_free_from_src():
    result = run_from_src("-m", "MLPStrategy", "--help")
    assert result.returncode == 0, result.stderr
    assert "--config" in result.stdout


def test_imported_chassis_proxy_follows_runtime_restart():
    result = run_from_src(
        "-c",
        "from types import SimpleNamespace; import LunaPre; "
        "from LunaPre import chassis; "
        "r = LunaPre._runtime; r._runtime_initialized = True; "
        "r.chassis = SimpleNamespace(value=1); LunaPre.ensure_runtime(); "
        "from LunaPre import chassis as second_import; "
        "assert second_import is chassis; assert chassis.value == 1; "
        "LunaPre.shutdown_runtime(); "
        "r.chassis = SimpleNamespace(value=2); r._runtime_initialized = True; "
        "LunaPre.ensure_runtime(); assert chassis.value == 2; "
        "assert second_import.value == 2",
    )
    assert result.returncode == 0, result.stderr
