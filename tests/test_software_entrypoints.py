import os
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def run_from_root(*args):
    environment = os.environ.copy()
    environment["PYTHONPATH"] = str(ROOT / "src")
    return subprocess.run(
        [sys.executable, *args],
        cwd=ROOT,
        env=environment,
        text=True,
        capture_output=True,
        timeout=10,
        check=False,
    )


def test_lunapre_import_works_from_src_layout():
    result = run_from_root("-c", "from LunaPre import *; print(Car.__name__)")
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip().splitlines()[-1] == "Car"


def test_mlp_strategy_help_is_hardware_free_from_repository_root():
    result = run_from_root("-m", "MLPStrategy", "--help")
    assert result.returncode == 0, result.stderr
    assert "--config" in result.stdout


def test_teleop_module_help_is_hardware_free_from_repository_root():
    result = run_from_root("-m", "teleop", "--help")
    assert result.returncode == 0, result.stderr
    assert "DM-S2325" in result.stdout
