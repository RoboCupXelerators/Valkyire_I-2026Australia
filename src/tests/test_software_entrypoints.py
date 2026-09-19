import os
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"


def run_from_src(*args, extra_env=None):
    environment = os.environ.copy()
    environment.pop("PYTHONPATH", None)
    # LunaPre initializes its runtime eagerly now, so these subprocesses must
    # deterministically pick the SIM backend on ports the main pytest process
    # does not hold (the suite may keep default SIM sockets open).
    environment["ROBOCUPX_BITS"] = "SIM"
    environment["XEL_ROBOT_ID"] = "9"
    environment["XEL_CMD_PORT"] = "21001"
    environment["XEL_STATUS_PORT"] = "21002"
    environment.update(extra_env or {})
    return subprocess.run(
        [sys.executable, *args],
        cwd=SRC,
        env=environment,
        text=True,
        capture_output=True,
        timeout=10,
        check=False,
    )


def test_lunapre_import_eagerly_initializes_runtime_from_src():
    # LunaPre no longer lazy-loads: importing the package must initialize the
    # runtime and expose concrete runtime objects for debugging.
    result = run_from_src(
        "-c",
        "import LunaPre; "
        "assert LunaPre._runtime._runtime_initialized; "
        "assert LunaPre.chassis is LunaPre._runtime.chassis; "
        "assert not hasattr(LunaPre, '_RuntimeObjectProxy'); "
        "from LunaPre import *; "
        "assert chassis is LunaPre.chassis; print(Car.__name__)",
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip().splitlines()[-1] == "Car"


def test_lunapre_autostart_vision_opt_in_from_src():
    # Vision must stay off by default on import, and start only when
    # XEL_AUTO_VISION is explicitly requested (IDLE/REPL debugging).
    off = run_from_src(
        "-c",
        "import LunaPre; "
        "assert not LunaPre.UnitedPosition._vision_started; print('off')",
    )
    assert off.returncode == 0, off.stderr
    assert off.stdout.strip().splitlines()[-1] == "off"

    on = run_from_src(
        "-c",
        "import LunaPre; "
        "assert LunaPre.UnitedPosition._vision_started; print('on')",
        extra_env={"XEL_AUTO_VISION": "1"},
    )
    assert on.returncode == 0, on.stderr
    assert on.stdout.strip().splitlines()[-1] == "on"


def test_mlp_strategy_help_is_hardware_free_from_src():
    result = run_from_src("-m", "MLPStrategy", "--help")
    assert result.returncode == 0, result.stderr
    assert "--config" in result.stdout


def test_module_bindings_follow_runtime_restart():
    # Lazy proxies are gone: LunaPre module globals must re-bind to the
    # current runtime objects on every ensure/shutdown call. Previously
    # imported names intentionally stay stale across a restart.
    result = run_from_src(
        "-c",
        "from types import SimpleNamespace; import LunaPre; "
        "r = LunaPre._runtime; "
        "first = LunaPre.chassis; assert first is r.chassis; "
        "LunaPre.shutdown_runtime(); assert LunaPre.chassis is None; "
        "r.chassis = SimpleNamespace(value=1); r._runtime_initialized = True; "
        "LunaPre.ensure_runtime(); "
        "assert LunaPre.chassis is r.chassis; assert LunaPre.chassis.value == 1; "
        "assert first is not LunaPre.chassis",
    )
    assert result.returncode == 0, result.stderr
