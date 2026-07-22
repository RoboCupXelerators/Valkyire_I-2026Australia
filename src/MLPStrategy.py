"""Production MLP policy entry point, parallel to Hyperion.py."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import time
from typing import Callable, Optional

from mlp.adapters import observation_from_lunapre
from mlp.config import RuntimeConfig
from mlp.contracts import PolicyObservation, RoleId
from mlp.runtime import PolicyRuntime
from mlp.skill_executor import SkillExecutor, SkillExecutionResult


_runtime: Optional[PolicyRuntime] = None
_executor: Optional[SkillExecutor] = None
_observation_getter: Optional[Callable[[], PolicyObservation]] = None


def configure(runtime: PolicyRuntime, executor: SkillExecutor, observation_getter: Callable[[], PolicyObservation]) -> None:
    global _runtime, _executor, _observation_getter
    _runtime = runtime
    _executor = executor
    _observation_getter = observation_getter


def logic_step() -> SkillExecutionResult:
    if _runtime is None or _executor is None or _observation_getter is None:
        return SkillExecutor().safe_stop()
    try:
        observation = _observation_getter()
        return _executor.execute(_runtime.infer(observation, deterministic=True))
    except Exception:
        return _executor.safe_stop()


def main(config_path: str | None = None) -> None:
    path = Path(config_path or os.environ.get("XEL_MLP_CONFIG", "config/mlp/macos_cpu.json"))
    config = RuntimeConfig.load(path)
    os.environ.setdefault("ROBOCUPX_REFEREE_SOURCE", config.referee_source)
    os.environ.setdefault("ROBOCUPX_REFEREE_BAUD", str(config.referee_baudrate))
    if config.referee_port:
        os.environ.setdefault("ROBOCUPX_REFEREE_PORT", config.referee_port)
    role = int(RoleId.ATTACKER if config.role == "attacker" else RoleId.DEFENDER)
    executor = SkillExecutor()
    try:
        runtime = PolicyRuntime(config)
    except Exception:
        executor.safe_stop()
        raise
    configure(runtime, executor, lambda: observation_from_lunapre(role))
    period_s = config.control_period_ms / 1000.0
    deadline = time.monotonic()
    try:
        while True:
            logic_step()
            deadline += period_s
            time.sleep(max(0.0, deadline - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        executor.safe_stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run the Xelerators MLP V2 policy")
    parser.add_argument("--config", default=None)
    args = parser.parse_args()
    main(args.config)
