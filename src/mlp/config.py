"""Side-effect-free runtime configuration for the MLP policy."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Any, Dict


@dataclass(frozen=True)
class RuntimeConfig:
    platform: str
    device: str
    model_format: str
    bundle_path: str
    control_period_ms: int = 30
    observation_timeout_ms: int = 120
    sim_host: str = "127.0.0.1"
    sim_command_port: int = 20001
    sim_status_port: int = 20002
    scene_control_port: int = 20100
    referee_source: str = "auto"
    referee_port: str = ""
    referee_baudrate: int = 115200
    robot_id: int = 1
    role: str = "attacker"
    threads: int = 1
    training_enabled: bool = False
    latency_budget_ms: float = 10.0

    def __post_init__(self) -> None:
        if self.control_period_ms <= 0:
            raise ValueError("control_period_ms must be positive")
        if self.observation_timeout_ms < self.control_period_ms:
            raise ValueError("observation_timeout_ms must cover one control period")
        if self.model_format not in {"pytorch", "onnx", "tensorrt"}:
            raise ValueError(f"unsupported model_format: {self.model_format}")
        if self.role not in {"attacker", "defender"}:
            raise ValueError(f"unsupported role: {self.role}")
        if self.referee_source not in {"auto", "serial", "io"}:
            raise ValueError(f"unsupported referee_source: {self.referee_source}")
        if self.referee_baudrate <= 0:
            raise ValueError("referee_baudrate must be positive")
        for port in (self.sim_command_port, self.sim_status_port, self.scene_control_port):
            if not 1 <= port <= 65535:
                raise ValueError(f"invalid port: {port}")

    @classmethod
    def from_dict(cls, values: Dict[str, Any]) -> "RuntimeConfig":
        known = cls.__dataclass_fields__
        unknown = sorted(set(values) - set(known))
        if unknown:
            raise ValueError(f"unknown runtime config fields: {unknown}")
        return cls(**values)

    @classmethod
    def load(cls, path: str | Path) -> "RuntimeConfig":
        with Path(path).open("r", encoding="utf-8") as handle:
            values = json.load(handle)
        if not isinstance(values, dict):
            raise ValueError("runtime config must be a JSON object")
        return cls.from_dict(values)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)
