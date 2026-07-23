"""Ray-free policy runtime and referee-priority control gate."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import time
from typing import Any, Callable, Optional

from .actions import decode_policy_action
from .bundle import load_manifest, load_pytorch_bundle
from .config import RuntimeConfig
from .contracts import DecisionPacket, DribbleMode, FaceMode, PolicyObservation, RefereeState, RoleId, ShootIntent
from .features import FeatureEncoder


@dataclass(frozen=True)
class GateDecision:
    enabled: bool
    reason: str


class ControlGate:
    def __init__(self, observation_timeout_ms: int):
        self.observation_timeout_ms = observation_timeout_ms

    def evaluate(self, observation: PolicyObservation, now_ms: Optional[int] = None) -> GateDecision:
        if not observation.control_enabled:
            return GateDecision(False, "referee_control_disabled")
        if observation.penalty_active:
            return GateDecision(False, f"penalty:{observation.penalty_reason}")
        if int(observation.referee_state) != int(RefereeState.PLAY):
            return GateDecision(False, f"referee_state:{observation.referee_state}")
        if observation.observed_at_ms > 0:
            now_ms = now_ms if now_ms is not None else int(time.monotonic() * 1000)
            if now_ms - observation.observed_at_ms > self.observation_timeout_ms:
                return GateDecision(False, "stale_observation")
        return GateDecision(True, "play")


class PolicyRuntime:
    def __init__(self, config: RuntimeConfig, model: Any = None):
        self.config = config
        self.encoder = FeatureEncoder()
        self.gate = ControlGate(config.observation_timeout_ms)
        self.backend = "pytorch"
        self.session = None
        if model is None and config.model_format == "pytorch":
            model, self.manifest = load_pytorch_bundle(config.bundle_path, config.device)
        elif model is None:
            self.manifest = load_manifest(config.bundle_path)
            if self.manifest.model_format != "onnx":
                raise ValueError(f"{config.model_format} runtime requires an ONNX bundle")
            try:
                import onnxruntime as ort
            except ImportError as exc:
                raise RuntimeError("onnxruntime is required for ONNX/TensorRT inference") from exc
            providers = ["CPUExecutionProvider"]
            if config.model_format == "tensorrt":
                available = ort.get_available_providers()
                if "TensorrtExecutionProvider" not in available:
                    raise RuntimeError("TensorrtExecutionProvider is not available")
                providers = ["TensorrtExecutionProvider", "CUDAExecutionProvider", "CPUExecutionProvider"]
            self.session = ort.InferenceSession(
                str(Path(config.bundle_path) / self.manifest.model_file),
                providers=providers,
            )
            self.backend = "onnx"
            model = None
        else:
            self.manifest = None
        self.model = model

    def infer(self, observation: PolicyObservation, deterministic: bool = True) -> DecisionPacket:
        gate = self.gate.evaluate(observation)
        if not gate.enabled:
            return DecisionPacket.stopped()

        encoded = self.encoder.encode(observation)
        if self.manifest is not None:
            encoded = [
                (value - float(mean)) / max(abs(float(std)), 1e-6)
                for value, mean, std in zip(
                    encoded,
                    self.manifest.normalization_mean,
                    self.manifest.normalization_std,
                )
            ]
        if self.backend == "onnx":
            continuous, categories, entropy = self._infer_onnx(encoded, int(observation.role))
        else:
            import torch
            features = torch.tensor([encoded], dtype=torch.float32, device=self.config.device)
            role = torch.tensor([int(observation.role)], dtype=torch.long, device=self.config.device)
            with torch.inference_mode():
                action, _, entropy_tensor, _ = self.model.sample(features, role, deterministic=deterministic)
            continuous = action["continuous"][0].detach().cpu().tolist()
            categories = {name: int(action[name][0]) for name in ("intent", "face", "dribble", "shoot")}
            entropy = float(entropy_tensor[0].detach().cpu())
        confidence = self._confidence(entropy)
        return decode_policy_action(
            continuous,
            categories,
            confidence,
            self.config.control_period_ms,
        )

    def _infer_onnx(self, features: list[float], role: int):
        import numpy as np

        outputs = self.session.run(
            None,
            {"features": np.asarray([features], dtype=np.float32), "role": np.asarray([role], dtype=np.int64)},
        )
        continuous = outputs[0][0].tolist()
        names = ("intent", "face", "dribble", "shoot")
        categories = {name: int(np.argmax(logits[0])) for name, logits in zip(names, outputs[1:5])}
        entropy = 0.0
        for logits in outputs[1:5]:
            shifted = logits[0] - np.max(logits[0])
            probabilities = np.exp(shifted) / np.exp(shifted).sum()
            entropy -= float(np.sum(probabilities * np.log(probabilities + 1e-8)))
        if len(outputs) > 6:
            entropy += float(np.sum(0.5 * (1.0 + math.log(2.0 * math.pi)) + outputs[6][0]))
        return continuous, categories, entropy

    @staticmethod
    def _confidence(entropy: float) -> float:
        return max(0.0, min(1.0, math.exp(-max(0.0, entropy) / 10.0)))
