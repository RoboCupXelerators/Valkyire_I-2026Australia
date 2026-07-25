"""Versioned inference bundle loading and export."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Any, Dict, Iterable, Sequence

from .contracts import ActionSpec, CONTRACT_VERSION, FeatureSpec, combined_contract_hash


@dataclass(frozen=True)
class BundleManifest:
    bundle_version: str
    model_format: str
    model_file: str
    contract_hash: str
    feature_spec: Dict[str, Any]
    action_spec: Dict[str, Any]
    normalization_mean: Sequence[float]
    normalization_std: Sequence[float]
    training_commit: str = "local"

    @classmethod
    def from_dict(cls, values: Dict[str, Any]) -> "BundleManifest":
        return cls(**values)


def create_manifest(model_format: str = "pytorch", model_file: str = "policy.pt", training_commit: str = "local") -> BundleManifest:
    feature_spec = FeatureSpec()
    action_spec = ActionSpec()
    return BundleManifest(
        bundle_version=CONTRACT_VERSION,
        model_format=model_format,
        model_file=model_file,
        contract_hash=combined_contract_hash(feature_spec, action_spec),
        feature_spec=feature_spec.to_dict(),
        action_spec=action_spec.to_dict(),
        normalization_mean=[0.0] * feature_spec.dimension,
        normalization_std=[1.0] * feature_spec.dimension,
        training_commit=training_commit,
    )


def save_pytorch_bundle(model: Any, directory: str | Path, training_commit: str = "local") -> Path:
    import torch

    destination = Path(directory)
    destination.mkdir(parents=True, exist_ok=True)
    manifest = create_manifest(training_commit=training_commit)
    torch.save(model.state_dict(), destination / manifest.model_file)
    (destination / "manifest.json").write_text(
        json.dumps(asdict(manifest), indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return destination


def load_manifest(directory: str | Path) -> BundleManifest:
    path = Path(directory) / "manifest.json"
    manifest = BundleManifest.from_dict(json.loads(path.read_text(encoding="utf-8")))
    expected = combined_contract_hash()
    if manifest.contract_hash != expected:
        raise ValueError(f"policy contract mismatch: bundle={manifest.contract_hash}, runtime={expected}")
    dimension = FeatureSpec().dimension
    if len(manifest.normalization_mean) != dimension or len(manifest.normalization_std) != dimension:
        raise ValueError("bundle normalization width does not match FeatureSpec")
    return manifest


def load_pytorch_bundle(directory: str | Path, device: str = "cpu") -> tuple[Any, BundleManifest]:
    import torch
    from .model import PolicyNetwork

    manifest = load_manifest(directory)
    if manifest.model_format != "pytorch":
        raise ValueError(f"bundle format is {manifest.model_format}, not pytorch")
    model = PolicyNetwork()
    state = torch.load(Path(directory) / manifest.model_file, map_location=device, weights_only=True)
    model.load_state_dict(state)
    model.to(device).eval()
    return model, manifest


def export_onnx(model: Any, destination: str | Path, opset: int = 17) -> Path:
    import torch
    from .model import OnnxPolicyWrapper

    path = Path(destination)
    path.parent.mkdir(parents=True, exist_ok=True)
    wrapper = OnnxPolicyWrapper(model).eval()
    torch.onnx.export(
        wrapper,
        (torch.zeros(1, 49), torch.zeros(1, dtype=torch.long)),
        path,
        input_names=["features", "role"],
        output_names=["continuous", "intent", "face", "dribble", "shoot", "value", "continuous_log_std"],
        dynamic_axes={"features": {0: "batch"}, "role": {0: "batch"}},
        opset_version=opset,
    )
    return path


def save_onnx_bundle(model: Any, directory: str | Path, training_commit: str = "local", opset: int = 17) -> Path:
    destination = Path(directory)
    destination.mkdir(parents=True, exist_ok=True)
    model_file = "policy.onnx"
    export_onnx(model, destination / model_file, opset=opset)
    manifest = create_manifest(model_format="onnx", model_file=model_file, training_commit=training_commit)
    (destination / "manifest.json").write_text(
        json.dumps(asdict(manifest), indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return destination
