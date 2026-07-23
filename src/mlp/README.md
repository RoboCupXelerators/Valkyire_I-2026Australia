# MLP V2 inference runtime

This package is the canonical policy contract and inference implementation. It
does not depend on Ray, Gymnasium, or the training repository.

Key properties:

- 49 deterministic observation features plus a separate attacker/defender role.
- A shared `256 -> 256` MLP trunk and independent 128-wide action heads.
- Tanh-squashed continuous actions with consistent sampling and log-probability.
- A referee-priority control gate that returns a stopped action before inference.
- Versioned bundles with feature/action contract hashes.
- PyTorch and ONNX Runtime backends; Jetson can select ONNX Runtime's TensorRT
  execution provider without changing the bundle contract.

`src/MLPStrategy.py` is the deployment entry point. Select a complete platform
configuration with `--config` or `XEL_MLP_CONFIG`. A missing or incompatible
bundle is a hard error; runtime exceptions lead to a safe stop.

Training code belongs in `TreeDiagramDataset`. Only promoted inference bundles
may be placed under `.runtime/policies`, which is not source-controlled.
