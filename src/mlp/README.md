# MLP V2 inference runtime

This package is the canonical policy contract and inference implementation. It
does not depend on Ray, Gymnasium, or the training repository.

Key properties:

- 49 deterministic observation features plus a separate attacker/defender role.
- A versioned `VectorObservationEncoder -> ControlInterface[256] -> ControlCore`
  path, followed by independent 128-wide action heads and a value head.
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

The actor contract deliberately exposes only the official referee wire semantics:
`PLAY` or `STOP`. Penalty detail, score, and match time remain fixed deployment
defaults in policy observations; Unity may use them only for rewards, masks,
resets, and evaluation.

## ControlInterface V1

`ControlInterface` is the explicit V2/V3 warm-start boundary. It is a
float32, 256-wide tensor produced by `VectorObservationEncoder` after the
bundle's 49-feature normalization and role conditioning. `ControlCore`, all
actor heads, the value head, the role embedding, and `log_std` are preserved
when a future encoder is introduced. Bundle manifests record architecture ID,
ControlInterface version/width/dtype, and the stable module prefixes.

Pre-modular V2 checkpoints use a documented one-time key migration during
PyTorch bundle loading. The migration is strict: unknown, missing, or
shape-incompatible keys fail loading rather than being silently discarded.
