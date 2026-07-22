# HybridVision TODO

This document tracks the proposed YOLO + Blob + tracker architecture for high-rate
robot vision. The goal is to raise the control-facing vision update rate without
requiring every camera frame to run through YOLO.

## Goal

- Provide a stable 60 Hz fused vision output for strategy/control.
- Keep YOLO as the reliable but slower "tick" detector.
- Use Blob/ROI tracking as the fast "subtick" tracker for ball motion.
- Keep the public `UnitedVision`/control panel API compatible while allowing
  hardware-specific acceleration profiles.

Target rates:

- Orin profile: 4 cameras at 60 fps capture, YOLO at 20-30 Hz, Blob/ROI and fused
  output at 60 Hz.
- RPi5 profile: Blob/ROI at 30-60 Hz, YOLO disabled or low-frequency CPU fallback
  only for validation/relocalization.

## Architecture

Proposed file layout:

```text
src/vision_utils/
  HybridVision.py
  hybrid/
    __init__.py
    types.py
    scheduler.py
    fusion.py
    kalman.py
    roi.py
    blob_tracker.py
    yolo_detector.py
    perf.py
    profiles.py
    backends/
      __init__.py
      orin_tensorrt.py
      cuda_torch.py
      cpu_blob.py
```

Responsibilities:

- `HybridVision.py`: compatibility wrapper exposing existing methods such as
  `GetBallPos()`, `GetChassisPos()`, `GetBallBBoxes()`, `GetGoalBBoxes()`, and
  performance fields such as `PreProcessMs`, `InferMs`, `PostProcessMs`.
- `types.py`: shared dataclasses for frames, detections, tracks, timestamps,
  confidence, and fused output.
- `scheduler.py`: tick/subtick timing. YOLO runs at low frequency, Blob/ROI runs
  every frame, fusion emits fixed-rate output.
- `yolo_detector.py`: abstract detector interface. It must hide TensorRT/CUDA/CPU
  details from the fusion layer.
- `blob_tracker.py`: high-rate ball tracker, preferably ROI-first and full-frame
  only when relocalizing.
- `roi.py`: ROI growth/shrink, speed-aware prediction window, and gating.
- `kalman.py`: ball position/velocity estimator.
- `fusion.py`: confidence arbitration between YOLO, Blob, and prediction.
- `perf.py`: metrics for control panel.
- `profiles.py`: configuration presets for Orin, CUDA fallback, and RPi5.

## Data Flow

```text
Camera frames
  -> YOLO tick detector, 20-30 Hz on Orin
  -> Blob subtick tracker, 60 Hz or 120 Hz depending on camera
  -> ROI gate + Kalman fusion
  -> Fused VisionOutput at control rate
  -> Existing strategy/control panel API
```

Ball handling:

- YOLO confirms object identity and resets drift.
- Blob tracks the ball inside a ROI between YOLO ticks.
- Kalman predicts short gaps and estimates velocity.
- If Blob and YOLO disagree beyond a gate, prefer YOLO and reset the ROI.
- If there is no valid observation for too long, lower confidence and stop
  publishing a trusted ball position.

Chassis/goal handling:

- Keep YOLO as the primary detector.
- Lower frequency is acceptable; chassis and goal do not need 60 Hz YOLO.
- Optional tracker can be added later if strategy needs smoother opponent motion.

## Configuration

Proposed config keys under `Vision`:

```json
{
  "Mode": "hybrid",
  "Profile": "orin",
  "CaptureFps": 60,
  "OutputHz": 60,
  "YoloHz": 20,
  "TrackerHz": 60,
  "YoloModelSize": "v8n",
  "YoloInputSize": 640,
  "YoloBatch": 4,
  "AdaptiveFrameDistribution": true,
  "CameraMinYoloHz": 4,
  "CameraFocusYoloHz": 20,
  "CameraFocusHoldMs": 250,
  "YoloMaxFramesPerTick": 4,
  "TrackerRoiScale": 2.0,
  "TrackerLostMs": 120,
  "TrackerPredictMs": 50
}
```

Profile behavior:

- `orin`: TensorRT batch=4 YOLO, Blob/ROI at camera rate, 60 Hz fused output.
- `cuda`: PyTorch/CUDA fallback, lower YOLO rate, same API.
- `rpi5`: Blob-first operation. YOLO is optional, low-rate, and should not block
  the output loop.

When `AdaptiveFrameDistribution` is enabled, every camera keeps the configured
low-rate YOLO search budget. A camera whose Blob or YOLO result contains a ball
gets the focus budget for `CameraFocusHoldMs`; other cameras continue low-rate
search so the target can be reacquired after it moves. `YoloMaxFramesPerTick`
limits how many new camera frames enter one YOLO call.

## TODO

### Phase 0: Baseline and Interfaces

- [ ] Define `VisionOutput`, `Detection`, `TrackState`, and `PerfSnapshot` in
  `hybrid/types.py`.
- [ ] Document existing sentinel values and coordinate semantics used by
  `UnitedVision`.
- [ ] Add a `HybridVision` skeleton that returns current placeholder values but
  exposes the same API as `UnitedVision`.
- [ ] Add config parsing for `Vision.Mode`, `Vision.Profile`, `Vision.OutputHz`,
  `Vision.YoloHz`, and `Vision.TrackerHz`.
- [ ] Add control panel fields for `blob_ms`, `fusion_ms`, `output_hz`,
  `yolo_hz`, `tracker_hz`, and `vision_confidence`.

### Phase 1: Blob/ROI Subtick Tracker

- [ ] Extract reusable ball blob logic from `Blob.py` into
  `hybrid/blob_tracker.py`.
- [ ] Implement ROI-first tracking with full-frame relocalization fallback.
- [ ] Add blob confidence based on color mask area, circularity, bbox size,
  continuity, and ROI gate distance.
- [ ] Add speed-aware ROI expansion so fast ball movement does not immediately
  leave the search area.
- [ ] Add rejection rules for impossible jumps and low-confidence flicker.
- [ ] Benchmark Blob/ROI at 60 Hz and 120 Hz on real camera frames.

### Phase 2: YOLO Tick Detector

- [ ] Create `YoloDetector` interface with `detect_batch(frames)`.
- [ ] Move Orin TensorRT-specific behavior into `hybrid/backends/orin_tensorrt.py`.
- [ ] Keep `TunaIntelligence` behavior available as a backend, not as the fusion
  owner.
- [ ] Support YOLOv8n/v8s model selection and input size selection.
- [ ] Export and benchmark YOLOv8n TensorRT batch=4 engines at 640, 512, and 416.
- [ ] Decide whether end-to-end TensorRT NMS is worth using for lower CPU cost.

### Phase 3: Fusion and Kalman

- [ ] Implement ball Kalman filter with position and velocity state.
- [ ] Fuse YOLO detections as high-confidence corrections.
- [ ] Fuse Blob detections as high-rate local observations.
- [ ] Emit prediction-only output for a short gap, then degrade confidence.
- [ ] Proposed trust windows:
  - valid observation age `< 50 ms`: trusted output.
  - `50-120 ms`: predicted output with lower confidence.
  - `> 120 ms`: ball lost sentinel.
- [ ] Add conflict arbitration:
  - YOLO vs Blob large disagreement: reset Blob ROI to YOLO.
  - Blob-only large jump: reject unless velocity gate allows it.
  - Repeated Blob low confidence: wait for YOLO relocalization.

### Phase 4: Scheduler

- [ ] Implement fixed-rate output loop independent of YOLO completion.
- [ ] Run camera capture at camera rate.
- [ ] Run Blob/ROI every frame or every output tick.
- [ ] Run YOLO tick at 20-30 Hz on Orin without blocking fusion output.
- [ ] Use latest-frame semantics so slow YOLO does not process stale frames.
- [ ] Add queue size limits of 1 for real-time paths.
- [ ] Add timestamps to all detections and fused outputs.

### Phase 5: Hardware Profiles

- [ ] Orin profile:
  - [ ] TensorRT YOLOv8n batch=4.
  - [ ] 4 cameras at 60 fps capture.
  - [ ] Blob/ROI at 60 Hz.
  - [ ] Fused output at 60 Hz.
- [ ] RPi5 profile:
  - [ ] Blob/ROI at 30-60 Hz.
  - [ ] Optional CPU YOLO at low rate only when enabled.
  - [ ] No TensorRT-specific imports.
- [ ] Add runtime capability detection without importing hardware drivers that
  initialize lidar or chassis.
- [ ] Make unsupported acceleration fall back cleanly and visibly in control panel.

### Phase 6: Validation

- [ ] Add offline replay benchmark using recorded frames.
- [ ] Add live benchmark summary:
  - camera fps
  - yolo tick fps
  - blob tracker fps
  - fused output fps
  - end-to-end age in ms
  - lost-ball duration
  - false jump count
- [ ] Compare pure YOLO vs hybrid output:
  - latency
  - smoothness
  - ball loss recovery time
  - false positive rate
  - control behavior during fast ball movement
- [ ] Validate under lighting changes, reflections, white lines, and ball occlusion.
- [ ] Add a control panel overlay that shows YOLO bbox, Blob ROI, fused ball point,
  and confidence state.

## Design Constraints

- Do not let the vision-only control panel initialize LunaPre runtime, lidar, or
  chassis.
- Keep strategy-facing semantics stable: robot-frame centimeters and existing
  sentinel values must remain compatible.
- Do not block fused output on YOLO.
- Prefer dropping stale frames over building latency.
- Treat Blob as a short-term tracker, not as an unconditional truth source.
- Store performance metrics separately for YOLO, Blob, fusion, and output.

## Open Questions

- Should the first implementation target 60 Hz only, or support 120 Hz output
  from the start?
- Is YOLOv8n at 640 accurate enough for small ball detection, or do we need
  512/640 adaptive input by camera/ROI?
- Should chassis/opponent detections get a separate lightweight tracker later?
- Do we want TensorRT end-to-end NMS, or keep Python-side NMS for easier debugging?
- Should RPi5 run a low-rate model at all, or stay Blob-only for match runtime?
