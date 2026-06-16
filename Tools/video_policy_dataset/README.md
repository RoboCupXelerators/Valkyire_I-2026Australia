# Video Policy Dataset Demo

This is a small offline demo for the planned match-video dataset tool.

It proves the intended data path without requiring a real match video yet:

```text
synthetic match frames
  -> mock YOLO detections
  -> field-space tracklets
  -> mock Gemma/Qwen segment labels
  -> DeployObservationV1
  -> transitions_*.npz for mlp.behavior_cloning
```

The mock detector and mock VLM are deliberately isolated behind small classes
so they can be replaced later with:

- local YOLO inference from `src/vision_utils/models`
- real camera calibration / homography
- a local VLM/LLM pass that labels short trajectory segments

## Run

From `D:\github\Software`:

```powershell
python tools\video_policy_dataset\demo_pipeline.py --frames 240 --output-dir Records\video_policy_dataset_demo
```

The command writes:

- `detections_*.jsonl`: mock YOLO output
- `tracklets_*.jsonl`: field-space object tracks
- `labels_*.jsonl`: segment-level mock Gemma/Qwen labels
- `transitions_*.npz`: current MLP behavior-cloning data
- `summary_*.json`: output paths and counts
- `report_*.html`: standalone visual report

## Smoke Train

After generating an `.npz`, run:

```powershell
python src\mlp\behavior_cloning.py --data Records\video_policy_dataset_demo\transitions_YYYYMMDD_HHMMSS.npz --epochs 2 --batch-size 32
```

The current model still consumes the legacy 5D action vector
`[vx, vy, wz, kick, dribble]`. The demo keeps that compatibility while also
recording the high-level `DecisionPacketV1` labels needed for the next model
head.

