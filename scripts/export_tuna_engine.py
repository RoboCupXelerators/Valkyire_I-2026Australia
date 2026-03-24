"""Export TunaIntelligence YOLOv8 model to TensorRT .engine.

Usage:
  source venv/bin/activate
  python scripts/export_tuna_engine.py \
    --pt src/vision_utils/models/yolov8nForX5.pt \
    --engine src/vision_utils/models/yolov8nForX5.engine \
    --imgsz 640 --batch 4 --half

Notes:
  - This uses Ultralytics exporter (pt -> onnx -> tensorrt).
  - For Jetson/Orin, batch=4 allows 4-camera batching.
  - INT8 export is not enabled here (needs calibration dataset).
"""

from __future__ import annotations

import argparse
import os
import shutil
from pathlib import Path


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--pt", type=str, required=True, help="Path to .pt weights")
    p.add_argument("--engine", type=str, required=True, help="Target .engine path")
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--batch", type=int, default=4)
    p.add_argument("--device", type=str, default="0")
    p.add_argument("--half", action="store_true", help="Export FP16 engine")
    p.add_argument("--workspace", type=int, default=2048, help="TensorRT workspace MB")
    p.add_argument("--nms", action="store_true", help="Export end2end engine with NMS")
    return p.parse_args()


def main():
    args = parse_args()
    pt = Path(args.pt)
    engine = Path(args.engine)
    engine.parent.mkdir(parents=True, exist_ok=True)

    if not pt.exists():
        raise FileNotFoundError(pt)

    from ultralytics import YOLO

    m = YOLO(str(pt), task="detect")
    print("Model names:", getattr(m.model, "names", None))
    print(f"Exporting: {pt} -> {engine}")

    exported = m.export(
        format="engine",
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        half=bool(args.half),
        workspace=args.workspace,
        nms=bool(args.nms),
        int8=False,
    )

    exported = Path(str(exported))
    if exported.resolve() != engine.resolve():
        # Ultralytics exports next to weights by default.
        # IMPORTANT: use copy (not move) to avoid deleting an existing *.engine
        # that other code paths expect (e.g. yolov8nForX5.engine).
        if exported.exists():
            shutil.copy2(exported, engine)
            print("Copied exported engine to:", engine)
        else:
            print("WARNING: exporter returned path but file not found:", exported)

    print("Done. Engine:", engine, "size=", engine.stat().st_size)


if __name__ == "__main__":
    main()
