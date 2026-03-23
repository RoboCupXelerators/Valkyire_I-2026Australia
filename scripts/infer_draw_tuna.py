"""Run Tuna YOLOv8 TensorRT engine on sample images and save visualizations.

This script loads the TensorRT engine exported from Ultralytics and runs inference
on test images, then draws bounding boxes and labels and writes output images.

Example:
  source venv/bin/activate
  python scripts/infer_draw_tuna.py \
    --engine src/vision_utils/models/yolov8nForX5.engine \
    --images src/vision_utils/test1.jpg src/vision_utils/test2.jpg \
    --outdir outputs/tuna_vis
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--engine", type=str, required=True)
    p.add_argument("--images", type=str, nargs="+", required=True)
    p.add_argument("--outdir", type=str, default="outputs/tuna_vis")
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument(
        "--batch",
        type=int,
        default=4,
        help="Fixed batch size for TRT engine. If engine was exported with batch=4, keep this 4.",
    )
    p.add_argument("--device", type=str, default="0")
    p.add_argument("--conf", type=float, default=0.25)
    return p.parse_args()


def main():
    args = parse_args()

    from ultralytics import YOLO

    engine = Path(args.engine)
    if not engine.exists():
        raise FileNotFoundError(engine)

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    model = YOLO(str(engine), task="detect")
    print("Loaded engine:", engine)
    print("Names:", model.names)

    for img_path in args.images:
        p = Path(img_path)
        if not p.exists():
            print("[SKIP] missing", p)
            continue

        # If engine is fixed-batch (e.g., batch=4), Ultralytics/TensorRT will assert when
        # given a smaller batch. We therefore duplicate the same image to match batch.
        source = [str(p)] * int(args.batch)

        results = model.predict(
            source=source,
            imgsz=args.imgsz,
            conf=args.conf,
            device=args.device,
            batch=int(args.batch),
            verbose=False,
        )
        r0 = results[0]
        drawn = r0.plot()  # BGR uint8

        out_path = outdir / f"{p.stem}_trt.jpg"
        ok = cv2.imwrite(str(out_path), drawn)
        if not ok:
            raise RuntimeError(f"Failed to write {out_path}")

        n = 0 if r0.boxes is None else len(r0.boxes)
        print(f"{p.name}: dets={n} -> {out_path}")


if __name__ == "__main__":
    main()
