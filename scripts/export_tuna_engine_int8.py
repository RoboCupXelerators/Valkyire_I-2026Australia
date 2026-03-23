"""Export YOLOv8 (.pt) to TensorRT INT8 (.engine) using Ultralytics.

Why this script
---------------
Ultralytics supports `YOLO(...).export(format='engine', int8=True)` but INT8
requires a `data` argument (dataset yaml) to build an INT8 calibration
dataloader.

This script supports two calibration data sources:

1) --data <data.yaml> (recommended)
   Use your real training/val dataset yaml. Best accuracy.

2) --calib-dir <dir_with_images>
   If you only have unlabeled images, we auto-generate a temporary YOLO dataset
   with empty label files for each image (background images). This is valid for
   Ultralytics calibration code path (labels may be empty).

Examples
--------
1) Using your dataset yaml (recommended):

  python scripts/export_tuna_engine_int8.py \
    --pt src/vision_utils/models/yolov8s.pt \
    --engine src/vision_utils/models/yolov8s_int8.engine \
    --data /path/to/your/data.yaml \
    --imgsz 640 --batch 4 --device 0

2) Using an image folder for calibration:

  python scripts/export_tuna_engine_int8.py \
    --pt src/vision_utils/models/yolov8s.pt \
    --engine src/vision_utils/models/yolov8s_int8.engine \
    --calib-dir /path/to/calib_images \
    --imgsz 640 --batch 4 --device 0

Notes
-----
- INT8 engines should be exported on the target device (Jetson) because
  TensorRT engines are hardware-specific.
- `half=True` and `int8=True` are mutually exclusive in Ultralytics.
"""

from __future__ import annotations

import argparse
import os
import shutil
import tempfile
from pathlib import Path


IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp"}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--pt", type=str, required=True, help="Path to .pt weights")
    p.add_argument("--engine", type=str, required=True, help="Target .engine path")

    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--data", type=str, default=None, help="Ultralytics data.yaml for INT8 calibration")
    g.add_argument("--calib-dir", type=str, default=None, help="Folder of calibration images (unlabeled ok)")

    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--batch", type=int, default=4)
    p.add_argument("--device", type=str, default="0", help="'0'/'cuda:0'/'cpu'")
    p.add_argument("--workspace", type=int, default=2048, help="TensorRT workspace MB")
    p.add_argument("--dynamic", action="store_true", help="Export dynamic-shape engine")
    p.add_argument("--nms", action="store_true", help="Export end2end engine with NMS")
    p.add_argument(
        "--fraction",
        type=float,
        default=1.0,
        help="Use a fraction of calibration split images (0-1).",
    )
    p.add_argument(
        "--split",
        type=str,
        default="val",
        choices=["train", "val", "test"],
        help="Which split in data.yaml to use for calibration",
    )
    return p.parse_args()


def _list_images(d: Path) -> list[Path]:
    imgs = [p for p in sorted(d.rglob("*")) if p.is_file() and p.suffix.lower() in IMG_EXTS]
    return imgs


def _write_data_yaml(
    root: Path,
    names: dict[int, str] | None = None,
    split: str = "val",
) -> Path:
    """Write a minimal Ultralytics data.yaml.

    root layout is expected:
      root/images/{split}/*.jpg
      root/labels/{split}/*.txt
    """
    # ultralytics expects dict-like names in YAML
    names = names or {0: "object"}
    # simple YAML writer (avoid adding PyYAML dependency)
    yaml_lines = [
        f"path: {root.as_posix()}",
        f"train: images/{split}",
        f"val: images/{split}",
        "names:",
    ]
    for k in sorted(names.keys()):
        yaml_lines.append(f"  {int(k)}: {str(names[k])}")
    data_yaml = root / "data.yaml"
    data_yaml.write_text("\n".join(yaml_lines) + "\n", encoding="utf-8")
    return data_yaml


def _prepare_unlabeled_calib_dataset(calib_dir: Path, split: str = "val") -> Path:
    """Create a temporary YOLO-style dataset from an image folder.

    - Copies images into temp/images/{split}
    - Creates empty label files into temp/labels/{split}

    Returns path to generated data.yaml.
    """
    imgs = _list_images(calib_dir)
    if not imgs:
        raise FileNotFoundError(f"No images found in calib-dir: {calib_dir}")

    tmp_root = Path(tempfile.mkdtemp(prefix="ultra_int8_calib_"))
    img_out = tmp_root / "images" / split
    lbl_out = tmp_root / "labels" / split
    img_out.mkdir(parents=True, exist_ok=True)
    lbl_out.mkdir(parents=True, exist_ok=True)

    for p in imgs:
        # keep unique filename to avoid collisions
        dst = img_out / p.name
        if dst.exists():
            stem = p.stem
            dst = img_out / f"{stem}_{abs(hash(str(p))) % 10**8}{p.suffix.lower()}"
        shutil.copy2(p, dst)
        (lbl_out / f"{dst.stem}.txt").write_text("", encoding="utf-8")

    # For calibration, actual names do not affect preprocessing.
    data_yaml = _write_data_yaml(tmp_root, names={0: "object"}, split=split)
    return data_yaml


def main() -> None:
    args = parse_args()
    pt = Path(args.pt)
    engine = Path(args.engine)
    engine.parent.mkdir(parents=True, exist_ok=True)

    if not pt.exists():
        raise FileNotFoundError(pt)

    from ultralytics import YOLO

    data_yaml: str | None = None
    tmp_root: Path | None = None

    if args.data:
        data_yaml = str(Path(args.data))
        if not os.path.exists(data_yaml):
            raise FileNotFoundError(data_yaml)
    else:
        # build temporary dataset from calib images
        calib_dir = Path(args.calib_dir)
        if not calib_dir.exists():
            raise FileNotFoundError(calib_dir)
        data_path = _prepare_unlabeled_calib_dataset(calib_dir, split=args.split)
        tmp_root = data_path.parent
        data_yaml = str(data_path)
        print(f"[INFO] Generated temporary calibration dataset: {data_yaml}")

    print("[INFO] Loading model:", pt)
    m = YOLO(str(pt), task="detect")
    print("[INFO] Model names:", getattr(m.model, "names", None))
    print("[INFO] Export config:")
    print(
        "  imgsz=", args.imgsz,
        " batch=", args.batch,
        " device=", args.device,
        " int8=True",
        " dynamic=", bool(args.dynamic),
        " nms=", bool(args.nms),
        " workspace=", args.workspace,
        " split=", args.split,
        " fraction=", args.fraction,
        " data=", data_yaml,
    )

    exported = m.export(
        format="engine",
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        half=False,
        int8=True,
        data=data_yaml,
        workspace=args.workspace,
        dynamic=bool(args.dynamic),
        nms=bool(args.nms),
        split=args.split,
        fraction=float(args.fraction),
    )

    exported = Path(str(exported))
    if exported.resolve() != engine.resolve():
        if exported.exists():
            shutil.copy2(exported, engine)
            print("[INFO] Copied exported engine to:", engine)
        else:
            raise FileNotFoundError(f"Exporter returned {exported} but file not found")

    print("[OK] Engine:", engine, "size=", engine.stat().st_size)

    if tmp_root is not None:
        print(f"[INFO] Temporary calibration dataset kept at: {tmp_root}")
        print("       You may delete it manually after confirming engine quality.")


if __name__ == "__main__":
    main()
