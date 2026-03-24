"""Debug live camera ball detections and save annotated frames.

This helps verify that:
  - cameras are online
  - TunaIntelligence detects ball on a specific camera
  - bbox output pipeline is correct

It reads frames from TunaIntelligence (same pipeline as Vision.py), fetches
latest BallBBoxQueue, draws the bbox on the corresponding 640x640 resized image,
and saves a few frames to disk.

Example:
  source venv/bin/activate
  python scripts/debug_cam_ball_bbox.py --cam-index 1 --outdir outputs/cam6_debug

Note:
  cam-index is the index in CamPorts list (0..3). If CamPorts=[4,6,0,2], then
  cam-index=1 corresponds to /dev/video6.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path
import os
import sys

import cv2


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--cam-index", type=int, default=1, help="0..3 index in CamPorts")
    p.add_argument("--frames", type=int, default=30, help="number of frames to save")
    p.add_argument("--stride", type=int, default=3, help="save every Nth frame")
    p.add_argument("--outdir", type=str, default="outputs/cam_ball_debug")
    p.add_argument("--conf", type=float, default=0.25)
    return p.parse_args()


def main():
    args = parse_args()

    # Allow running from repo root without installing as a package
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    if repo_root not in sys.path:
        sys.path.insert(0, repo_root)

    from src.vision_utils.TunaIntelligence import TunaIntelligence

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    ti = TunaIntelligence(init_cams=True, prefer_engine=True, export_engine_if_missing=False)
    print("CamPorts:", ti.CamPorts)
    cam_i = int(args.cam_index)

    saved = 0
    k = 0
    try:
        while saved < int(args.frames):
            k += 1
            # get latest resized frame by reusing TI resize (matches inference input)
            f = ti.Frames[cam_i]
            if f is None:
                time.sleep(0.02)
                continue
            f640 = ti.Resize(f, (640, 640))

            # get latest bbox list (non-blocking)
            bbox = None
            try:
                out = ti.BallBBoxQueue.get_nowait()
                if out and len(out) == 4:
                    bbox = out[cam_i]
            except Exception:
                pass

            if bbox is not None:
                try:
                    _, x1, y1, x2, y2, conf = bbox
                    if conf is not None and conf >= float(args.conf):
                        cv2.rectangle(f640, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                        cv2.putText(
                            f640,
                            f"Ball {conf:.2f}",
                            (max(0, int(x1)), max(20, int(y1) - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (0, 255, 0),
                            2,
                        )
                except Exception:
                    pass

            if k % int(args.stride) == 0:
                outp = outdir / f"cam{cam_i}_{saved:03d}.jpg"
                cv2.imwrite(str(outp), f640)
                saved += 1
                print("saved", outp)
            time.sleep(0.02)
    finally:
        # stop threads to reduce chance of abort
        try:
            ti.close(join_timeout=1.0)
        except Exception:
            pass


if __name__ == "__main__":
    main()
