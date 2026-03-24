"""Offline test for segmented multi-cam recording.

This test does NOT require real cameras.
It feeds synthetic frames into `SegmentedMultiCamRecorder` and verifies
that video files are created and finalized (tmp -> final rename).

Run:
  python src/tests/test_record_offline.py
"""

from __future__ import annotations

import os
import shutil
import sys
import time
from pathlib import Path

import numpy as np


def main():
    # Allow running directly via `python src/tests/...py`
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    if repo_root not in sys.path:
        sys.path.insert(0, repo_root)

    from src.vision_utils.PreProcess import SegmentedMultiCamRecorder, _RecordConfig
    from src.vision_utils.VisionDatas import logger

    out_dir = Path("/tmp/vision_record_test")
    if out_dir.exists():
        shutil.rmtree(out_dir, ignore_errors=True)
    out_dir.mkdir(parents=True, exist_ok=True)

    cfg = _RecordConfig(
        enabled=True,
        record_dir=out_dir,
        cams=[0, 1, 2, 3],
        record_size=(160, 120),
        segment_seconds=1,  # rotate quickly
        fps=10,
        codec="mp4v",
        ext="mp4",
    )
    rec = SegmentedMultiCamRecorder(logger=logger, cam_ports=[0, 1, 2, 3], cfg=cfg)

    # feed frames for ~2.5s to ensure at least one rotation happens
    t0 = time.time()
    n = 0
    while time.time() - t0 < 2.5:
        frames = []
        for cam_i in range(4):
            # create a unique, valid BGR frame
            fr = np.zeros((240, 320, 3), dtype=np.uint8)
            fr[..., 0] = (cam_i * 50) % 255
            fr[..., 1] = (n * 10) % 255
            fr[..., 2] = 200
            frames.append(fr)
        rec.write_frames(frames)
        n += 1
        time.sleep(0.1)

    rec.close()

    # verify files
    finals = list(out_dir.rglob("*.mp4")) + list(out_dir.rglob("*.avi"))
    tmps = list(out_dir.rglob("*.tmp.*"))
    assert len(tmps) == 0, f"tmp files should be finalized, still have: {tmps}"
    assert len(finals) > 0, "no video files created"
    # ensure files are non-empty
    assert all(p.stat().st_size > 0 for p in finals), "some output videos are empty"

    print("OK", "files=", len(finals), "dir=", out_dir)


if __name__ == "__main__":
    main()
