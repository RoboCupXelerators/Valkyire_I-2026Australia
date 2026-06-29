"""Offline smoke test for HybridVision.

This test avoids camera initialization and runs the hybrid component on synthetic
orange-ball frames. It validates import, thread startup, Blob/ROI tracking,
fusion output, and the UnitedVision-compatible public API.
"""

import os
import sys
import time

import cv2
import numpy as np


def main():
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    if repo_root not in sys.path:
        sys.path.insert(0, repo_root)

    from src.vision_utils.HybridVision import HybridVision

    vision = HybridVision(
        init_cams=False,
        yolo_enabled=False,
        config_overrides={
            "output_hz": 60,
            "tracker_hz": 120,
            "yolo_hz": 0,
            "min_ball_area": 40,
        },
    )
    try:
        for i in range(24):
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            # BGR color converted from HSV(10, 220, 255), inside the production
            # orange threshold used by BlobTracker.
            cv2.circle(frame, (120 + i, 120), 18, (35, 108, 255), -1)
            vision.Frames = [frame.copy(), None, None, None]
            vision._latest_timestamps = [time.time(), 0.0, 0.0, 0.0]
            time.sleep(0.012)

        ball = vision.GetBallPos()
        bboxes = vision.GetBallBBoxes()
        perf = vision.GetHybridPerf()
        print("ball:", ball)
        print("ball_bboxes:", bboxes)
        print("hybrid_perf:", perf)

        assert isinstance(ball, list) and len(ball) == 2
        assert any(item is not None for item in bboxes)
        assert perf["blob_ms"] > 0
        assert perf["fusion_ms"] >= 0
        assert perf["tracker_hz"] > 0
        assert "vision_confidence" in perf
        print("OK")
    finally:
        vision.close(join_timeout=2.0)


if __name__ == "__main__":
    main()
