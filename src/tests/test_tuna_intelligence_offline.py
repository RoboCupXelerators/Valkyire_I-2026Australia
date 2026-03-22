"""Offline smoke test for TunaIntelligence.

Runs TunaIntelligence without initializing cameras, feeds the same test image
to all 4 camera slots, and verifies that queues produce expected shapes.

This is meant to be runnable both on dev PC (CPU) and Jetson (CUDA), assuming
`torch` + `ultralytics` are installed.
"""

import time
import os
import sys

import cv2


def main():
    # Allow running directly via `python src/tests/...py`
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    if repo_root not in sys.path:
        sys.path.insert(0, repo_root)

    # Import module directly to avoid triggering optional backends in
    # `src/vision_utils/__init__.py`.
    from src.vision_utils.TunaIntelligence import TunaIntelligence

    img_paths = [
        os.path.join(repo_root, "src", "vision_utils", "test1.jpg"),
        os.path.join(repo_root, "src", "vision_utils", "test2.jpg"),
    ]
    imgs = []
    for p in img_paths:
        im = cv2.imread(p)
        if im is None:
            raise FileNotFoundError(f"Cannot read {p}")
        imgs.append(im)

    try:
        import ultralytics  # noqa: F401
    except Exception as e:
        print("[SKIP] ultralytics not installed, cannot run inference in this environment.")
        print("Reason:", e)
        print("On Jetson, install torch + ultralytics, then re-run this test.")
        return

    det = TunaIntelligence(init_cams=False, device="cpu", export_engine_if_missing=False)

    # feed a few frames (alternating images)
    for _ in range(6):
        im = imgs[_ % len(imgs)]
        det.Frames = [im.copy(), im.copy(), im.copy(), im.copy()]
        time.sleep(0.05)

    chassis = det.ChassisQueue.get(timeout=5)
    ball = det.BallQueue.get(timeout=5)

    print("chassis:", chassis)
    print("ball:", ball)

    assert isinstance(chassis, list) and len(chassis) == 4
    assert isinstance(ball, list) and len(ball) == 4

    # each entry: [cam, cx, bottom_y, w, h, conf]
    assert all(len(x) == 6 for x in chassis)
    assert all(len(x) == 6 for x in ball)

    print("OK")

    # Stop background threads before interpreter shutdown (avoids abort on some builds)
    det.close(join_timeout=2.0)


if __name__ == "__main__":
    main()
