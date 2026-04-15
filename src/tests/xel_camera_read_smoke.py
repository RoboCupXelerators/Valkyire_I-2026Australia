"""xel_camera_read_smoke

最小摄像头联调脚本：验证 Unity 侧单端口 MJPEG 流能否被 OpenCV 的 `cam.read()` 捕捉。

Unity 侧要求：
- 在一个 Camera GameObject 上挂 `XelCameraStream`
- 确保 Unity Play 后日志里出现类似：
  [XelCameraStream] MJPEG server started on http://127.0.0.1:20801/stream

用法：
    python Software/src/tests/xel_camera_read_smoke.py

可选环境变量：
- XEL_CAM_URL: 默认 http://127.0.0.1:20801/stream?id=main
- XEL_CAM_ID: 默认 main，用于多摄像头时指定 streamId
- XEL_CAM_INDEX: 备用本地摄像头索引，默认空（不启用）
"""

from __future__ import annotations

import os
import sys
import time

import cv2


def open_capture():
    cam_id = os.environ.get("XEL_CAM_ID", "robot2").strip()
    base_url = os.environ.get("XEL_CAM_URL", "http://127.0.0.1:20801/stream")
    cam_index = os.environ.get("XEL_CAM_INDEX", "0").strip()

    # 1) 优先打开 Unity MJPEG HTTP 流（默认带 streamId）
    candidates = []
    if cam_id:
        candidates.append(f"{base_url}?id={cam_id}_{cam_index}")
    candidates.append(base_url)

    for url in candidates:
        cap = cv2.VideoCapture(url)
        if cap.isOpened():
            return cap, f"url={url}"
        cap.release()

    # 2) 可选：回退到本机摄像头索引，便于排查 OpenCV 环境本身是否正常
    if cam_index:
        try:
            idx = int(cam_index)
        except ValueError:
            raise RuntimeError(f"XEL_CAM_INDEX 无效: {cam_index}")

        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            return cap, f"index={idx}"
        cap.release()

    raise RuntimeError(
        "无法打开摄像头/视频流。请确认 Unity 已开启 XelCameraStream，或检查 XEL_CAM_URL/XEL_CAM_ID。"
    )


def main() -> int:
    print("[cam-smoke] opening capture...")
    cap, source = open_capture()
    print(f"[cam-smoke] opened: {source}")

    try:
        ok_count = 0
        for i in range(320):
            t0 = time.time()
            ret, frame = cap.read()
            dt = (time.time() - t0) * 1000.0

            if not ret or frame is None:
                print(f"[cam-smoke] frame {i}: read failed")
                time.sleep(0.05)
                continue

            ok_count += 1
            h, w = frame.shape[:2]
            mean_val = float(frame.mean())
            print(f"[cam-smoke] frame {i}: ok size={w}x{h} mean={mean_val:.1f} read={dt:.1f}ms")

            # 简单展示：按 q 退出；无 GUI 环境下会自动跳过
            try:
                cv2.imshow("XEL Camera Smoke", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            except Exception:
                pass

            time.sleep(0.03)

        print(f"[cam-smoke] success frames={ok_count}")
        return 0 if ok_count > 0 else 2
    finally:
        cap.release()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())