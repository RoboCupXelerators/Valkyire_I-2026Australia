"""Web-based chessboard calibration for Pixel2CM.

This tool replaces the old single-frame heuristic with a proper calibration
pipeline:

1. Stream the camera in a browser.
2. Capture multiple chessboard views.
3. Estimate intrinsics + distortion from all samples.
4. Estimate the final board pose / homography from a selected sample.
5. Save a modern calibration file that pixel2cm.py can use directly.

Recommended chessboard settings:
- inner corners: 9 x 9
- square size: 15 mm

That means the distance between the first and last inner corner in either
direction is 8 * 15 mm = 120 mm = 12 cm.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import cv2
import gradio as gr
import numpy as np

APP_DIR = Path(__file__).parent
CALIB_DIR = APP_DIR / "Calibration"
CALIB_DIR.mkdir(parents=True, exist_ok=True)

DEFAULT_CAM_INDEX = 0
DEFAULT_BOARD_W = 9
DEFAULT_BOARD_H = 9
DEFAULT_SQUARE_MM = 15.0
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480


@dataclass
class Sample:
    frame: np.ndarray
    corners: np.ndarray
    image_size: tuple[int, int]
    area: float
    ts: float = field(default_factory=time.time)


@dataclass
class CalibrationResult:
    ok: bool = False
    K: Optional[np.ndarray] = None
    dist: Optional[np.ndarray] = None
    rvec: Optional[np.ndarray] = None
    tvec: Optional[np.ndarray] = None
    H: Optional[np.ndarray] = None
    rmse: float = 0.0
    board_size: tuple[int, int] = (DEFAULT_BOARD_W, DEFAULT_BOARD_H)
    square_size_mm: float = DEFAULT_SQUARE_MM
    origin: str = "bottom_left_inner_corner"
    image_size: tuple[int, int] = (DEFAULT_WIDTH, DEFAULT_HEIGHT)


class CalibrationSession:
    def __init__(self):
        self.lock = threading.Lock()
        self.running = False
        self.cap: Optional[cv2.VideoCapture] = None
        self.latest_frame: Optional[np.ndarray] = None
        self.latest_corners: Optional[np.ndarray] = None
        self.latest_score: float = 0.0
        self.latest_found: bool = False
        self.latest_size: tuple[int, int] = (DEFAULT_WIDTH, DEFAULT_HEIGHT)
        self.samples: list[Sample] = []
        self.result = CalibrationResult()
        self.status = "Idle"

    def reset_samples(self):
        with self.lock:
            self.samples.clear()


SESSION = CalibrationSession()


def find_available_cameras(max_index: int = 20) -> list[str]:
    out = []
    try:
        cv2.setLogLevel(0)
    except Exception:
        pass
    for i in range(max_index):
        cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            continue
        ret, _ = cap.read()
        cap.release()
        if ret:
            out.append(str(i))
    try:
        cv2.setLogLevel(3)
    except Exception:
        pass
    return out or ["0"]


def board_object_points(board_w: int, board_h: int, square_mm: float) -> np.ndarray:
    """Return chessboard object points with origin at bottom-left inner corner.

    The detected image points are ordered row-major from top-left to bottom-right.
    We map them to a board coordinate system where:
      - X goes to the right
      - Y goes upward (bottom-left corner is the origin)
      - Z is 0 on the board plane
    """
    objp = np.zeros((board_w * board_h, 3), np.float32)
    for row in range(board_h):
        for col in range(board_w):
            idx = row * board_w + col
            objp[idx, 0] = col * square_mm
            objp[idx, 1] = (board_h - 1 - row) * square_mm
    return objp


def detect_chessboard(frame_bgr: np.ndarray, board_size: tuple[int, int]):
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCornersSB(gray, board_size, None)
    if not found or corners is None:
        return False, None
    return True, corners.astype(np.float32)


def corners_area(corners: np.ndarray, board_w: int, board_h: int) -> float:
    if corners is None or corners.shape[0] < board_w * board_h:
        return 0.0
    tl = corners[0, 0]
    tr = corners[board_w - 1, 0]
    bl = corners[board_w * (board_h - 1), 0]
    br = corners[-1, 0]
    quad = np.array([tl, tr, br, bl], dtype=np.float32)
    return float(abs(cv2.contourArea(quad.reshape(-1, 1, 2))))


def draw_overlay(frame_bgr: np.ndarray, corners: Optional[np.ndarray], text: str):
    out = frame_bgr.copy()
    if corners is not None:
        for i, c in enumerate(corners.reshape(-1, 2)):
            x, y = int(round(c[0])), int(round(c[1]))
            cv2.circle(out, (x, y), 3, (0, 255, 0), -1)
            if i % 10 == 0:
                cv2.putText(out, str(i), (x + 4, y - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)

    if text:
        y0 = 25
        for line in text.splitlines()[:6]:
            cv2.putText(out, line, (10, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(out, line, (10, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
            y0 += 24
    return out


def _open_camera(cam_index: int, width: int, height: int):
    cap = cv2.VideoCapture(cam_index, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(cam_index)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        try:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        except Exception:
            pass
        try:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
            cap.set(cv2.CAP_PROP_EXPOSURE, 157)
            cap.set(cv2.CAP_PROP_BRIGHTNESS, 0)
            cap.set(cv2.CAP_PROP_CONTRAST, 32)
            cap.set(cv2.CAP_PROP_SATURATION, 64)
        except Exception:
            pass
    return cap


def start_stream(cam_index, width, height, board_w, board_h):
    cam_index = int(cam_index)
    width = int(width)
    height = int(height)
    board_w = int(board_w)
    board_h = int(board_h)

    SESSION.cap = _open_camera(cam_index, width, height)
    if not SESSION.cap or not SESSION.cap.isOpened():
        SESSION.status = f"无法打开摄像头 {cam_index}"
        placeholder = np.zeros((height, width, 3), dtype=np.uint8)
        cv2.putText(placeholder, SESSION.status, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        yield cv2.cvtColor(placeholder, cv2.COLOR_BGR2RGB), SESSION.status
        return

    SESSION.running = True
    SESSION.status = "Streaming"
    while SESSION.running:
        ret, frame = SESSION.cap.read()
        if not ret or frame is None:
            time.sleep(0.03)
            continue

        found, corners = detect_chessboard(frame, (board_w, board_h))
        score = corners_area(corners, board_w, board_h) if found else 0.0

        with SESSION.lock:
            SESSION.latest_frame = frame.copy()
            SESSION.latest_corners = corners.copy() if corners is not None else None
            SESSION.latest_score = score
            SESSION.latest_found = found
            SESSION.latest_size = (frame.shape[1], frame.shape[0])

        lines = [
            f"Camera={cam_index} | frame={frame.shape[1]}x{frame.shape[0]}",
            f"board={board_w}x{board_h} inner corners | square={DEFAULT_SQUARE_MM:.1f}mm",
            f"found={found} | samples={len(SESSION.samples)}",
        ]
        if SESSION.result.ok:
            lines.append(f"rmse={SESSION.result.rmse:.3f}px")

        preview = draw_overlay(frame, corners, "\n".join(lines))
        yield cv2.cvtColor(preview, cv2.COLOR_BGR2RGB), "\n".join(lines)

    if SESSION.cap is not None:
        SESSION.cap.release()
        SESSION.cap = None


def stop_stream():
    SESSION.running = False
    if SESSION.cap is not None:
        try:
            SESSION.cap.release()
        except Exception:
            pass
        SESSION.cap = None
    return "Stopped"


def capture_sample():
    with SESSION.lock:
        frame = None if SESSION.latest_frame is None else SESSION.latest_frame.copy()
        corners = None if SESSION.latest_corners is None else SESSION.latest_corners.copy()
        size = SESSION.latest_size
        score = SESSION.latest_score
        found = SESSION.latest_found

    if frame is None or corners is None or not found:
        return "No chessboard detected — cannot capture sample yet."

    sample = Sample(frame=frame, corners=corners, image_size=size, area=score)
    with SESSION.lock:
        SESSION.samples.append(sample)

    return f"Captured sample #{len(SESSION.samples)} | area={score:.0f}"


def clear_samples():
    SESSION.reset_samples()
    SESSION.result = CalibrationResult()
    return "Cleared all samples"


def add_uploaded_image(file_obj, board_w, board_h):
    if file_obj is None:
        return "Please choose an image first"
    path = Path(file_obj.name if hasattr(file_obj, "name") else str(file_obj))
    img = cv2.imread(str(path))
    if img is None:
        return f"Failed to read {path}"

    found, corners = detect_chessboard(img, (int(board_w), int(board_h)))
    if not found or corners is None:
        return f"No chessboard found in {path.name}"

    sample = Sample(frame=img, corners=corners, image_size=(img.shape[1], img.shape[0]), area=corners_area(corners, int(board_w), int(board_h)))
    with SESSION.lock:
        SESSION.samples.append(sample)
    return f"Imported {path.name} as sample #{len(SESSION.samples)}"


def _calibrate_intrinsics(samples: list[Sample], board_w: int, board_h: int, square_mm: float):
    if len(samples) < 3:
        return None, None, None, "Need at least 3 valid samples"

    objp = board_object_points(board_w, board_h, square_mm)
    objpoints = []
    imgpoints = []
    image_size = samples[0].image_size

    for s in samples:
        if s.corners is None:
            continue
        objpoints.append(objp)
        imgpoints.append(s.corners)

    if len(objpoints) < 3:
        return None, None, None, "Need at least 3 valid detected samples"

    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, image_size, None, None)

    # Reprojection RMSE
    sq_err = 0.0
    n_pts = 0
    for obj, img, rv, tv in zip(objpoints, imgpoints, rvecs, tvecs):
        proj, _ = cv2.projectPoints(obj, rv, tv, K, dist)
        err = proj.reshape(-1, 2) - img.reshape(-1, 2)
        sq_err += float(np.sum(err * err))
        n_pts += int(obj.shape[0])
    rmse = math.sqrt(sq_err / max(1, n_pts))
    return K, dist, rmse, "OK"


def _solve_pose_from_sample(sample: Sample, K: np.ndarray, dist: np.ndarray, board_w: int, board_h: int, square_mm: float):
    objp = board_object_points(board_w, board_h, square_mm)
    ok, rvec, tvec = cv2.solvePnP(objp, sample.corners, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None, None, None
    R, _ = cv2.Rodrigues(rvec)
    H = K @ np.column_stack((R[:, 0], R[:, 1], tvec.reshape(3)))
    return rvec, tvec, H


def calibrate_now(board_w, board_h, square_mm):
    board_w = int(board_w)
    board_h = int(board_h)
    square_mm = float(square_mm)

    with SESSION.lock:
        samples = list(SESSION.samples)

    K, dist, rmse, msg = _calibrate_intrinsics(samples, board_w, board_h, square_mm)
    if K is None:
        return msg

    # Choose the best-quality sample to estimate the chessboard pose.
    best = max(samples, key=lambda s: s.area)
    rvec, tvec, H = _solve_pose_from_sample(best, K, dist, board_w, board_h, square_mm)
    if rvec is None or tvec is None or H is None:
        return "Intrinsic calibration succeeded, but pose estimation failed on the selected sample."

    SESSION.result = CalibrationResult(
        ok=True,
        K=K,
        dist=dist,
        rvec=rvec,
        tvec=tvec,
        H=H,
        rmse=float(rmse),
        board_size=(board_w, board_h),
        square_size_mm=square_mm,
        origin="bottom_left_inner_corner",
        image_size=best.image_size,
    )

    return f"Calibration OK | rmse={rmse:.3f}px | samples={len(samples)}"


def _legacy_compat_fields(result: CalibrationResult):
    # Keep legacy keys present so older readers can at least load the file.
    # They will not be as accurate as the new path, but the file remains usable.
    if result.H is None:
        legacy_matrix = np.eye(3, dtype=np.float64)
    else:
        # Map board-mm directly (with Y flipped) into the legacy matrix slot.
        flip = np.diag([1.0, -1.0, 1.0])
        legacy_matrix = flip @ np.linalg.inv(result.H)
    legacy_p2c = np.array([0.1, 0.0, 0.0], dtype=np.float64)
    return legacy_matrix, legacy_p2c


def save_calibration(cam_index, board_w, board_h, square_mm):
    cam_index = int(cam_index)
    board_w = int(board_w)
    board_h = int(board_h)
    square_mm = float(square_mm)

    if not SESSION.result.ok:
        return "Please calibrate first"

    result = SESSION.result
    legacy_matrix, legacy_p2c = _legacy_compat_fields(result)

    out_path = CALIB_DIR / f"CalibrationData{cam_index}.npz"
    np.savez(
        out_path,
        version=np.array([2], dtype=np.int32),
        format="pixel2cm_v2",
        camera_index=np.array([cam_index], dtype=np.int32),
        board_size=np.array([board_w, board_h], dtype=np.int32),
        square_size_mm=np.array([square_mm], dtype=np.float32),
        origin=np.array([result.origin]),
        image_size=np.array(result.image_size, dtype=np.int32),
        K=result.K,
        dist=result.dist,
        rvec=result.rvec,
        tvec=result.tvec,
        H=result.H,
        rmse=np.array([result.rmse], dtype=np.float32),
        samples_used=np.array([len(SESSION.samples)], dtype=np.int32),
        # legacy compatibility keys
        matrix=legacy_matrix,
        p2c=legacy_p2c,
    )

    return f"Saved calibration to {out_path}"


def status_text():
    with SESSION.lock:
        samples = len(SESSION.samples)
        found = SESSION.latest_found
        score = SESSION.latest_score
        result = SESSION.result
    lines = [
        f"streaming={SESSION.running}",
        f"samples={samples}",
        f"found={found}",
        f"latest_area={score:.0f}",
    ]
    if result.ok:
        lines.append(f"calibrated rmse={result.rmse:.3f}px")
    return "\n".join(lines)


with gr.Blocks(title="Pixel2CM Web Calibrator") as demo:
    gr.Markdown(
        """
        # Pixel2CM Web Calibrator
        这个页面替代了旧的单帧经验式标定。建议使用 **9×9 内角点、15mm 格子** 的棋盘格。

        核心变化：
        - 先做 **相机内参 + 畸变** 标定
        - 再用棋盘平面求 **棋盘姿态 / 单应矩阵**
        - 输出的新 `.npz` 会被 `pixel2cm.py` 直接读取
        """
    )

    with gr.Row():
        with gr.Column(scale=3):
            preview = gr.Image(label="Camera Preview", type="numpy")
            info_box = gr.Textbox(label="Status", lines=10, interactive=False)
        with gr.Column(scale=2):
            cam_index = gr.Number(value=DEFAULT_CAM_INDEX, label="Camera Index", precision=0)
            width = gr.Number(value=DEFAULT_WIDTH, label="Frame Width", precision=0)
            height = gr.Number(value=DEFAULT_HEIGHT, label="Frame Height", precision=0)
            board_w = gr.Number(value=DEFAULT_BOARD_W, label="Chessboard Width (inner corners)", precision=0)
            board_h = gr.Number(value=DEFAULT_BOARD_H, label="Chessboard Height (inner corners)", precision=0)
            square_mm = gr.Number(value=DEFAULT_SQUARE_MM, label="Square Size (mm)")

            with gr.Row():
                start_btn = gr.Button("Start Stream", variant="primary")
                stop_btn = gr.Button("Stop")

            with gr.Row():
                capture_btn = gr.Button("Capture Sample")
                clear_btn = gr.Button("Clear Samples")

            calibrate_btn = gr.Button("Calibrate Intrinsics + Pose", variant="primary")
            save_btn = gr.Button("Save Calibration")
            refresh_btn = gr.Button("Refresh Status")

            upload = gr.File(label="Or upload chessboard image", file_count="single", file_types=["image"])
            upload_btn = gr.Button("Import Uploaded Image")

            result_box = gr.Textbox(label="Result", lines=6, interactive=False)

    def _stream_wrapper(cam_index, width, height, board_w, board_h):
        placeholder = np.zeros((int(height), int(width), 3), dtype=np.uint8)
        cv2.putText(placeholder, "Starting...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        yield placeholder, "Starting..."
        yield from start_stream(cam_index, width, height, board_w, board_h)

    start_evt = start_btn.click(
        _stream_wrapper,
        inputs=[cam_index, width, height, board_w, board_h],
        outputs=[preview, info_box],
    )
    stop_btn.click(fn=stop_stream, outputs=result_box, cancels=[start_evt])

    capture_btn.click(fn=capture_sample, outputs=result_box)
    clear_btn.click(fn=clear_samples, outputs=result_box)
    calibrate_btn.click(fn=calibrate_now, inputs=[board_w, board_h, square_mm], outputs=result_box)
    save_btn.click(fn=save_calibration, inputs=[cam_index, board_w, board_h, square_mm], outputs=result_box)
    refresh_btn.click(fn=status_text, outputs=info_box)
    upload_btn.click(fn=add_uploaded_image, inputs=[upload, board_w, board_h], outputs=result_box)


if __name__ == "__main__":
    demo.launch(server_name="0.0.0.0", server_port=7863, share=False)