"""Live calibration preview tool.

This utility applies saved camera calibration files to a real-time camera feed
and renders a bird's-eye preview of the calibrated board plane.

Supported calibration formats:
- Modern format from Tools/Pixel2CM/CameraCalibrate_Nexus.py (version >= 2)
  keys: version, K, dist, H, rvec, tvec, board_size, square_size_mm
- Legacy format from the old single-frame calibrator
  keys: matrix, p2c
"""

from __future__ import annotations

import time
from collections import OrderedDict
from pathlib import Path
from typing import Any

import cv2
import gradio as gr
import numpy as np

APP_DIR = Path(__file__).resolve().parent
CALIB_DIR = APP_DIR / "Pixel2CM" / "Calibration"

DEFAULT_CAM_INDEX = 0
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_FPS = 30
DEFAULT_PIXELS_PER_MM = 4.0
DEFAULT_GRID_MM = 10.0

RUNNING = False
CAPS: list[cv2.VideoCapture] = []


def _as_float_array(value: Any, dtype=np.float64) -> np.ndarray:
    return np.asarray(value, dtype=dtype)


def list_calibration_files() -> list[str]:
    files = sorted(CALIB_DIR.glob("CalibrationData*.npz"))
    if not files:
        return [str(CALIB_DIR / "CalibrationData0.npz")]
    return [str(p) for p in files]


def open_camera(cam_index: int, width: int, height: int, use_mjpg: bool) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(int(cam_index), cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(int(cam_index))

    if not cap.isOpened():
        return cap

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))

    if use_mjpg:
        try:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        except Exception:
            pass

    try:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
    except Exception:
        pass

    return cap


def load_calibration(calib_path: str) -> dict[str, Any]:
    path = Path(calib_path)
    if not path.exists():
        return {
            "mode": "missing",
            "path": str(path),
        }

    data = np.load(path, allow_pickle=True)
    keys = set(data.files)

    if {"K", "dist", "H"}.issubset(keys) or ("version" in keys and int(np.atleast_1d(data["version"])[0]) >= 2):
        return {
            "mode": "modern",
            "path": str(path),
            "version": int(np.atleast_1d(data["version"])[0]) if "version" in keys else 2,
            "K": _as_float_array(data["K"]),
            "dist": _as_float_array(data["dist"]),
            "H": _as_float_array(data["H"]),
            "rvec": _as_float_array(data["rvec"]) if "rvec" in keys else None,
            "tvec": _as_float_array(data["tvec"]) if "tvec" in keys else None,
            "board_size": tuple(np.asarray(data["board_size"]).astype(int).tolist()) if "board_size" in keys else (9, 9),
            "square_size_mm": float(np.atleast_1d(data["square_size_mm"])[0]) if "square_size_mm" in keys else 15.0,
            "origin": str(np.atleast_1d(data["origin"])[0]) if "origin" in keys else "bottom_left_inner_corner",
            "image_size": tuple(np.asarray(data["image_size"]).astype(int).tolist()) if "image_size" in keys else None,
        }

    if {"matrix", "p2c"}.issubset(keys):
        return {
            "mode": "legacy",
            "path": str(path),
            "matrix": _as_float_array(data["matrix"]),
            "p2c": _as_float_array(data["p2c"]),
        }

    return {
        "mode": "unknown",
        "path": str(path),
        "matrix": np.eye(3, dtype=np.float64),
        "p2c": np.array([1.0, 0.0, 0.0], dtype=np.float64),
    }


def _draw_text_box(img: np.ndarray, lines: list[str], origin=(10, 24), color=(255, 255, 255)):
    y = origin[1]
    for line in lines:
        cv2.putText(img, line, (origin[0], y), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, line, (origin[0], y), cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 1, cv2.LINE_AA)
        y += 22


def _draw_grid(img: np.ndarray, step_px: int, color=(0, 180, 255)):
    if step_px <= 0:
        return img
    out = img.copy()
    h, w = out.shape[:2]
    for x in range(0, w, step_px):
        cv2.line(out, (x, 0), (x, h - 1), color, 1, cv2.LINE_AA)
    for y in range(0, h, step_px):
        cv2.line(out, (0, y), (w - 1, y), color, 1, cv2.LINE_AA)
    cv2.line(out, (0, 0), (w - 1, 0), (0, 255, 0), 2)
    cv2.line(out, (0, 0), (0, h - 1), (255, 0, 0), 2)
    return out


def _letterbox_resize(img: np.ndarray, target_w: int, target_h: int, pad_color=(20, 20, 20)) -> np.ndarray:
    target_w = max(1, int(target_w))
    target_h = max(1, int(target_h))
    h, w = img.shape[:2]
    scale = min(target_w / max(1, w), target_h / max(1, h))
    new_w = max(1, int(round(w * scale)))
    new_h = max(1, int(round(h * scale)))
    resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR)
    canvas = np.full((target_h, target_w, 3), pad_color, dtype=np.uint8)
    x0 = (target_w - new_w) // 2
    y0 = (target_h - new_h) // 2
    canvas[y0:y0 + new_h, x0:x0 + new_w] = resized
    return canvas


def _stack_four_views(views: list[np.ndarray], labels: list[str], cell_w: int = 640, cell_h: int = 360) -> np.ndarray:
    cells = []
    for i in range(4):
        if i < len(views) and views[i] is not None:
            cell = _letterbox_resize(views[i], cell_w, cell_h)
            _draw_text_box(cell, [labels[i] if i < len(labels) else f"cam{i}"], origin=(10, 24))
        else:
            cell = np.zeros((cell_h, cell_w, 3), dtype=np.uint8)
            _draw_text_box(cell, [f"cam{i}: unavailable"], origin=(10, 24), color=(0, 0, 255))
        cells.append(cell)
    top = np.hstack([cells[0], cells[1]])
    bottom = np.hstack([cells[2], cells[3]])
    return np.vstack([top, bottom])



def _modern_bird_eye(frame_bgr: np.ndarray, calib: dict[str, Any], pixels_per_mm: float, grid_mm: float):
    K = calib.get("K")
    dist = calib.get("dist")
    H = calib.get("H")
    board_size = calib.get("board_size", (9, 9))
    square_mm = float(calib.get("square_size_mm", 15.0))

    board_w_mm = max(1.0, (int(board_size[0]) - 1) * square_mm)
    board_h_mm = max(1.0, (int(board_size[1]) - 1) * square_mm)
    ppm = max(0.1, float(pixels_per_mm))
    # Shrink the BEV resolution to ensure boundaries are visible in the UI
    zoom_factor = 0.8
    effective_ppm = ppm * zoom_factor
    out_w = max(1, int(round(board_w_mm * effective_ppm)))
    out_h = max(1, int(round(board_h_mm * effective_ppm)))

    src = frame_bgr
    if K is not None and dist is not None:
        try:
            src = cv2.undistort(frame_bgr, K, dist)
        except Exception:
            src = frame_bgr

    try:
        H_inv = np.linalg.inv(H)
    except Exception:
        H_inv = np.eye(3, dtype=np.float64)

    # World plane mm -> display px:
    # x_display = x_mm * effective_ppm
    # y_display = (board_h_mm - y_mm) * effective_ppm
    scale = np.array([
        [effective_ppm, 0.0, 0.0],
        [0.0, -effective_ppm, board_h_mm * effective_ppm],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    M = scale @ H_inv
    bird = cv2.warpPerspective(
        src,
        M,
        (out_w, out_h),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(0, 0, 0),
    )

    grid_px = max(1, int(round(grid_mm * ppm)))
    bird = _draw_grid(bird, grid_px)
    _draw_text_box(
        bird,
        [
            f"mode=modern | board={board_size[0]}x{board_size[1]} | square={square_mm:.1f}mm",
            f"bird-eye size={out_w}x{out_h} px | scale={ppm:.2f} px/mm",
            f"grid={grid_mm:.1f}mm",
        ],
        origin=(10, 24),
    )
    return bird


def _legacy_bird_eye(frame_bgr: np.ndarray, calib: dict[str, Any]):
    matrix = calib.get("matrix", np.eye(3, dtype=np.float64))
    try:
        bird = cv2.warpPerspective(frame_bgr, matrix, (frame_bgr.shape[1], frame_bgr.shape[0]))
    except Exception:
        bird = frame_bgr.copy()
    _draw_text_box(
        bird,
        [
            "mode=legacy | approximate bird-eye",
            "Tip: use the modern Pixel2CM web calibrator for accurate preview.",
        ],
        origin=(10, 24),
    )
    return bird


def make_bird_eye(frame_bgr: np.ndarray, calib: dict[str, Any], pixels_per_mm: float, grid_mm: float):
    mode = calib.get("mode")
    if mode == "modern":
        return _modern_bird_eye(frame_bgr, calib, pixels_per_mm, grid_mm)
    if mode == "legacy":
        return _legacy_bird_eye(frame_bgr, calib)

    out = frame_bgr.copy()
    _draw_text_box(out, [f"Calibration missing: {calib.get('path', '')}"], origin=(10, 24), color=(0, 0, 255))
    return out


def stream_preview(cam_index, calib_file, width, height, use_mjpg, target_fps, pixels_per_mm, grid_mm, display_mode, mosaic_cams):
    global RUNNING, CAPS

    cam_index = int(cam_index)
    width = int(width)
    height = int(height)
    target_fps = max(1, int(target_fps))
    pixels_per_mm = float(pixels_per_mm)
    grid_mm = float(grid_mm)
    display_mode = str(display_mode)
    mosaic_indices = []
    if mosaic_cams:
        for part in str(mosaic_cams).replace('，', ',').split(','):
            part = part.strip()
            if part:
                try:
                    mosaic_indices.append(int(part))
                except Exception:
                    pass
    if not mosaic_indices:
        mosaic_indices = [0, 2, 4, 6]

    if display_mode == 'single':
        calib = load_calibration(str(calib_file))
        CAPS = [open_camera(cam_index, width, height, bool(use_mjpg))]
        if not CAPS[0].isOpened():
            placeholder = np.zeros((height, width, 3), dtype=np.uint8)
            cv2.putText(placeholder, f"Cannot open camera {cam_index}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            yield cv2.cvtColor(placeholder, cv2.COLOR_BGR2RGB), cv2.cvtColor(placeholder, cv2.COLOR_BGR2RGB), f"Failed to open camera {cam_index}"
            return
        RUNNING = True
        frame_interval = 1.0 / float(target_fps)
        while RUNNING:
            t0 = time.perf_counter()
            ret, frame = CAPS[0].read()
            if not ret or frame is None:
                time.sleep(0.03)
                continue
            raw = frame.copy()
            bird = make_bird_eye(frame, calib, pixels_per_mm, grid_mm)
            info = [
                f"mode=single | camera={cam_index} | frame={frame.shape[1]}x{frame.shape[0]} | fps_target={target_fps}",
                f"calib={calib.get('path', '')}",
                f"calib_mode={calib.get('mode', 'unknown')}",
            ]
            if calib.get('mode') == 'modern':
                board_size = calib.get('board_size', (9, 9))
                square_mm = float(calib.get('square_size_mm', 15.0))
                info.append(f"board={board_size[0]}x{board_size[1]} inner corners | square={square_mm:.1f}mm")
            raw = _draw_grid(raw, 0)
            _draw_text_box(raw, info, origin=(10, 24))
            yield cv2.cvtColor(raw, cv2.COLOR_BGR2RGB), cv2.cvtColor(bird, cv2.COLOR_BGR2RGB), '\n'.join(info)
            elapsed = time.perf_counter() - t0
            remain = frame_interval - elapsed
            if remain > 0:
                time.sleep(remain)
    else:
        # mosaic mode: open up to 4 cameras and compose a 2x2 bird-eye mosaic
        selected = mosaic_indices[:4]
        while len(selected) < 4:
            selected.append(selected[-1] if selected else cam_index)

        calibs = [load_calibration(str(calib_file)) if i == 0 else load_calibration(str(CALIB_DIR / f"CalibrationData{cam}.npz")) for i, cam in enumerate(selected)]
        CAPS = [open_camera(cam, width, height, bool(use_mjpg)) for cam in selected]
        if not all(cap.isOpened() for cap in CAPS):
            bad = [str(selected[i]) for i, cap in enumerate(CAPS) if not cap.isOpened()]
            placeholder = np.zeros((height, width, 3), dtype=np.uint8)
            cv2.putText(placeholder, f"Cannot open cameras: {','.join(bad)}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            rgb = cv2.cvtColor(placeholder, cv2.COLOR_BGR2RGB)
            yield rgb, rgb, f"Failed to open: {','.join(bad)}"
            return
        RUNNING = True
        frame_interval = 1.0 / float(target_fps)
        while RUNNING:
            t0 = time.perf_counter()
            raw_views = []
            bird_views = []
            labels = []
            for idx, (cam, cap, calib) in enumerate(zip(selected, CAPS, calibs)):
                ret, frame = cap.read()
                if not ret or frame is None:
                    frame = np.zeros((height, width, 3), dtype=np.uint8)
                    cv2.putText(frame, f"cam {cam}: no frame", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                raw_views.append(frame.copy())
                bird_views.append(make_bird_eye(frame, calib, pixels_per_mm, grid_mm))
                labels.append(f"cam={cam} | {calib.get('mode', 'unknown')}")
            raw = _stack_four_views(raw_views, labels, cell_w=width, cell_h=height)
            bird = _stack_four_views(bird_views, labels, cell_w=640, cell_h=360)
            summary = [f"mode=mosaic | cams={selected}", f"calib_files={[calib.get('path', '') for calib in calibs]}", f"bird-eye mosaic 2x2"]
            _draw_text_box(raw, summary, origin=(10, 24))
            _draw_text_box(bird, summary, origin=(10, 24))
            yield cv2.cvtColor(raw, cv2.COLOR_BGR2RGB), cv2.cvtColor(bird, cv2.COLOR_BGR2RGB), '\n'.join(summary)
            elapsed = time.perf_counter() - t0
            remain = frame_interval - elapsed
            if remain > 0:
                time.sleep(remain)

    for cap in CAPS:
        try:
            cap.release()
        except Exception:
            pass
    CAPS = []


def stop_preview():
    global RUNNING, CAPS
    RUNNING = False
    for cap in CAPS:
        try:
            cap.release()
        except Exception:
            pass
    CAPS = []
    return "Stopped"


def refresh_calibration_choices():
    choices = list_calibration_files()
    return gr.Dropdown(choices=choices, value=choices[0] if choices else None)


calib_choices = list_calibration_files()

with gr.Blocks(title="Bird-Eye Calibration Preview") as demo:
    gr.Markdown(
        """
        # Real-time Calibration Preview / 鸟瞰图预览
        选择摄像头和标定文件后，可以同时看到：
        - 左侧：原始实时画面
        - 右侧：应用标定后的鸟瞰图（bird-eye / BEV）
        """
    )

    with gr.Row():
        with gr.Column(scale=3):
            raw_view = gr.Image(label="Raw Camera View", type="numpy")
            bird_view = gr.Image(label="Bird-Eye View", type="numpy")
        with gr.Column(scale=2):
            display_mode = gr.Radio(choices=["single", "mosaic"], value="mosaic", label="Display Mode")
            cam_index = gr.Number(value=DEFAULT_CAM_INDEX, label="Single Camera Index", precision=0)
            mosaic_cams = gr.Textbox(value="0,2,4,6", label="Mosaic Camera Indices (comma separated)")
            calib_file = gr.Dropdown(label="Calibration File (single mode)", choices=calib_choices, value=calib_choices[0] if calib_choices else None)
            width = gr.Number(value=DEFAULT_WIDTH, label="Frame Width", precision=0)
            height = gr.Number(value=DEFAULT_HEIGHT, label="Frame Height", precision=0)
            target_fps = gr.Slider(1, 60, value=DEFAULT_FPS, step=1, label="Target FPS")
            pixels_per_mm = gr.Slider(0.5, 10.0, value=DEFAULT_PIXELS_PER_MM, step=0.5, label="Bird-eye scale (px/mm)")
            grid_mm = gr.Slider(5.0, 50.0, value=DEFAULT_GRID_MM, step=5.0, label="Grid spacing (mm)")
            use_mjpg = gr.Checkbox(label="Use MJPG", value=False)

            with gr.Row():
                start_btn = gr.Button("Start Preview", variant="primary")
                stop_btn = gr.Button("Stop")

            refresh_btn = gr.Button("Refresh Calibration List")
            status_box = gr.Textbox(label="Status", lines=8, interactive=False)

    def _start(cam_index, calib_file, width, height, use_mjpg, target_fps, pixels_per_mm, grid_mm, display_mode, mosaic_cams):
        placeholder = np.zeros((int(height), int(width), 3), dtype=np.uint8)
        cv2.putText(placeholder, "Starting...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        yield placeholder, placeholder, "Starting..."
        yield from stream_preview(cam_index, calib_file, width, height, use_mjpg, target_fps, pixels_per_mm, grid_mm, display_mode, mosaic_cams)

    start_evt = start_btn.click(
        _start,
        inputs=[cam_index, calib_file, width, height, use_mjpg, target_fps, pixels_per_mm, grid_mm, display_mode, mosaic_cams],
        outputs=[raw_view, bird_view, status_box],
    )
    stop_btn.click(fn=stop_preview, outputs=status_box, cancels=[start_evt])
    refresh_btn.click(fn=refresh_calibration_choices, outputs=calib_file)

if __name__ == "__main__":
    demo.launch(server_name="0.0.0.0", server_port=7864, share=False)
