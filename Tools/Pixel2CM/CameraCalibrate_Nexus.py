"""
Gradio Camera Calibrator with Full Calibration & Real-time Mapping

功能：
- 实时摄像头预览（支持 V4L2）
- 采集棋盘图像用于标定
- 执行完整相机标定（内参 + 畸变）
- 实时去畸变 + 计算 Homography（像素 → 世界坐标 cm）
- 显示每个角点在机器人坐标系下的 (X, Y) 位置
- 保存/加载标定结果到 ./Calibration/

坐标系定义：
- 棋盘左下角 = (0, 0) mm
- X 轴向右，Y 轴向前
- 机器人位于 (0, -Intercept_cm)，即棋盘前方 Intercept_cm 处
- 因此：机器人坐标系中，点 P 的位置为：
    robot_x = world_x_mm / 10.0          # cm
    robot_y = Intercept_cm + world_y_mm / 10.0  # cm

依赖：opencv-python, numpy, gradio
"""

import cv2
import numpy as np
import math
import time
from pathlib import Path
import gradio as gr
import threading
import os
import time

# ================== 配置 ==================
DEFAULT_CAM_INDEX = 0
DEFAULT_INTERCEPT_CM = 15.0  # 机器人到棋盘第一个内角点的距离（cm）
CHESSBOARD_SIZE = (9, 9)     # 内部角点数 (10x10 格子)
SQUARE_SIZE_MM = 15.0        # 每格 15mm

# ================== 全局状态 ==================
running = False
cap = None
calibration_data = None  # {'K': ..., 'dist': ..., 'H': ..., 'intercept': ...}
capture_buffer = []      # 存储用于标定的图像
Date = time.strftime("%Y%m", time.localtime())
APP_DIR = Path(__file__).parent
CALIB_DIR = APP_DIR / f"Calibration{Date}"
CALIB_DIR.mkdir(exist_ok=True)

# ================== 工具函数 ==================

def pixel_to_robot_coord(x, y, H, intercept_cm):
    """将像素点 (x,y) 映射到机器人坐标系 (X_cm, Y_cm)"""
    if H is None:
        return None, None
    pt = np.array([[[x, y]]], dtype=np.float32)
    world_mm = cv2.perspectiveTransform(pt, H)
    wx, wy = world_mm[0][0]
    robot_x = wx / 10.0  # mm -> cm
    robot_y = intercept_cm + wy / 10.0
    return robot_x, robot_y

def draw_robot_coords(img, corners, H, intercept_cm):
    """在图像上绘制机器人坐标"""
    out = img.copy()
    if corners is None or H is None:
        return out
    for i, corner in enumerate(corners):
        x, y = int(corner[0][0]), int(corner[0][1])
        rx, ry = pixel_to_robot_coord(x, y, H, intercept_cm)
        if rx is not None:
            label = f"({rx:.1f}, {ry:.1f})"
            cv2.putText(out, label, (x + 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
    return out

def format_robot_coords(corners, H, intercept_cm, max_points=50):
    lines = []
    if corners is None or H is None:
        return "No valid homography"
    for i, corner in enumerate(corners):
        x, y = corner[0]
        rx, ry = pixel_to_robot_coord(x, y, H, intercept_cm)
        if rx is not None:
            lines.append(f"#{i:02d}: px=({x:.0f},{y:.0f}) → robot=({rx:.2f}cm, {ry:.2f}cm)")
        if i >= max_points:
            lines.append("... truncated ...")
            break
    return "\n".join(lines)

# ================== 标定逻辑 ==================

def perform_calibration(images, board_w, board_h, square_size_mm):
    objp = np.zeros((board_w * board_h, 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_w, 0:board_h].T.reshape(-1, 2)
    objp *= square_size_mm

    objpoints = []
    imgpoints = []

    for img in images:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCornersSB(gray, (board_w, board_h), None)
        if ret:
            objpoints.append(objp)
            imgpoints.append(corners)

    if len(objpoints) < 3:
        return None, "Need at least 3 valid images"

    try:
        ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None
        )
        return {"K": K, "dist": dist}, "Success"
    except Exception as e:
        return None, f"Calibration failed: {str(e)}"

def compute_homography_from_frame(frame, K, dist, board_w, board_h, square_size_mm):
    h, w = frame.shape[:2]
    new_K, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 1, (w, h))
    undistorted = cv2.undistort(frame, K, dist, None, new_K)

    gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCornersSB(gray, (board_w, board_h), None)
    if not ret or corners is None:
        return undistorted, None

    # 构建真实世界坐标 (mm)
    obj_2d = np.zeros((board_w * board_h, 2), np.float32)
    for i in range(board_h):
        for j in range(board_w):
            idx = i * board_w + j
            obj_2d[idx] = [j * square_size_mm, i * square_size_mm]

    img_points = corners.reshape(-1, 2)
    H, _ = cv2.findHomography(img_points, obj_2d)
    return undistorted, H

# ================== 视频流 ==================

def video_generator(cam_index, intercept_cm):
    global running, cap, calibration_data
    cap = cv2.VideoCapture(int(cam_index), cv2.CAP_V4L2)
    try:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        cap.set(cv2.CAP_PROP_BRIGHTNESS, -64)
        cap.set(cv2.CAP_PROP_CONTRAST, 32)
        cap.set(cv2.CAP_PROP_SATURATION, 64)
        cap.set(cv2.CAP_PROP_SHARPNESS, 2)
    except:
        pass

    running = True
    while running:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        display_img = frame.copy()
        info_text = "No calibration loaded"

        if calibration_data is not None:
            K = calibration_data['K']
            dist = calibration_data['dist']
            intercept = calibration_data['intercept']

            h, w = frame.shape[:2]
            new_K, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 1, (w, h))
            undistorted = cv2.undistort(frame, K, dist, None, new_K)

            gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCornersSB(gray, CHESSBOARD_SIZE, None)

            if ret and corners is not None:
                # 计算当前 Homography
                obj_2d = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 2), np.float32)
                for i in range(CHESSBOARD_SIZE[1]):
                    for j in range(CHESSBOARD_SIZE[0]):
                        idx = i * CHESSBOARD_SIZE[0] + j
                        obj_2d[idx] = [j * SQUARE_SIZE_MM, i * SQUARE_SIZE_MM]
                img_points = corners.reshape(-1, 2)
                H, _ = cv2.findHomography(img_points, obj_2d)

                # 绘制坐标
                display_img = draw_robot_coords(undistorted, corners, H, intercept)
                info_text = format_robot_coords(corners, H, intercept)
            else:
                display_img = undistorted
                cv2.putText(display_img, "No chessboard", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        else:
            cv2.putText(display_img, "Load calibration first", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        rgb = cv2.cvtColor(display_img, cv2.COLOR_BGR2RGB)
        yield rgb, info_text

    cap.release()

# ================== Gradio 回调 ==================

def start_stream(cam_index, intercept_cm):
    placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(placeholder, "Starting...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    yield placeholder, "Starting"
    yield from video_generator(cam_index, intercept_cm)

def stop_stream():
    global running
    running = False
    return "Stopped"

def capture_frame():
    global cap, capture_buffer
    if cap is None or not cap.isOpened():
        return "Start stream first"
    ret, frame = cap.read()
    if ret:
        capture_buffer.append(frame.copy())
        return f"Captured! Total: {len(capture_buffer)}"
    return "Failed to capture"

def calibrate_now():
    global capture_buffer, calibration_data
    if len(capture_buffer) < 3:
        return f"Need ≥3 images, got {len(capture_buffer)}"
    
    params, msg = perform_calibration(
        capture_buffer,
        CHESSBOARD_SIZE[0],
        CHESSBOARD_SIZE[1],
        SQUARE_SIZE_MM
    )
    if params is None:
        return msg
    
    # 保存到全局
    calibration_data = {
        'K': params['K'],
        'dist': params['dist'],
        'intercept': DEFAULT_INTERCEPT_CM  # 可后续调整
    }
    
    # 保存到文件
    np.savez(CALIB_DIR / "camera_calibration.npz", 
             K=params['K'], 
             dist=params['dist'],
             intercept=DEFAULT_INTERCEPT_CM)
    return f"✅ Calibration success! Saved to {CALIB_DIR}/camera_calibration.npz"

def apply_perspective_transform_to_point(x, y, matrix):
    pt = np.array([x, y, 1.0], dtype=np.float64)
    res = matrix @ pt
    if res[2] == 0:
        return None
    res = res / res[2]
    return float(res[0]), float(res[1])

def compute_calibration_from_corners(corners, board_w, board_h, intercept):
    """
    给定找到的角点（N x 1 x 2），计算透视变换矩阵和像素->厘米系数
    返回 (matrix, p2c) 或 (None, None) 当失败
    p2c = [fPixelToCM, fHorizontalB, fVerticalB]
    """
    # corners 的排列：从 (0,0) 到 (w-1,h-1) 行优先
    n = corners.shape[0]
    if n < 4:
        return None, None

    # 取四个角：top-left, top-right, bottom-left, bottom-right
    tl = corners[0][0]
    tr = corners[board_w - 1][0]
    bl = corners[board_w * (board_h - 1)][0]
    br = corners[-1][0]

    # 根据原脚本的计算，使用左侧两个点与右侧两个点来估计像素间距
    # 选择左一和右一作为基线
    # 保证顺序：左-右
    lCorners = [tuple(bl), tuple(tl), tuple(br), tuple(tr)]
    # 以 x 排序（左到右），以恢复原算法的形式
    lCorners.sort(key=lambda item: item[0])

    # bottom y 平均
    fBottomY = (lCorners[0][1] + lCorners[3][1]) / 2.0

    fDistance = math.hypot(lCorners[0][0] - lCorners[3][0], lCorners[0][1] - lCorners[3][1])
    if fDistance == 0:
        return None, None
    # 原脚本假定两个格子为12cm
    fPixelToCM = 12.0 / fDistance

    fHorizontalSlope = (lCorners[0][1] - lCorners[3][1]) / (lCorners[0][0] - lCorners[3][0])
    # 防止除0
    if fHorizontalSlope == 0:
        fVerticalSlope = 0
    else:
        fVerticalSlope = -1.0 / fHorizontalSlope

    if fVerticalSlope > 0:
        iTheta = math.atan(fVerticalSlope)
    else:
        iTheta = math.atan(fVerticalSlope) + math.pi

    fDeltaX = math.cos(iTheta) * fDistance
    fDeltaY = math.sin(iTheta) * fDistance

    fCorrectedLFX = lCorners[0][0] + fDeltaX
    fCorrectedLFY = lCorners[0][1] - fDeltaY
    fCorrectedRFX = lCorners[3][0] + fDeltaX
    fCorrectedRFY = lCorners[3][1] - fDeltaY

    lSRCPoints = np.float32(lCorners)
    lDSTPoints = np.float32([lCorners[0], (fCorrectedLFX, fCorrectedLFY), (fCorrectedRFX, fCorrectedRFY), lCorners[3]])

    try:
        aPerspectiveMatrix = cv2.getPerspectiveTransform(lSRCPoints, lDSTPoints)
    except Exception as e:
        return None, None

    # 应用与原脚本相同的像素->厘米关系
    iX, iY = apply_perspective_transform_to_point(320, fBottomY, aPerspectiveMatrix)
    fHorizontalB = -fPixelToCM * iX
    fVerticalB = intercept + fPixelToCM * iY

    aPixelToCM = np.array([fPixelToCM, fHorizontalB, fVerticalB], dtype=np.float64)

    return aPerspectiveMatrix, aPixelToCM


def save_compatible_calibration(cam_index, intercept_cm):
    """
    保存与你原始程序完全兼容的 CalibrationData{cam}.npz
    内部使用去畸变图像计算，但输出格式不变
    """
    global capture_buffer, calibration_data
    if len(capture_buffer) == 0:
        return "No frames captured"

    # 第一步：先做完整标定（用于去畸变）
    params, msg = perform_calibration(
        capture_buffer,
        CHESSBOARD_SIZE[0],
        CHESSBOARD_SIZE[1],
        SQUARE_SIZE_MM
    )
    if params is None:
        return msg

    K = params['K']
    dist = params['dist']

    # 第二步：用最后一张图（或最佳图）计算兼容的 matrix 和 p2c
    frame = capture_buffer[-1]  # 或选一张清晰的
    h, w = frame.shape[:2]
    new_K, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 1, (w, h))
    undistorted = cv2.undistort(frame, K, dist, None, new_K)

    gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCornersSB(gray, CHESSBOARD_SIZE, None)
    if not ret or corners is None:
        return "No chessboard in last frame"

    # ⭐ 关键：使用去畸变后的角点，调用你原来的函数！
    matrix, p2c = compute_calibration_from_corners(
        corners,
        CHESSBOARD_SIZE[0],
        CHESSBOARD_SIZE[1],
        float(intercept_cm)
    )

    if matrix is None or p2c is None:
        return "Failed to compute compatible calibration"

    # 保存为原始格式
    file_path = CALIB_DIR / f"CalibrationData{int(cam_index)}.npz"
    np.savez(file_path, matrix=matrix, p2c=p2c)

    # 同时保存完整参数（供未来升级用）
    np.savez(CALIB_DIR / "camera_calibration_full.npz", 
             K=K, dist=dist, intercept=intercept_cm)

    return f"✅ Compatible calibration saved to {file_path} (original format preserved)"

def load_calibration():
    global calibration_data
    path = CALIB_DIR / "camera_calibration.npz"
    if not path.exists():
        return "No saved calibration"
    data = np.load(path)
    calibration_data = {
        'K': data['K'],
        'dist': data['dist'],
        'intercept': float(data['intercept'])
    }
    return f"✅ Loaded calibration from {path}"

def save_intercept(intercept_cm):
    global calibration_data
    if calibration_data is None:
        return "Load calibration first"
    calibration_data['intercept'] = float(intercept_cm)
    # 更新文件
    path = CALIB_DIR / "camera_calibration.npz"
    data = np.load(path)
    np.savez(path, K=data['K'], dist=data['dist'], intercept=float(intercept_cm))
    return f"Intercept updated to {intercept_cm} cm"

def calibrate_compatible(cam_index, intercept_cm):
    return save_compatible_calibration(cam_index, intercept_cm)


# ================== Gradio UI ==================

with gr.Blocks(title="RCJ Camera Calibrator") as demo:
    gr.Markdown("## 🤖 RCJ Camera Calibrator - Full Calibration + Real-time Mapping")
    
    with gr.Row():
        with gr.Column(scale=3):
            image_out = gr.Image(label="Camera Preview (Undistorted)", type="numpy")
        with gr.Column(scale=2):
            cam_index = gr.Number(value=DEFAULT_CAM_INDEX, label="Camera Index", precision=0)
            intercept_input = gr.Number(value=DEFAULT_INTERCEPT_CM, label="Intercept (cm)", info="Robot to front edge of board")
            with gr.Row():
                start_btn = gr.Button("▶️ Start Stream")
                stop_btn = gr.Button("⏹️ Stop")
            with gr.Row():
                capture_btn = gr.Button("📸 Capture Frame")
                calibrate_btn = gr.Button("🎯 Calibrate Now")
            load_btn = gr.Button("📂 Load Calibration")
            save_intercept_btn = gr.Button("💾 Save Intercept")
            status = gr.Textbox(label="Status", value="Idle")

    info_out = gr.Textbox(label="Robot Coordinates (X_cm, Y_cm)", lines=10)

    start_btn.click(start_stream, inputs=[cam_index, intercept_input], outputs=[image_out, info_out])
    stop_btn.click(stop_stream, outputs=status)
    calibrate_btn.click(calibrate_compatible, inputs=[cam_index, intercept_input], outputs=status)
    capture_btn.click(capture_frame, outputs=status)
    load_btn.click(load_calibration, outputs=status)
    save_intercept_btn.click(save_intercept, inputs=intercept_input, outputs=status)

if __name__ == '__main__':
    demo.launch(server_name="0.0.0.0", server_port=7863)