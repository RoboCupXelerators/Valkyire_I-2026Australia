"""
Gradio-based camera calibrator

功能：
- 使用摄像头实时预览棋盘格
- 在图像上绘制检测到的棋盘格交点
- 计算并显示透视变换矩阵下每个交点的变换后像素坐标与以像素->厘米换算得到的坐标
- 可保存当前标定数据（matrix 和 p2c）到本工程的 `Calibration` 目录中

使用：
- 运行此脚本后会启动 Gradio 页面，点击 "Start" 开始流式预览，点击 "Stop" 停止
- 选择合适的摄像头索引、棋盘格尺寸（宽、高）和拦截值（Intercept），然后预览窗口会显示角点与变换数据

备注：此脚本依赖 opencv-python、numpy、gradio
"""

import cv2
import numpy as np
import math
import time
from pathlib import Path
import gradio as gr
import threading

# 默认参数
DEFAULT_CAM_INDEX = 0
DEFAULT_INTERCEPT = 15
DEFAULT_BOARD_W = 9
DEFAULT_BOARD_H = 9

running = False
cap = None
current_state = {}


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


def format_transformed_points(corners, matrix, p2c, max_points=200):
    """返回格式化文本。"""
    if matrix is None or p2c is None:
        return "No valid perspective / calibration yet"
    fPixelToCM, fHorizontalB, fVerticalB = p2c
    lines = []
    n = corners.shape[0]
    for i in range(n):
        x, y = corners[i][0]
        tx, ty = apply_perspective_transform_to_point(float(x), float(y), matrix)
        # 用与原脚本一致的像素->厘米映射
        h_cm = -fPixelToCM * tx
        v_cm = fHorizontalB  # 这里只保留两个方向示例：水平和垂直计算
        v_cm = fVerticalB + fPixelToCM * ty
        lines.append(f"#{i:02d}: px=({tx:.1f},{ty:.1f})  cm=(H={h_cm:.2f}, V={v_cm:.2f})")
        if i + 1 >= max_points:
            lines.append("... truncated ...")
            break
    return "\n".join(lines)


def overlay_points_on_image(img, corners, matrix, p2c):
    out = img.copy()
    if corners is None:
        return out
    # 画原始角点
    for i in range(corners.shape[0]):
        x, y = int(corners[i][0][0]), int(corners[i][0][1])
        cv2.circle(out, (x, y), 3, (0, 255, 0), -1)

    if matrix is not None:
        # 画透视变换后的点及编号
        for i in range(corners.shape[0]):
            x, y = corners[i][0]
            res = apply_perspective_transform_to_point(float(x), float(y), matrix)
            if res is None:
                continue
            tx, ty = int(res[0]), int(res[1])
            cv2.circle(out, (tx, ty), 4, (0, 0, 255), -1)
            cv2.putText(out, f"{i}", (tx + 4, ty - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)

    return out


def video_generator(cam_index=DEFAULT_CAM_INDEX, intercept=DEFAULT_INTERCEPT, board_w=DEFAULT_BOARD_W, board_h=DEFAULT_BOARD_H):
    global running, cap, current_state
    # 打开摄像头
    cap = cv2.VideoCapture(int(cam_index), cv2.CAP_V4L2)
    # 尝试一些常用设置
    try:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        cap.set(cv2.CAP_PROP_BRIGHTNESS, -64)
        cap.set(cv2.CAP_PROP_CONTRAST, 32)
        cap.set(cv2.CAP_PROP_SATURATION, 64)
        cap.set(cv2.CAP_PROP_SHARPNESS, 2)
    except Exception:
        pass

    running = True

    # 连续产生帧，直至外部将 running 设为 False
    while running:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        # 为检测做灰度
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCornersSB(gray, (int(board_w), int(board_h)), None)

        matrix = None
        p2c = None
        display_img = frame.copy()
        info_text = "No chessboard found"

        if found and corners is not None:
            # 计算矩阵与 p2c
            matrix, p2c = compute_calibration_from_corners(corners, int(board_w), int(board_h), float(intercept))
            # 绘制原点和变换后点
            display_img = overlay_points_on_image(display_img, corners, matrix, p2c)
            info_text = format_transformed_points(corners, matrix, p2c)
        else:
            # 无角点，仅显示原始图像
            cv2.putText(display_img, "No chessboard detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # 转换为 RGB 供 Gradio 显示
        display_rgb = cv2.cvtColor(display_img, cv2.COLOR_BGR2RGB)

        # 保存当前状态，供 Save 按钮使用
        current_state['matrix'] = matrix
        current_state['p2c'] = p2c

        # 生成输出：图像与文本
        yield display_rgb, info_text

    # 退出循环后清理
    try:
        cap.release()
    except Exception:
        pass


def stop_stream():
    global running
    running = False
    return "Stopped"


def save_calibration(cam_index=DEFAULT_CAM_INDEX):
    global current_state
    matrix = current_state.get('matrix')
    p2c = current_state.get('p2c')
    if matrix is None or p2c is None:
        return "No calibration to save"

    APP_DIR = Path(__file__).parent
    DATA_DIR = APP_DIR / "Calibration"
    DATA_DIR.mkdir(parents=True, exist_ok=True)

    file_path = DATA_DIR / f"CalibrationData{int(cam_index)}.npz"
    np.savez(file_path, matrix=matrix, p2c=p2c)
    return f"Saved: {file_path}"


with gr.Blocks(title="Camera Calibrator (Gradio)") as demo:
    gr.Markdown("## 📷 Camera Calibrator - 实时透视变换演示")
    with gr.Row():
        with gr.Column(scale=3):
            image_out = gr.Image(label="Camera Preview", type="numpy")
        with gr.Column(scale=2):
            cam_index_input = gr.Number(value=DEFAULT_CAM_INDEX, label="Camera Index", precision=0)
            intercept_input = gr.Number(value=DEFAULT_INTERCEPT, label="Intercept (cm)")
            board_w_input = gr.Number(value=DEFAULT_BOARD_W, label="Chessboard Width (inner corners)", precision=0)
            board_h_input = gr.Number(value=DEFAULT_BOARD_H, label="Chessboard Height (inner corners)", precision=0)
            start_btn = gr.Button("Start")
            stop_btn = gr.Button("Stop")
            save_btn = gr.Button("Save Calibration")
            status_txt = gr.Textbox(label="Status", value="Idle", interactive=False)

    info_out = gr.Textbox(label="Transformed Points (index: px=(x,y) cm=(H,V))", lines=12)

    def _start_stream(cam_index, intercept, board_w, board_h):
        # 作为 generator：先返回一个占位帧表明正在启动，然后开始流式输出真实帧（每个 yield 返回 (image, info_text)）
        placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(placeholder, "Starting...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        yield placeholder, "Starting"
        yield from video_generator(cam_index, intercept, board_w, board_h)

    start_btn.click(_start_stream, inputs=[cam_index_input, intercept_input, board_w_input, board_h_input], outputs=[image_out, info_out])

    def _stop_click():
        stop_stream()
        return "Stopped"

    stop_btn.click(_stop_click, outputs=[status_txt])
    save_btn.click(save_calibration, inputs=[cam_index_input], outputs=[status_txt])

if __name__ == '__main__':
    demo.launch(share=False)
