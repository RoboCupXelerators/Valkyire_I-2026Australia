import gradio as gr
import cv2
import time
from collections import deque

def find_available_cameras():
    """
    扫描若干 /dev/videoX 索引，返回可用的摄像头索引列表。
    """
    available_cameras = []
    # 检查索引 0 到 19
    # 说明：有些设备节点（例如 /dev/video1）可能不是 capture device，
    # 用 CAP_V4L2 并且额外 read 一帧来验证，可减少误判。
    for i in range(20):
        # 避免 OpenCV 在尝试不存在的索引时刷屏 warning
        try:
            cv2.setLogLevel(0)  # SILENT
        except Exception:
            pass

        cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            continue

        # 读一帧做有效性验证
        ret, _ = cap.read()
        cap.release()

        if ret:
            available_cameras.append(str(i))

    # 恢复默认日志等级
    try:
        cv2.setLogLevel(3)  # INFO
    except Exception:
        pass
    
    if not available_cameras:
        print("警告：未找到任何摄像头。将默认使用索引 0。")
        return ["0"]
        
    print(f"找到的摄像头: {available_cameras}")
    return available_cameras

def stream_video(
    camera_index_str,
    show_stats=True,
    target_fps=30,
    use_mjpg=False,
    Width=640,
    Height=480,
    AutoExposure=3,
    Exposure=157,
    Brightness=0,
    Contrast=32,
    Saturation=64,
):
    """
    一个生成器函数，用于从指定的摄像头索引捕获并持续产生（yield）视频帧。
    """
    
    # 将下拉菜单传来的字符串索引转换为整数
    try:
        camera_index = int(camera_index_str)
    except ValueError:
        print(f"无效的摄像头索引: {camera_index_str}")
        yield None, f"无效的摄像头索引: {camera_index_str}"
        return

    # 初始化摄像头：优先使用 V4L2（更稳定/更符合 Linux /dev/video 习惯），失败再 fallback
    cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(camera_index)
    
    if not cap.isOpened():
        print(f"错误：无法打开摄像头索引 {camera_index}")
        # （可选）返回一张表示错误的图片
        yield None, f"错误：无法打开摄像头索引 {camera_index}"
        return

    backend = "V4L2" if cap.getBackendName().lower().find("v4l") >= 0 else cap.getBackendName()
    print(f"成功打开摄像头 {camera_index} (backend={backend})，开始推流...")

    # 尽量把 cap.set 放在循环外，避免每帧重复设置造成额外延时
    # 注意：不同摄像头/驱动对这些属性支持程度不同（可能 set 失败但不会报错）
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, Width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, Height)

    # 很多 USB 摄像头在高分辨率下如果使用 raw(YUYV) 会被带宽卡住，FPS 会大幅降低。
    # 试试 MJPG 往往能明显提升 FPS（前提是摄像头支持 MJPG）。
    if use_mjpg:
        try:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        except Exception:
            pass

    # 只是“请求”帧率，驱动不一定会按这个来。
    if target_fps and target_fps > 0:
        try:
            cap.set(cv2.CAP_PROP_FPS, float(target_fps))
        except Exception:
            pass
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, AutoExposure)
    cap.set(cv2.CAP_PROP_EXPOSURE, Exposure)
    cap.set(cv2.CAP_PROP_BRIGHTNESS, Brightness)
    cap.set(cv2.CAP_PROP_CONTRAST, Contrast)
    cap.set(cv2.CAP_PROP_SATURATION, Saturation)

    # FPS 统计：
    # - cap_ts：以 read() 完成时刻统计“采集端 FPS”（更接近摄像头真实产出）
    # - out_ts：以 yield 前时刻统计“输出 FPS”（更接近 UI 看到的刷新）
    cap_ts = deque(maxlen=60)
    out_ts = deque(maxlen=60)

    try:
        while True:
            loop_t0 = time.perf_counter()

            # 读取一帧画面
            read_t0 = time.perf_counter()
            ret, frame = cap.read()
            read_t1 = time.perf_counter()
            cap_ts.append(read_t1)
            
            # 如果 ret 为 False，表示读取失败（可能摄像头断开）
            if not ret:
                print("错误：无法从摄像头读取画面。")
                break

            # 叠加统计信息（在 BGR 上写字，避免 RGB/BGR 颜色混乱）
            out_t = time.perf_counter()
            out_ts.append(out_t)

            # 这里的“延时”是：从 cap.read() 成功返回（拿到图像）到准备输出（yield 前）
            latency_ms = (out_t - read_t1) * 1000.0
            read_block_ms = (read_t1 - read_t0) * 1000.0

            if len(cap_ts) >= 2 and (cap_ts[-1] - cap_ts[0]) > 0:
                cap_fps = (len(cap_ts) - 1) / (cap_ts[-1] - cap_ts[0])
            else:
                cap_fps = 0.0

            if len(out_ts) >= 2 and (out_ts[-1] - out_ts[0]) > 0:
                out_fps = (len(out_ts) - 1) / (out_ts[-1] - out_ts[0])
            else:
                out_fps = 0.0

            # 额外统计一次转换耗时，帮助定位瓶颈
            cvt_t0 = time.perf_counter()
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            cvt_ms = (time.perf_counter() - cvt_t0) * 1000.0

            stats_text = (
                f"CapFPS: {cap_fps:5.1f} | OutFPS: {out_fps:5.1f} | "
                f"Latency(read->yield): {latency_ms:6.1f} ms | "
                f"Read: {read_block_ms:6.1f} ms | Cvt: {cvt_ms:5.1f} ms"
            )

            if show_stats:
                # 两次 putText 做描边，提高可读性
                org = (10, 30)
                cv2.putText(
                    frame,
                    stats_text,
                    org,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 0),
                    3,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    frame,
                    stats_text,
                    org,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
                
            # 产生（yield）这一帧画面，它将被推送到 Gradio 的 Image 组件
            yield frame_rgb, (stats_text if show_stats else "")
            
            # 尽量控制输出帧率，防止 CPU 占用过高
            if target_fps and target_fps > 0:
                target_dt = 1.0 / float(target_fps)
                elapsed = time.perf_counter() - loop_t0
                remain = target_dt - elapsed
                if remain > 0:
                    time.sleep(remain)
            
    except Exception as e:
        print(f"推流时发生错误: {e}")
    finally:
        # 循环结束（无论是因为错误还是被Gradio停止），释放摄像头
        print(f"释放摄像头 {camera_index}")
        cap.release()
        # yield None # 清空图像

# --- 构建 Gradio 界面 ---

# 1. 在脚本启动时，先扫描一次可用的摄像头
available_cameras_list = find_available_cameras()

with gr.Blocks() as demo:
    gr.Markdown(
        """
        # 实时 Linux 摄像头查看器
        请从下拉菜单中选择一个摄像头端口（索引号），然后点击“开始”按钮。
        """
    )
    
    with gr.Row():
        # 下拉菜单，用于选择摄像头
        cam_selector = gr.Dropdown(
            label="选择摄像头端口",
            choices=available_cameras_list,
            value=available_cameras_list[0] if available_cameras_list else None
        )
        
    with gr.Row():
        # “开始”和“停止”按钮
        start_button = gr.Button("开始", variant="primary")
        stop_button = gr.Button("停止")

    with gr.Row():
        show_stats_cb = gr.Checkbox(label="显示 FPS/延时", value=True)

    with gr.Row():
        use_mjpg_cb = gr.Checkbox(label="使用 MJPG(可能提升FPS)", value=False)

    with gr.Row():
        target_fps_slider = gr.Slider(
            1,
            60,
            value=30,
            step=1,
            label="目标输出帧率 (target_fps)",
        )

    # 用于显示视频流的图像组件
    output_image = gr.Image(label="实时画面", type="pil")
    stats_box = gr.Textbox(label="统计信息", interactive=False)

    # --- 事件处理 ---
    
    # 当“开始”按钮被点击时：
    # 1. 调用 `stream_video` 函数
    # 2. 将 `cam_selector` 的当前值作为输入
    # 3. 将 `stream_video` 函数 `yield` 的所有值（即视频帧）
    #    持续更新到 `output_image` 组件上
    start_event = start_button.click(
        fn=stream_video,
        inputs=[cam_selector, show_stats_cb, target_fps_slider, use_mjpg_cb],
        outputs=[output_image, stats_box],
    )
    
    # 当“停止”按钮被点击时：
    # 1. 使用 `cancels=[start_event]` 来取消（停止）
    #    由 `start_button.click` 触发的 `stream_video` 函数的
    #    `while True` 循环。
    # 2. 这将触发 `stream_video` 函数中的 `finally` 块，
    #    从而安全地执行 `cap.release()`
    stop_button.click(
        fn=None,  # 不需要调用新函数
        inputs=None,
        outputs=None,
        cancels=[start_event] # 关键：取消开始事件
    )

if __name__ == "__main__":
    # 启动 Gradio 应用
    # share=True 会生成一个公开链接，方便从其他设备访问
    demo.launch(share=False)