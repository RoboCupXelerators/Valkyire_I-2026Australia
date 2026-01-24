import gradio as gr
import cv2
import time

def find_available_cameras():
    """
    检查 /dev/video0 到 /dev/video9，返回可用的摄像头索引列表。
    """
    available_cameras = []
    # 检查索引 0 到 9
    for i in range(20):
        # 尝试打开摄像头
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            # 如果成功打开，说明这个索引是可用的
            available_cameras.append(str(i))
            # 立即释放它
            cap.release()
    
    if not available_cameras:
        print("警告：未找到任何摄像头。将默认使用索引 0。")
        return ["0"]
        
    print(f"找到的摄像头: {available_cameras}")
    return available_cameras

def stream_video(camera_index_str):
    """
    一个生成器函数，用于从指定的摄像头索引捕获并持续产生（yield）视频帧。
    """
    
    # 将下拉菜单传来的字符串索引转换为整数
    try:
        camera_index = int(camera_index_str)
    except ValueError:
        print(f"无效的摄像头索引: {camera_index_str}")
        yield None
        return

    # 初始化摄像头
    cap = cv2.VideoCapture(camera_index)
    
    if not cap.isOpened():
        print(f"错误：无法打开摄像头索引 {camera_index}")
        # （可选）返回一张表示错误的图片
        yield None
        return

    print(f"成功打开摄像头 {camera_index}，开始推流...")

    try:
        while True:
            # 读取一帧画面
            ret, frame = cap.read()
            
            # 如果 ret 为 False，表示读取失败（可能摄像头断开）
            if not ret:
                print("错误：无法从摄像头读取画面。")
                break
                
            # OpenCV 默认使用 BGR 格式，Gradio 的 Image 组件需要 RGB 格式
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # 产生（yield）这一帧画面，它将被推送到 Gradio 的 Image 组件
            yield frame_rgb
            
            # 等待一小段时间，以控制帧率（例如约 30fps）
            # 这也可以防止 CPU 占用过高
            time.sleep(0.03)
            
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

    # 用于显示视频流的图像组件
    output_image = gr.Image(label="实时画面", type="pil")

    # --- 事件处理 ---
    
    # 当“开始”按钮被点击时：
    # 1. 调用 `stream_video` 函数
    # 2. 将 `cam_selector` 的当前值作为输入
    # 3. 将 `stream_video` 函数 `yield` 的所有值（即视频帧）
    #    持续更新到 `output_image` 组件上
    start_event = start_button.click(
        fn=stream_video,
        inputs=cam_selector,
        outputs=output_image
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