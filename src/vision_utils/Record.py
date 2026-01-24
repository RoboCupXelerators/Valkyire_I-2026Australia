import cv2
import time

def record_video():
    # 1. 打开摄像头 (0 通常是默认摄像头)
    # 如果你有多个摄像头，尝试改为 1 或 2
    cap = cv2.VideoCapture(4)

    # 检查摄像头是否成功打开
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    # 2. 获取摄像头默认的分辨率（宽度和高度）
    # 必须确保 VideoWriter 的分辨率与输入源完全一致，否则无法保存
    frame_width = 640
    frame_height = 480
    
    print(f"录制分辨率: {frame_width}x{frame_height}")

    # 3. 定义视频编码器和输出文件
    # XVID 是一个在树莓派上兼容性较好的编码格式，生成 .avi 文件
    # 如果想要 mp4，可以使用 'mp4v'，但在某些旧系统上可能需要额外库
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    
    # 参数说明: '文件名', 编码器, 帧率(FPS), (宽, 高)
    # 注意：如果实际录制帧率低于设置的20.0，视频播放时会变快
    out = cv2.VideoWriter('output.avi', fourcc, 30.0, (frame_width, frame_height))

    print("开始录制... 按 'q' 键停止。")

    try:
        while True:
            # 逐帧捕获
            ret, frame = cap.read()

            if not ret:
                print("无法接收帧 (流结束?). 退出中...")
                break

            # (可选) 在这里可以对 frame 进行处理，比如转灰度、画框等
            # frame = cv2.flip(frame, 1) # 如果画面是镜像的，取消注释这行来翻转

            # 4. 写入帧到文件
            out.write(frame)

            # 5. 显示实时画面 (如果你在没有显示器的模式下运行，请注释掉下面两行)
            # cv2.imshow('Video Recording', frame)

            # # 按 'q' 键退出循环
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        # 允许使用 Ctrl+C 退出
        print("检测到键盘中断，停止录制")

    # 6. 释放资源
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    print("视频已保存为 output.avi")

if __name__ == "__main__":
    record_video()