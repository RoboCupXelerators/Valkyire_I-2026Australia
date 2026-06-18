# TunaIntelligence（Jetson Nano / Orin 系列）

> Coordinate, pose, sentinel, and live-debug semantics are documented in
> [`../../docs/interface_semantics.md`](../../docs/interface_semantics.md).

## 目标

在 Jetson 平台上使用 YOLOv8 + TensorRT 进行推理，并保持与 `ArisuIntelligence` 相同的接口：

- `self.ChassisQueue`: put `[[cam, cx, bottom_y, w, h, conf] * 4]`
- `self.BallQueue`: put `[[cam, cx, bottom_y, w, h, conf] * 4]`

这样 `src/Vision.py` 不需要改动即可读取结果。

## 配置切换

在 `src/vision_utils/VisionDatas/data/vision.json` 内设置：

```json
{
  "RoboInfo": {
    "Rec": "TI"
  }
}
```

`vision_utils/__init__.py` 已支持：

- `AI` -> `ArisuIntelligence`
- `TI` -> `TunaIntelligence`
- `XV` -> `XeleratorVision`
- `SY` -> `SimYoloVision`
- 其他 -> `BlobBased`

## 依赖说明

仓库 `requirements.txt` 未强制安装 `torch/ultralytics`，因为 Jetson 上需要安装与 JetPack 匹配的 PyTorch Wheel。

在 Jetson 上请自行安装：

- `torch`（NVIDIA/JetPack 对应版本）
- `ultralytics`

并确认 TensorRT 工具可用：

```bash
trtexec --version
```

## TensorRT Engine 构建（推荐在目标 Jetson 设备本机执行）

### 0) 前置说明（非常重要）

1. **engine 必须在目标设备上导出**：TensorRT `.engine` 和 GPU/驱动/TensorRT 版本强绑定。
   把别的机器导出的 engine 拷过来，通常会有警告甚至性能变差/无法运行。
2. **本项目推荐 batch=4**：因为我们默认 4 路相机，`TunaIntelligence` 会尽量把 4 路拼成 batch 一次推理。
3. **强烈建议 FP16**：Orin/Nano 这类平台 FP16 一般性价比最好。
4. 如果你想进一步降低 CPU 占用/延迟：建议导出 `--nms`（end2end NMS 下沉到 TensorRT）。

### 1) 推荐方式：使用仓库脚本导出

本仓库提供了导出脚本：`scripts/export_tuna_engine.py`。

以 4 类自训模型为例（把路径换成你的新训练权重）：

```bash
source venv/bin/activate

# 4 路相机批处理，FP16（推荐）
python scripts/export_tuna_engine.py \
  --pt src/vision_utils/models/yolov8nForX5.pt \
  --engine src/vision_utils/models/yolov8nForX5.engine \
  --imgsz 640 --batch 4 --half

# 可选：导出 end2end（包含 NMS），进一步降低 CPU/NMS 开销
python scripts/export_tuna_engine.py \
  --pt src/vision_utils/models/yolov8nForX5.pt \
  --engine src/vision_utils/models/yolov8nForX5_end2end.engine \
  --imgsz 640 --batch 4 --half --nms
```

脚本会打印：
- 模型 names（用于确认是否为项目 4 类模型）
- 实际导出的 engine 路径和文件大小

### 2) 直接用 Ultralytics 命令导出（不推荐，但可用于验证）

在 Jetson 上（已安装好 torch/ultralytics/TensorRT）运行：

```bash
python -c "from ultralytics import YOLO; YOLO('YOUR_MODEL.pt', task='detect').export(format='engine', imgsz=640, batch=4, half=True, int8=False, device=0)"
```

### 3) 常见坑 / FAQ

**Q0: 我的相机输出是 MJPG（USB 常见），Jetson Nano 如何硬解码？**

本仓库的采集（`VisionPreUntil.InitCam`）在 **检测到 Jetson Nano** 且配置为 `CaptureBackend=auto` 时，会优先尝试使用 GStreamer 管线：

1) `nvv4l2decoder mjpeg=1`（优先，若 JetPack 镜像提供）
2) `nvjpegdec`（次选）
3) `jpegdec`（CPU fallback，但仍保持 gstreamer 链路）

任意一种管线打开失败会自动回退到旧的 `cv2.CAP_V4L2`，所以不会影响其它平台运行。

**Q1: 为什么我用单张图片 predict 会报 shape 不匹配？**

如果你导出的 engine 是固定 batch=4（输入 shape 为 `(4,3,640,640)`），那么用单张图片 `(1,3,640,640)` 去 warmup/predict 会触发 Ultralytics 断言。

解决：
- 对线上 4 路相机推理来说这是正常的（正好 batch=4）。
- 如果你希望离线单张也能用，需要导出 dynamic engine（需要额外配置；本仓库当前以 4 路固定 batch 为主）。

**Q2: 我的 yolov8s.pt 能直接用于项目吗？**

不一定。
- 本项目后处理期待的是 **4 类自训**：Ball/BlueGoal/YellowGoal/Chassis。
- 如果你的 `yolov8s.pt` 是 COCO 80 类（person/bicycle/...），则不能直接替换。

## 离线测试（无摄像头）

```bash
python src/tests/test_tuna_intelligence_offline.py
```

该测试会：

1. `init_cams=False` 启动 `TunaIntelligence`
2. 读取 `src/vision_utils/test1.jpg`、`src/vision_utils/test2.jpg`
3. 将该图填充到 4 个 camera slot
4. 检查 `ChassisQueue/BallQueue` 的输出格式

## 注意（类别映射）

当前实现按类别 ID 直接筛选（约定，自训 4 类模型）：

- `0 = ball`
- `1 = chassis`
- `2/3` 忽略

如后续训练集修正导致类别变化，请同步修改 `TunaIntelligence._init_class_mapping()` 或后处理筛选逻辑。
