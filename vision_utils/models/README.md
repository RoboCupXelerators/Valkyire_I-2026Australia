# Vision Models 说明（TunaIntelligence / Hailo）

本目录同时包含 **Jetson(TensorRT)** 和 **Hailo8** 两套推理所用的模型文件。

## 当前文件与用途

### 1) RoboCup 4 类自训模型（推荐给 TunaIntelligence）

#### `yolov8nForX5.pt / yolov8nForX5.onnx / yolov8nForX5.engine`

- **类别数**：4
- **names**：`{0: Ball, 1: BlueGoal, 2: YellowGoal, 3: Chassis}`
- **用途**：`src/vision_utils/TunaIntelligence.py` 默认使用该模型。
- 说明：
  - `.pt` 是 PyTorch 权重
  - `.onnx` 为导出中间产物
  - `.engine` 为在 Jetson 上导出的 TensorRT 引擎（建议在目标设备上重新导出）

导出/重导出 engine（建议在 Orin Nano 上执行）：

```bash
source venv/bin/activate
python scripts/export_tuna_engine.py \
  --pt src/vision_utils/models/yolov8nForX5.pt \
  --engine src/vision_utils/models/yolov8nForX5.engine \
  --imgsz 640 --batch 4 --half
```

> 备注：如果你想达到类似 Hailo8 的 4x30FPS（=120FPS 总吞吐），通常需要更低 CPU 开销的解码/预处理流水线（NVDEC + NVMM 零拷贝）以及更紧凑的 TensorRT 推理路径。

### 2) COCO 80 类通用模型（不是本项目 4 类自训）

#### `yolov8s.pt / yolov8s.onnx / yolov8s.engine`

- **类别数**：80（COCO）
- **names**：`person/bicycle/car/.../sports ball/...`
- **用途**：通用演示/对比。
- 警告：
  - 该模型 **不包含** 我们项目的 `BlueGoal/YellowGoal/Chassis` 语义类；直接替换会导致后处理/映射不匹配。
  - `yolov8s.engine` 如果不是在当前设备导出，会出现 TensorRT 警告：跨设备使用 engine 可能影响性能甚至出错。

如果要使用 **YOLOv8s 结构** 来做 RoboCup 4 类检测，需要训练得到类似：

- `yolov8sForX5.pt`（4 类 names 同上）

然后再导出到 `.engine`。

### 3) Hailo8 / 其它加速器

#### `yolov8s.hef`

- Hailo8 HEF 格式模型（由 Hailo 工具链编译得到）

#### `yolov8n.bin` / `yolov8n_detect_bayese_640x640_nchw.bin`

- 其它平台/推理框架使用的二进制模型（与 TensorRT 不同）

## 性能建议（Orin Nano 8GB）

### 1) 先确保系统跑在最大功耗/满频

```bash
sudo nvpmodel -m 0   # 以实际机型输出为准，0 常见是 MAXN
sudo jetson_clocks
tegrastats
```

### 2) 降低 CPU 占用：使用硬解码 + 零拷贝

- 推荐把相机采集从 OpenCV `VideoCapture` 切到 GStreamer（NVDEC/NVMM），并尽可能在 GPU 上完成 resize/色彩转换。
- 如果目标是“接近 Hailo8 的吞吐”，建议进一步考虑：DeepStream 或直接 TensorRT + CUDA 前后处理（减少 Python/Ultralytics 开销）。
