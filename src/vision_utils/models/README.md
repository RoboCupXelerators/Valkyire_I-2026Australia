# Vision Models 说明（TunaIntelligence / Hailo）

> Coordinate, pose, sentinel, and live-debug semantics are documented in
> [`../../../docs/interface_semantics.md`](../../../docs/interface_semantics.md).

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

## INT8 量化（TensorRT）

我们使用 **Ultralytics 自带的 TensorRT INT8 导出**（会走校准流程）。

### 重要前提

- **必须提供校准数据集 `data.yaml`**（Ultralytics 用它构建 INT8 calibration dataloader）。
- TensorRT `.engine` **强依赖硬件**，请尽量在目标机（Jetson）上导出。
- `int8=True` 与 `half=True` **互斥**（Ultralytics 会强制关闭 half）。
- 校准集图片数量：
  - Ultralytics 提示 **建议 >300 张**（少了也能做，但精度更不稳定）。

### 导出 INT8 engine（推荐：用你训练时的数据集 yaml）

假设你训练好的权重是 `yolov8s.pt`，并且你有训练/验证数据集 yaml：`/path/to/data.yaml`

```bash
source venv/bin/activate

python scripts/export_tuna_engine_int8.py \
  --pt src/vision_utils/models/yolov8s.pt \
  --engine src/vision_utils/models/yolov8s_int8.engine \
  --data /path/to/data.yaml \
  --imgsz 640 --batch 4 --device 0
```

### 让 TunaIntelligence 使用你自己的模型（写进 config）

在 `src/ReasonData/data/config.json` 里配置：

```json
"VisionVals": {
  "TunaModelPath": "src/vision_utils/models/yolov8s.pt",
  "TunaEnginePath": "src/vision_utils/models/yolov8s_int8.engine"
}
```

然后 TunaIntelligence 默认会优先读取这两个路径（除非你在构造函数里显式传入 model_path/engine_path）。

### 只有图片、没有标注怎么办？（可用，但不如带标注的数据可靠）

如果你只有一堆“真实相机图片”（没有 labels），也可以用于校准：

```bash
source venv/bin/activate

python scripts/export_tuna_engine_int8.py \
  --pt src/vision_utils/models/yolov8s.pt \
  --engine src/vision_utils/models/yolov8s_int8.engine \
  --calib-dir /path/to/calib_images \
  --imgsz 640 --batch 4 --device 0
```

脚本会自动生成一个临时 YOLO 数据集结构，并为每张图创建空 label（背景图），以满足 Ultralytics 的 dataloader。

### 导出后验证（快速 sanity-check）

```bash
python - <<"PY"
from ultralytics import YOLO
m = YOLO("src/vision_utils/models/yolov8s_int8.engine", task="detect")
r = m.predict("src/vision_utils/test1.jpg", imgsz=640, device=0, verbose=False)
print("OK, got", len(r), "result(s)")
PY

```

### 常见坑

1) **batch 大于校准图片数量会直接报错**：校准集至少要 >= batch。
2) **跨设备 engine**：在 PC 上导出的 engine 拿到 Jetson 上可能报错或性能很差。
3) **动态 shape**：如需动态输入（不固定 640），可以加 `--dynamic`，但一般会牺牲一些性能。
4) **NMS end2end**：加 `--nms` 会把 NMS 融进 engine（便于部署），但导出更慢、也可能受版本影响。

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
