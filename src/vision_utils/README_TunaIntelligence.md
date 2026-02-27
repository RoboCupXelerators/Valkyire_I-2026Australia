# TunaIntelligence (Jetson Nano)

## 目标

在 Jetson Nano 上使用 `src/vision_utils/models/yolov8s.pt` 进行 YOLOv8 推理，并保持与 `ArisuIntelligence` 相同的接口：

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
- `TV` -> `TunaVision`
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

## TensorRT Engine 构建（推荐 Jetson 本机执行）

在 Jetson 上（已安装好 torch/ultralytics/TensorRT）运行：

```bash
python -c "from ultralytics import YOLO; YOLO('src/vision_utils/models/yolov8s.pt').export(format='engine', imgsz=640, half=True, int8=False, device=0)"
```

导出成功后会生成 `yolov8s.engine`（通常与 pt 同目录）。`TunaIntelligence` 会优先加载该 engine。

## 离线测试（无摄像头）

```bash
python src/tests/test_tuna_intelligence_offline.py
```

该测试会：

1. `init_cams=False` 启动 `TunaIntelligence`
2. 读取 `src/vision_utils/test1.jpg`、`src/vision_utils/test2.jpg`
3. 将该图填充到 4 个 camera slot
4. 检查 `ChassisQueue/BallQueue` 的输出格式

## 注意

当前实现按类别 ID 直接筛选（约定）：

- `0 = ball`
- `1 = chassis`
- `2/3` 忽略

如后续训练集修正导致类别变化，请同步修改 `TunaIntelligence.ModelInfer()`。
