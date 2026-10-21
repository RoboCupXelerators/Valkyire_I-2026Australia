# scripts

项目维护、部署、模型导出和现场调试脚本放在这里。测试和平台验证工具统一放在 src/tools/tests/；
一次性提示词、临时实验和已经断掉的入口不要放进这两个目录。

Hailo 的 `.whl` / `.deb` 安装包保留在本目录中，方便在 Jetson / Hailo
机器上离线安装驱动和 runtime。

## 目录

```
scripts/
├── debug/                         # 调试工具
│   └── debug_ball_distance.py
│
├── debug_cam_ball_bbox.py         # 摄像头球框调试
├── export_tuna_engine.py          # 导出 Tuna TensorRT 模型
├── export_tuna_engine_int8.py     # 导出 Tuna INT8 模型
├── fetch_packages.py              # 拉取外部 packages 仓库里的大文件
├── infer_draw_tuna.py             # Tuna 推理可视化
├── run_lockball_pid.py            # 真机锁球 PID 调试
├── setup_env_auto.sh              # 通用环境初始化
├── setup_venv_jetson_orin_nano.sh # Jetson venv 初始化
├── install_cuda.sh                # Jetson CUDA/JetPack 依赖
├── install_system_service.sh      # 系统服务安装
└── udev-gen.sh                    # 设备 udev 规则辅助
```

## 使用方法

### 调试

```bash
cd Software
python scripts/debug/debug_ball_distance.py
```

### 测试与平台检查
运行通用测试：src/tools/tests/run_tests.sh
ROS2、LD06/LD07 探测和 DM2325 工具请切换到 `legacy` 分支（本公开快照未包含）。
安装 CH344 驱动：bash scripts/install_ch344_driver.sh
运行 JetPack 检查：src/tools/tests/jetpack_test.sh
### 开机服务

从仓库根目录显式指定需要启动的策略，例如：

```bash
bash scripts/install_system_service.sh src/Hyperion.py
```

安装器使用本仓库 `venv/bin/python`，不依赖 ROS 环境。

### 虚拟裁判串口

```bash
python src/tools/tests/virtual_referee.py
```

脚本第一行输出 PTY 设备路径。将该路径设置为
`ROBOCUPX_REFEREE_PORT`，然后在脚本中输入 `play`、`stop` 或 `penalty`。
线路内容严格保持为官方协议的 `PLAY`/`STOP`。

### 模型 / Hailo

```bash
python scripts/export_tuna_engine.py
python scripts/export_tuna_engine_int8.py
python scripts/infer_draw_tuna.py
```

Hailo 安装包在本目录下保留：

```text
hailort-4.23.0-cp312-cp312-linux_aarch64.whl
hailort_4.23.0_arm64.deb
hailort-pcie-driver_4.23.0_all.deb
```
