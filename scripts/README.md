# scripts

项目维护、部署、模型导出和现场调试脚本放在这里。脚本应当是可重复运行的工具；
一次性提示词、临时实验和已经断掉的入口不要放进这个目录。

Hailo 的 `.whl` / `.deb` 安装包保留在本目录中，方便在 Jetson / Hailo
机器上离线安装驱动和 runtime。

## 目录

```
scripts/
├── build_codegraph_view.py        # 生成 docs/codegraph_view.html
├── debug/                         # 调试工具
│   ├── debug_ball_distance.py
│   └── debug_teacher.py
│
├── strategy_test/                 # Unity 策略调试入口
│   ├── run_alpha_vs_lambda.py
│   ├── test_alpha_single.py
│   ├── test_alpha_vs_lambda.py
│   ├── test_alpha_vs_teacher.py
│   └── test_single_with_reset.py
│
├── debug_cam_ball_bbox.py         # 摄像头球框调试
├── export_tuna_engine.py          # 导出 Tuna TensorRT 模型
├── export_tuna_engine_int8.py     # 导出 Tuna INT8 模型
├── fetch_packages.py              # 拉取外部 packages 仓库里的大文件
├── generate_offline_teacher_bc.py # 生成离线教师数据
├── infer_draw_tuna.py             # Tuna 推理可视化
├── plot_mlp_training.py           # MLP 训练曲线
├── run_lockball_pid.py            # 真机锁球 PID 调试
├── run_sim_lockball_pid_sweep.py  # 仿真锁球 PID 扫描
├── run_tests.sh                   # 测试入口
├── setup_env_auto.sh              # 通用环境初始化
├── setup_venv_jetson_orin_nano.sh # Jetson venv 初始化
├── install_cuda.sh                # Jetson CUDA/JetPack 依赖
├── install_system_service.sh      # 系统服务安装
├── jetpack_test.sh                # JetPack 环境检查
└── udev-gen.sh                    # 设备 udev 规则辅助
```

## 使用方法

### Codegraph

```bash
cd Software
python scripts/build_codegraph_view.py
```

输出文件：

```text
docs/codegraph_view.html
```

### 调试

```bash
cd Software
python scripts/debug/debug_ball_distance.py

python scripts/debug/debug_teacher.py
```

### 策略测试

```bash
# Alpha vs Lambda 对打（需要 Unity 仿真器）
cd Software
python scripts/strategy_test/run_alpha_vs_lambda.py

# Alpha 单机测试
python scripts/strategy_test/test_alpha_single.py
```

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
