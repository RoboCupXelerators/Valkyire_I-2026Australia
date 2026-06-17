# scripts - 工具脚本目录

## 目录结构

```
scripts/
├── debug/                  # 调试工具
│   ├── debug_ball_distance.py    # 球距离调试
│   └── debug_teacher.py          # Teacher 模型调试
│
├── strategy_test/          # 策略测试脚本
│   ├── run_alpha_vs_lambda.py    # Alpha vs Lambda 对打
│   ├── test_alpha_single.py      # Alpha 单机测试
│   ├── test_alpha_vs_lambda.py   # Alpha vs Lambda
│   ├── test_alpha_vs_teacher.py  # Alpha vs Teacher
│   └── test_single_with_reset.py # 单机重置测试
│
├── debug_cam_ball_bbox.py        # 摄像头球边界框调试
├── export_tuna_engine.py         # 导出 Tuna TensorRT 模型
├── export_tuna_engine_int8.py    # 导出 Tuna INT8 模型
├── fetch_packages.py             # 下载依赖包
├── generate_offline_teacher_bc.py # 生成离线教师 BC 数据
├── infer_draw_tuna.py            # Tuna 模型推理可视化
├── plot_mlp_training.py          # MLP 训练曲线绘制
├── run_lockball_pid.py           # 锁球 PID 调试
└── run_sim_lockball_pid_sweep.py # 仿真锁球 PID 扫描
```

## 使用方法

### 调试工具

```bash
# 调试球距离（需要机器人运行）
cd Software
python scripts/debug/debug_ball_distance.py

# 调试 Teacher 模型（需要 Unity 仿真器）
cd Software
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

### 模型导出

```bash
# 导出 TensorRT 模型（Jetson）
python scripts/export_tuna_engine.py
```