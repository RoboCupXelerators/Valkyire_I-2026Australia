# Software 目录结构

这份文件只记录当前仍然推荐阅读和维护的结构。更细的调用关系请看
`docs/codegraph_view.html`。

## 顶层目录

```text
Software/
├── src/                    # 机器人运行代码、策略、视觉、数据和测试
├── docs/                   # 当前维护文档与 codegraph
├── scripts/                # 可重复运行的部署、调试、模型和图生成脚本
├── web/control_panel/      # control_panel.py 使用的前端资源
├── Tools/                  # 历史标定数据、归档说明和许可
├── Calibration_2605/       # 标定数据
├── Records/                # 本地运行/采样记录，默认不作为代码入口
├── logs/                   # 本地训练/运行日志
├── requirements*.txt       # Python 依赖
├── packages_config.json    # 外部 packages 仓库配置
├── pytest.ini              # 测试配置
├── README.md
└── STRUCTURE.md
```

## 快速命令

```bash
# 跑当前保留的自动化测试
../venv/bin/python3 -m pytest

# 生成代码关系图
python3 scripts/build_codegraph_view.py

# 启动控制面板
python3 src/control_panel.py
```

## src/

```text
src/
├── BasicFuc.py             # 历史策略常用函数，后续应继续减肥
├── Hyperion.py             # Hyperion 策略入口
├── offence.py              # 进攻策略
├── defence.py              # 防守策略
├── alpha.py                # Alpha 策略
├── lambda_strategy.py      # Lambda 策略
├── control_panel.py        # Web 控制面板后端
├── STRATEGY_CODE_STYLE.md  # 当前策略代码风格说明
├── LunaPre/                # 运行时 facade、定位、传感器、底盘抽象
├── ReasonData/             # 配置、记录、回放数据
├── mlp/                    # MLP 策略接口、推理和状态编码
├── tests/                  # 当前保留的 pytest 测试
├── utils/                  # Bits、仿真适配、OTA、场景控制等工具
└── vision_utils/           # 视觉后端、检测、融合和模拟器视觉适配
```

### LunaPre/

```text
src/LunaPre/
├── __init__.py             # 对策略暴露 UnitedPosition/chassis/peripheral 等 facade
├── runtime.py              # 具体硬件/模拟器装配层
├── positions.py            # 语义位置、球、路径和 Pos2Pos API
├── position_recorder.py    # Positions 记录包装
├── sensor.py               # Lidar facade
├── lidar_driver_rplidar.py # 直接 RPLidar 驱动
├── lidar_driver_ros2.py    # ROS2 /scan 驱动
├── strategy_tools.py       # 策略复用工具
└── motion/
    ├── chassis.py          # 底盘/外设/按键 facade
    ├── motion_backend.py   # 轮速混控、限幅、滤波
    ├── motion_recorder.py  # 运动记录
    └── recording_chassis.py
```

边界约定：

- `runtime.py` 负责选择 `ArisuBits` / `IceLoongBits` / `XeleratorSimBits`。
- `runtime.py` 也负责装配 `DM2325Bits`；具体串口驱动不进入策略层。
- `chassis.py` 不直接导入具体 Bits，只接收 runtime 注入的 callable。
- `positions.py` 消费 runtime 提供的语义接口，不直接碰具体 Bits。
- `sensor.py` 只负责 lidar facade 和 lidar driver 选择，不消费 vision。

### vision_utils/

```text
src/vision_utils/
├── Vision.py               # 真实视觉主入口
├── XeleratorVision.py      # Unity/SIM 视觉适配
├── HybridVision.py         # 混合视觉入口
├── LunaIntelligence.py     # Luna YOLO 后端
├── TunaIntelligence.py     # Tuna YOLO 后端
├── ArisuIntelligence.py    # 历史 Arisu 视觉后端
├── PreProcess.py           # 预处理/标定相关逻辑
├── hybrid/                 # 融合、Kalman、ROI、调度、YOLO detector
├── models/                 # 视觉模型文件
└── VisionDatas/            # 视觉数据与独立配置兼容层
```

### tests/

当前保留的是小而直接的回归测试：

```text
src/tests/
├── test_defence_strategy.py
├── test_navigation_path.py
├── test_offence_boundary.py
├── test_positions_ball_filter.py
├── test_restricted_area_geometry.py
├── test_slipsideshot.py
└── test_xelerator_sim_bits.py
```

## docs/

```text
docs/
├── codegraph_view.html     # 代码关系可视化
├── DM2325_ARCHITECTURE.md  # DM2325 历史、仓库职责和迁移边界
├── interface_semantics.md  # 坐标、哨兵值、语义接口约定
├── MULTI_ROBOT_SCRIMMAGE.md
├── OTA_GUIDE.md
├── PACKAGES_AND_OTA.md
└── SIMULATOR_ACCEPTANCE.md
```

已过期的流水账、草稿报告、一次性提示词和临时脚本不再放在 `docs/`。

## scripts/

```text
scripts/
├── build_codegraph_view.py
├── fetch_packages.py
├── setup_env_auto.sh
├── setup_venv_jetson_orin_nano.sh
├── install_cuda.sh
├── install_system_service.sh
├── jetpack_test.sh
├── udev-gen.sh
├── run_tests.sh
├── run_lockball_pid.py
├── run_sim_lockball_pid_sweep.py
├── debug/
└── hailort*.whl / hailort*.deb
```

Hailo `.whl` / `.deb` 安装包保留在仓库里，供 Jetson/Hailo 机器离线安装驱动和 runtime。

## 本地生成内容

这些目录/文件主要是本地运行产物或索引，不应当作为架构入口：

```text
.codegraph/
.pytest_cache/
__pycache__/
log/
logs/
tmp_strategy_backshot_check/
```
