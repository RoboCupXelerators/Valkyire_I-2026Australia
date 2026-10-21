# Software 目录结构

这份文件只记录当前仍然推荐阅读和维护的结构。

## 顶层目录

```text
Software/
├── src/                    # 机器人运行代码、策略、视觉、数据和测试
├── docs/                   # 当前维护文档
├── scripts/                # 可重复运行的部署、调试、模型脚本
├── web/control_panel/      # control_panel.py 使用的前端资源
├── Tools/                  # 历史标定数据、归档说明和许可
├── Records/                # 本地运行/采样记录，默认不作为代码入口
├── log/                    # 本地运行日志
├── requirements*.txt       # Python 依赖
├── packages_config.json    # 外部 packages 仓库配置
├── pytest.ini              # 测试配置
├── README.md
└── STRUCTURE.md
```

## 快速命令

```bash
# 跑当前保留的自动化测试
venv/bin/python -m pytest

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
├── sensor.py               # Bits 位姿校验和统一墙距计算
├── strategy_tools.py       # 策略复用工具
└── motion/
    ├── chassis.py          # 底盘/外设/按键 facade
    ├── motion_backend.py   # 轮速混控、限幅、滤波
    ├── motion_recorder.py  # 运动记录
    └── recording_chassis.py
```

边界约定：

- `runtime.py` 负责选择 `IceLoongBits` / `XeleratorSimBits`。
- `chassis.py` 不直接导入具体 Bits，只接收 runtime 注入的 callable。
- `positions.py` 消费 runtime 提供的语义接口，不直接碰具体 Bits。
- `sensor.py` 只消费 Bits 位姿，提供四向墙距，不创建线程或扫描驱动。
- `runtime.lidar` 仅为 `runtime.sensor` 的旧名称，不再有第二个雷达对象。
- `Car` 直接调用 `MotionBackend` 混控，记录层通过输出 hook 记录，不重复实现运动。

### vision_utils/

```text
src/vision_utils/
├── Vision.py               # 真实视觉主入口
├── XeleratorVision.py      # Unity/SIM 视觉适配
├── LunaIntelligence.py     # Luna YOLO 后端
├── TunaIntelligence.py     # Tuna YOLO 后端
├── ArisuIntelligence.py    # 历史 Arisu 视觉后端
├── PreProcess.py           # 预处理/标定相关逻辑
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
├── interface_semantics.md  # 坐标、哨兵值、语义接口约定
├── MULTI_ROBOT_SCRIMMAGE.md
├── OTA_GUIDE.md
├── PACKAGES_AND_OTA.md
└── SIMULATOR_ACCEPTANCE.md
```

已过期的流水账、草稿报告、一次性提示词和临时脚本不再放在 `docs/`。

## legacy 分支

ROS2、LD07/LD06、DM2325 驱动及相关测试、现场工具保留在 `legacy` 分支。
本公开快照只包含 `main`，`legacy` 分支未包含。
main 不再携带这些驱动和 ArisuBits 归档。结构精简仅在 main 维护。

## scripts/

```text
scripts/
├── fetch_packages.py
├── setup_env_auto.sh
├── setup_venv_jetson_orin_nano.sh
├── install_cuda.sh
├── install_system_service.sh
├── udev-gen.sh
├── run_lockball_pid.py
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
