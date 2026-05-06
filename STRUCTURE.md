# Software 目录结构

```
Software/
├── src/                    # 源代码
├── scripts/                # 脚本
├── web/                    # Web 前端
├── docs/                   # 文档
├── packages/               # 安装包
├── Tools/                  # 开发工具
├── Calibration/            # 标定数据
├── requirements*.txt       # 依赖
├── README.md
└── STRUCTURE.md
```

## 快速开始

```bash
# 安装依赖
pip install -r requirements.txt

# 运行测试
pytest

# 启动服务
python src/vision_dashboard.py
python src/control_panel.py
```

## 详细结构

### src/ - 源代码

| 文件/目录 | 说明 |
|-----------|------|
| `BasicFuc.py` | 基础函数库 |
| `chassis.py` | 底盘控制 |
| `Sensor.py` | 传感器读取 |
| `LunaPre.py` | 运行时初始化 |
| `Vision.py` | 视觉处理 |
| `offence.py` | 进攻策略 |
| `defence.py` | 防守策略 |
| `exec.py` | 策略执行器 |
| `control_panel.py` | 控制面板 |
| `vision_dashboard.py` | 视觉仪表板 |
| `vision_utils/` | 视觉工具 |
| `ReasonData/` | 数据记录与回放 |
| `utils/` | 工具函数 |
| `tests/` | 测试脚本 |

### scripts/ - 脚本

| 脚本 | 说明 |
|------|------|
| `setup_env_auto.sh` | 环境安装 |
| `run_tests.sh` | 运行测试 |
| `install_cuda.sh` | CUDA 安装 |

### docs/ - 文档

- 日志、变更记录等

### packages/ - 安装包

- Hailo、Jetson 等平台的安装包

## 相关文档

- [MJPEG 修复指南](../SoccerVisonWiki/MJPEG_Camera_Stream_Fix.md)
- [比赛回放指南](src/ReasonData/REPLAY_GUIDE.md)
- [模拟器控制](../SoccerSimulatorForXels/README_XEL_UDP.md)