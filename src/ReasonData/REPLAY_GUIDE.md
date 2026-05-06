# 比赛记录与回放功能指南

## 概述

本功能允许您记录模拟器中机器人的动作，并在之后回放比赛。这对于：
- 分析比赛策略
- 调试机器人行为
- 演示和展示

## 功能组成

### 1. 数据库模块 (`ReasonData/data.py`)

增强的数据模型，包含以下表：

- **Positions**: 记录机器人位置、速度、球位置等
- **Outputs**: 记录电机输出和命令
- **MatchSession**: 记录比赛会话信息

### 2. 模拟器记录集成 (`utils/XeleratorSimBits.py`)

在 `XeleratorSimBits` 类中添加了记录 API：

```python
# 启用记录（自动创建会话）
bits.EnableRecording(session_id="my_match", record_every=5)

# 检查是否正在记录
if bits.IsRecording():
    print(f"Recording session: {bits.GetSessionId()}")

# 停止记录
bits.StopRecording()
```

### 3. 回放控制器 (`utils/XeleratorReplay.py`)

独立的回放模块，支持：
- 从数据库加载历史会话
- 变速播放（0.5x - 10x）
- 循环播放
- 导出/导入 JSON 格式

## 使用方法

### 方法一：在策略代码中启用记录

```python
from utils.XeleratorSimBits import XeleratorSimBits

# 创建模拟器连接
bits = XeleratorSimBits(unity_ip="127.0.0.1", robot_id=1)

# 启用记录（每 5 帧记录一次，减少数据库负载）
bits.EnableRecording(record_every=5)

# ... 运行策略代码 ...

# 比赛结束后停止记录
bits.StopRecording()
```

### 方法二：使用命令行工具回放

```bash
# 列出所有可用的比赛会话
cd Software/src
python -m utils.XeleratorReplay --list

# 回放最新会话
python -m utils.XeleratorReplay

# 回放指定会话，2倍速
python -m utils.XeleratorReplay --session 20260505_152249 --speed 2.0

# 循环播放
python -m utils.XeleratorReplay --loop

# 导出会话为 JSON
python -m utils.XeleratorReplay --export replay.json

# 从 JSON 导入并回放
python -m utils.XeleratorReplay --import replay.json
```

### 方法三：在代码中使用回放控制器

```python
from utils.XeleratorReplay import ReplayController

# 创建回放控制器
replay = ReplayController(unity_ip="127.0.0.1", replay_port=20003)

# 加载最新会话
frame_count = replay.LoadSession()
print(f"Loaded {frame_count} frames")

# 获取会话信息
info = replay.GetSessionInfo()
print(f"Duration: {info['duration_seconds']:.1f} seconds")

# 开始回放（2倍速，循环）
replay.Play(speed=2.0, loop=True)

# 暂停
replay.Pause()

# 恢复
replay.Resume()

# 跳转到指定时间
replay.SeekToTime(30.0)  # 跳转到第 30 秒

# 停止回放
replay.Stop()
```

## 数据字段说明

### Positions 表

| 字段 | 类型 | 说明 |
|------|------|------|
| Tick | float | 时间戳（秒） |
| RobotId | int | 机器人 ID |
| SessionId | string | 会话 ID |
| XPosition | float | 场地 X 坐标 (cm) |
| YPosition | float | 场地 Y 坐标 (cm) |
| YawDeg | float | 航向角（度） |
| Vx, Vy, Wz | float | 速度分量 |
| BallXPosition | float | 球相对 X 位置 (cm) |
| BallYPosition | float | 球相对 Y 位置 (cm) |
| HasBall | bool | 是否持球 |

## 数据库位置

数据库文件存储在：
```
Software/src/ReasonData/data/Xel-{机器人编号}.{年月}.sqlite
```

例如：`Xel-None.202605.sqlite`

## 注意事项

1. **记录频率**：建议使用 `record_every=5` 或更高，避免过多的数据库写入影响控制循环性能。

2. **磁盘空间**：长时间记录会占用磁盘空间，建议定期清理旧的数据库文件。

3. **Unity 回放端口**：回放默认使用端口 20003，需要在 Unity 端添加对应的接收逻辑。

4. **多机器人**：每个机器人有独立的 RobotId，可以区分不同机器人的数据。

## Unity 端集成（待实现）

要实现完整的回放功能，需要在 Unity 端添加：

1. 回放数据接收器（监听端口 20003）
2. 回放模式控制器
3. 机器人位置插值动画
4. 时间轴 UI 控件

## 测试

运行测试脚本验证功能：

```bash
cd Software/src
python tests/test_match_recording.py --all
```

测试选项：
- `--init`: 测试数据库初始化
- `--mock`: 使用模拟数据测试记录
- `--list`: 列出所有会话
- `--export`: 导出最新会话
- `--replay`: 测试回放控制器
- `--all`: 运行所有测试