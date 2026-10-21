# Xelerators Software

> **公开快照**：本仓库是 Xelerators Software 截至 **2026-09-30** 的 `main` 历史公开快照。仅包含 `main` 分支；模型权重、安装包、标定数据、录包和测试照片未包含，作者信息已匿名化。详见 [PUBLIC_RELEASE.md](PUBLIC_RELEASE.md)。

> **从这里开始**：机器人现场运行时与策略主仓库。它负责 Bits 通信、LunaPre、运动控制、视觉接入、策略和控制面板；模拟器、训练与硬件固件分别在独立仓库维护。

## 快速导航

| 目标 | 入口 |
| --- | --- |
| 第一次配置和运行 | `GETTING_STARTED.md` |
| 首次部署环境 | `bash scripts/setup_env_auto.sh` |
| 启动控制面板 | `source venv/bin/activate` 后运行 `python src/control_panel.py` |
| 策略/运动/裁判运行时 | `src/LunaPre/`、`src/offence.py`、`src/defence.py` |
| MLP 推理契约 | `src/mlp/README.md` |
| 本地配置与回放数据 | `src/ReasonData/`（不提交本机运行数据） |
| 验证改动 | `venv/bin/python -m pytest` |
| 查目录职责 | `STRUCTURE.md` |

`main` 面向 IceLoong 与 Unity/Bits。本公开快照只包含 `main`；需要 ROS2、LD07/LD06 或旧 DM2325 兼容运行时的 `legacy` 分支不在本仓库中。新人请先阅读 [新人成员连接与运行指南](./新人成员连接与运行指南.md)。

机器人足球运行时、策略、视觉和控制面板。

## 分支与硬件

| 分支 | 支持范围 |
| --- | --- |
| `main` | IceLoong (`IL`)、Unity 模拟器 (`SIM`)，定位统一来自 Bits |
| `legacy` | 私有仓库保留；本公开快照未包含 |

ArisuBits 已从 main 删除；ArisuIntelligence/Hailo 视觉后端仍保留。
main 不再安装、探测或启动 ROS2，也不提供原生串口雷达驱动。

已有配置的 `Sensor.Source=auto/bits` 会迁移到 Bits 定位，原配置保留为
`cfg.cache.json`。需要 ROS2/LD07 的机器人请使用私有仓库的 `legacy` 分支（本快照未包含）。
main 的 `Sensor`、`Ports.LidarID/LidarPort` 已移除；PID、场地尺寸和相机映射继续保留。

## 安装与运行

```bash
# 首次部署：自动识别 Jetson/Pi 并准备运行环境
bash scripts/setup_env_auto.sh

# 日常启动
source venv/bin/activate
python src/control_panel.py
```

硬件视觉按 [scripts/README.md](scripts/README.md) 安装对应平台依赖。
配置在 `src/ReasonData/data/config.json`，可用 `XEL_RUNTIME_DATA_DIR` 指定独立目录。
`RoboInfo.Bit` 选择 `IL` 或 `SIM`；`ROBOCUPX_BITS` 可覆盖运行时后端。
Unity 模拟器需单独启动；串口裁判和手动控制保持现有接口。

```bash
venv/bin/python -m pytest
```

默认测试跳过 camera 标记；缺少 PyTorch 时跳过模型测试。本地 UDP/HTTP
测试需要允许端口绑定。物理硬件及真实 Unity 比赛验收需在对应环境执行。

## 代码结构

运行时装配、Sensor 观测、Positions 策略语义和 MotionBackend 混控各负责一层；
SIM/IL 共用墙距计算。详见 [STRUCTURE.md](STRUCTURE.md) 和
[本次审计](docs/ARCHITECTURE_AUDIT.md)。

## 策略快速上手

> 🌟 **新手入门？请先阅读 [GETTING_STARTED.md](./GETTING_STARTED.md) - 专为初学者编写的友好指南！**

本指南旨在帮助开发者快速上手编写机器人足球策略，重点介绍最常用的核心函数和典型应用场景。

## 🚀 快速开始

### 1. 核心对象初始化
在策略脚本中，通常通过 `from LunaPre import *` 导入运行时 facade；锁球工具从 `BasicFuc` 导入，哨兵常量从 `LunaPre.strategy_tools` 导入。
- `UnitedPosition`: 机器人位置、视觉数据获取及运动控制核心对象。
- `chassis`: 底盘运动控制接口。
- `peripheral`: 外设控制（如踢球、运球）。
- `lockball`: 锁球控制对象（`from BasicFuc import lockball`）。

### 2. 最常用函数速查表

#### 📍 机器人定位与视觉 (UnitedPosition)
| 函数 | 功能 | 返回值/示例 |
| :--- | :--- | :--- |
| `RobotBallPos()` | 获取球在**机体坐标系**下的位置 | `[x, y]` (cm) |
| `FieldRobotPose()` | 获取机器人在**场地坐标系**下的位姿 | `[x, y, yaw]` (cm, cm, °) |
| `FieldBallPos()` | 获取球在**场地坐标系**下的位置 | `[x, y]` (cm) |
| `PredictBallLanding(horizon_s=0.45)` | 预测球的短期拦截落点 | `{"valid": bool, "field_pos": [x, y], ...}` |
| `Pos2Pos([x, y, yaw], speed_gain=None)` | 移动到场地坐标系的指定点 | `True` (到达) / `False` (未到达) |
| `Move2Path(points, iWaitMs=0.1)` | 顺序移动到路径点列表（阻塞） | `True` (完成) |

#### 🛞 底盘控制 (chassis)
| 函数 | 功能 | 返回值/示例 |
| :--- | :--- | :--- |
| `RobotMoveVector(x, y, z)` | 机体坐标系向量移动 (x/y: m/s, z: deg/s) | 无 |
| `FieldMoveVector(x, y, facing, kp=None)` | 场地坐标系向量移动 (x/y: m/s) + 航向闭环 | 无 |
| `FieldTurn(angle)` | 原地转向到场地绝对角度 | 无 |
| `stop()` | 停止所有电机 | 无 |

#### ⚽ 外设控制 (peripheral)
| 函数 | 功能 | 返回值/示例 |
| :--- | :--- | :--- |
| `ShootBall()` | 触发电磁铁脉冲踢球 | 无 |
| `Dribble(status)` | 设置运球器状态 (True:开, False:关) | 无 |

#### 🧠 锁球控制 (lockball)
| 函数 | 功能 | 返回值/示例 |
| :--- | :--- | :--- |
| `LockBallSlip()` | 自动追踪并锁定球 (非阻塞) | 无 |

---

## 🛠️ 典型应用场景

### 场景 A: 简单追踪球并踢球
这是最基础的进攻逻辑：
```python
from LunaPre import *
from BasicFuc import lockball

if UnitedPosition.RestrictedArea(): # 确保不在限制区/撞墙保护
    ball_pos = UnitedPosition.RobotBallPos()
    ball_state = UnitedPosition.BallState(ball_pos)
    
    if ball_state == "lost":
        UnitedPosition.Pos2Pos([0, 0, 0])
    elif ball_state == "held":
        peripheral.Dribble(0.7) # 开启运球/吸球
        peripheral.ShootBall()  # 触发踢球
    else:
        lockball.LockBallSlip() # 自动对准并锁定球 (非阻塞)
```

### 场景 B: 场地走点 (Waypoints)
可以通过 `Move2Path` 实现多个路点的路径规划：
```python
# 示例：依次经过五个点
UnitedPosition.Move2Path([
    [40, 40, 0], 
    [-70, 50, 80], 
    [-80, -50, 180], 
    [80, -50, 270], 
    [80, 50, 0]
], 0.8)
```

### 场景 C: 基础防御逻辑 (横向拦截)
利用场地坐标系，使机器人在球门线附近跟随球的 Y 轴位置：
```python
# 假设 pan_y 是一个 PID 控制器
ball_pos = UnitedPosition.RobotBallPos()
if UnitedPosition.BallState(ball_pos) == "free":
    # 使用 PID 计算 Y 轴移动速度，保持在球的正对面
    speed_y = pan_y(ball_pos[1]) 
    # 保持航向角为 0 (面向对方球门)，沿 Y 轴进行 PID 修正移动
    chassis.FieldMoveVector(0, -speed_y, 0) 
```

## 🧭 策略文件组织建议

比赛策略应该优先让人能顺着 `Step()` / `step()` 从上到下读完整个决策流程。

推荐写法：

- 主流程里保留关键分支：丢球、持球、可见球、边界恢复、射门/拦截/回位。
- 私有函数只封装动作或几何计算，例如 `_clear_ball()`、`_intercept_target()`。
- 不要把高层决策拆成太多 `_handle_xxx()`，否则研究员很难直接判断机器人下一步会做什么。
- 多个策略共用的哨兵值和小工具放在 `LunaPre.strategy_tools`，不要在每个策略文件里重复定义。

简化判断标准：如果读者只看一个 `step()` 函数，就应该能大致复述机器人一轮控制周期的行为。

## 🧪 真实场景调试

`src/tools/tests/sim_robot_role_runner.py` 可以在 Unity 场景里直接跑单个角色，并输出 JSONL trace：

```bash
PYTHONDONTWRITEBYTECODE=1 XEL_SIM_BALL_STATUS_FALLBACK=1 \
venv/bin/python src/tools/tests/sim_robot_role_runner.py \
  --robot-id 1 --role offence --duration 8 \
  --trace Records/sim_runs/offence_real.jsonl \
  --summary Records/sim_runs/offence_real.summary.json \
  --require-motion --min-motion 5 --min-command-frames 3
```

进攻策略的 trace 会额外写入 `offence_decision` 字段。重点看这些值：

- `action`: 本 tick 的实际策略动作，例如 `chase_ball`、`attack_dodge`、`quick_shot`。
- `shot_gate`: 主动射门门控原因，例如 `open_goal`、`quick_zone`、`opportunity_not_ready`。
- `force_gate`: 被贴防/受压时的强制射门门控原因，例如 `forced_shot_before_threat_zone`。
- `progress` / `y` / `yaw_error` / `lane_score`: 判断射门是否合理的核心几何量。

当前 offence 约定：空门或开阔线路可以提前射门；被对手贴近时，必须至少推进到威胁区域，不能在己方半场或刚过中线因为“有人贴近”就乱射。

分层约定：`offence.py` 只写比赛策略和动作选择；Unity 状态包缺字段、持球漏报、敌人过滤、坐标兜底这类仿真适配问题放在 `src/utils/XeleratorSimBits.py` 或测试 runner 里处理。

## 模拟器四档弹射

SIM 模式下，`Bits.Kick(1)` 至 `Bits.Kick(4)` 分别使用 Unity 机器人配置
`kick_speed_m_s` 的 1/2、2/3、5/6、1。配置 6 m/s 时就是 3、4、5、6 m/s。
`SetKickPowerLevel(level)` 设置后续默认档位；省略档位时仍使用该默认值。
适配器发送归一化比例，旧 `Sim.KickStrength` / `XEL_SIM_KICK_STRENGTH`
不再重复放大 Unity 的速度设置。请与支持该语义的模拟器版本一起更新。

## ⚠️ 重要提示
- **坐标系约定**：
    - **机体坐标系**：$x$ 为前， $y$ 为左。
    - **场地坐标系**：原点在场地中心，方向定义见 `docs/interface_semantics.md`。
- **安全停止**：在策略脚本的 `finally` 块中，务必调用 `chassis.stop()` 和 `UnitedPosition.Stop()` 释放资源。
- **球状态**：新策略优先用 `UnitedPosition.BallState()` 判断，返回 `"lost"` / `"held"` / `"free"`；需要常量时从 `LunaPre.strategy_tools` 导入 `BALL_NOT_FOUND` / `BALL_HELD`，不要在新代码里硬编码原始哨兵。

## 许可证

团队自有代码采用 [Apache License 2.0](LICENSE)。第三方组件保留其原始许可证，例如 `Tools/LICENSE`；版权与归属说明见 [NOTICE](NOTICE)。
