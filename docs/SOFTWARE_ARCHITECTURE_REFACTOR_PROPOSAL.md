# Software Architecture Refactor Proposal

状态: Draft v1
日期: 2026-06-21
适用范围: `Software/src/LunaPre/`、`Software/src/utils/*Bits.py`、策略模块与调试工具

## 1. 背景

当前软件栈的问题不是某个函数写得不够好，而是历史分层主要来自“某段时间新增了哪些功能”，不是来自清晰的职责边界。

典型例子是 `chassis.py`：

- 早期 `MotionRecorder` 引入前约 207 行，主要包含 `Car` / `Peripherals` / `Key`。
- `bf04439 feat(data): add motion recording for match replay` 后增加约 211 行，记录逻辑进入 `chassis.py`。
- 后续又加入状态同步线程、牵引控制、轮速限幅、轮速分配等逻辑。
- 当前 `chassis.py` 同时承担运动 API、轮速分配、运动保护、记录、状态同步、外设与按键，阅读和维护成本明显过高。

这类问题也出现在定位、仿真、真机 Bits、策略输入等路径中：功能是逐步叠上去的，但边界没有同步收敛。

## 2. 重构目标

本轮重构的目标不是“删代码”，而是重新定义软件栈的职责边界，让后续开发更可维护。

核心目标：

- 策略层只表达比赛意图，不碰轮速、电机端口或 Bits 细节。
- 运动层负责把策略意图翻译成底盘速度或轮速。
- Bits 层只负责上下位机通信和硬件能力暴露。
- 观测、记录、状态同步等旁路功能不能进入主控制链。
- 保持现有比赛代码可渐进迁移，不做一次性大破坏。

一句话概括：

> 策略说“我要做什么”，运动层说“怎么变成可执行运动”，Bits 层说“怎么发给硬件”。

## 3. 非目标

本提案不做以下事情：

- 不把 `chassis.stop()` 定义为真正 emergency stop。真正急停必须由物理开关或硬件禁能处理。
- 不要求所有运动 API 立即改成场地参考系。
- 不立即删除 `RobotMoveVector()` 等低层接口。
- 不把 `Pos2Pos` 下放到下位机。当前需求是 `GetPose` 在下位机，`Pos2Pos` 仍由上位机做位置闭环。
- 不在旧 `IceLoongBits` 上继续堆新协议。旧 Bits 组件应保留稳定，新标准应通过 fork / 新 Bits 组件实现。

## 4. 建议目标分层

```text
strategy 层
  offence / defence / alpha / lambda
  只表达比赛意图，不直接接触 Bits / 电机 / 记录系统

motion intent 层
  场地参考系运动、面向目标、Pos2Pos
  这是策略推荐入口

chassis 层
  统一底盘运动语义
  将运动意图转换为机器人速度或轮速
  提供普通软件 stop

motion backend 层
  轮速混控、限幅、滤波、牵引控制
  未来可切换为下位机速度控制

bits 层
  上下位机通信
  SetWheelSpeed / GetYaw / GetPose / Kick / Dribble

localization 层
  Bits.GetPose 优先
  Sensor.Lidar + DirectDistance 兜底

observability 层
  MotionRecorder / robot_state / control_panel telemetry
  只观察或订阅，不进入主控制链
```

## 5. 关键边界规则

### 5.1 策略层

策略模块推荐使用场地参考系表达意图：

- 去某个场地位置
- 面向球门或目标点
- 防守某条线
- 执行 `Pos2Pos`

策略层不应直接调用：

- `Bits.SetWheelSpeed`
- 电机端口映射
- 原始四轮速度
- 记录器或 telemetry 写入

### 5.2 Chassis 层

`chassis` 的意义是策略意图到安全底盘执行的翻译层。

它应该保留：

- `FieldMoveVector`
- `FieldMoveAngle`
- `FieldTurn`
- `RobotMoveVector`
- `RobotMoveAngle`
- `RobotTurn`
- `stop`
- `SetMotor`

但接口应分级：

| 级别 | 接口 | 用途 |
| --- | --- | --- |
| 推荐策略入口 | `FieldMoveVector` / `FieldTurn` / `Pos2Pos` | 比赛策略 |
| 低层接口 | `RobotMoveVector` / `RobotTurn` | 调试、视觉伺服、模型输出、采样 |
| 最低层接口 | `SetMotor` | Car 内部与硬件测试 |

不建议把机器人参考系接口一次性删除。原因：

- 控制面板、采样脚本、真机调试仍需要低层运动。
- 部分视觉伺服逻辑天然来自机器人坐标系。
- MLP legacy action 路径仍可能输出 `vx / vy / wz`。
- 定位不可用时，robot-frame 运动仍可用于底层测试。

真正要统一的是策略表达，而不是砍掉底层能力。

### 5.3 Stop 语义

`chassis.stop()` 是普通软件停止命令，不是 emergency stop。

推荐规则：

```text
chassis.stop() is a normal software stop command, not an emergency stop.
Physical emergency stop must be handled by hardware.
Software motion guards may refuse or zero motion commands, but must not be documented as emergency stop.
```

软件 stop 只负责：

- 发送零速度或零轮速
- 清理上位机速度缓存
- 用于策略切换、目标丢失、任务结束、普通停车

### 5.4 Bits 与定位

当前定位方向：

- 下位机未来可能完成雷达解析与定位，输出 `[x, y, yaw]`。
- 上位机仍负责 `Pos2Pos` 闭环。
- 旧 Bits 组件保留，不把新协议直接塞进历史 `IceLoongBits`。
- 新下位机协议应 fork / 新建一个新标准 Bits 组件，并按统一标准暴露能力。

因此职责应为：

- 新 Bits 标准：暴露下位机姿态入口 `GetPose()`，返回 `[x, y, yaw]`。
- 旧 `IceLoongBits`：保持现有命令和数据结构稳定，用于已有机器人和 demo。
- `LunaPre.Positions.FieldRobotPos()`：姿态兼容入口。
- `Sensor.Lidar.GetDists()`：只负责距离来源兜底，不承担姿态语义。

fallback 顺序：

```text
FieldRobotPos()
  -> try Bits.GetPose()
  -> if valid: return [x, y, yaw]
  -> else: DirectDistance()
  -> Sensor.Lidar.GetDists()
  -> compute field pose from wall distances
```

### 5.5 Observability

记录、回放、状态同步、控制面板 telemetry 都属于旁路能力。

它们可以读取运动输出或订阅状态，但不应成为 `Car` 的主体职责。

建议迁移方向：

- `MotionRecorder` 从 `chassis.py` 移出。
- `robot_state` 更新线程从 `Car.__init__` 移出，改成显式启动或订阅式更新。
- `control_panel` 不应要求 `Car` 自动承担 telemetry 生命周期。

## 6. 当前主要问题

### 6.1 `chassis.py` 职责过宽

当前 `Car` 同时做：

- 对外运动 API
- 速度滤波
- 轮速限幅
- 轮速混控
- 输出记录
- 状态线程更新
- 停止缓存清理

这些职责混在一起后，新增功能很容易继续堆进去。

### 6.2 内部 helper 过多且语义不明显

`_filter_chassis_command`、`_apply_wheel_slew_limit`、`_allocate_wheel_mix` 等不是死代码，但它们的领域是 motion backend，不是 `Car` 对外语义。

它们应该被归类并抽出，而不是继续散在 `Car` 中。

### 6.3 历史 wrapper 存在语义风险

`RobotXMove` / `RobotYMove` 等历史接口可读性差，并且存在签名不匹配风险。

这类接口应该：

- 标记为 legacy。
- 查清外部调用。
- 若无人使用，逐步删除。
- 若仍需保留，改成明确语义或加测试约束。

## 7. 建议迁移顺序

### Phase 0: 文档与共识

目标：

- 团队确认分层边界。
- 明确 `stop`、`Field*`、`Robot*`、`Bits.GetPose` 的语义。
- 决定哪些 API 是策略推荐入口，哪些是低层调试入口。

产出：

- 本文档定稿。
- 更新 `docs/interface_semantics.md` 中相关术语。

### Phase 1: 下位机定位接口占位

目标：

- 定义新 Bits 标准中的 `GetPose()` 能力。
- fork / 新建符合新标准的 Bits 组件，旧 `IceLoongBits` 保留稳定。
- 不在旧 `IceLoongBits` 中修改串口协议和数据结构。
- 在 `LunaPre.Positions.FieldRobotPos()` 中强化 `Bits.GetPose()` 合法性检查和 fallback。

约束：

- `Pos2Pos` 仍在上位机。
- `LunaPre.sensor` 仍只做距离兜底。

### Phase 1.5: LunaPre package 化

目标：

- 保留 `from LunaPre import *`、`from LunaPre import UnitedPosition, chassis` 这类历史入口。
- 将 `LunaPre.py` 从单文件模块升级为 `LunaPre/` package。
- 按职责拆分 runtime 初始化、定位/运动封装、队内通信、记录器。
- 将 motion、sensor、vision facade 收敛到 `LunaPre/` 下；`src/` 顶层不保留 `chassis` / `Sensor` / `Vision` 兼容门面。
- `src/` 顶层只保留策略、control panel、MLP/teacher 等独立库或入口。
- SIM runtime 细节由 `utils.XeleratorSimBits.create_runtime()` 负责，`LunaPre.runtime` 只选择 backend 并挂接通用 `chassis/peripheral/key`。

当前分支实施：

```text
src/LunaPre/
  __init__.py          兼容旧顶层导出
  runtime.py           Bits / chassis / lidar / peripheral 初始化
  sensor.py            Lidar / ROS2 sensor facade
  vision.py            UnitedVision facade
  positions.py         Positions 主体逻辑
  position_recorder.py UnitedPosition 的外挂记录器包装
  communication.py     队内通信协议
  motion/
    chassis.py         Car / Peripherals / Key
    recording_chassis.py
    motion_backend.py
    motion_recorder.py
    robot_state_sync.py
```

记录器规则：

- `Positions` 不直接写 CSV、不持有记录生命周期。
- 默认导出的 `UnitedPosition` 使用 `RecordingPositions()`，记录器作为外挂类覆盖 `Pos2Pos`。
- 后续其它记录器也应按 wrapper / subclass / observer 方式挂载，不进入业务主体类。

### Phase 2: 移出旁路功能

目标：

- 把 `MotionRecorder` 从 `chassis.py` 移到独立模块。
- 将状态同步线程从 `Car.__init__` 移出，避免创建 `Car` 自动产生隐藏线程。

验收：

- `chassis.py` 不再直接定义记录器。
- `Car` 构造不启动非运动必需的后台线程。
- 原有记录功能通过显式入口或订阅机制保留。

当前分支实施：

- `MotionRecorder` 已拆到 `src/LunaPre/motion/motion_recorder.py`。
- 默认 runtime 使用 `src/LunaPre/motion/recording_chassis.py` 中的 `RecordingCar` 挂载记录器，`Car` 本体只保留 motor output hook。
- `robot_state` 同步线程已拆到 `src/LunaPre/motion/robot_state_sync.py`。
- `Car.__init__` 默认不再自动启动状态同步；`LunaPre` runtime 初始化后显式调用 `chassis.start_state_update()`。

### Phase 3: 抽出 motion backend

目标：

- 把轮速限幅、牵引控制、分配策略从 `Car` 中抽出。
- 保持 `Car` 对外 API 不变。

建议模块：

```text
src/LunaPre/motion/motion_backend.py
  clamp_motor_vector
  apply_wheel_slew_limit
  filter_chassis_command
  allocate_wheel_mix
```

或先作为 `chassis.py` 内清晰分区，待稳定后再拆文件。

当前分支实施：

- 已直接拆出 `src/LunaPre/motion/motion_backend.py`。
- `Car` 保留 `_allocate_wheel_mix()`、`_angle_error_deg()` 等 legacy 私有方法名作为代理，避免旧检查脚本和临时调试代码立即失效。

### Phase 4: 策略 API 收敛

目标：

- 策略模块优先使用场地参考系 API。
- `RobotMoveVector` 保留为低层接口，不作为新策略默认入口。
- 清理或标记 `RobotXMove` / `RobotYMove` 等历史 wrapper。

验收：

- 新策略代码不直接使用 `SetMotor`。
- 新策略代码优先使用 `Field*` / `Pos2Pos`。
- robot-frame 调用仅出现在调试、采样、模型 legacy action、明确的局部视觉伺服中。

## 8. 团队评估问题

已确认前提：

- 旧 Bits 组件保留稳定，不在旧 `IceLoongBits` 上继续扩展新协议。
- 新下位机协议 fork / 新建一个新标准 Bits 组件，并用新标准重写。
- 新标准 Bits 暴露 `GetPose()` 作为下位机定位入口。
- `Pos2Pos` 继续由上位机闭环。

下周评估建议集中回答这些问题：

1. 是否认可“策略主语言使用场地参考系，robot-frame 保留为低层接口”？
2. 是否认可 `chassis.stop()` 只作为普通软件 stop，不承担 emergency stop 语义？
3. 是否接受本分支把 `MotionRecorder`、`robot_state` 同步、`motion_backend` 从 `chassis.py` 拆出？
4. 是否接受 `LunaPre.py` 升级为 `LunaPre/` package，同时保持 `from LunaPre import *` 兼容？
5. `LunaPre` 显式启动状态同步是否是合适入口，还是应该由 control panel / app runtime 自己启动？
6. `Car` 保留 legacy 私有代理方法多久合适？
7. 哪些 legacy API 可以标记废弃或删除？

## 9. 推荐决策

推荐本次重构采用保守渐进方案：

- 先定边界和文档。
- 再接入 `GetPose` 兼容。
- 然后移出记录器与状态线程。
- 最后拆 motion backend。

不建议一次性重写 `chassis.py`。它处在运动主链路上，应该通过小步迁移、行为等价测试和真机确认来降低风险。

最终目标不是把代码拆得更多，而是让每一层都有清楚的意义：

```text
strategy: decide what to do
motion intent: express movement in soccer terms
chassis: translate movement to executable chassis commands
motion backend: allocate and limit motor output
bits: talk to hardware
observability: watch, record, and debug
```
