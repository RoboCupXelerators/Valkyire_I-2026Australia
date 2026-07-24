# DM2325 架构边界与历史梳理

本文记录 DM2325/C 型板相关代码的来源、当前职责和后续迁移边界。它是维护约束，不是硬件验收报告。

## 1. 历史问题

`e359215dd92c156c005b9743653a7c7d3688f7a8`（2026-07-20）一次加入了以下互不相同的内容：

- C 型板协议、状态机、安全和硬件验收报告；
- Raspberry Pi USB CDC 主机驱动 `src/utils/DM2325Bits.py`；
- LunaPre 底盘/IMU 适配；
- 遥操作入口和 Control Panel 扩展；
- 第二套独立 `tools/imu_web` 控制页面及网页专用 JSON 流；
- 指向当时并不存在的 `tools.motor_control` 的脚本。

该提交的提交信息也是生成过程残留，而不是可读的工程摘要。后续提交
`50b61d534fd1fb9d6281becfa7662aeae26603f8` 又把视觉和激光雷达修改叠加在同一分支上。
因此，Git 提交边界不能再当作模块边界。

当前工作区没有找到 STM32 `.c`、`.h`、`.ioc` 或 CMake 工程。已有报告提到的
`bmi088_driver`、`cboard_imu` 和四电机固件实现并不在 Software 中。报告只能作为历史证据，不能替代可构建源码。

## 2. 当前唯一运行路径

```text
Strategy / Control Panel
          |
          v
LunaPre facade and runtime
          |
          v
DM2325Bits (host protocol and typed state)
          |
          v
USB CDC transport
          |
          v
C-board firmware
```

- 策略只调用 LunaPre 暴露的底盘、IMU 和外设接口。
- `src/LunaPre/runtime.py` 选择 DM2325 backend，并注入 callable。
- `src/utils/DM2325Bits.py` 是 Software 内唯一的 C 型板主机驱动，负责帧编解码、串口事务、状态对象和安全生命周期。
- `src/control_panel.py` 是唯一网页控制后端，负责人工控制租约、展示层 JSON 和页面状态。
- `web/control_panel/` 是唯一网页界面。
- 仓库根目录不提供 `LunaPre` 兼容包装；运行时只从 `src/LunaPre` 导入。

## 3. 仓库职责

| 内容 | Software | C-type-Firmware |
| --- | --- | --- |
| STM32 源码、链接脚本、Cube 配置 | 禁止 | 唯一所有者 |
| CDC 消息号、payload 和状态机的固件实现 | 消费契约 | 唯一实现者 |
| 主机串口 transport 和 Python codec | 唯一所有者 | 不复制 |
| LunaPre/策略适配 | 唯一所有者 | 禁止 |
| 底盘几何和主机侧运动语义 | 唯一所有者 | 只消费生成契约 |
| 操作员 Control Panel | 唯一所有者 | 禁止 |
| 板级 build/flash/USB/硬件验收记录 | 迁出后仅链接 | 唯一所有者 |
| 固件开发用只读诊断工具 | 不维护副本 | 所有者 |

`C-type-Firmware` 当前由另一个任务维护。未协调前，不从本仓库跨仓库移动文件，也不修改该仓库。

## 4. imu_web 决策

独立 `tools/imu_web` 已从 Software 删除，不原样迁移。

原因：它直接导入 `DM2325Bits`、独占同一串口，并实现 Arm、Move、Stop、Disable、浏览器 deadman 和底盘遥测。这些功能与 Control Panel 重复，且把网页展示、主机协议和电机安全状态机绑定在一个文件中。迁入固件仓库只会把依赖方向反转为“固件仓库依赖 Software”。

保留能力的方式：

- `DM2325Bits.GetIMUState()` 继续返回类型化的硬件状态；
- `/api/control/telemetry` 在 motor telemetry 旁返回可选的 `imu` 字段；
- Control Panel 统一显示 IMU、底盘和外设状态；
- IMU 不可用时仅标记 `imu.available=false`，不影响四电机反馈；
- JSON 非有限值在展示边界转换为 `null`，不污染驱动协议层。

如果固件仓库以后需要 bring-up viewer，应使用固件仓库自己的测试协议客户端，并保持只读。它不得包含 Arm/Move 等机器人控制功能，也不得导入 Software。

## 5. 待协调迁移清单

以下内容语义上属于固件/硬件证据，但在固件源码恢复并确认目录之前暂留 Software，避免丢失：

- `docs/cboard_imu_final_report.md`
- `docs/four_motor_protocol.md`
- `docs/four_motor_state_machine.md`
- `docs/four_motor_safety.md`
- `docs/four_motor_id_mapping.md`
- `docs/four_motor_zero_output_verified.md`
- `docs/four_wheel_final_report.md`
- `scripts/capture_c_board_usb_snapshot.sh`
- `scripts/watch_c_board_usb.sh`

`docs/dm2325_capabilities.md`、`docs/dm2325_diagnostics.md`、
`docs/dm2325_driver_status.md`、`config/chassis_geometry.json` 和
`docs/nonorthogonal_*` 描述主机驱动或机器人几何，继续由 Software 维护。

## 6. 禁止事项

- 不再增加第二套 DM2325 网页、串口 owner 或电机 deadman。
- 不在 `DM2325Bits.py` 放 HTML、HTTP、SSE 或网页专用 JSON 代码。
- 不让策略直接导入 `DM2325Bits` 或读写串口。
- 不复制消息常量和 payload decoder 到 Control Panel。
- 不把缺失的固件源码描述为“已迁移”或“可构建”。
- 不改写已有硬件证据；迁移时保留原内容并记录来源 commit。

## English Summary

Software owns the Python host driver, LunaPre adapter, robot semantics, and the single operator Control Panel. C-type-Firmware must own buildable MCU sources, firmware-side protocol/state-machine code, and board-level evidence. The former standalone `imu_web` duplicated motor control and serial ownership, so it was removed rather than copied. IMU telemetry now reaches the existing Control Panel through a presentation-only adapter; the typed protocol state remains in `DM2325Bits`.
