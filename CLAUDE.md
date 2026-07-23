# CLAUDE.md

## House Rules for AI

1. 这是 Jetson 上的 **30 Hz 控制循环**；任何建议都必须优先保证实时性和安全停止。
2. **不要新增 class / dataclass / enum / 抽象层**；确需复用时优先写模块级函数或复用现有函数。
3. 主循环里不要每帧分配大对象、不要每帧 `print` / 打日志；调试输出必须节流。
4. 保持最小 diff，不重写已工作的模块，不回滚或覆盖他人的未提交修改。
5. 球状态使用 `LunaPre.strategy_tools` 里的 `BALL_NOT_FOUND` / `BALL_HELD`，分支判断优先用 `UnitedPosition.BallState()`（返回 `"lost"` / `"held"` / `"free"`），不要在新代码里硬编码十六进制哨兵。
6. 写路径和 API 名前先核对真实源码；例如定位方法统一写 `UnitedPosition.FieldRobotPose()`。
7. 策略示例必须包含 `try/finally`，退出时调用 `chassis.stop()`、必要时关闭 `peripheral.Dribble(False)` 并 `UnitedPosition.Stop()`。

8. 所有新写或修改的应用代码、策略、可执行模块、CLI 入口和平台适配器必须放在 `src/` 下；不得在仓库根目录或 `scripts/`、`tools/`、`web/` 中新增实现代码。
9. 运行 LunaPre、策略和模块时必须以 `src/` 为工作目录和 import root，例如 `cd src && python -m ...`。不要执行仓库根目录的旧副本，也不要用根目录 `sys.path` hack 绕过布局。
10. `src/LunaPre/` 是本项目核心源码，不是溢出目录或无关改动；涉及它的修改必须按核心运行时处理。
11. 测试按平台选择。默认跑通用或 SIM 测试；DM-S2325、IMU、真实相机和 Jetson 专项测试按 pytest marker 单独执行，不要把未运行的专项测试报告成已验证。
## 当前入口速记

- 策略常用 facade：`from LunaPre import *`
- 锁球/带球转向：`from BasicFuc import lockball, TurnTo_with_ball`
- 哨兵常量：`from LunaPre.strategy_tools import BALL_NOT_FOUND, BALL_HELD`
- 当前结构总览：`STRUCTURE.md`
- 接口语义：`docs/interface_semantics.md`
