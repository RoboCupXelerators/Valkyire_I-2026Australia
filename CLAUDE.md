# CLAUDE.md

## House Rules for AI

1. 这是 Jetson 上的 **30 Hz 控制循环**；任何建议都必须优先保证实时性和安全停止。
2. **不要新增 class / dataclass / enum / 抽象层**；确需复用时优先写模块级函数或复用现有函数。
3. 主循环里不要每帧分配大对象、不要每帧 `print` / 打日志；调试输出必须节流。
4. 保持最小 diff，不重写已工作的模块，不回滚或覆盖他人的未提交修改。
5. 球状态使用 `LunaPre.strategy_tools` 里的 `BALL_NOT_FOUND` / `BALL_HELD`，分支判断优先用 `UnitedPosition.BallState()`（返回 `"lost"` / `"held"` / `"free"`），不要在新代码里硬编码十六进制哨兵。
6. 写路径和 API 名前先核对真实源码；例如定位方法统一写 `UnitedPosition.FieldRobotPose()`。
7. 策略示例必须包含 `try/finally`，退出时调用 `chassis.stop()`、必要时关闭 `peripheral.Dribble(False)` 并 `UnitedPosition.Stop()`。

## 当前入口速记

- 策略常用 facade：`from LunaPre import *`
- 锁球/带球转向：`from BasicFuc import lockball, TurnTo_with_ball`
- 哨兵常量：`from LunaPre.strategy_tools import BALL_NOT_FOUND, BALL_HELD`
- 当前结构总览：`STRUCTURE.md`
- 接口语义：`docs/interface_semantics.md`
