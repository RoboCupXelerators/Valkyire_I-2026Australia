# 澳大利亚实机策略审查与改写说明

## 交付结论

本次按“保留原战术行为、改为无参数启动、协同不可用时自动降级”的方向改写。新入口位于 `src/strategy_real_australia.py`，原始下载文件没有覆盖。

- 启动不解析命令行参数。机器人编号取 `RoboInfo.number`；当前默认配置为 1。若两台机器人同时协同，需在各自设备的 `config.json` 中分别设为 1 和 2。
- 赛制/物理/战术参数内联在 `src/strategy_real_australia.py` 顶部的 `AUSTRALIA` 命名空间中；不再写入 `ReasonData` 配置，避免污染运行时 `config.json`。
- 无 Team Link API、配置关闭 Team Link、启动后 1 秒仍未配对，或已配对后心跳连续丢失 0.75 秒，都会进入单机接管并清除传球等协同计划。心跳恢复后允许重新启用双机战术。
- 视觉有效期和包间隔从 `Positions.VisionRxAge()` / `VisionRxIntervals()` 读取，只认成功接收的 UDP 包；超时发布的空字典不会刷新有效期。

## 原文件发现

`<downloads>/strategy_real_australia.py` 有 8,549 行、281 个顶层函数定义和 430 个不同的顶层赋值名。`reset_strategy()` 重复定义两次，后定义通过别名调用前一实现。文件包含约 4,800 行生成的澳大利亚战术图，以及物理执行、安全边界、持球恢复、双机协议、遥测和进程启动代码。

这不是单纯的包装函数膨胀：生成战术图包含传球协商、争球、澳门进攻、墙反弹、死球和多阶段防守。Software 里现有 `offence.py`、`defence.py` 和 `candidate_coordinated_lanes.py` 没有这些分支的等价实现。直接换成现成策略会改变比赛行为。

## 本次代码调整

- 移除参数解析、`--robot-id`、`--solo`、`--check` 和同时启动两台进程的 `run_all()`。入口只调用 `main()`；身份及运行配置读 JSON。
- 将可调阈值、速度、距离、超时和角色表内联为策略文件顶部的 `AUSTRALIA.Real` / `Team` / `Tactics` / `Runtime` 命名空间字段，不再污染 `ReasonData` 配置。机器人实时状态仍保留在模块变量中，因为它们会在控制循环中持续变化。
- 合并两个 `reset_strategy()`，删除为了串接两个重置函数而设置的 `_TACTICAL_RESET` 别名。
- 去掉策略文件向 `sys.path` 插入路径的代码；运行位置遵循项目约定，从 `Software/src` 启动。
- 删除策略里重复实现视觉接收新鲜度的类，改用 `LunaPre.positions.Positions.VisionRxAge()` 和 `VisionRxIntervals()`。
- 保留原 16 字节版本化 Team Link 报文。当前 `LunaPre.Communication` 是 10 字节旧协议，直接替换会导致两端协议不一致。
- 沿用 LunaPre 的 `safe_stop`、位置与裁判接口、底盘安全检查和关闭流程。

## 500 行目标

当前保留完整战术分支的策略文件约 7,574 行，尚未达到 500 行。把代码搬到其他文件只能缩短入口，不能减少实现总量；改用现有通用策略则会丢失原战术。若必须让总实现小于 500 行，需要先接受战术行为删减，或把现有 Software 运行时扩展为可复用的持球执行器、传球协议和 Australia 战术组件，再逐项验证行为一致。本次优先避免把简化版误标成逻辑等价版。

## 配置位置

- 机器人编号：`RoboInfo.number`
- 场地尺寸：`Field.Long` / `Field.Short`（现已为 243 / 182）
- 球门与球径：`Vision.GoalWidth` / `Vision.GoalHeight` / `Vision.BallDiameter`（现已为 45 / 14 / 4.2）
- 是否尝试双机链路：`AUSTRALIA.Runtime.TeamLinkEnabled`
- 墙线、边界和实机控制限值：`AUSTRALIA.Real`
- 配对超时、心跳超时、传球和攻防参数：`AUSTRALIA.Team` / `AUSTRALIA.Tactics`
- 其余参数定义：`src/strategy_real_australia.py` 顶部的 `AUSTRALIA` 命名空间

`src/ReasonData/data/config.json` 是本地运行配置目录，受 `.gitignore` 忽略。场地/球门几何归入公共 `Field` / `Vision` 段；其余澳洲策略参数不再写入该文件。旧配置迁移时会一次性采纳上面的实机几何。

## 静态检查与限制

已完成 Python AST 语法解析、配置 JSON 解析、214 个代码配置属性引用与默认值键核对，以及 `git diff --check`。没有运行 pytest，也没有接入实机；硬件 I/O、相机、裁判锁和双机无线链路仍需在目标设备上确认。

## 启动

```bash
cd Software/src
python strategy_real_australia.py
```
