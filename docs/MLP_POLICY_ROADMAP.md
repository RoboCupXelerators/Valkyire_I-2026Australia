# MLP 策略路线图

状态: Draft v1  
日期: 2026-05-18  
适用范围: `Software/src/mlp`、`Software/src/LunaPre/`、`Software/src/vision_utils/`、`SoccerSimulatorForXels`

## 1. 目标定义

本项目中的 MLP 不应被定义为“底盘动作控制器”，而应被定义为“高层策略决策器”。

最终目标:

- 在真实部署时，策略模型直接吃满 `IL + AI/TI + LunaPre` 能稳定提供的信息。
- 模型输出“下一步该做什么”，而不是直接输出最终电机控制。
- 底盘跟踪、转向、追球、靠近、面向目标等连续控制细节交给 `PID + skill executor`。
- Unity 模拟器仅作为训练与验证基础设施，不应成为部署输入接口的真相来源。

一句话概括:

> 模型负责“决策”，规则/PID 负责“执行”。

## 2. 核心原则

### 2.1 部署优先

训练时 actor 能看到的观测，必须尽可能等价于真实比赛中 `LunaPre/UnitedPosition` 能拿到的观测。

禁止:

- actor 直接读取 simulator 真值坐标作为正式训练输入。
- actor 依赖真实部署时拿不到的字段。
- actor 直接输出轮速或原始电机级控制。

允许:

- trainer / critic / reward 模块使用 simulator 真值。
- teacher policy 使用 simulator 真值。
- imitation 数据可以从规则策略或真值标注生成。

### 2.2 分层控制

控制栈应拆成 4 层:

1. 传感器层: `IL + AI/TI + LunaPre`
2. 观测适配层: 将原始观测编码为策略输入
3. 高层策略层: 输出技能选择与子目标
4. 低层执行层: `PID + chassis + BasicFuc/skill executor`

### 2.3 模拟器双视角

模拟器对训练应同时提供两种信息面:

- `policy view`: 部署对齐视角，只暴露真实比赛可见的观测
- `truth view`: 仿真真值视角，用于 reward、终局判断、teacher、debug

### 2.4 部分可观测

真实系统并不是全知全能。

因此最终策略不要假设:

- 球永远可见
- 对手永远可见
- 检测永远无延迟
- 定位永远无误差

如果继续坚持纯前馈 MLP，至少要支持:

- 最近若干帧堆叠
- 数据年龄 / 可见性标记
- 上一步动作

中期建议预留到 `MLP + GRU` 或其它轻量时序结构。

## 3. 当前现状判断

### 3.1 已有真实部署侧信息接口

目前 `LunaPre.Positions` 已经暴露了高价值接口:

- `RobotBallPos`
- `FieldRobotPose`
- `FieldBallPos`
- `DirectDistance`
- `RobotChassisPos`
- `FieldChassisPos`
- `FieldGoalsPos`

这些接口是后续策略输入设计的基准面。

### 3.2 MLP 方案的关键约束

早期 `src/mlp` 偏向“动作控制器”思路:

- 输出空间是 `vx / vy / wz / kick / dribble`
- `MLPInference` 最后直接调用 `chassis.RobotMoveVector`
- `gym_env` 主要围绕低层动作执行来设计

这条 legacy action 路径仍保留为兼容模式，但长期主路线应是“高层技能决策器”。
也就是: MLP 输出 `DecisionPacketV1`，再由 `SkillExecutor` 把策略意图翻译成现有控制函数能执行的动作。

### 3.3 当前训练环境的问题

当前训练环境存在以下结构性问题:

- actor 观测没有严格对齐真实部署接口
- simulator 已有的部分状态没有完整接入 `gym_env`
- 训练控制面还缺少权威的 episode 状态回传
- 缺少 Python 侧可编程 reset 场景接口
- 训练逻辑仍偏单机器人追球/射门，不足以支持长期策略学习

## 4. 目标架构

## 4.1 策略栈结构

```text
Vision / LunaPre / Unity SIM
        |
        v
DeployObservationV1
  - self pose / velocity
  - ball rel pos + visible / age / confidence
  - enemies rel pos + visible / age / confidence
        |
        v
StateEncoder
  - 44-dim actor input
  - strict dim guard
        |
        v
MLPActor
  - legacy_action: vx / vy / wz / kick / dribble
  - decision_packet: continuous target + categorical intent heads
        |
        v
DecisionPacketV1
        |
        v
SkillExecutor
        |
        v
BasicFuc / LunaPre / chassis / peripheral
```

训练、评估和调试可以使用更丰富的真值链路，但这条链路不应直接喂给 actor:

```text
Unity truth / reset / episode state
        |
        v
TruthStateV1 / ResetScenarioV1 / EpisodeStatusV1
        |
        v
reward / critic / teacher / regression tests / debug UI
```

其中:

- `DeployObservationV1` 是部署侧和模拟器侧共同遵守的 actor 输入契约。
- `StateEncoder` 负责把稀疏、带质量标记的观测压成固定 44 维向量。
- `MLPActor` 可以走旧的低层动作模式，也可以走新的 `decision_packet` 模式。
- `DecisionPacketV1` 是中间契约，不直接等于电机指令。
- `SkillExecutor` 负责把决策翻译成当前代码库能执行的控制流程。

## 4.2 部署对齐观测契约 v1

以下字段是 actor 最终应该看到的主要输入。  
来源必须能对应到真实系统。

| 类别 | 字段 | 来源 | 备注 |
|---|---|---|---|
| 自身状态 | `self_x, self_y, self_yaw` | `FieldRobotPose` | 场地坐标 |
| 自身状态 | `self_vx, self_vy, self_wz` | `IL / chassis / estimator` | 若真实侧暂不稳定，可先降级为可选 |
| 球观测 | `ball_rel_x, ball_rel_y` | `RobotBallPos` | 不可见时使用哨兵或 mask |
| 球观测 | `ball_visible` | 由 `RobotBallPos` 推导 | 非真值 |
| 球观测 | `ball_age_ms` | 观测层新增 | 必补 |
| 球观测 | `ball_confidence` | 视觉/模拟器观测质量 | 用于 noisy vision / YOLO 置信度 |
| 球观测 | `has_ball` | `RobotBallPos == [0xDDD,0xDDD]` | 已存在 |
| 边界 | `wall_front, wall_left, wall_back, wall_right` | `DirectDistance` | lidar / stub |
| 敌方 | `enemy_i_rel_x, enemy_i_rel_y` | `RobotChassisPos` 或 `FieldChassisPos` | 最多 N 个 |
| 敌方 | `enemy_i_visible, enemy_i_age_ms, enemy_i_conf` | 观测层新增 | 必补 |
| 球门 | `blue_goal_*`, `yellow_goal_*` | `FieldGoalsPos` / vision | 建议带 visible/age |
| 队友 | `mate_i_*` | 队内通信 | 多机器人策略必需 |
| 比赛状态 | `referee_state, penalty_state, role` | IL / 通信 / 配置 | 必补 |
| 历史 | `last_decision`, `last_action`, `stuck_flag` | 策略层缓存 | 便于时序建模 |

说明:

- actor 不应直接吃 simulator 的绝对球真值。
- actor 不应默认任何观测永远存在。
- 观测契约中要系统化加入 `visible / age / confidence`。

## 4.3 trainer / critic 可见的真值契约

以下信息可以只给训练基础设施使用，不给 actor:

- 真实球绝对坐标
- 真实敌方绝对坐标与速度
- 真实队友绝对坐标与速度
- 真实是否进球 / 丢球 / 出界 / 超时
- reset 前后场景 id
- 所有动力学参数与噪声参数

用途:

- reward 计算
- episode 终局判断
- curriculum 控制
- teacher policy
- debug / 可视化 / 对照评估

## 4.4 高层动作契约 v1

策略输出不再是低层速度，而是 `Decision Packet`。

建议字段:

| 字段 | 含义 |
|---|---|
| `intent_id` | 当前要执行的技能/策略意图 |
| `target_x, target_y` | 场地目标点 |
| `target_yaw` | 希望面向方向 |
| `face_mode` | 面向球 / 面向球门 / 面向目标点 / 保持 |
| `move_gain` | 执行强度或速度权重 |
| `dribble_mode` | 关 / 弱 / 强 |
| `shoot_intent` | 不射 / 尝试射门 |
| `decision_horizon` | 本决策建议持续多久 |

建议的 `intent_id` 初始集合:

- `SearchBall`
- `ApproachBall`
- `SecureBall`
- `CarryBallToPose`
- `Shoot`
- `BlockGoalLane`
- `MarkEnemy`
- `ReturnHome`
- `AvoidWall`
- `SupportMate`

## 4.5 Skill Executor 职责

`Skill Executor` 负责把高层决策转成当前代码库能执行的动作。

执行层可以复用:

- `UnitedPosition.Pos2Pos`
- `UnitedPosition.Move2Path`
- `lockball.LockBallSlip`
- `chassis.FieldMoveVector`
- `chassis.RobotMoveVector`
- `peripheral.Dribble`
- `peripheral.ShootBall`

策略网络不负责:

- PID 参数调节
- 轮速分配
- 原地纠偏细节
- 单步避障微分控制

## 5. 模拟器需要承担的职责

## 5.1 模拟器不是部署接口

模拟器不是最终观测来源。  
它的职责是:

- 训练场景生成器
- 真值提供者
- 规则/teacher 对照环境
- 可复现的回放器

## 5.2 必须补齐的训练控制接口

Python 侧必须能够直接控制 reset 场景，而不是只发一个粗粒度 `RESET`。

最低要求:

- 设置本机位姿
- 设置球位姿、速度、自旋
- 设置敌方位姿、朝向、速度
- 设置队友位姿、朝向、速度
- 设置当前半场 / 颜色 / 开局类型
- 设置摩擦、阻尼、kick/dribble 强度、延迟
- 查询当前 episode id、result、elapsed time、penalty 状态

建议新增统一协议:

```text
RESET_SCENARIO {json payload}
GET_EPISODE_STATUS
GET_TRUTH_STATE
SET_DOMAIN_RANDOMIZATION {json payload}
```

建议 `RESET_SCENARIO` payload 至少包含:

- `seed`
- `self_pose`
- `ball_state`
- `enemies`
- `teammates`
- `field_side`
- `referee_state`
- `physics_profile`

## 5.3 必须支持两种训练模式

### 模式 A: deploy-aligned sensor mode

给 actor 的观测必须尽量模拟真实 `IL + AI/TI + LunaPre`。

需要支持:

- 遮挡
- 丢检
- 观测延迟
- 噪声
- 目标丢失后的 age 增长
- lidar 无效或异常帧

### 模式 B: privileged truth mode

给 reward / critic / teacher 用。

需要支持:

- 全场真值
- 终局事件
- 角色标签
- episode meta

## 6. 训练路线

## 6.1 阶段 0: 接口对齐

目标:

- 明确 actor 最终可见信息面
- 明确 simulator 真值面
- 明确高层动作契约

交付物:

- 观测契约文档
- 决策包契约文档
- 训练协议文档

退出条件:

- 不再把 actor 当成低层速度控制器
- 团队对 `policy view` 与 `truth view` 分离达成一致

## 6.2 阶段 1: 场景控制与 episode 协议

目标:

- 让 Python 能精确 reset 场景
- 让训练端拿到权威 episode 状态

交付物:

- Unity 训练控制接口
- Python `TrainingEnvController` 协议实现
- episode id / result / timeout / out-of-bounds 回传

退出条件:

- Python 可以构造固定开局
- Python 可以准确知道 episode 的开始、结束和结果

## 6.3 阶段 2: 观测适配层

目标:

- 做一个统一的 observation adapter
- 让模拟器输出形状与真实部署一致

交付物:

- `real observation adapter`
- `sim observation adapter`
- `sensor noise / occlusion / age` 注入机制

退出条件:

- actor 输入不再依赖 simulator-only 真值
- 同一套 encoder 能同时用于 sim 与 real

## 6.4 阶段 3: 高层技能接口

目标:

- 把动作空间从 `vx/vy/wz/kick/dribble` 升级为 `intent + target`

交付物:

- `Decision Packet`
- `Skill Executor`
- `intent_id` 枚举

退出条件:

- inference 层不再直接调用连续低层速度作为策略唯一输出
- PID/规则层接管连续跟踪与微调

## 6.5 阶段 4: imitation 预训练

目标:

- 先学会模仿已有规则策略

数据来源:

- `offence.py`
- `defence.py`
- 人工脚本 / 专家策略
- 可选 teacher policy

交付物:

- 数据采集脚本
- 行为克隆 baseline
- 离线评估指标

退出条件:

- 模型在常见开局下能复现规则策略主干行为

## 6.6 阶段 5: RL 微调

目标:

- 在 imitation 的基础上做 RL 提升

建议方案:

- actor 使用 deploy-aligned observation
- critic 使用 privileged truth
- curriculum 从单球追踪逐步扩展到防守、逼抢、反击

退出条件:

- 比 imitation baseline 更稳定
- 对噪声、遮挡、初始位姿变化更鲁棒

## 6.7 阶段 6: 多智能体与 self-play

目标:

- 让策略真正具备博弈能力，而不是只会追球

需要支持:

- 多机器人同时训练
- 队友通信输入
- 对手策略池
- self-play / league training

退出条件:

- 面对固定脚本与历史策略能稳定提升胜率

## 6.8 阶段 7: 真实部署验证

目标:

- 在 `IL + AI/TI` 环境中验证训练结果

验证维度:

- 掉检时是否稳定
- 遮挡时是否稳定
- 传感器延迟是否导致行为崩溃
- 半场切换 / 颜色切换是否稳定
- 真实底盘与模拟器动力学差异是否可接受

退出条件:

- 真实机行为与仿真行为的策略意图基本一致

## 7. 模型形态建议

短期:

- 可以保留 `MLP` 名义
- 但至少支持多帧堆叠与上一步动作输入

中期:

- 升级为 `MLP + GRU`

长期:

- actor 负责高层意图
- 可选加入 tree search / option policy / planner head

注意:

- “像 AlphaGo 一样”不应理解为一开始就做复杂搜索。
- 这里的真正对应物是“策略层决定下一步意图”，而不是“神经网络直接控制执行器”。

## 8. 不应该做的事情

- 不要把 simulator 真值直接喂给最终 actor 再拿去真机部署
- 不要继续让策略网络直接学习轮速级控制
- 不要在观测契约没定之前就大规模调 reward
- 不要在没有 reset 协议的情况下做大量 RL 训练
- 不要把高层策略性能问题归因于 PID 未调好

## 9. 多 agent 协作建议

后续如需多个 agent 并行协作，推荐按“文件写入边界”拆分。

### Agent A: 观测契约与编码

负责:

- `src/mlp/state_encoder.py`
- 新增 `src/mlp/observation_contract.py`
- `LunaPre` 侧真实可用字段梳理

目标:

- 定义 deploy-aligned observation v1
- 补 `visible / age / confidence`

### Agent B: Unity 训练协议

负责:

- `SoccerSimulatorForXels/Assets/Scripts/XelTrainingEnv.cs`
- `src/mlp/gym_env.py` 中的 `TrainingEnvController`

目标:

- 实现 `RESET_SCENARIO`
- 实现 episode 状态回包
- 实现 truth state 拉取

### Agent C: 高层动作与执行器

负责:

- `src/mlp/inference.py`
- 新增 `src/mlp/skill_executor.py`
- 必要时补 `BasicFuc.py`

目标:

- 把动作空间改成 `intent + target`
- 完成 skill executor

### Agent D: imitation 数据与基线

负责:

- 新增数据采集脚本
- 规则策略 rollout 采样
- 行为克隆 baseline

目标:

- 快速产出第一版“会踢”的策略基线

### Agent E: RL 训练与评估

负责:

- `src/mlp/train.py`
- `src/mlp/model.py`
- 评估脚本与 TensorBoard 指标

目标:

- 做 actor/critic 分视角训练
- 建立 curriculum 与 checkpoint 评估

### Agent F: simulator 拟真增强

负责:

- 遮挡、噪声、延迟、domain randomization
- 多机器人场景与 self-play 训练支撑

目标:

- 缩小 sim-to-real gap

协作规则:

- 尽量让 agent 拥有互不重叠的写入文件集合
- 文档与协议先行，训练和模型修改后做
- 每个 agent 交付时必须说明“对 actor 可见信息面是否有变化”

## 10. 里程碑验收标准

### M1: 接口统一

- 有正式 observation contract
- 有正式 decision packet contract
- 有 reset / episode / truth 协议

### M2: 策略分层

- actor 不再直接输出低层连续控制为唯一接口
- skill executor 可稳定运行

### M3: baseline 可用

- imitation baseline 能稳定完成常规追球、拿球、射门
- 能处理一定噪声和遮挡

### M4: RL 提升

- RL 微调后优于 imitation baseline
- 常见开局更稳健

### M5: 多智能体

- 支持队友/敌人博弈
- 支持 self-play 或 league

### M6: 真机闭环

- 在 `IL + AI/TI` 下策略意图稳定
- sim-to-real 差距可接受

## 11. 下一步建议

按优先级，下一批工作建议是:

1. 先定义 observation contract v1 与 decision packet v1
2. 再补 Unity/Python 的 `RESET_SCENARIO + GET_EPISODE_STATUS`
3. 然后把 `src/mlp` 从低层动作空间改成高层意图空间
4. 之后再做 imitation baseline
5. 最后才进入 RL 微调与 self-play

如果顺序反了，后面大概率会反复返工。

## 12. 当前契约落地状态

**更新日期**: 2026-06-19
**里程碑**: M2 Decision Packet + observation-quality foundation

### 12.1 已完成

| 文件 | 状态 | 说明 |
|------|------|------|
| `src/mlp/contracts.py` | ✅ 已落地 | `DeployObservationV1`、`DecisionPacketV1`、truth/reset/episode 契约 |
| `src/mlp/state_encoder.py` | ✅ 已落地 | 支持 `DeployObservationV1`，默认 44 维并做严格维度检查 |
| `src/mlp/model.py` | ✅ 已落地 | 支持 `legacy_action` 与 `decision_packet` 两种输出模式 |
| `src/mlp/inference.py` | ✅ 已落地 | 支持 `DecisionPacketV1` 推理、固定频率循环和技能执行入口 |
| `src/mlp/skill_executor.py` | ✅ 已落地 | 把高层决策包翻译到现有控制栈 |
| `src/tests/test_mlp_contracts.py` | ✅ 已落地 | 契约、编码器和基础推理测试 |
| `src/tests/test_mlp_output_modes.py` | ✅ 已落地 | 输出模式、决策包解析和推理循环测试 |

### 12.2 已定义契约

| 契约 | 状态 | 说明 |
|------|------|------|
| `DeployObservationV1` | ✅ 完成 | actor 观测契约，包含球/敌方 visible、age、confidence |
| `TruthStateV1` | ✅ 完成 | trainer/critic 真值契约 |
| `DecisionPacketV1` | ✅ 完成 | 高层决策包契约，可由 MLP 原生输出 |
| `EpisodeStatusV1` | ✅ 完成 | episode 状态契约 |
| `ResetScenarioV1` | ✅ 完成 | 场景重置契约 |

### 12.3 已接入与待继续对接

以下字段已经进入 actor 契约与编码器:

**DeployObservationV1 actor 输入**:
- `ball_visible`, `ball_age_ms`, `ball_confidence`
- `enemies[i].visible`, `enemies[i].age_ms`, `enemies[i].confidence`
- `last_decision_id`, `last_action_age_ms`, `stuck_flag`

以下字段仍需要后续接入真实数据源或训练环境:

**DeployObservationV1**:
- `velocity_valid`: 速度数据有效性标记 (真实侧可能不稳定)
- `blue_goal`, `yellow_goal`: 球门观测 (需对接 `FieldGoalsPos`)
- `teammates`: 队友观测 (需队内通信)
- `referee_state`, `penalty_state`: 裁判状态 (需对接裁判盒)
- `role`: 角色 ID (需配置/通信)
- 真实 YOLO/雷达链路中的 `confidence` 需要继续从实际检测置信度映射，而不是只用默认值。

**TruthStateV1**:
- `enemies_true`, `teammates_true`: 敌方/队友真值 (Unity 协议需实现)
- `scenario_id`: 场景ID (Unity 协议需实现)

**ResetScenarioV1**:
- `physics_profile`: 物理参数配置 (Unity 协议需实现)

### 12.4 编码维度说明

当前 `DeployObservationV1` 默认编码维度为 **44 维**:

| 特征组 | 维度 | 说明 |
|--------|------|------|
| 自身位姿 | 3 | x, y, yaw |
| 自身速度 | 3 | vx, vy, wz |
| 球位置 | 2 | rel_x, rel_y |
| 持球状态 | 1 | has_ball |
| 球质量 | 3 | visible, age, confidence |
| 敌方 (×4) | 20 | rel_x, rel_y, visible, age, confidence × 4 |
| 边界距离 | 4 | front, left, back, right |
| 球门方向 | 2 | dx, dy |
| 比赛状态 | 3 | referee, penalty, role |
| 历史信息 | 3 | last_decision, last_action_age, stuck |

`MLPConfig.state_dim` 已更新为 44。编码器会在 `strict_dim=True` 时阻止字段数量变化后静默训练，避免模型输入和契约漂移。

### 12.5 下一步工作

按优先级:

1. **Unity 训练协议实现**
   - 实现 `RESET_SCENARIO` 协议
   - 实现 `GET_EPISODE_STATUS` 协议
   - 实现 `GET_TRUTH_STATE` 协议
   - 文件: `SoccerSimulatorForXels/Assets/Scripts/XelTrainingEnv.cs`

2. **真实视觉/模拟视觉质量对接**
   - 将 YOLO 置信度映射到 `ball_confidence` 和 `enemy.confidence`
   - 将帧延迟、丢包率、机器人速度相关丢检注入 `age/confidence`
   - 对接 `FieldGoalsPos` 球门观测

3. **训练数据与 teacher 对齐**
   - 让行为克隆数据优先标注 `DecisionPacketV1`
   - 保留 `legacy_action` 作为兼容和回归对照
   - 补 reward / curriculum / scenario seed 的一致性测试

4. **模型结构评估**
   - 先用当前前馈 MLP 建 baseline
   - 如果 partial observability 明显影响策略，再加多帧堆叠或轻量 GRU

### 12.6 向后兼容性

- `RobotState` 仍然可用，`StateEncoder.encode()` 自动检测输入类型
- `legacy_action` 输出模式仍然可用，旧训练脚本无需立即全部迁移
- 新的训练和部署代码应优先使用 `DeployObservationV1` + `decision_packet`
