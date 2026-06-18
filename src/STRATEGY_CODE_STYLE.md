# 策略代码风格规范

> 适用于机器人足球策略模块的代码编写标准，兼顾可维护性、Agent 友好性和比赛临场调试。

---

## 核心原则

### 1. 单帧决策模型

每个策略文件的核心是一个 `Step()` 函数，处理**一帧**的决策：

- 主循环只负责调度，不包含复杂逻辑
- 长动作用**状态 + 时间戳**表达，不要用长 `sleep()`
- 每个分支短小、条件清晰，改一个行为不影响其他逻辑

### 2. 显式状态管理

使用字符串状态 + 时间戳，而非隐式状态或深层嵌套：

```python
state = "boot"
state_since = time.time()

def set_state(next_state):
    global state, state_since
    if state != next_state:
        state = next_state
        state_since = time.time()
    return state
```

**优点：**
- 日志和遥测直接可读
- 状态转换有迹可循
- 便于调试和回放分析

### 3. 参数环境变量化

关键参数走环境变量，改阈值不需要动代码：

```python
ROLE_DT = float(os.environ.get("ROLE_DT", "0.03"))
```

---

## 标准文件结构

```python
from __future__ import annotations

import os
import time

try:
    from .BasicFuc import *
except ImportError:
    from BasicFuc import *

# ==================== 配置区 ====================

ROLE_DT = float(os.environ.get("ROLE_DT", "0.03"))

# 哨兵值（与仓库保持一致）
BALL_NOT_FOUND = [0xFFF, 0xFFF]
BALL_HELD = [0xDDD, 0xDDD]

# ==================== 状态区 ====================

state = "boot"
state_since = time.time()

def set_state(next_state):
    """状态切换，记录时间戳"""
    global state, state_since
    if state != next_state:
        state = next_state
        state_since = time.time()
    return state

# ==================== Step 函数 ====================

def OffenceStep():
    """单帧决策核心"""
    ball = UnitedPosition.RobotBallPos()
    robot = UnitedPosition.FieldRobotPos()

    # 分支逻辑：条件 -> 动作 -> 状态
    if ball == BALL_NOT_FOUND:
        UnitedPosition.Pos2Pos(
            [Settings.CheckPoints.Home[0] + 20, Settings.CheckPoints.Home[1], 0],
            4.5,
        )
        return set_state("return_home")

    if ball != BALL_HELD:
        lockball.LockBallSlip()
        return set_state("lock_ball")

    # ... 其他逻辑
    
    return set_state("idle")

# ==================== 主循环 ====================

def main():
    try:
        while True:
            OffenceStep()
            time.sleep(ROLE_DT)
    finally:
        UnitedPosition.Stop()

if __name__ == "__main__":
    main()
```

---

## 编码规范

### 条件分支写法

**推荐：** 每个条件后立即 `return set_state()`

```python
if ball == BALL_NOT_FOUND:
    # 动作
    UnitedPosition.Pos2Pos([...], 4.5)
    return set_state("return_home")
```

**不推荐：** 深层嵌套 if/elif

```python
# 反例：难读、难改、难调试
if condition1:
    if condition2:
        if condition3:
            ...
```

### 长动作处理

**推荐：** 用状态 + 时间戳

```python
if state == "aiming":
    if time.time() - state_since > 1.0:  # 等待 1 秒
        # 完成瞄准，执行动作
        peripheral.ShootBall()
        return set_state("shooting")
    # 继续等待
    return set_state("aiming")
```

**不推荐：** 长时间 sleep

```python
# 反例：阻塞主循环，无法响应其他事件
time.sleep(2)  # ❌
```

### 注释要求

- **配置区：** 说明参数用途和单位
- **状态区：** 列出所有可能状态及其含义
- **Step 函数：** 每个主要分支加一行注释说明意图

---

## 接口规范

### 输入：观测接口

从 `UnitedPosition` 获取观测，不要直接依赖仿真器真值：

| 接口 | 返回 | 说明 |
|------|------|------|
| `RobotBallPos()` | `[x, y]` 或哨兵值 | 机体坐标系下球位置 |
| `FieldRobotPos()` | `[x, y, θ]` | 场地坐标系下机器人位姿 |
| `FieldBallPos()` | `[x, y]` | 场地坐标系下球位置 |

### 输出：动作接口

通过 `chassis` / `peripheral` / `lockball` 下发指令：

| 接口 | 说明 |
|------|------|
| `chassis.stop()` | 停止底盘 |
| `peripheral.Dribble(speed)` | 运球器 |
| `peripheral.ShootBall()` | 击球 |
| `lockball.LockBallSlip()` | 锁球 |

### 哨兵值约定

| 值 | 含义 |
|---|------|
| `[0xFFF, 0xFFF]` | 未检测到球 |
| `[0xDDD, 0xDDD]` | 已抱球 |

---

## 为什么选择这种风格

| 对象 | 优势 |
|------|------|
| **维护者** | 文件结构固定，读起来就是"配置、状态、Step、main"，不用追复杂类 |
| **Agent** | 每个分支短、条件清楚，改一个行为不容易碰坏别的逻辑 |
| **仓库兼容** | 兼容现在的运行方式，不强行引入 dataclass、复杂类型、框架调度 |
| **调试** | state 是字符串，日志或遥测直接看得懂 |
| **比赛临场** | 参数走环境变量，改阈值不用动代码 |

---

## 反模式（不要这样写）

### ❌ 深层嵌套

```python
if ball:
    if robot:
        if goal:
            if angle:
                ...
```

### ❌ 长时间阻塞

```python
time.sleep(2)  # 阻塞主循环
```

### ❌ 隐式状态

```python
# 靠代码位置推断状态，不可读
if counter > 10:
    ...
elif counter > 5:
    ...
```

### ❌ 过度工程化

```python
# 一上来就搞复杂状态机类
class StateMachine:
    def __init__(self):
        self.transitions = {...}
    ...
```

---

## 总结

> **稳是第一位。**
> 
> 一个 Step 处理一帧，主循环只负责调度，长动作用状态和时间戳表达。

这种风格最适合当前仓库和团队现状：
- 不改变现有运行方式
- 不引入额外依赖
- Agent 容易理解和修改
- 人类容易阅读和调试

---

*文档版本：2026-06-18*  
*来源：策略组讨论*