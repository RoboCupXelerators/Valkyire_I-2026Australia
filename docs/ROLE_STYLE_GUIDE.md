# 策略模块风格指南

面向 agent 的代码规范，目标是**简单、可读、易维护**。

---

## 核心原则

1. **简单数据类型**：用 `float`、`int`、`str`、`list`，避免复杂类型注解和 dataclass
2. **状态机模式**：字符串状态 + 早返回，不用嵌套 if-else
3. **环境变量可调**：关键参数从 `os.environ` 读取，方便临场改
4. **每步立即返回**：一个 Step 只做一个决策，立即返回状态

---

## 标准模板

```python
from BasicFuc import *
import os
import time

# ========== 配置（环境变量 + 默认值）==========
ROLE_DT = float(os.environ.get("ROLE_DT", "0.03"))
SOME_THRESHOLD = float(os.environ.get("SOME_THRESHOLD", "50.0"))

# ========== 状态 ==========
state = "boot"
started_at = time.time()

# ========== 主循环 ==========
def RoleStep():
    global state

    # 优先级1：找不到球
    ball = UnitedPosition.RobotBallPos()
    if ball == [0xFFF, 0xFFF]:
        peripheral.Dribble(0.7)
        # 搜索行为
        state = "search"
        return state

    # 优先级2：未持球
    if ball != [0xDDD, 0xDDD]:
        lockball.LockBallSlip()
        state = "approach"
        return state

    # 优先级3：特殊机会（射门/过人等）
    if special_condition():
        do_special_action()
        state = "special"
        return state

    # 默认：推进
    peripheral.Dribble(0.8)
    chassis.FieldMoveVector(45, 0, 0)
    state = "drive"
    return state

def RoleStop():
    global state
    peripheral.Dribble(False)
    chassis.stop()
    state = "stopped"

if __name__ == "__main__":
    try:
        while True:
            RoleStep()
            time.sleep(ROLE_DT)
    finally:
        RoleStop()
```

---

## 命名约定

| 类型 | 风格 | 示例 |
|------|------|------|
| 常量 | 大写下划线 | `BALL_NOT_FOUND`、`ROLE_DT` |
| 状态字符串 | 小写下划线 | `"search"`、`"approach"`、`"quick_shot"` |
| Step 函数 | `<Role>Step()` | `OffenceStep()`、`GoalieStep()` |
| Stop 函数 | `<Role>Stop()` | `OffenceStop()` |
| 子动作函数 | 动词短语 | `TurnTo_with_ball()`、`Slipsideshot()` |

---

## 状态机写法

**正确**（早返回，清晰优先级）：
```python
def RoleStep():
    global state
    
    if condition_a():
        action_a()
        state = "a"
        return state
    
    if condition_b():
        action_b()
        state = "b"
        return state
    
    default_action()
    state = "default"
    return state
```

**避免**（嵌套 elif，难读难改）：
```python
def RoleStep():
    global state
    if condition_a():
        action_a()
        state = "a"
    elif condition_b():
        action_b()
        state = "b"
    else:
        default_action()
        state = "default"
    return state
```

---

## 球状态常量

```python
BALL_NOT_FOUND = [0xFFF, 0xFFF]  # 未检测到球
BALL_HELD = [0xDDD, 0xDDD]       # 已持球
```

判断方式：
```python
ball = UnitedPosition.RobotBallPos()
if ball == BALL_NOT_FOUND:
    # 找不到球
elif ball == BALL_HELD:
    # 已持球
else:
    # 看到球但未持球
```

---

## 常用接口速查

### 定位/感知
- `UnitedPosition.RobotBallPos()` → `[x, y]` 机体坐标系球位置
- `UnitedPosition.FieldRobotPos()` → `[x, y, angle]` 场地坐标系机器人位置
- `UnitedPosition.FieldBallPos()` → `[x, y]` 场地坐标系球位置
- `UnitedPosition.Pos2Pos(target, speed)` → 移动到目标点
- `UnitedPosition.Local2Angle(pos)` → 本地坐标转角度

### 执行器
- `peripheral.Dribble(speed)` → 运球，`False` 或 `0` 停止
- `peripheral.ShootBall()` → 击球
- `chassis.FieldMoveVector(vx, vy, angle)` → 场地坐标系移动
- `chassis.stop()` → 停止底盘

### 锁球
- `lockball.LockBallSlip()` → 锁球（带滑动处理）
- `lockball.Lockballangle()` → 锁球角度

---

## 时间与周期

周期性动作（如过人套路）：
```python
trick_slot = int((time.time() - started_at) / TRICK_CYCLE)
if trick_slot % 2 == 0:
    # 奇数周期
else:
    # 偶数周期
```

---

## 异常处理

简单粗暴，失败就用默认值：
```python
try:
    x, y, _ = UnitedPosition.FieldRobotPos()
except Exception:
    x, y = 0, 0
```

---

## 参数调优建议

| 参数 | 含义 | 典型范围 |
|------|------|----------|
| `ROLE_DT` | 循环周期（秒） | 0.02 ~ 0.05 |
| `ATTACK_DIR` | 进攻方向（1 或 -1） | 1（正向）/ -1（反向） |
| `QUICK_SHOT_X` | 快速射门 X 阈值 | 60 ~ 90 |
| `SLIP_Y` | 侧滑 Y 阈值 | 20 ~ 35 |

---

## 不做什么

- 不用复杂类型注解（`List[Tuple[float, ...]]` 等）
- 不用 dataclass、NamedTuple
- 不在 Step 内做阻塞等待
- 不用深层嵌套 if-else
- 不依赖仿真器专有接口（保持实机兼容）

---

## 检查清单

写完一个策略模块后检查：

- [ ] 所有状态都用字符串常量
- [ ] 每个分支都 `return state`
- [ ] 关键参数可从环境变量调整
- [ ] 没有复杂类型注解
- [ ] 异常有 try/except 兜底
- [ ] Stop 函数清理了所有执行器