# 🤖 机器人足球-快速上手指南

本指南旨在帮助开发者快速上手编写机器人足球策略，重点介绍最常用的核心函数和典型应用场景。

## 🚀 快速开始

### 1. 核心对象初始化
在策略脚本中，通常通过 `from LunaPre import *` 导入所有常用工具。
- `UnitedPosition`: 机器人位置、视觉数据获取及运动控制核心对象。
- `chassis`: 底盘运动控制接口。
- `peripheral`: 外设控制（如踢球、运球）。
- `lockball`: 锁球控制类。

### 2. 最常用函数速查表

#### 📍 机器人定位与视觉 (UnitedPosition)
| 函数 | 功能 | 返回值/示例 |
| :--- | :--- | :--- |
| `RobotBallPos()` | 获取球在**机体坐标系**下的位置 | `[x, y]` (cm) |
| `RobotBallAngle()` | 获取球相对机器人角度 | `int` (°) |
| `FieldRobotPos()` | 获取机器人在**场地坐标系**下的位置 | `[x, y, yaw]` (cm, cm, °) |
| `FieldBallPos()` | 获取球在**场地坐标系**下的位置 | `[x, y]` (cm) |
| `Pos2Pos([x, y, yaw], speed_gain=None)` | 移动到场地坐标系的指定点 | `True` (到达) / `False` (未到达) |
| `Move2Path(points, iWaitMs=0.1)` | 顺序移动到路径点列表（阻塞） | `True` (完成) |

#### 🛞 底盘控制 (chassis)
| 函数 | 功能 | 返回值/示例 |
| :--- | :--- | :--- |
| `RobotMoveVector(x, y, z)` | 机体坐标系向量移动 (x:前, y:左, z:自转) | 无 |
| `FieldMoveVector(x, y, facing, kp=None)` | 场地坐标系向量移动 + 航向闭环 | 无 |
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
if UnitedPosition.RestrictedArea(): # 确保不在限制区/撞墙保护
    ball_pos = UnitedPosition.RobotBallPos()
    
    if ball_pos == [0xFFF, 0xFFF]:
        print("未发现球")
    elif ball_pos == [0xDDD, 0xDDD]:
        print("已抱球，准备射门")
        peripheral.Dribble(0.7) # 开启运球/吸球
        peripheral.ShootBall()  # 触发踢球
    else:
        print("追踪中...")
        lockball.LockBallSlip() # 自动对准并锁定球 (非阻塞)
```

### 场景 B: 场地走点 (Waypoints)
如 `exec.py` 所示，可以通过 `Move2Path` 实现复杂的路径规划：
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
if ball_pos != [0xFFF, 0xFFF] and ball_pos != [0xDDD, 0xDDD]:
    # 使用 PID 计算 Y 轴移动速度，保持在球的正对面
    speed_y = pan_y(ball_pos[1]) 
    # 保持航向角为 0 (面向对方球门)，沿 Y 轴进行 PID 修正移动
    chassis.FieldMoveVector(0, -speed_y, 0) 
```

## ⚠️ 重要提示
- **坐标系约定**：
    - **机体坐标系**：$x$ 为前， $y$ 为左。
    - **场地坐标系**：原点在场地中心，方向定义见 `README_DETAILED.md`。
- **安全停止**：在策略脚本的 `finally` 块中，务必调用 `chassis.stop()` 和 `UnitedPosition.Stop()` 释放资源。
- **哨兵值**：
    - `0xFFF`: 未检测到球。
    - `0xDDD`: 已经抱球。