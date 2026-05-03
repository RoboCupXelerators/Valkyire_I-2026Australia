# 🤖 机器人足球-快速上手指南

[![LICENSE](https://img.shields.io/badge/license-MIT-green)](./LICENSE)

> 📝 本文档将 `doc.txt` 中的常用命令和函数速查内容，整理为更适合查阅的详细版 README。

## ✨ 介绍

这里收录的是机器人足球项目中最常用的自动运行命令，以及 `LunaPre` / `UnitedPosition` / `chassis` / `peripheral` / `lockball` 等对象的常用函数速查，方便你快速编写上层策略和现场题逻辑。

在阅读本文档前，你最好已经了解：

- 🧠 基本的 Python 语法
- 💻 如何运行脚本与查看日志
- 🔍 如何使用搜索引擎定位问题
- ❓ 如何提问与排查环境问题

## 🚀 自动运行与日志查看

### 1. 自动运行 `main.py` 里的攻防

```bash
./scripts/install_system_service.sh
```

### 2. 查看攻防输出

```bash
journalctl -u ENDFIELD -f
```

### 3. 取消 `main.py` 攻防自动运行

```bash
systemctl disable ENDFIELD.service
```

## 🧭 现场题函数速查

### 📍 数据读取（使用前已自动初始化）

| 函数 | 作用 | 返回值 / 说明 | 可用性 |
| :--- | :--- | :--- | :---: |
| `UnitedPosition.RobotBallPos()` | 获取球在机体坐标系下的位置 | `[x, y]`，单位 cm；哨兵值：`[0xfff, 0xfff]` 表示无球，`[0xddd, 0xddd]` 表示抱球 | ✅ |
| `UnitedPosition.FieldRobotPos()` | 获取自己在场地坐标系下的位置 | `[x, y, yaw]`，单位 cm, cm, ° | ✅ |
| `UnitedPosition.FieldBallPos()` | 获取球在场地坐标系下的位置 | `[x, y]`，单位 cm；哨兵值同上 | ✅ |
| `UnitedPosition.Pos2Pos([x, y, yaw])` | 非阻塞移动到场地目标点 | 返回 `True` 表示到达，`False` 表示未到达 | ✅ |
| `UnitedPosition.Move2Path([[x1, y1, yaw1], ...], wait_s)` | 阻塞方式按顺序移动到多个目标点 | 逐点执行路径 | ✅ |
| `UnitedPosition.RestrictedArea()` | 边界保护 / 限制区保护 | 放在主循环最前面使用；返回 `True` 表示安全，`False` 表示已自动调整 | ✅ |

### 🛞 底盘移动（全局对象 `chassis`）

| 函数 | 作用 | 说明 | 可用性 |
| :--- | :--- | :--- | :---: |
| `chassis.RobotMoveVector(SpeedX, SpeedY, SpeedZ)` | 机体坐标系下的移动 | 前后 / 左右 / 旋转，无返回值 | ✅ |
| `chassis.RobotMoveAngle(Angle, Speed)` | 按角度移动 | `Angle=0` 表示前方，`90` 表示左侧，无返回值 | ✅ |
| `chassis.RobotTurn(Speed)` | 原地旋转 | 无返回值 | ✅ |
| `chassis.FieldMoveVector(SpeedX, SpeedY, FacingAngle)` | 场地坐标系下移动 | 自动补偿航向角，`FacingAngle` 为车头绝对朝向 | ✅ |
| `chassis.stop()` | 立即停车 | 紧急停止底盘 | ✅ |

### 🧠 高级策略动作（`BasicFuc.py`）

| 函数 | 作用 | 说明 | 可用性 |
| :--- | :--- | :--- | :---: |
| `lockball.LockBallSlip()` | 精确吸球 / 锁球 | 返回 `True` 表示抱球，`False` 表示正在移动 | ✅ |

### ⚽ 外设（全局对象 `peripheral`）

| 函数 | 作用 | 说明 | 可用性 |
| :--- | :--- | :--- | :---: |
| `peripheral.Dribble(pwm)` | 设置运球器 | 控制运球器 PWM | ✅ |
| `peripheral.ShootBall()` | 踢球 | 触发电磁铁脉冲 | ✅ |
| `mose("str")` | 发摩斯码 | 可用于调试输出 | ✅ |
| `key() -> bool` | 按键检测 | 返回按键是否按下 | ✅ |

## 🧩 快速示例

### 示例 1：读取球的位置

```python
bx, by = UnitedPosition.RobotBallPos()
print(bx, by)
```

### 示例 2：读取机器人在场地坐标系中的位置

```python
x, y, yaw = UnitedPosition.FieldRobotPos()
print(x, y, yaw)
```

### 示例 3：让机器人走到指定点

```python
arrived = UnitedPosition.Pos2Pos([x, y, yaw])
if arrived:
    print("arrived")
```

### 示例 4：按路径点移动

```python
UnitedPosition.Move2Path([[x1, y1, yaw1], [x2, y2, yaw2]], wait_s=0.1)
```

### 示例 5：使用限制区保护

```python
if not UnitedPosition.RestrictedArea():
    print("robot adjusted by safety guard")
```

## ⚠️ 重要提示

- **哨兵值约定**：
  - `0xfff`：无球 / 无效数据
  - `0xddd`：已抱球 / 已抓球
- **推荐调用顺序**：在主循环中优先调用 `UnitedPosition.RestrictedArea()`，再执行具体策略逻辑。
- **安全停止**：退出策略时，建议调用 `chassis.stop()` 停止底盘，避免持续运动。
