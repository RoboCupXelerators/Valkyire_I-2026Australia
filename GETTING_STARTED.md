# 🚀 Xelerators 机器人足球系统 - 新手入门指南

> 写给第一次接触这套系统的同学们。如果你觉得其他文档太复杂，从这里开始就对了！

---

## 📋 目录

1. [这是什么？](#这是什么)
2. [开始之前](#开始之前)
3. [快速上手（5分钟）](#快速上手5分钟)
4. [核心概念（必读）](#核心概念必读)
5. [写你的第一个策略](#写你的第一个策略)
6. [常用功能速查](#常用功能速查)
7. [模拟器使用](#模拟器使用)
8. [常见问题](#常见问题)

---

## 这是什么？

这是一套**机器人足球控制系统**，让你可以用 Python 代码控制机器人在场地上踢球。

### 你需要知道的三件事

| 概念 | 简单解释 |
|------|----------|
| **视觉系统** | 机器人的"眼睛"，识别球、球门、其他机器人的位置 |
| **底盘控制** | 机器人的"腿"，控制前后左右移动和转向 |
| **策略代码** | 机器人的"大脑"，你写的 Python 代码决定它怎么踢球 |

---

## 开始之前

### 你需要准备

- ✅ 一台电脑（Windows/Mac/Linux 都可以）
- ✅ 已安装 Python 3.8 或更高版本
- ✅ 一个代码编辑器（推荐 VS Code）
- ✅ 基础的 Python 知识（会写 if/else、循环、函数就行）

### 安装步骤

```bash
# 1. 进入项目目录
cd Software

# 2. 安装依赖
pip install -r requirements.txt

# 完成！
```

> 💡 **提示**：如果你用的是 Jetson 开发板或 Hailo 设备，请用对应的 requirements_jetson.txt 或 requirements_hailo.txt

---

## 快速上手（5分钟）

### 方式一：使用模拟器（推荐新手）

模拟器让你不需要真实机器人就能测试代码！

**步骤：**

1. **打开 Unity 模拟器**
   - 用 Unity Hub 打开 `SoccerSimulatorForXels` 文件夹
   - 打开场景 `Assets/Scenes/SampleScene.unity`
   - 点击 Unity 编辑器上方的 ▶️ 播放按钮

2. **修改配置文件**
   
   打开 `Software/src/ReasonData/data/config.json`，确认以下设置：
   ```json
   {
     "RoboInfo": {
       "Bit": "SIM"
     },
     "Sim": {
       "UnityIP": "127.0.0.1",
       "RobotId": 1
     }
   }
   ```

3. **运行进攻策略**
   ```bash
   cd Software/src
   python offence.py
   ```

4. **观察结果**
   - 在 Unity 窗口中，机器人应该开始自动追球了！
   - 按 `Ctrl+C` 停止程序

### 方式二：使用真实机器人

1. **连接机器人**
   - 确保机器人已开机
   - 通过 USB 或网络连接

2. **修改配置**
   ```json
   {
     "RoboInfo": {
       "Bit": "IceLoong"  // 或其他下位机类型
     }
   }
   ```

3. **运行策略**
   ```bash
   python offence.py
   ```

---

## 核心概念（必读）

### 🗺️ 坐标系统（最重要！）

理解坐标系统是使用这套系统的关键。

#### 两种坐标系

| 坐标系 | 原点位置 | 用途 |
|--------|----------|------|
| **机体坐标系** | 机器人中心 | "球在我前方 10cm" |
| **场地坐标系** | 场地中心 | "我在场地 X=90cm, Y=0cm 的位置" |

#### 方向定义

```
        前方 (X正方向, 0°)
              ↑
              |
 左 (Y正) ←---+---→ 右 (Y负)
    (90°)     |
              ↓
        后方 (X负, 180°)
```

- **X 轴**：前方为正，后方为负
- **Y 轴**：左侧为正，右侧为负
- **角度**：逆时针为正，0° 是正前方

#### 举例

```python
# 球在机器人前方 10cm，左边 5cm
ball_pos = [10, 5]  # [X, Y]

# 机器人在场地右半场，靠近对方球门
robot_pos = [90, 0]  # X=90cm（前方），Y=0cm（中间）
```

### 🎯 核心对象

在你的策略代码中，会用到这几个核心对象：

```python
from LunaPre import *
```

| 对象名 | 作用 | 常用操作 |
|--------|------|----------|
| `UnitedPosition` | 定位与导航 | 获取球的位置、机器人位置、移动到目标点 |
| `chassis` | 底盘控制 | 控制机器人移动、转向、停止 |
| `peripheral` | 外设控制 | 踢球、运球 |
| `lockball` | 锁球控制 | 自动追踪并锁定球 |

---

## 写你的第一个策略

### 最简单的策略：追球并踢球

创建文件 `my_first_strategy.py`：

```python
from LunaPre import *

try:
    while True:
        # 1. 检查是否在安全区域
        if UnitedPosition.RestrictedArea():
            
            # 2. 获取球的位置（相对于机器人）
            ball_pos = UnitedPosition.RobotBallPos()
            
            # 3. 根据球的位置做决策
            if ball_pos == [0xFFF, 0xFFF]:
                # 没看到球，回到场地中心
                print("找不到球，回中心...")
                UnitedPosition.Pos2Pos([0, 0, 0])
                
            elif ball_pos == [0xDDD, 0xDDD]:
                # 已经抱住球了，踢球！
                print("抱住球了，射门！")
                peripheral.Dribble(0.7)  # 开启运球器
                peripheral.ShootBall()    # 踢球
                
            else:
                # 看到球了，追球！
                print(f"追球中... 球在 [{ball_pos[0]:.1f}, {ball_pos[1]:.1f}]")
                lockball.LockBallSlip()  # 自动追球
        
        # 等待一小段时间（约 30Hz）
        time.sleep(0.03)
        
finally:
    # 程序结束时一定要停止机器人！
    chassis.stop()
    peripheral.Dribble(False)
    UnitedPosition.Stop()
```

### 运行你的策略

```bash
python my_first_strategy.py
```

### 代码解释

| 代码 | 含义 |
|------|------|
| `UnitedPosition.RestrictedArea()` | 检查机器人是否在安全区域（不会撞墙） |
| `RobotBallPos()` | 获取球相对于机器人的位置 |
| `[0xFFF, 0xFFF]` | 特殊值，表示"没看到球" |
| `[0xDDD, 0xDDD]` | 特殊值，表示"已经抱住球" |
| `LockBallSlip()` | 自动追球功能 |
| `finally` 块 | 确保程序退出时机器人停止 |

---

## 常用功能速查

### 📍 获取位置信息

```python
# 球的位置（相对于机器人）
ball = UnitedPosition.RobotBallPos()      # 返回 [x, y] cm

# 机器人的位置（在场地上的绝对位置）
pose = UnitedPosition.FieldRobotPose()    # 返回 [x, y, yaw]

# 球的位置（在场地上的绝对位置）
ball_field = UnitedPosition.FieldBallPos() # 返回 [x, y]
```

### 🚗 移动控制

```python
# 移动到场地上的某个点
UnitedPosition.Pos2Pos([x, y, yaw])  # 返回 True 表示到达

# 按路径走多个点
UnitedPosition.Move2Path([
    [50, 0, 0],    # 第一个点
    [50, 50, 90],  # 第二个点
    [0, 50, 180]   # 第三个点
])

# 直接控制底盘
chassis.RobotMoveVector(x_speed, y_speed, rotation)  # 机体坐标系
chassis.FieldMoveVector(x_speed, y_speed, facing)    # 场地坐标系

# 停止
chassis.stop()
```

### ⚽ 踢球和运球

```python
# 开启运球器（吸球）
peripheral.Dribble(True)   # 或 peripheral.Dribble(0.7)

# 关闭运球器
peripheral.Dribble(False)

# 踢球
peripheral.ShootBall()
```

---

## 模拟器使用

### 为什么用模拟器？

- ✅ 不需要真实机器人
- ✅ 可以随时测试代码
- ✅ 可以观察机器人的行为
- ✅ 安全，不会撞坏东西

### 启动模拟器

1. 打开 Unity Hub
2. 添加项目：`SoccerSimulatorForXels`
3. 打开场景：`Assets/Scenes/SampleScene.unity`
4. 点击播放按钮 ▶️

### 配置 Python 连接模拟器

在 `config.json` 中设置：

```json
{
  "RoboInfo": {
    "Bit": "SIM"
  },
  "Sim": {
    "UnityIP": "127.0.0.1",
    "RobotId": 1
  }
}
```

### 模拟器快捷操作

- 在运行时 UI 中可以调整球的位置
- 可以切换相机视角
- 可以重置机器人位置

---

## 常见问题

### ❓ 程序报错 "找不到模块"

**原因**：没有在正确的目录运行，或者没安装依赖

**解决**：
```bash
cd Software/src
pip install -r ../requirements.txt
python offence.py
```

### ❓ 机器人不动

**可能原因**：
1. 配置文件中 `Bit` 设置错误
2. 模拟器没有运行
3. 端口被占用

**解决方法**：
```bash
# 检查端口是否被占用
lsof -i :20001

# 确认配置正确
cat src/ReasonData/data/config.json
```

### ❓ 找不到球 (返回 0xFFF)

**可能原因**：
1. 视觉系统没有启动
2. 相机没有正确连接
3. 在模拟器中，球可能不在视野内

**解决方法**：
- 真实机器人：检查相机连接
- 模拟器：调整相机角度或球的位置

### ❓ 机器人方向反了

**原因**：坐标系或电机配置问题

**解决方法**：
- 检查 `config.json` 中的电机端口配置
- 在模拟器中检查 `invertYawSignForSoftware` 设置

### ❓ 程序退出后机器人还在动

**原因**：没有正确执行清理代码

**解决方法**：确保你的代码有 `try/finally` 块：

```python
try:
    # 你的策略代码
    pass
finally:
    chassis.stop()
    peripheral.Dribble(False)
    UnitedPosition.Stop()
```

---

## 📚 下一步

当你熟悉了基础操作，可以：

1. 阅读 [README_DETAILED.md](./README_DETAILED.md) 了解更多高级功能
2. 查看 `offence.py` 和 `defence.py` 学习更复杂的策略
3. 尝试修改现有策略，实现自己的想法

---

## 🆘 需要帮助？

- 查看 [详细文档](./README_DETAILED.md)
- 查看 [项目结构](./STRUCTURE.md)
- 查看 [模拟器文档](../SoccerSimulatorForXels/README.md)

---

> 💡 **小贴士**：学习编程最好的方式是动手尝试。不要怕出错，多试几次就熟悉了！
