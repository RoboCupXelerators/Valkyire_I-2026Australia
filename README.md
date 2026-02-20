# 🧠⚽ 机器人足球-策略/编程进阶文档

<!-- 在这里可以添加一些徽章，例如构建状态、许可证等 -->
[![LICENSE](https://img.shields.io/badge/license-MIT-green)](./LICENSE)

> 📝 还没想好放啥

## ✨ 介绍

这里存放的是「策略 / 编程」进阶相关的内容：包括基础规则、坐标系、命名规范，以及我们封装的一些常用模块，目标是让你更快上手写**上层策略**、更容易解决**现场题**。

在观看本文档前，你必须要了解：

- 📜 **组委会提供的规则**

- 💻 如何使用一台电脑

- 🔍 如何使用搜索引擎

- 🧠 一颗完整的大脑

- ❓ [《如何智慧的提问》](https://manual.njust.wiki/notes/freshGuide/smart-questions/)

- [`Python`基本语法](https://www.runoob.com/python3/python3-basic-syntax.html)

- [`Python`如何导入库](https://www.runoob.com/python3/python3-module.html)

- [`Python`条件控制](https://www.runoob.com/python3/python3-conditional-statements.html)

- [`Python`循环语句](https://www.runoob.com/python3/python3-loop.html)

## 🧭 基础规则

机器人的方向描述统一为：**从提手处往下看**。

- 航向角 `0˚`：正前方
- 逆时针为正方向
- `90˚`：左侧
- `270˚`：右侧

> 实际上是为了遵循`ROS`的规则，为后续封装节点做准备。

### 🗺️ 方向与坐标系统

方向与坐标系统分为两类：**绝对（场地坐标系）** 和 **相对（机体坐标系）**。

<image src="../RoboCupJuniorModels/.readme/images/field.png" width=30%/>

| 名称   |   场地坐标系   |   机体坐标系   |
| :----: | :----: | :----: |
| 参考点 | 以球场中圈圆心为原点 | 以机器人中心为原点 |
| 参考方向 | 对方球门中心为正方向 | 指南针0˚为正方向 |


## 🧱 基础命名规范

- ✅ 函数/方法：使用 **大驼峰（PascalCase）**，英文命名，例如 `ChassisAngle`、`ChasingBall`。
- ✅ 变量：使用 **蛇形命名（snake_case）**，英文命名，例如 `my_robot`、`my_position`。

### 🏷️ 命名前缀

| 名称   |   含义   |
| :----: | :----: |
| field | 针对场地坐标系 |
| robot | 针对机体坐标系 |
| direct | 原始数据 |
| _ | class内的变量，不推荐直接读取 |


### 📚 其他内部约定名词
| 名称   |   含义   |
| :----: | :----: |
| Self | 机器人自己 |
| Peer | 队友 |
| Ball | 球 |
| Chassis | 所有机器人底盘(包括队友) |
| Enemy | 敌方机器人底盘 |

## 🧰 基础用法（LunaPre）

为了方便进行**上层策略开发**以及**处理现场题**，我们把一些常用功能逐渐封装成模块，并集中放在 `/src/LunaPre.py` 中，模块名为 `LunaPre`。

```python
from LunaPre import *
```

推荐使用这种方式[**导入模块**](https://www.runoob.com/python3/python3-module.html)，后续写策略会更省事。

### 🤖 机体坐标系（Robot Frame）

> 还在为现场题的“走点”而头疼？还在因为场地不同就要重新量距离？还在用 _卷尺_ 去测场地里的点？大人，时代变了。
>
> 我们的**集成坐标系统**可以让你用很少的代码完成：定位、换算、移动到指定点等常见操作。

```python
UnitedPosition = Positions()
```

`UnitedPosition` 就是你接下来会频繁使用的对象，其中包含很多实用的 **方法（methods）/ 功能函数**。

### ⚽ `RobotBallPos`：获取球在 _机体坐标系_ 下的位置
获取 `球` 在机体坐标系下的位置。输出示例：
```python
print(UnitedPosition.RobotBallPos())
>>>[7.2,0.0]
```
输出是一个由两个浮点数组成的列表：第 0 项是 **X**，第 1 项是 **Y**。

上面的结果表示：球在机器人正前方 **7.2cm**，左右偏移 **0cm**。

#### ⚽ `RobotBallAngle`：球相对机器人（机体坐标系）的角度

- **返回**：`int`（单位 °）
- **含义**：根据视觉的 `ball_pos=[x, y]` 计算角度，常用于“对准球”的控制。

### 🧩 `RobotChassisPos`：获取底盘在 _机体坐标系_ 下的位置
```python
    def RobotChassisPos(self):
        return self.vision_data["chassis_pos"]
```
获取 `所有底盘` 在机体坐标系下的位置。输出示例：
```python
print(UnitedPosition.RobotChassisPos())
>>>[[9.2,0.0,20,20,0.85378],[15.3,0.0,20,20,0.85378]]
```
输出是一个“列表的列表”。其中每个子列表通常包含 5 个元素：

- 第 0 项：**X**
- 第 1 项：**Y**
- 其余项：尺寸/置信度等信息（一般写上层策略时不必深究）

上面的第一组数据表示：有一个底盘在机器人正前方 **9.2cm**。

#### 🤖 `RobotChassisAngle`：其他底盘相对机器人角度列表

- **返回**：`list[int]`（每个元素是一个角度）
- **说明**：对视觉检测到的每个底盘计算一个相对角度。

### 🏟️ 场地坐标系（Field Frame）

### 📍 `FieldRobotPos`：获取机器人在 _场地坐标系_ 下的位置
获取 `自己（机器人）` 在场地坐标系下的位置。输出示例：
```python
print(UnitedPosition.FieldRobotPos())
>>> [90.3, 0.0, 1.324542]
```
输出是一个由 3 个浮点数组成的列表：

- 第 0 项：**X**（cm）
- 第 1 项：**Y**（cm）
- 第 2 项：**航向角**（°）

上面的结果表示：机器人在场地坐标 **X=90.3cm，Y=0cm** 的位置，航向角约为 **1.32°**。

#### 📍⚽ `FieldBallPos`：球在场地坐标系下的绝对位置

- **返回**：`[x, y]`（cm）
- **哨兵值**：同上：`0xfff`（无球）、`0xddd`（抱球）

#### 🧭⚽ `FieldBallAngle`：球在场地坐标系下的绝对角度（叠加航向）

- **返回**：`int`（单位 °，范围约 `0~360`）
- **计算**：`(RelBallAngle() + compass()) % 360`

#### 📐⚽ `FieldBallDistance`：球与机器人的距离

- **返回**：`float | int`
- **哨兵值**：
  - `0xfff`：没看到球 / 无效
  - `0xddd`：判断为“已抱球/已抓球”（CatchBall）

示例：

```python
dist = UnitedPosition.AbsBallDistance()
if dist == 0xfff:
    print("No ball")
elif dist == 0xddd:
    print("Catch ball")
else:
    print(f"Ball distance = {dist:.1f} cm")
```

### 🧭 `FieldChassisPos`：获取其他底盘在 _场地坐标系_ 下的位置
获取 `所有底盘（不包含自己）` 在场地坐标系下的位置。输出示例：
```python
print(UnitedPosition.FieldChassisPos())
>>>[[9.2,0.0],[15.3,0.0]]
```

> 兼容性说明：为了兼容历史代码，`LunaPre.py` 里仍保留了旧方法名（例如 `Robot_Ball_Pos`、`Field_Robot_Pos` 等）作为别名；新代码建议使用本文档里的 **PascalCase** 名称。
输出是一个“列表的列表”，每个子列表包含两个元素：

- 第 0 项：**X**（cm）
- 第 1 项：**Y**（cm）

上面的第一组数据表示：有一个底盘在场地坐标 **X=9.2cm，Y=0.0cm**。

#### 🧭🤖 `FieldChassisAngle`：其他底盘在场地坐标系下的绝对角度列表

- **返回**：`list[int]`
- **计算**：`RelChassisAngle()` 的每个角度叠加 `compass()`。

### 🧩 其他常用方法（Positions）

#### 🧭🚗 `Pos2Pos`：移动到场地坐标系的某个目标点

```python
arrived = UnitedPosition.Pos2Pos([x, y, yaw], speed_gain=None)
```

- **参数**：
  - `AimPos=[x, y, yaw]`（cm, cm, °）
  - `speed_gain`：可选速度增益（越大走得越“猛”，但更容易抖/冲过头）
- **返回**：
  - `True`：到达（在误差范围内）
  - `False`：未到达（函数内部会发底盘移动/转向指令）

典型写法：

```python
target = [90, 0, 0]
if UnitedPosition.Pos2Pos(target):
    print("Arrived")
```

#### 🧵🚗 `Move2Path`：按路径点依次走完（阻塞）

```python
UnitedPosition.Move2Path(
    Posistions=[
        [60,  0, 0],
        [90, 20, 0],
    ],
    iWaitMs=0.1,
)
```

- **特性**：阻塞直到走完最后一个点才返回 `True`
- **注意**：比赛策略里不建议长时间阻塞；更推荐在主循环中用 `Pos2Pos` 自己做状态机。

### 🦾 其他进阶方法（Positions）

下面这些函数同样来自 `Positions` 类（也就是 `UnitedPosition` 对象）。它们在写上层策略、处理现场题时非常常用。

> 说明：`LunaPre.py` 中很多函数的注释里会写“非阻塞/阻塞”。
>
> - **非阻塞**：调用一次就返回（但可能会发出底盘控制命令）。
> - **阻塞**：内部有 `while` 循环，会一直等到条件满足才返回；在主策略中使用要谨慎。

#### 🧾 `GetVisionData`：获取视觉原始数据（球 + 底盘）

```python
def GetVisionData(self):
    return self.vision_data
```

- **返回**：`dict`，形如：

```python
{
  "ball_pos": [x, y],
  "chassis_pos": [[x, y, w, h, conf], ...]
}
```

- **注意**：
  - `Positions()` 初始化时会启动一个后台线程监听 `UDP 127.0.0.1:5005`。
  - 若超时收不到数据，会尝试自动拉起 `src/Vision.py` 作为视觉服务。
  - 需要退出程序/重启策略时，建议调用 `UnitedPosition.Stop()` 释放线程与进程。

示例：

```python
data = UnitedPosition.GetVisionData()
print(data["ball_pos"], data["chassis_pos"]) 
```

#### 📏 `DirectDistance`：获取激光雷达到四面墙的“原始距离”

```python
def DirectDistance(self):
    return lidar.GetDists()
```

- **返回**：`[d0, d1, d2, d3]`
- **常见用法**：用于定位（`FieldRobotPos` 内部就是用它算的）。
- **异常/哨兵值**：
  - 若雷达未启动/无数据，会返回 `[0xfff, 0xfff, 0xfff, 0xfff]`。
  - 内部会重试，超过 `cfg.ExpectedVals.MaxWarnCount` 会 `raise RuntimeError`。

> 距离的**顺序与单位**由 `lidar.GetDists()` 决定；在当前实现里：
>
> - `FieldRobotPos()` 会把它当作“长边方向两面墙 + 短边方向两面墙”的四个距离来使用（`0 & 2` 对应长边方向，`1 & 3` 对应短边方向）。
> - 并通过 `LidarScale=10` 把距离换算到厘米级（内部最后会 `/10`）。

#### 🧭 `Local2Angle`：从“自己当前场地坐标”指向目标点的角度

```python
def Local2Angle(self, AimPos: list) -> int:
    ...
```

- **参数**：`AimPos=[x, y]`（场地坐标系，单位 cm）
- **返回**：`int`，`atan2(deltaY, deltaX)` 的角度（单位 °）
- **用途**：比如做一个“面向某点”的控制量，或用于决策（不直接控制底盘）。

#### 🧭 `Pos2Angle`：从任意起点指向目标点的角度

```python
def Pos2Angle(self, lInputPos: list, lAimPos: list) -> int:
    ...
```

- **参数**：
  - `lInputPos=[x, y]` 起点
  - `lAimPos=[x, y]` 终点
- **返回**：`int` 角度（单位 °）

#### 🧭🧱 `RestrictedArea`：限制区/撞墙保护（会直接控制底盘）

```python
ok = UnitedPosition.RestrictedArea()
```

- **返回**：`True/False`
  - `True`：当前位置安全
  - `False`：检测到接近边界/限制区，函数内部会调用 `chassis.AbsMoveAngle(...)` 做规避

> 这是一个“保护函数”，通常放在策略主循环最前面：只要返回 `False`，就说明它刚刚发了一个避障动作，你的策略这帧最好不要再强行给别的移动指令。

#### 🛑 `Cover2Start`：检测“前方被遮挡”后才返回（阻塞）

- **返回**：`True`
- **用途**：常用于“遮挡启动”类的开赛动作（例如裁判遮住前方传感器后，机器人才能动）。

#### 🧯 `Stop`：安全停止并释放视觉线程/进程

```python
UnitedPosition.Stop()
```

- 会执行：底盘停止、关运球器（`peripheral.Dribble(False)`）、停止雷达、结束视觉线程与视觉进程。


### 📡 通讯模块（Communication）

`LunaPre.py` 同时封装了队内通信逻辑（目前主要是 Wi-Fi）。你可以用它来同步“是否持球 / 自己位置”等关键信息。

```python
from LunaPre import Communication

comm = Communication()
comm.StartConncetion()  # 注意：函数名就是这个拼写
```

#### 🏁 `StartConncetion`：一键启动收发线程

- 内部会启动两个后台线程：
  - `SendStatusThreadFunc`：周期发送自己的状态
  - `PeerStatusThreadFunc`：周期解析队友发来的消息

#### ⚽ `BallOwner`：更新本机“是否持球”标志位

```python
comm.BallOwner()
print(comm.BallFlag)  # [self_has_ball, peer_has_ball]
```

- **说明**：该函数不返回值，只会根据 `UnitedPosition.AbsBallPos()` 的结果更新 `comm.BallFlag[0]`。

#### 📤 `SendStatus`：启动“发送线程”（非阻塞）

```python
comm.SendStatus()
```

- **返回**：`True`（线程已启动/启动成功）
- **发送内容**：当前实现会发送类似：

```
BS:1;PX:90;PY:0
```

字段含义：

| 字段 | 含义 |
|---|---|
| `BS` | self 是否持球（0/1） |
| `PX` | self 位置 X（int，cm） |
| `PY` | self 位置 Y（int，cm） |

#### 📥 `PeerStatus`：启动“接收/解析线程”（非阻塞）

```python
comm.PeerStatus()
```

- 会持续读取 `transimission.MessageCache` 并解析更新：
  - `comm.PeerPosition = [x, y]`
  - `comm.BallFlag[1]`（队友是否持球）

#### 📌 通信模块常用数据

- `comm.BallFlag`：`[self_has_ball, peer_has_ball]`
- `comm.PeerPosition`：`[peer_x, peer_y]`（cm，若无数据为 `[0xfff, 0xfff]`）

> ⚠️ 线程注意事项：
>
> - `SendStatusThreadFunc` / `PeerStatusThreadFunc` 是“死循环线程函数”，**不要在主线程里直接调用**。
> - 请使用 `SendStatus()` / `PeerStatus()` / `StartConncetion()` 来启动后台线程。
> - 由于采用全局启动标志位（`SendstatusThreadFuncStarted` 等），同一进程里多次创建 `Communication()` 并重复启动线程可能会导致混乱；建议全局只创建一个 `comm` 实例。


## 🗒️ 其他碎碎念

### 📡 通讯
通讯目前常见有两种方式：**蓝牙**、**Wi-Fi**。两者在比赛环境下都可能不稳定，后续会考虑补充 **UART** 通讯。

> 上面「📡 通讯模块（Communication）」是当前代码里已经落地的一版封装；如果你准备写更通用的通讯层，建议至少包含以下两个核心能力：

通讯相关函数建议至少包含两个核心能力：

- `MessageCache`：消息缓存/队列
- `Send`：发送

## 🧾 ReasonData（运行数据）

### 🪵 日志（logger）
日志等级分为：`info`、`debug`、`warning`、`success`、`error` 五个等级。
<table>
    <tr>
        <td>info</td>
        <td>记录普通信息 例如球的坐标 自己的坐标等基础数据</td>
    </tr>
    <tr>
        <td>debug</td>
        <td>记录只会在调试阶段用到的变量 例如电机编码器数值</td>
    </tr>
    <tr>
        <td>warning</td>
        <td>记录不符合预期 但是不值得阻塞主程序继续进行的异常情况</td>
    </tr>
    <tr>
        <td>success</td>
        <td>记录关键流程已完成/状态良好的信息（例如初始化完成、任务成功等）</td>
    </tr>
    <tr>
        <td>error</td>
        <td>记录不符合预期 必须要阻塞程序/报错之前的所有变量</td>
    </tr>

</table>

### ⚙️ 配置文件
配置文件一般是 JSON，用于存储每台机器人可能不同的变量，例如：机器人编号、下位机类型、端口号等。

> 具体字段含义见文件内注释，这里不再展开。

### 🗃️ 数据库
预留：支援未来。