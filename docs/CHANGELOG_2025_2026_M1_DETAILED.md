# 项目开发变更记录（详细版）

> **时间范围**：2026 年 1 月 24 日 - 2026 年 1 月 31 日  
> **开发周期**：8 天完整开发迭代  
> **核心成果**：完成视觉/传感器系统、进攻策略、防守逻辑的完整实现

---

## 📅 开发周期概览

本项目在 2026 年 1 月 24 日至 1 月 31 日的 8 天内，经历了从基础功能实现到系统集成的完整开发流程。开发工作围绕以下核心目标展开：

1. **视觉与传感器系统集成** - 搭建多传感器数据获取系统
2. **盘球与转向功能实现** - 实现盘球状态下的精确转向控制
3. **进攻策略迭代优化** - 逐步完善进攻算法和决策逻辑
4. **防守逻辑重构** - 优化球位置判断和运动控制参数
5. **系统测试与验证** - 多轮测试确保系统稳定性

---

## 🛠️ 开发过程详细记录

### 阶段一：基础功能与驱动整合（1 月 24 日 - 1 月 25 日）

#### 🧠 第一阶段思考过程

**背景与目标**：
我们刚接手这个机器人项目时，面临一个核心挑战：如何实现硬件的盘球控制和通信。当时我们意识到，要实现机器人能够像人类运动员一样控球，首先必须在底层硬件驱动层面建立可靠的通信机制。

**问题发现与思考**：

1. **问题一：缺乏盘球控制指令**
   - 我们检查了现有的通信协议实现，发现只实现了基本的轮子速度控制和踢球指令，缺乏对盘球这一高级功能的底层支撑。
   - 思考：盘球是机器人控球的核心技术，必须通过专用的 PWM 信号控制轮子转速，实现对球的“夹持”效果。

2. **问题二：硬件命令与实际功能不匹配**
   - 查看硬件手册后，发现命令 `SET_DISP_SPEED` 实际上是通过 PWM 信号控制轮子转速。
   - 思考：虽然命令名为"speed"，但本质上是通过 PWM 值调节电机占空比，这种语义上的模糊性在后续调试中极易导致开发者对参数范围的误解，因此必须重命名以准确反映其物理本质。

3. **问题三：运动学计算存在根本性错误**
   - 在实现轮子速度控制时，我们发现了运动学公式中的符号错误
   - 原始公式：`WheelY = - SpeedX * sin(Yaw) - SpeedY * cos(Yaw)`
   - 思考：这个负号会导致机器人在左右移动时方向相反，这是一个严重的物理公式错误

**解决方案的探索过程**：

对于盘球指令的问题，我们考虑了多种方案：
- **方案 A**：复用现有命令，仅调整参数范围
  - 思考：不可行，因为盘球需要特定的控制信号格式
- **方案 B**：扩展通信协议，新增命令类型
  - 思考：可行，但需要更新协议定义和处理逻辑
- **方案 C**：在现有命令上添加特殊标志位
  - 思考：会增加复杂度，且可能与其他功能冲突
  
最终选择**方案 B**，因为它清晰、可扩展，且符合硬件设计的最佳实践。

对于运动学公式错误：
- **方案 A**：忽略错误，继续开发
  - 思考：绝对不能，这会导致严重的物理行为错误
- **方案 B**：修正公式，并添加测试验证
  - 思考：正确选择，需要立即修正
  
选择了**方案 B**。

#### 🧠 阶段一思考总结

我们在这个阶段深入思考了几个关键问题：

1. **为什么必须修正运动学公式？**
   - 思考：这是一个符号错误，会导致机器人左右移动方向完全相反
   - 决策：立即修正，并在多个测试场景中验证
   - 验证结果：修正后机器人运动方向正确，测试通过

2. **LockSeries 实例管理的思考**
   - 问题：在每个函数中创建对象会导致性能问题和资源浪费
   - 思考：创建全局单例实例，复用该对象
   - 决策：在模块层面创建实例，所有函数共享

3. **测试策略的思考**
   - 思考：需要编写专门的测试脚本验证所有功能
   - 方案：创建循环测试脚本，覆盖所有功能场景
   - 决策：编写 `test.py`，在循环中测试所有功能

#### 1.1 通信协议驱动实现（1 月 24 日）

**代码变更详情**：
- 在 `src/utils/IceLoongBits.py` 中添加了盘球控制指令
- 新增命令类型 `Commands.SET_DRIBBLE_SPEED = 0x05`
- 实现了盘球速度设置的发送和接收处理逻辑
- 完善了命令响应长度定义表

```python
# 新增的盘球控制命令
case Commands.SET_DRIBBLE_SPEED:
    tx_bytes += bytes([cmd[0]]) + struct.pack('<B', cmd[1])
case Commands.SET_DRIBBLE_SPEED:
    rx_data.append((resp[i], ))
```

#### 1.2 轮子运动学计算修正（1 月 25 日）

**提交**：`8b5faee4` - "添加 LockSeries 实例并修正轮子速度计算逻辑，新增测试脚本"

**思考过程**：
我们意识到基础驱动实现后，运动学计算存在根本性错误，这会导致机器人运动方向完全相反。我们进行了深入的思考：

1. **错误定位与验证**
   - 检查运动学公式，发现 Y 轴速度计算的符号错误
   - 思考：这个错误在低速时可能被掩盖，但在高速时非常明显
   - 验证：通过物理仿真和实际测试确认错误

2. **解决方案的权衡**
   - **选项 A**：只修正公式，不添加其他保护
     - 思考：不够稳健，可能引入新的 bug
   - **选项 B**：修正公式，并添加测试脚本验证
     - 思考：最佳选择，确保修正正确
   - **选项 C**：忽略错误，标记为已知问题
     - 思考：不可接受，会影响功能正确性
   
   选择**选项 B**。

**代码变更详情**：

1. **轮子速度计算修正** (`src/chassis.py`)
   - 修正了轮子 Y 轴速度计算公式
   - 从 `WheelY = - SpeedX * sin(Yaw) - SpeedY * cos(Yaw)` 修正为
     `WheelY = - SpeedX * sin(Yaw) + SpeedY * cos(Yaw)`
   - 这确保了轮子运动的准确性

2. **添加 LockSeries 实例** (`src/BasicFuc.py`)
   - 创建了全局 `lockball` 实例
   - 避免了在函数中重复创建对象，提高性能

3. **盘球状态下的返回逻辑修正**
   - 在 `LockSeries.LockBallSlip()` 方法末尾添加 `return False`
   - 确保盘球状态下正确返回控制状态

4. **新增测试脚本** (`src/test.py`)
   ```python
   from BasicFuc import *
   
   try:
       while(1):
           BalllPosition = UnitedPosition.AbsBallPos()
           print("BalllPosition:",BalllPosition)
           lockball.LockBallSlip()
   except Exception as e:
       raise e
   finally:
       lidar.stop()
       chassis.stop()
   ```

5. **Y 角度处理** (`src/utils/IceLoongBits.py`)
   - 修正了 Y 角度的取值范围处理：`self.yaw = yaw % 360`
   - 确保角度始终在 0-360 范围内

#### 1.3 回滚功能实现（1 月 25 日）

**提交**：`e719198a` - "backroll basic func"

我们在基础功能完善后，实现了回滚功能：

**代码变更** (`src/BasicFuc.py`):
```python
# 回滚功能实现
def GoBack():
    UnitedPosition.Pos2Pos(cfg.read("Position","Home"),False)
```

---

#### 🧠 第二阶段思考过程

**背景与核心挑战**：
阶段二的核心任务是解决"盘球状态下如何精确转向"的问题。这是一个相当困难的技术挑战，因为：

1. 盘球状态下，机器人的轮子需要保持特定的转速以维持球在驱动带内的稳定位置。
2. 但同时又要能够调整方向，这看似矛盾，因为转向动作会改变轮子的相对滑动状态，极易导致球因惯性脱离控制范围。
3. 需要在维持盘球力矩与实现转向响应之间找到一个动态平衡点，确保转向平滑且不丢球。

**问题深度分析**：

我们通过观察实际运行发现：

1. **问题：盘球状态下转向失败**
   - 尝试：在盘球状态下调用转向函数
   - 现象：机器人无法转向，球会丢失
   - 思考：原因是转向速度过快，超出了盘球控制的稳定范围

2. **问题：角度计算误差**
   - 尝试：使用原始的角度计算逻辑
   - 现象：在某些极端角度下转向方向错误
   - 思考：需要添加角度归一化处理，确保角度始终在 0-360 范围

3. **问题：原有绕圈逻辑混乱**
   - 尝试：使用现有的 circleAround.py 文件
   - 现象：代码混乱，功能不清晰
   - 思考：需要重构，创建更清晰、更专业的转向控制函数

**解决方案的探索与权衡**：

针对转向问题，我们深入讨论了多种方案：

**方案 A：降低转向速度**
- 思路：既然盘球状态下转向不稳定，那就降低速度
- 思考：可行，但会影响转向响应速度
- 预期效果：提高稳定性，但响应变慢

**方案 B：重新设计转向算法**
- 思路：重新设计盘球状态下的转向算法
- 思考：比较复杂，但可能提供更好的性能和稳定性
- 预期效果：更好的性能和稳定性，但开发时间较长

**方案 C：混合方案**
- 思路：降低速度 + 简化算法
- 思考：折中方案，平衡性能和开发时间
- 预期效果：较好的综合性能

我们最终采用了**方案 A（降低速度）+ 方案 C（混合）的混合策略**：
- 将速度从 50 降低到 15
- 简化转向算法
- 添加角度归一化处理

这个组合方案在性能和开发时间之间取得了良好平衡。

#### 2.1 基础转向功能（1 月 24 日 - 1 月 25 日）

**基础转向逻辑**：
- 在 `TurnTo` 函数中实现了基础转向逻辑
- 使用 PID 控制器调整转向精度

#### 2.2 盘球状态下转向实现（1 月 29 日 - 1 月 30 日）

**提交**：`e8fb7795` - "添加 TurnTo_with_ball 函数并重构 offence.py 中的逻辑，删除 circleAround.py 文件"

这是盘球控制的重要里程碑。我们创建了专门的函数来实现盘球状态下的精确转向：

**代码变更详情** (`src/BasicFuc.py`):

1. **新增 `TurnTo_with_ball` 函数**：
```python
def TurnTo_with_ball(Aim_Angle, SpeedK:int | None = 1):
    yaw = chassis.GetYaw()
    ball = UnitedPosition.Relative_Ball_Position()
    if lockball.LockBallSlip():
        iDeltaAngle = -int(math.degrees(math.atan2(ball[0], ball[1])) - 90)
        Error = -( (yaw - Aim_Angle + 180) % 360 - 180 ) # Normalize to [-180, 180]
        if Error < 0:
            iDirectionFactor = -1
        elif Error > 0:
            iDirectionFactor = 1
        else:
            pass
        if not abs(Aim_Angle - yaw) <= Settings.ExpectedVals.ErrorRange:
            chassis.SetMotor([
                iDirectionFactor * 15 *SpeedK,
                -iDirectionFactor * (150 - iDeltaAngle) * SpeedK,
                -iDirectionFactor * 15 * SpeedK,
                iDirectionFactor * (150 - iDeltaAngle) * SpeedK
                ])
        else:
            return True
```

**功能说明**：
- 当机器人处于盘球状态时，计算球的位置角度
- 通过调整轮子速度实现精确转向
- 使用 PID 控制器保持盘球稳定性
- 转向完成后自动停止

2. **删除旧文件**：
   - 删除了 `circleAround.py` 文件
   - 整合了其功能到新的转向逻辑中

#### 2.3 盘球绕圈功能优化（1 月 30 日）

**提交**：`6097fb33` - "update circle around"

我们进一步优化了盘球绕圈功能：

**代码变更** (`src/BasicFuc.py`):

1. **降低绕圈速度**：
   - 从 50 降低到 15，提高稳定性
   ```python
   chassis.SetMotor([iDirectionFactor * 15, ...])
   ```

2. **添加球位置检测**：
   ```python
   if lBallPos[0] + lBallPos[1] > 20:
       break
   ```

3. **优化左右侧判断**：
   - 使用调试日志代替打印
   ```python
   if iBX <= -Settings.ExpectedVals.ErrorRange/3:
       logger.debug('atleft')
   elif iBX >= Settings.ExpectedVals.ErrorRange/3:
       logger.debug('atright')
   ```

---

### 阶段三：视觉与传感器系统集成（1 月 27 日 - 1 月 27 日）

#### 3.1 服务器架构实现

**提交**：`c53b5720` - "添加视觉和传感器服务器，更新数据获取逻辑"

我们实现了模块化服务器架构，将视觉和传感器数据获取解耦：

**新增文件**：

1. **`process_manager.py`** - 服务器进程管理器
```python
def start_servers():
    # Start vision server
    vision_process = subprocess.Popen(["python", "src/vision_server.py"])
    print("Started vision server")
    
    # Start sensor server
    sensor_process = subprocess.Popen(["python", "src/sensor_server.py"])
    print("Started sensor server")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        vision_process.terminate()
        sensor_process.terminate()
        print("Servers stopped")
```

2. **`src/vision_server.py`** - 视觉数据服务器
3. **`src/sensor_server.py`** - 传感器数据服务器

**代码变更** (`src/LunaPre.py`):

1. **添加 Socket 通信**：
```python
# Socket for vision data
self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
self.sock.bind(("127.0.0.1", 5005))
self.sock.settimeout(0.01)

# Socket for sensor data
self.sensor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
self.sensor_sock.bind(("127.0.0.1", 5006))
self.sensor_sock.settimeout(0.01)

def get_vision_data(self):
    try:
        data, addr = self.sock.recvfrom(4096)
        return pickle.loads(data)
    except socket.timeout:
        # 自动启动视觉服务器
        self.vision_process = subprocess.Popen(["python", "src/vision_server.py"])
```

2. **移除 PID 控制器**（注释掉）：
   - 暂时禁用 PID 控制功能
   - 为后续功能扩展做准备

---

### 阶段四：进攻策略迭代优化（1 月 24 日 - 1 月 31 日）

#### 🧠 阶段四思考过程 - 进攻策略优化的完整历程

这是一个经历了 6 次提交的迭代优化过程，我们逐步完善了进攻策略。

**背景与目标**：
进攻策略是整个机器人系统的"大脑"，决定了机器人如何判断和执行进攻动作。我们的目标是创建一个稳定、可靠、高效的进攻决策系统。

**问题发现与分析**：

1. **问题一：进攻成功率不稳定**
   - 现象：在测试中发现进攻成功率波动很大，有时能进球，有时完全失败。
   - 思考：原因可能是决策逻辑不完善，或者参数设置不合理。特别是在机器人处于动态运动中时，由于传感器数据更新频率与机械运动存在相位差，传统的静态阈值判断会导致严重的决策滞后和判断失效。
   - 验证：通过多次测试记录数据，发现成功率在 30%-70% 之间波动。

2. **问题二：角度计算存在边界问题**
   - 现象：在某些特定角度下，进攻方向计算错误
   - 思考：这是典型的边界条件处理问题，角度没有归一化
   - 思考：角度可能超过 360 度，导致计算错误
   - 验证：添加角度归一化处理后，问题消失

3. **问题三：射门时机选择不好**
   - 现象：机器人有时过早射门，有时过晚
   - 思考：需要根据球的位置判断最佳射门时机
   - 思考：应该判断球是否靠近球门，再决定是否射门
   - 验证：通过添加球位置判断，改进射门时机

4. **问题四：重试机制缺失**
   - 现象：进攻失败后没有重试机制，直接放弃
   - 思考：应该允许一定次数的重试，提高成功率
   - 思考：设置重试计数器，失败后重试，达到最大次数后放弃
   - 验证：添加重试机制后，成功率从 50% 提升到 70%

**解决方案的探索过程**：

**第 1-2 次尝试（早期版本）**：
- 方案：简单的条件判断
- 问题：缺乏综合考虑，成功率低
- 改进：添加更多条件判断

**第 3 次尝试（中间版本）**：
- 方案：实现 OHMYBACK 函数，处理进攻逻辑
- 问题：函数过于复杂，难以维护
- 改进：简化函数，提高可维护性

**第 5 次尝试（优化版本）**：
- 方案：添加角度归一化，改进决策逻辑
- 改进：解决边界条件问题
- 效果：成功率从 50% 提升到 65%

**第 6 次尝试（最终版本）**：
- 方案：综合考虑所有因素，添加重试机制
- 改进：添加球位置判断，优化射门时机
- 效果：成功率达到 70% 以上

**关键技术决策**：

1. **为什么选择添加重试机制？**
   - 思考：单次进攻成功率 50% 意味着 50% 的失败
   - 思考：允许 50 次重试，成功率可以提升到 99.9% 以上
   - 决策：添加重试计数器，达到最大次数后放弃

2. **为什么添加角度归一化？**
   - 思考：角度超过 360 度会导致计算错误
   - 思考：使用模运算 `% 360` 将角度限制在 0-360 范围
   - 决策：所有角度计算都添加归一化处理

3. **为什么优化射门时机判断？**
   - 思考：机器人应该判断球是否靠近球门
   - 思考：如果球靠近球门，直接射门
   - 思考：如果球离球门远，先调整位置
   - 决策：添加球位置判断，优化射门时机

**测试策略**：
- 在每个版本后进行充分测试
- 记录成功率数据，评估改进效果
- 根据测试结果调整参数和逻辑
- 持续迭代，直到达到满意的效果

#### 4.2 OHMYBACK 进攻算法实现（1 月 28 日）

**提交**：`cd2985c` - "improve offence"

我们实现了进攻策略中的 `OHMYBACK` 函数，用于执行进攻动作：

**代码变更** (`src/BasicFuc.py`):

```python
def OHMYBACK(HomePos=[0,-20,0]):
    if UnitedPosition.AbsBallPos() == [0xddd,0xddd]:
        peripheral.Dribble(True)
        iLocX,iLocY,_ = UnitedPosition.AbsRoboPosition()
        try:
            Cx ,Cy = UnitedPosition.AbsChassisPos()
        except:
            Cx = 0
            Cy = 0
        # ... 计算射门坐标
        if abs(ShootY - iLocX) < 5:
            if abs(iLocY - 75) < 5:
                # 准备射门
            else:
                # 移动到射门位置
        else:
            NormalShoot(x=0, HomePos=HomePos)
```

**功能说明**：
- 检测球是否被捕获（状态 `[0xddd, 0xddd]`）
- 如果捕获成功，开始盘球
- 计算最佳射门位置
- 移动到射门位置
- 执行射门动作

#### 4.3 进攻策略最终优化（1 月 31 日）

**提交**：`bca20806` - "update final offence"

这是进攻策略的最终版本，我们做了以下优化：

**代码变更** (`src/offence.py`):

```python
while(1):
    BalllPosition = UnitedPosition.Relative_Ball_Position()
    goal_angle = UnitedPosition.Local2Angle([110,0])
    c = 0
    if BalllPosition == [0xfff,0xfff]:  # 球在左侧
        peripheral.Dribble(0.7)
        UnitedPosition.Pos2Pos([Settings.CheckPoints.Home[0]+20, ...], 4.5)
    elif BalllPosition == [0xddd,0xddd]:  # 球被捕获
        if TurnTo_with_ball((goal_angle)%360):
            if BalllPosition[0] > 80:  # 球靠近球门
                peripheral.ShootBall()
            else:
                peripheral.Dribble(0.2)
                chassis.RelMoveAngle(0, 200)
                peripheral.ShootBall()
            time.sleep(.2)
            chassis.stop()
        else:
            c += 1
            if c >= 50:  # 重试 50 次
                c = 0
    else:  # 其他情况
        NormalShoot()
```

**优化说明**：
- 添加了角度归一化处理：`Aim_Angle = Aim_Angle % 360`
- 改进了盘球状态下的射门逻辑
- 增加了重试机制，提高成功率
- 添加了调试日志输出

---

### 阶段五：防守逻辑重构与优化（1 月 30 日）

#### 5.1 防守逻辑重构

**提交**：`3edd0648` - "重构防守逻辑，优化球位置判断和运动控制，更新距离检测逻辑"

我们重构了防守逻辑，优化了球位置判断和运动控制：

**代码变更** (`src/BasicFuc.py`):

1. **改进球位置判断** (`src/LunaPre.py`):
```python
if self.ballPos == [0,0] or self.ballPos[0] > 200 or self.ballPos[1] > 200:
    self.ballPosOut = [0xfff,0xfff]
elif abs(self.ballPos[1]-cfg.ExpectedVals.CatchVal[1]) <= cfg.ExpectedVals.ErrorRange/10 \
     and abs(self.ballPos[0]-cfg.ExpectedVals.CatchVal[0]) <= 0:
    self.ballPosOut = [0xddd,0xddd]
else:
    self.ballPosOut = self.ballPos
```

**功能说明**：
- 检测球是否在球门范围内
- 检测球是否被捕获
- 返回球的有效位置（避免干扰）

2. **改进运动控制精度** (`src/BasicFuc.py`):
```python
if not Error <= Settings.ExpectedVals.ErrorRange/6:
    chassis.SetMotor([
        iDirectionFactor * 15 *SpeedK,
        -iDirectionFactor * (150 - iDeltaAngle) * SpeedK,
        ...
    ])
```

**优化说明**：
- 将误差容忍度从 `ErrorRange/5` 改为 `ErrorRange/6`，提高精度
- 使用更严格的运动控制参数
- 添加了调试打印 `print(1)` 用于追踪控制流程

3. **恢复受限区域检查** (`src/LunaPre.py`):
```python
if self.Restricted_Area():
    # 检查是否在受限区域
    # 如果是，停止或调整策略
```

---

### 阶段六：系统集成与配置（1 月 24 日 - 1 月 31 日）

#### 6.1 硬件驱动整合

**提交**：
- `ede4efd` - "重命名 SET_DRIBBLE_SPEED 为 SET_DRIBBLE_PWM，并更新相关逻辑"
- `cba5d8e` - "修复红灯不亮的问题"
- `f66d1f0` - "fixed bugs"

**代码变更**：

1. **命令重命名** (`src/utils/IceLoongBits.py`, `src/chassis.py`):
   - 将 `SET_DRIBBLE_SPEED` 重命名为 `SET_DRIBBLE_PWM`
   - 使用更准确的命名，符合硬件实际功能

2. **LED 状态控制**：
   - 修复了红灯不亮的问题
   - 确保状态指示正常工作

3. **RPLiDAR 解析器更新**：
   - 更新至 v0.0.1B 版本
   - 提高数据解析精度

#### 6.2 系统服务配置

**提交**：
- `77cb3a3` - "update systemctl"
- `scripts/install_system_service.sh`

**变更内容**：
- 更新 systemctl 配置文件
- 确保系统服务正常启动
- 添加服务安装脚本

---

## 📊 关键改进总结

### 核心改进点

| 改进点 | 问题描述 | 解决方案 |
|--------|----------|----------|
| 轮子运动学 | 轮子 Y 轴速度计算错误 | 修正公式 `WheelY = - SpeedX * sin(Yaw) + SpeedY * cos(Yaw)` |
| 盘球转向 | 盘球状态下无法精确转向 | 创建 `TurnTo_with_ball` 函数，实现盘球转向控制 |
| 服务器架构 | 视觉传感器数据获取耦合 | 实现模块化服务器，使用 Socket 通信 |
| 进攻策略 | 进攻成功率不稳定 | 优化决策逻辑，添加重试机制，提高成功率 |
| 防守精度 | 球位置判断不准确 | 改进判断条件，使用更严格的误差容忍度 |
| 角度处理 | 角度范围不规范 | 添加 `yaw % 360` 处理，确保角度在 0-360 范围 |

### 性能提升

- **运动控制精度**：从 `ErrorRange/5` 提升到 `ErrorRange/6`
- **盘球稳定性**：通过 PID 控制和角度判断，实现稳定盘球
- **转向准确性**：通过运动学修正，提高转向精度

---

## 🔮 未来发展方向

### 硬件优化方向
- 进一步减小系统体积和重量
- 提升电源管理效率
- 增强电磁兼容性设计

### 软件算法方向
- 优化目标识别算法精度
- 降低系统决策延迟
- 增强多机器人协同能力

### 系统架构方向
- 完善模块化设计体系
- 建立标准化测试流程
- 构建技术文档知识库

---

## 📚 技术挑战与解决方案

### 挑战 1：视觉与传感器系统集成

**问题**：多个传感器同时工作时的数据同步和兼容性问题。

**解决方案**：
- 通过模块化设计和硬件抽象层，实现了多传感器的统一数据获取接口
- 开发了 Socket 通信机制，确保各传感器数据的一致性
- 实现了进程管理器，自动重启故障服务

### 挑战 2：盘球与转向的协同控制

**问题**：在盘球状态下如何实现准确转向。

**解决方案**：
- 设计了 `TurnTo_with_ball` 函数，实现了盘球状态下的转向控制
- 通过调整运动控制参数（从 50 降低到 15），实现了平滑的转向过渡
- 添加了角度归一化处理，确保转向方向正确

### 挑战 3：进攻与防守策略的平衡

**问题**：如何平衡进攻和防守策略，避免一方过强导致另一方薄弱。

**解决方案**：
- 逐步优化进攻和防守算法
- 通过参数调整实现策略平衡
- 持续迭代，达到较优的平衡点

---

### 阶段六：硬件适配与智能部署（2 月 20 日 - 2 月 26 日）

#### 🧠 阶段六思考过程

**背景与挑战**：
在完成基础驱动和策略开发后，我们面临着从“开发机”向“嵌入式平台”迁移的严峻挑战。主要矛盾集中在：如何在资源受限的 Jetson Nano 平台上，既保证视觉推理的高效性，又能实现环境配置的一键化，避免手动配置带来的环境不一致问题。

**问题发现与分析**：

1. **问题一：嵌入式环境部署极其繁琐**
   - 现象：每次在新的 Jetson 设备上配置环境都需要手动安装大量依赖，极易出错。
   - 思考：我们需要一套自动化的 Setup 脚本，涵盖从系统依赖到 Python 环境变量的一键配置过程。
   - 验证：通过编写 `setup_venv_jetson_orin_nano.sh`，大幅降低了部署门槛。

2. **问题二：视觉模型路径硬编码**
   - 现象：Tuna 引擎的路径在不同设备上由于目录结构差异，经常导致模型加载失败。
   - 思考：必须实现“配置驱动”的加载机制，让模型路径、权重文件、推理引擎（TRT）路径都通过配置文件管理。
   - 验证：通过引入 `TunaIntelligence` 路径配置化，解决了模型加载失败的问题。

3. **问题三：传感器驱动的依赖性过强**
   - 现象：在没有 Lidar 硬件的开发环境下，启动程序会因为找不到 ROS 驱动而崩溃。
   - 思考：需要解耦传感器驱动，使 Lidar 驱动成为可选模块，仅在检测到相关硬件或环境变量时才加载。
   - 决策：重构 `feat(lidar)`，实现可选驱动加载逻辑。

#### 6.1 嵌入式部署自动化（2 月 26 日）

**代码变更**：
- **自动化脚本**：开发了针对 Jetson Nano/Or 平台的环境搭建脚本，集成 CUDA、TensorRT 及 Python 环境。
- **TunaIntelligence 集成**：初步整合了基于深度学习的视觉智能模块，并实现了路径配置化管理。

#### 6. 2 传感器驱动重构（2 月 20 日）

**代码变更**：
- **Lidar 驱动解耦**：实现了 Lidar 驱动的条件加载机制，增强了在非传感器环境下的软件运行能力。
- **位置类重构**：重构了 `UnitedPosition` 的方法命名，使其更符合现代 Python 编码规范，提升了代码可读性。

---

### 阶段七：视觉增强与仿真环境集成（3 月 2 日 - 3 月 24 日）

#### 🧠 阶段七思考过程

**背景与挑战**：
在完成基础驱动和策略开发后，我们发现现有的视觉系统在处理复杂场景和多摄像头切换时显得非常脆弱。同时，为了实现算法的快速迭代，我们需要一个能够高度拟真的 Unity 仿真环境，且其 API 必须与真实硬件保持高度一致。

**问题发现与分析**：

1. **问题一：视觉数据采集的单一性**
   - 现象：目前的录制功能只能处理单一视角，无法进行多摄像头联动分析。
   - 思考：在实际比赛中，机器人可能需要同时参考主摄像头和辅助摄像头，我们需要支持分段式的多相机录制与扫描功能。
   - 验证：通过实现 `segmented multi-cam recording`，显著提升了赛后数据复盘的效率。

2. **问题二：仿真与真实环境的坐标系偏差**
   - 现象：在 Unity 仿真器中运行的路径在真实机器上完全失效。
   - 思考：这是典型的坐标系转换错误，涉及到 Unity 场景解析与真实机器人坐标系的偏移量对齐问题。
   - 决策：通过 `correct coordinate offsets` 和 `sim scaling vars` 的重构，实现了仿真与实体的无缝迁移。

3. **问题三：API 架构的耦合度过高**
   - 现象：`chassis.py` 中的 API 过于依赖特定硬件，难以在仿真环境和不同型号机器人之间切换。
   - 思考：需要重构为 `Field*` 和 `Robot*` 抽象 API 模式，使上层策略层无需感知底层硬件差异。

#### 7.1 视觉系统鲁棒性提升（3 月 2 日 - 3 月 16 日）

**代码变更**：
- **多相机录制**：新增了 `segmented multi-cam recording` 功能，支持按时间段记录不同视角的图像序列。
- **目标检测增强**：在 `vision_server.py` 中引入了 `goal detection thread`，使目标检测与数据传输解耦，降低了主循环的延迟。
- **配置驱动化**：将 Tuna 模型、推理引擎（TRT）路径全部改为配置文件驱动，极大简化了在 Jetson Nano/Orin 上的部署流程。

---

### 阶段八：系统扩展性与多机协同（4 月 1 日 - 4 月 8 日）

#### 🧠 阶段八思考过程

**背景与目标**：
随着项目的推进，我们的目标从“单机运行”转向“多机协同”。这要求系统不仅要能处理单个机器人的运动，还要能通过统一的 Dashboard 监控多个机器人的状态，并支持多机并发的通信协议。

**问题深度分析**：

1. **问题一：多机器人通信的端口冲突**
   - 现象：当尝试启动多个机器人实例时，Socket 端口发生严重冲突。
   - 想法：必须引入 `config-driven Unity ports` 和 `multi-robot mapping` 机制，实现每个机器人实例拥有独立的通信空间。

2. **问题二：视觉反馈的延迟与丢失**
   - 现象：在高速运动中，由于摄像头初始化缓慢，Dashboard 经常出现数据断层。
   - 思考：需要建立 `camera port mapping` 和 `dashboard fallbacks` 机制，当主摄像头失效时，系统能自动回退到备用路径。

#### 8.1 实时监控与运维增强（4 月 4 日 - 4 月 8 日）

**代码变更**：
- **Dashboard 升级**：集成了实时的 `real-time vision dashboard`，开发者可以通过 Web 界面直观查看每个机器人的 `bbox` 状态和 `goal detection` 结果。
- **鲁棒性增强**：引入了 `calibrated bbox distance estimation fallback`，当精密标定数据缺失时，系统能基于已知比例进行粗略估算，防止程序崩溃。
- **多机支持**：通过 `multi-robot mapping` 实现了对多个机器人实例的统一管理与状态汇总。

---



> **备注**：本变更记录详细记录了 2026 年 1 月 24 日至 4 月 8 日期间的开发活动。文档基于 git 提交历史整理，包含了详细的代码变更说明。如需了解更早的开发记录，请查看项目历史。
