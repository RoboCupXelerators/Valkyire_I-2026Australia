# Xelerator 四轮底盘 Web 控制

这个页面显示四轮状态、编码器测速和可选的 C 型板 IMU 数据，并提供前后左右控制。HTTP 层只使用 Python 标准库，所有串口协议、状态机、电机控制和 80°/100°底盘测速都由 `~/Software/src/utils/DM2325Bits.py` 提供。未安装 IMU 不影响底盘控制或速度测量。

## 硬件模式

首次启动时不要添加方向确认参数：

```bash
cd ~/Software
PYTHONPATH=. python3 tools/imu_web/server.py \
  --hardware --host 0.0.0.0 --port 8765
```

浏览器打开 `http://198.51.100.81:8765`。此时可以查看 IMU 和四轮 Disabled 状态，但服务器会拒绝 Arm。

只有 FL、FR、RL、RR 的 `motor_sign` 已经肉眼确认、写入固件并经过重新构建刷写后，才允许：

```bash
cd ~/Software
PYTHONPATH=. python3 tools/imu_web/server.py \
  --hardware --direction-confirmed --host 0.0.0.0 --port 8765
```

## 安全行为

- 单一硬件工作线程独占 `/dev/ttyACM0`，HTTP 线程不得直接读写 CDC。
- Arm 前由 `DM2325Bits.Arm()` 验证四轮在线、新鲜、Disabled 且无故障。
- 线速度限制为 `2.29 m/s`，对应 DM-S2325-1EC 在 24 V 下的 560 rpm 空载上限、30 mm 轮半径和当前 80°/100°底盘几何；这是轮子离地峰值测试上限，不是带载持续额定速度。
- 实际 `vx/vy/wz`、合速度和峰值由四个电机编码器反馈经正运动学计算，不依赖 IMU；该结果不包含轮胎打滑误差。
- 页面每 30 ms 刷新一次目标租约；树莓派硬件线程每 20 ms 向 C 板发送当前目标，避免浏览器或 Wi-Fi 抖动直接影响 C 板 100 ms watchdog。
- 浏览器目标超过 200 ms 未刷新时，树莓派立即归零并 Disable；页面隐藏或关闭时不等待超时，直接请求 Disable。
- 松开方向键、窗口失焦、页面隐藏或关闭时发送停止/Disable。
- Disabled 状态使用只读刷新维持四路反馈新鲜度，不发送 Enable。
- 本工具不修改 ESC ID、Master ID、控制模式、波特率或零点。

## 操作

1. 确认四轮离地、页面显示 `online_mask=0xF`、`enabled_mask=0x0`、`fault_mask=0x0`。
2. 点击 `Arm`。
3. 按住页面方向按钮或键盘 `W/A/S/D`；松开立即归零。
4. 点击 `Disable` 结束控制。

## 演示与旧 IMU 流

不连接硬件时可运行：

```bash
python3 tools/imu_web/server.py --demo
```

保留的只读 JSON Lines 兼容入口：

```bash
python3 tools/imu_web/server.py \
  --cmd "python3 src/utils/DM2325Bits.py --port /dev/ttyACM0 --json-lines"
```

`--cmd` 模式只显示 IMU，不提供电机控制。
