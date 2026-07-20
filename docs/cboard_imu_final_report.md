# RoboMaster C 型板 IMU 阶段报告

日期：2026-07-13。当前状态为“软件、构建、刷写、静态验收完成；人工轴向验证待用户操作”。

1. 实际 IMU：Bosch BMI088，accel ID `0x1E`、gyro ID `0x0F`。
2. 磁力计：板载 IST8310，但当前无 I2C3 驱动/校准，不参与姿态，mag 发布为 0 且 source 为 `GYRO_RELATIVE`。
3. 原始驱动依据：RoboMaster `Development-Board-C-Examples` commit `59d12b1a` 的 BMI088 driver/register/middleware、SPI/TIM/INS 配置。
4. 姿态算法：当前工程内可审计的 6DoF gyro+accel Mahony-like quaternion update；未链接官方闭源 AHRS.lib。
5. SPI/GPIO：SPI1 PB3/PB4/PA7 mode 3；accel CS PA4、gyro CS PB0；DRDY PC4/PC5；heater TIM10_CH1 PF6；MCU 温度 ADC1 internal。
6. 采样频率：1 ms poll；accel DRDY 有效样本约 `807.79 Hz`。
7. 融合频率：约 `807.79 Hz`，与 accel-ready 有效样本同步。
8. Pi 遥测频率：50 Hz 请求目标，完整快照实测 `45.45 Hz`。
9. 轴映射：当前 identity，仅用于静态验证，硬件安装轴向尚未确认。
10. yaw 正方向：软件定义为从上向下看逆时针增加；实机符号待人工旋转确认。
11. yaw 零点：启动相对零；`ZeroYaw`/`SetYawReference` 为 RAM 软件 reference，不改传感器零点。
12. gyro bias：静止 1000 有效样本 Welford 均值/方差校准；最终 bias `[0.00106633, 0.000346211, -0.00363788] rad/s`。
13. 温控：存在；TIM10/PF6、官方 PID 参数、MCU 温度+10°C 上限45°C、50°C 锁存保护；链路不可用立即 `DEGRADED/HEATER_UNAVAILABLE`；最终 BMI 温度 33.75→32.75°C。
14. 新 C 模块：`bmi088_driver`、`cboard_imu_math`、`cboard_imu`、SPI/TIM/ADC support、IMU protocol snapshot。
15. 新 Pi 模块：DM2325 IMU state/API（含 `GetGyroZ`）、strict static/axis capture 与会话判定、LunaPre health gate/cached lidar yaw/Pos2Pos status。
16. `GetYaw`：保留 float degree `[0,360)`；只用新鲜 C 板快照，stale 不返回 0。
17. `GetIMUState`：完整硬件状态并提供 online/ready/calibrated/stale/fault/age。
18. `FieldMoveVector`：接口兼容；IMU 不健康或轴向未确认时输出零并返回 False。
19. `FieldTurn`：同上。
20. 测试：当前全量 `260/260 + 49/49 = 309/309`；含 C 服务层并发快照/状态机、轴向动作正反例/只读调用约束、串口独占及硬件错误版本/CAN2 选择拒绝。
21. 固件 SHA256：ELF `24bd6e8171dfb611acc58a5fffde54def8d3c571718bad39d30f5ecb7f47b3f9`；BIN `c9c12a7094d4760a980485a92eb4ae6866c911b3a2773a1bbfa7e917f9e25f2b`。
22. SWD：OpenOCD Programming Finished，`Verified OK`，reset 成功。
23. 四轮最终 C 板被动状态：online `0x00`、enabled `0`、fault `0`、CAN ESR `0`；final5 的 Disabled 刷新曾因四电机无应答返回 WATCHDOG_TIMEOUT/ACK error、无 Bus-off，故当前不能重新证明 motor-side online/Disabled；CAN2 未初始化且切换被拒绝。
24. 120 秒静态 Pi 快照：5455 行数据（CSV 含 header 共5456行）；C 板有效样本推进96932。
25. IMU 有效频率：`807.7944 Hz`；间隔 P50/P95/P99 `1224.67/1280.28/1281.80 μs`。
26. SPI 错误：0→0。
27. missed samples：0→0。
28. yaw 漂移：`-1.51203°/120s`，`-0.75604°/min`；最大45Hz观测步进 `0.13889°`。
29. gyro 噪声 std：X/Y/Z `0.004486/0.004553/0.004668 rad/s`。
30. accel 模长：均值 `9.83378 m/s²`，std `0.04376 m/s²`。
31. 温度：start/max/end `33.75/33.75/32.75°C`。
32. health flags：仅 `0x00004000 AXIS_MAPPING_UNVERIFIED`。
33. static acceptance：`PASS_STATIC_AXIS_MAPPING_PENDING`，所有增强 checks 为 true。
34. 非零电机运动：否；未 Enable，未发送非零 MIT 目标。
35. 下一步人工动作：标记安装后正前方，C 板平放，缓慢俯视逆时针约90°、回零、顺时针约90°；可选抬前端/左侧。全程不 Enable 电机。
36. 文档：本目录所有 `cboard_imu_*` 审计、映射、坐标、协议、校准、CSV/TXT/JSON、兼容、build、flash、final report。
