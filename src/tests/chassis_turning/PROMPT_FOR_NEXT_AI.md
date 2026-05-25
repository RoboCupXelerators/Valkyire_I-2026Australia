# Prompt For Next AI On Real Robot

你现在接手的是 RoboCupX 真实机器人底盘“边走边转”验证任务。请先保护现场，不要回滚已有改动。

目标：
- 验证 `Software/src/chassis.py` 中的边走边转控制改动。
- 重点观察：突然启动是否打滑、边平移边转是否明显漂移、yaw 在 `+179/-179` 附近是否走最短角度。
- 通过真机 CSV 结果调整 `Software/src/ReasonData/data/config.json` 的 `Motion` 参数。

先读这些文件：
- `Software/src/chassis.py`
- `Software/src/ReasonData/data/config.json`
- `Software/src/tests/chassis_turning/offline_chassis_allocator_check.py`
- `Software/src/tests/chassis_turning/real_robot_turning_probe.py`

先在开发机或真机上跑离线检查：

```powershell
cd D:\Github\Software
python src\tests\chassis_turning\offline_chassis_allocator_check.py
```

真机 dry-run，不会动轮子：

```powershell
cd D:\Github\Software
python src\tests\chassis_turning\real_robot_turning_probe.py
```

确认机器人在空旷场地、有人盯急停、定位可用后，再跑低速真机 probe：

```powershell
cd D:\Github\Software
python src\tests\chassis_turning\real_robot_turning_probe.py --armed --duration-s 0.8 --speed-scale 0.8
```

输出 CSV 在：

```text
Software/Records/chassis_turning/
```

调参方向：
- 起步还是滑：降低 `TractionPlanarAccelLimit` 或 `TractionWheelSlewRate`。
- 边转边走时平移拖不住：提高 `TurnTranslationMinScale`，或把 `WheelAllocatorSaturationMode` 改成 `translation_first` 试一次。
- 转向优先级不够、朝向追不上：降低 `TurnTranslationMinScale`，或把 `WheelAllocatorSaturationMode` 改成 `turn_first`。
- 整体太肉：提高 `TractionPlanarAccelLimit`、`TractionAngularAccelLimit`、`TractionWheelSlewRate`。
- `+179/-179` 附近如果还大幅绕圈：优先检查 IMU yaw 符号和 `GetYaw()` 单位/范围。

验收标准：
- `offline_chassis_allocator_check.py` 全 PASS。
- 真机 probe 中每组动作起步没有明显猛滑。
- 左右转测试大致对称。
- CSV 中 yaw 变化连续，没有在 `+179/-179` 附近突然走大圈。
- 调参后记录最终使用的 `Motion` 参数值，并说明观察到的改进和剩余问题。
