# Restore Log

日期：2026-03-23

## 目标
将 `Software` 侧的运行表现尽量恢复到 `SoftwareSuccess` 版本的状态，并记录差异排查结果。

## 结论
经过对比，`Software/src` 与 `SoftwareSuccess` 的核心 Python 代码已经基本一致；`diff -qr` 排查后，去掉 `__pycache__`、`*.pyc` 和数据日志后，实际目录差异主要只剩：

- `SoftwareSuccess` 根目录存在 `.git`
- `SoftwareSuccess` 根目录存在 `requirements.txt`

为了让 `Software/src` 侧在独立运行/拷贝时也能保留依赖说明，本次补充了：

- `Software/src/requirements.txt`

## 已确认的关键稳定点

- `ReasonData/__init__.py` 在两边一致，配置/日志加载逻辑一致
- `utils/XeleratorSimBits.py` 的 SIM 协议主体逻辑一致，坐标与 UDP 通道一致
- `LunaPre.py` 的 SIM 初始化链路一致，`Bits -> chassis -> lidar -> vision` 的路径一致

## 备注

如果后续仍出现“同环境不同效果”，优先检查：

1. 启动时的 `cwd`
2. `PYTHONPATH`
3. `ROBOCUPX_BITS` / `XEL_*` 环境变量
4. Unity 场景里的原点和 yaw 约定

## 进一步排查结论（2026-03-24）

我继续检查了 `Software/src` 的启动入口和环境变量入口，结果如下：

- **不是“隐藏变量”很多**，主要就是这些显式变量在影响行为：
  - `ROBOCUPX_USE_MOCK`
  - `ROBOCUPX_BITS`
  - `ROBOCUPX_ENABLE_VISION`
  - `XEL_UNITY_IP`
  - `XEL_ROBOT_ID`
  - `XEL_CMD_PORT`
  - `XEL_STATUS_PORT`
  - `XEL_TWIST_FROM_WHEELS`
  - `XEL_WHEEL_TO_V`
  - `XEL_WHEEL_TO_WZ`
  - `XEL_YAW_OFFSET_DEG`
  - `XEL_YAW_ZERO_AT_START`
  - `XEL_SIM_KICK_STRENGTH`
  - `XEL_SIM_DRIBBLE_SCALE`
  - `XEL_SIM_PLANAR_MOVE_SCALE`
  - `XEL_UNITY_TO_CM`
  - `XEL_ORIGIN_X_CM`
  - `XEL_ORIGIN_Y_CM`
  - `XEL_POSE_SCALE`
  - `OFFENCE_DT`
  - `OFFENCE_LOG_EVERY`
  - `OFFENCE_PRINT_POSE`

- **真正容易出问题的路径依赖**只有一处特别明显：
  - `LunaPre.py` 在启动 vision 子进程时使用 `subprocess.Popen([sys.executable, "src/Vision.py"], cwd=project_root)`
  - 这意味着必须从“仓库根目录”启动，且 `project_root` 必须正确推导
  - 如果你从别的目录直接跑脚本，或者 `PYTHONPATH` 混入了另一套 `Software`，就可能导入到错误版本

- **目前最建议的启动方式**：
  1. 进入仓库根目录 `Xelerators`
  2. 只保留一套 `Software` 路径在 `PYTHONPATH`
  3. 先显式设置 `ROBOCUPX_BITS=SIM`
  4. 再运行最小联调脚本 `python Software/src/xel_unity_offence_smoke.py`

如果你愿意，我下一步可以直接给你补一个“启动前自动打印 cwd / sys.path / 所有 XEL_* 和 ROBOCUPX_* 变量”的诊断脚本，这样一跑就能立刻看出是不是环境或路径被污染了。
