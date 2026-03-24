"""diagnose_runtime

启动前诊断脚本：打印当前工作目录、导入路径、以及可能影响 SIM/Unity 联调的环境变量。

用途：
- 排查是否从错误目录启动
- 排查 PYTHONPATH 是否混入另一套 Software
- 排查 ROBOCUPX_* / XEL_* / OFFENCE_* 是否被覆盖
"""

from __future__ import annotations

import os
import sys
from pathlib import Path


def main() -> None:
    print("=== diagnose_runtime ===")
    print("cwd:", os.getcwd())
    print("__file__:", __file__)
    print("repo_root_guess:", Path(__file__).resolve().parents[1])
    print("python:", sys.executable)
    print("sys.path:")
    for i, p in enumerate(sys.path):
        print(f"  [{i}] {p}")

    keys = [
        "ROBOCUPX_USE_MOCK",
        "ROBOCUPX_BITS",
        "ROBOCUPX_ENABLE_VISION",
        "PYTHONPATH",
        "XEL_UNITY_IP",
        "XEL_ROBOT_ID",
        "XEL_CMD_PORT",
        "XEL_STATUS_PORT",
        "XEL_TWIST_FROM_WHEELS",
        "XEL_WHEEL_TO_V",
        "XEL_WHEEL_TO_WZ",
        "XEL_YAW_OFFSET_DEG",
        "XEL_YAW_ZERO_AT_START",
        "XEL_SIM_KICK_STRENGTH",
        "XEL_SIM_DRIBBLE_SCALE",
        "XEL_SIM_PLANAR_MOVE_SCALE",
        "XEL_UNITY_TO_CM",
        "XEL_ORIGIN_X_CM",
        "XEL_ORIGIN_Y_CM",
        "XEL_POSE_SCALE",
        "OFFENCE_DT",
        "OFFENCE_LOG_EVERY",
        "OFFENCE_PRINT_POSE",
    ]

    print("env:")
    for k in keys:
        print(f"  {k}={os.environ.get(k)}")


if __name__ == "__main__":
    main()
