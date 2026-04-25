"""exec

临时执行脚本（移动路径测试）。
避免 import 即执行。
"""

from __future__ import annotations

from LunaPre import *


def main():
    try:
        UnitedPosition.Move2Path([[40, 40, 0], [-90, 50, 0], [-90, -50, 0], [90, -50, 0], [-90, -50, 0]], 0.8)
    finally:
        chassis.stop()
        UnitedPosition.Stop()


if __name__ == "__main__":
    main()
