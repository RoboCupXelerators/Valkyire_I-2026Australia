"""exec

临时执行脚本（移动路径测试）。
避免 import 即执行。
"""

from __future__ import annotations

from BasicFuc import *


def main():
    try:
        a = time.time()
        UnitedPosition.Move2Path([[40, 40, 0], [-70, 50, 80], [-80, -50, 180], [80, -50, 270], [80, 50, 0]], 0.8)
        b = time.time() - a
        logger.info(f"Move2Path finished in {b:.2f} seconds")
        # while(1):
            # UnitedPosition.Pos2Pos([0, 0, 0])
        time.sleep(0.01)
    finally:
        chassis.stop()
        UnitedPosition.Stop()


if __name__ == "__main__":
    main()
