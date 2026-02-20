"""question

原文件更像一个临时路径测试脚本，不应在 import 时执行。
"""

from __future__ import annotations

from BasicFuc import *


def main():
    try:
        points_list = [
            [0, -80, 0],
            [-100, -80, 0],
            [-80, 0, 0],
            [-100, 80, 0],
            [0, 80, 0],
            [100, 80, 0],
            [80, 0, 0],
            [100, -80, 0],
        ]

        points_cache = []
        for _ in range(3):
            print(UnitedPosition.FieldRobotPos())
            time.sleep(1)

        position_point = UnitedPosition.FieldRobotPos()
        # position_point = [100, 80, 0]  # force start point for debugging

        start_idx = 0
        for i in range(len(points_list)):
            if (abs(position_point[0] - points_list[i][0]) < 10) and (abs(position_point[1] - points_list[i][1]) < 10):
                start_idx = i
                break

        i = start_idx
        for _ in range(int(len(points_list) / 2) + 1):
            points_cache.append(points_list[i % len(points_list)])
            i += 1

        points_cache.append([0, 0, 0])
        UnitedPosition.Move2Path(points_cache, 7)
        print(points_cache)
    finally:
        peripheral.Dribble(False)
        chassis.stop()
        UnitedPosition.Stop()
        chassis.stop()


if __name__ == "__main__":
    main()
