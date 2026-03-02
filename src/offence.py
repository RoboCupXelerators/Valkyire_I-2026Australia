"""offence

注意：原始版本在 import 时就会启动 while(1) 控制循环，这会导致：
- 任何地方只要 import offence 就会直接占用硬件/启动子进程
- 单元测试/静态检查难以进行

现在改为显式 main() 入口：只有 `python -m offence` 或直接运行脚本时才执行。
"""

from __future__ import annotations

from BasicFuc import *


def main():
    try:
        while True:
            print(UnitedPosition.FieldRobotPos())
            if UnitedPosition.RestrictedArea():
                ball_position = UnitedPosition.RobotBallPos()
                robo_position = UnitedPosition.FieldRobotPos()

                goal_angle_deep = UnitedPosition.Local2Angle([230, 0]) + 180
                goal_angle_home = UnitedPosition.Local2Angle([100, 0])

                if ball_position == [0xFFF, 0xFFF]:
                    UnitedPosition.Pos2Pos(
                        [Settings.CheckPoints.Home[0] + 20, Settings.CheckPoints.Home[1], 0],
                        4.5,
                    )
                elif ball_position == [0xDDD, 0xDDD]:
                    if robo_position[0] < -30:
                        peripheral.Dribble(0.7)
                        if TurnTo_with_ball((goal_angle_deep) % 360, 0.6):
                            chassis.stop()
                            time.sleep(2)
                            chassis.stop()
                            peripheral.ShootBall()
                            logger.warning("Processing :27")
                            peripheral.Dribble(0.0)
                            time.sleep(2)
                        else:
                            logger.warning("Processing :30")
                    else:
                        peripheral.Dribble(0.0)
                        if TurnTo_with_ball((goal_angle_home) % 360, 1):
                            chassis.RobotMoveAngle(0, 300)
                            time.sleep(0.2)
                            peripheral.ShootBall()
                            logger.warning("Processing :45")
                            chassis.stop()
                            time.sleep(1)
                        else:
                            logger.warning("Processing :47")
                else:
                    logger.warning("Processing :44")
                    lockball.LockBallSlip()
            time.sleep(0.03)
    finally:
        UnitedPosition.Stop()


if __name__ == "__main__":
    main()
