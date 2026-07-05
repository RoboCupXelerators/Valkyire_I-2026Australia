# -*- coding: utf-8 -*-
"""
debug_ball_distance.py - 调试脚本：记录球和机器人的距离

用于诊断盘球问题：显示球相对于机器人的实际距离和位置

作者: 克莱因  日期: 2026-06-06
"""

import time
import sys
import os
import math

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from loguru import logger
from LunaPre import UnitedPosition

# 球状态常量
BALL_NOT_FOUND = [0xFFF, 0xFFF]
BALL_HELD = [0xDDD, 0xDDD]


def main():
    """主循环 - 持续记录球和机器人的距离"""
    logger.info("=" * 60)
    logger.info("球距离调试工具")
    logger.info("=" * 60)
    logger.info("按 Ctrl+C 停止")
    logger.info("")
    
    # 统计
    min_distance = float('inf')
    max_distance = 0
    total_samples = 0
    distance_sum = 0
    held_count = 0
    
    try:
        while True:
            # 获取机器人位置
            robot_pos = UnitedPosition.FieldRobotPose()
            robot_ball = UnitedPosition.RobotBallPos()
            field_ball = UnitedPosition.FieldBallPos()
            
            if not robot_pos or robot_pos[0] > 500:
                time.sleep(0.1)
                continue
            
            rx, ry, ryaw = float(robot_pos[0]), float(robot_pos[1]), float(robot_pos[2])
            
            # 检查是否持球
            ball_held = robot_ball == BALL_HELD
            
            # 计算球到机器人的距离
            if robot_ball and robot_ball != BALL_NOT_FOUND and robot_ball != BALL_HELD:
                # 机体坐标系下的球位置
                bx_local = float(robot_ball[0])  # 前向距离
                by_local = float(robot_ball[1])  # 横向距离
                dist_local = math.sqrt(bx_local**2 + by_local**2)
                
                # 持球判断
                is_held = ball_held
                
                # 记录统计
                total_samples += 1
                distance_sum += dist_local
                min_distance = min(min_distance, dist_local)
                max_distance = max(max_distance, dist_local)
                if is_held:
                    held_count += 1
                
                # 打印详细信息
                avg_dist = distance_sum / total_samples if total_samples > 0 else 0
                held_pct = (held_count / total_samples * 100) if total_samples > 0 else 0
                
                # 每10次打印一次
                if total_samples % 10 == 0:
                    logger.info(
                        f"[{total_samples:4d}] "
                        f"机体球坐标: x={bx_local:+6.1f} y={by_local:+6.1f} | "
                        f"距离: {dist_local:5.1f}cm | "
                        f"持球: {is_held} | "
                        f"最小: {min_distance:5.1f} | "
                        f"最大: {max_distance:5.1f} | "
                        f"平均: {avg_dist:5.1f} | "
                        f"持球率: {held_pct:4.1f}%"
                    )
            
            elif ball_held:
                held_count += 1
                total_samples += 1
                if total_samples % 10 == 0:
                    logger.info(f"[{total_samples:4d}] ⚽ 持球中!")
            
            elif field_ball and field_ball[0] < 500:
                # 用场地坐标计算距离
                fx, fy = float(field_ball[0]), float(field_ball[1])
                dist_field = math.sqrt((fx - rx)**2 + (fy - ry)**2)
                
                total_samples += 1
                distance_sum += dist_field
                min_distance = min(min_distance, dist_field)
                max_distance = max(max_distance, dist_field)
                
                if total_samples % 10 == 0:
                    logger.info(
                        f"[{total_samples:4d}] "
                        f"场地坐标: 机器人({rx:+6.1f}, {ry:+6.1f}) 球({fx:+6.1f}, {fy:+6.1f}) | "
                        f"距离: {dist_field:5.1f}cm | "
                        f"最小: {min_distance:5.1f}"
                    )
            
            time.sleep(0.05)  # 20Hz
            
    except KeyboardInterrupt:
        logger.info("")
        logger.info("=" * 60)
        logger.info("调试结束 - 统计")
        logger.info("=" * 60)
        logger.info(f"总样本数: {total_samples}")
        logger.info(f"最小距离: {min_distance:.1f}cm")
        logger.info(f"最大距离: {max_distance:.1f}cm")
        logger.info(f"平均距离: {distance_sum / total_samples:.1f}cm" if total_samples > 0 else "N/A")
        logger.info(f"持球次数: {held_count}")
        logger.info(f"持球率: {held_count / total_samples * 100:.1f}%" if total_samples > 0 else "N/A")
        logger.info("")
        logger.info("💡 分析提示:")
        logger.info("  - 如果最小距离始终 > 15cm，说明机器人无法靠近球")
        logger.info("  - 如果距离在 7-8cm 但持球率为 0%，说明盘球判定参数不对")
        logger.info("  - 如果距离很大且持球率低，可能是碰撞箱问题")


if __name__ == "__main__":
    main()
