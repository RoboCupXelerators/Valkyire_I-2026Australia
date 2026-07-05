"""Unity调试教师模型脚本

启动Unity Play模式后运行此脚本，实时观察RuleTeacher的决策。

用法:
    cd D:\\Github\\Software
    python src/debug_teacher.py
"""

from __future__ import annotations

import os
import sys
import time
import json
import math
from dataclasses import asdict
from typing import Any, Dict, List, Optional

# 设置环境变量
os.environ["ROBOCUPX_BITS"] = "SIM"

from LunaPre import UnitedPosition, chassis, peripheral
from BasicFuc import lockball

from teacher.rule_teacher import RuleTeacher, TeacherConfig, TeacherDecision
from mlp.contracts import (
    DeployObservationV1,
    EntityObservationV1,
    DecisionPacketV1,
    IntentId,
    FaceMode,
    DribbleMode,
    ShootIntent,
    observation_from_simulator,
)
from mlp.skill_executor import SkillExecutor


def clear_screen():
    """清屏"""
    os.system('cls' if os.name == 'nt' else 'clear')


def format_decision(decision: TeacherDecision) -> str:
    """格式化决策信息用于显示"""
    packet = decision.packet
    try:
        intent_name = IntentId(packet.intent_id).name
    except ValueError:
        intent_name = f"UNKNOWN({packet.intent_id})"
    try:
        face_name = FaceMode(packet.face_mode).name
    except ValueError:
        face_name = f"UNKNOWN({packet.face_mode})"
    try:
        dribble_name = DribbleMode(packet.dribble_mode).name
    except ValueError:
        dribble_name = f"UNKNOWN({packet.dribble_mode})"
    try:
        shoot_name = ShootIntent(packet.shoot_intent).name
    except ValueError:
        shoot_name = f"UNKNOWN({packet.shoot_intent})"
    
    lines = [
        f"┌─────────────────────────────────────────────────────────┐",
        f"│ 状态: {decision.state:<47}│",
        f"├─────────────────────────────────────────────────────────┤",
        f"│ 意图: {intent_name:<47}│",
        f"│ 目标: ({packet.target_x:.1f}, {packet.target_y:.1f}) @ {packet.target_yaw:.1f}°{'':>22}│",
        f"│ 面向: {face_name:<47}│",
        f"│ 运球: {dribble_name:<47}│",
        f"│ 射门: {shoot_name:<47}│",
        f"│ 强度: {packet.move_gain:.2f}{'':>49}│",
        f"├─────────────────────────────────────────────────────────┤",
        f"│ 原因: {', '.join(decision.reasons[:3])[:47] if decision.reasons else 'N/A':<47}│",
        f"│ 安全: target_legal={decision.safety.get('target_legal', '?')}, path_legal={decision.safety.get('path_legal', '?')}",
        f"└─────────────────────────────────────────────────────────┘",
    ]
    return "\n".join(lines)


def get_observation_from_simulator() -> DeployObservationV1:
    """从Unity仿真器获取观测数据"""
    obs = DeployObservationV1()
    
    try:
        # 自身位姿
        pos = UnitedPosition.FieldRobotPose()
        if pos and len(pos) >= 3:
            obs.self_x = float(pos[0])
            obs.self_y = float(pos[1])
            obs.self_yaw = float(pos[2])
        
        # 球位置 (已经是机器人坐标系相对坐标)
        ball_pos = UnitedPosition.RobotBallPos()
        if ball_pos:
            bx, by = ball_pos[0], ball_pos[1]
            if (bx, by) == (0xFFF, 0xFFF):
                # 球不可见
                obs.ball_visible = False
                obs.has_ball = False
            elif (bx, by) == (0xDDD, 0xDDD):
                # 持球
                obs.has_ball = True
                obs.ball_visible = True
                obs.ball_rel_x = 0.0
                obs.ball_rel_y = 0.0
            else:
                obs.ball_rel_x = float(bx)
                obs.ball_rel_y = float(by)
                obs.ball_visible = True
                obs.has_ball = False
        
        # 边界距离 (mm -> cm)
        wall_dist = UnitedPosition.DirectDistance()
        if wall_dist and len(wall_dist) >= 4:
            # 检查是否有无效值
            if all(d != 0xFFF for d in wall_dist[:4]):
                obs.wall_front = float(wall_dist[0]) / 10.0
                obs.wall_left = float(wall_dist[1]) / 10.0
                obs.wall_back = float(wall_dist[2]) / 10.0
                obs.wall_right = float(wall_dist[3]) / 10.0
                obs.wall_valid = True
        
        # 敌方位置
        enemies_raw = UnitedPosition.FieldChassisPos()
        if enemies_raw:
            for i, enemy_pos in enumerate(enemies_raw[:obs.max_enemies]):
                if enemy_pos and len(enemy_pos) >= 2:
                    # 转换为相对坐标
                    rel_x = float(enemy_pos[0]) - obs.self_x
                    rel_y = float(enemy_pos[1]) - obs.self_y
                    enemy = EntityObservationV1(
                        rel_x=rel_x,
                        rel_y=rel_y,
                        visible=True,
                        age_ms=0,
                        confidence=1.0,
                    )
                    obs.enemies.append(enemy)
                    
    except Exception as e:
        print(f"[ERROR] 获取观测失败: {e}")
    
    return obs


def format_observation(obs: DeployObservationV1) -> str:
    """格式化观测数据用于显示"""
    ball_dist = math.hypot(obs.ball_rel_x, obs.ball_rel_y) if obs.ball_visible else -1
    
    lines = [
        f"┌─────────────────────────────────────────────────────────┐",
        f"│ 机器人位置: ({obs.self_x:.1f}, {obs.self_y:.1f}) @ {obs.self_yaw:.1f}°{'':>20}│",
        f"│ 球状态: {'持球' if obs.has_ball else ('可见' if obs.ball_visible else '不可见')}",
        f"  球距离: {ball_dist:.1f}cm{'':>18}│" if obs.ball_visible else f"{'':>46}│",
        f"│ 球位置: ({obs.ball_rel_x:.1f}, {obs.ball_rel_y:.1f}){'':>31}│" if obs.ball_visible else f"│ 球位置: N/A{'':>43}│",
        f"│ 敌方数量: {len(obs.enemies)}{'':>42}│",
        f"│ 边界: 前={obs.wall_front:.1f} 左={obs.wall_left:.1f} 后={obs.wall_back:.1f} 右={obs.wall_right:.1f}{'':>8}│" if obs.wall_valid else f"│ 边界: N/A{'':>44}│",
        f"└─────────────────────────────────────────────────────────┘",
    ]
    return "\n".join(lines)


def execute_decision(packet: DecisionPacketV1, executor: SkillExecutor) -> bool:
    """执行决策"""
    try:
        result = executor.execute(packet)
        return result.executed
    except Exception as e:
        print(f"[ERROR] 执行决策失败: {e}")
        return False


def main():
    """主循环"""
    print("=" * 60)
    print("  Unity 教师模型调试器")
    print("=" * 60)
    print()
    print("请确保 Unity 已进入 Play 模式")
    print("按 Ctrl+C 退出")
    print()
    
    # 初始化教师模型和执行器
    config = TeacherConfig()
    teacher = RuleTeacher(config)
    executor = SkillExecutor()
    
    print(f"[INFO] 教师配置:")
    print(f"  - 场地: {config.field_long} x {config.field_short} cm")
    print(f"  - 球门宽度: {config.goal_width} cm")
    print(f"  - 安全区 X 限制: {config.x_limit} cm")
    print(f"  - 安全区 Y 限制: {config.y_limit} cm")
    print(f"  - 射门 X 阈值: {config.deep_shoot_x_cm} cm")
    print()
    
    # 统计
    step_count = 0
    decision_counts: Dict[str, int] = {}
    
    try:
        while True:
            step_count += 1
            
            # 获取观测
            obs = get_observation_from_simulator()
            
            # 教师决策
            decision = teacher.decide(obs)
            
            # 统计
            state = decision.state
            decision_counts[state] = decision_counts.get(state, 0) + 1
            
            # 执行决策
            executed = execute_decision(decision.packet, executor)
            
            # 清屏并显示
            clear_screen()
            print(f"[Step {step_count}] Unity 教师模型调试")
            print()
            print(format_observation(obs))
            print()
            print(format_decision(decision))
            print()
            print(f"执行状态: {'✓ 成功' if executed else '✗ 失败'}")
            print()
            print("决策统计:")
            for s, c in sorted(decision_counts.items(), key=lambda x: -x[1]):
                print(f"  {s}: {c}")
            print()
            print("按 Ctrl+C 退出")
            
            # 控制循环频率
            time.sleep(0.05)  # 20Hz
            
    except KeyboardInterrupt:
        print("\n[INFO] 用户中断，退出调试")
    finally:
        # 停止机器人
        try:
            chassis.stop()
        except:
            pass
        print("[INFO] 已停止机器人")


if __name__ == "__main__":
    main()
