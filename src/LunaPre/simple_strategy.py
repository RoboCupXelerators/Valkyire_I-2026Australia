"""Small if/else strategy choices shared by offence and defence.

The functions intentionally return plain tuples.  They are easy to assert in
tests and keep the role modules free from repeated geometry thresholds.
"""

from __future__ import annotations

from LunaPre.strategy_tools import clamp


GOAL_X_CM = 110.0
GOAL_POST_Y_CM = 36.0
SIDE_WALL_Y_CM = 92.0


def choose_attack_plan(pose, enemies, attack_dir: float, direct_lane_score: float):
    """Return (action, target_x, target_y, speed_x, speed_y)."""

    if not pose or len(pose) < 2:
        return "", 0.0, 0.0, 0.0, 0.0

    x = float(pose[0])
    y = float(pose[1])
    attack_dir = 1.0 if float(attack_dir) >= 0 else -1.0
    progress = attack_dir * x
    abs_y = abs(y)
    side = 1.0 if y >= 0 else -1.0

    front_count = 0
    nearest_gap = 9999.0
    nearest_lateral = 0.0
    for enemy in enemies or []:
        if not enemy or len(enemy) < 2:
            continue
        ex = float(enemy[0])
        ey = float(enemy[1])
        forward_gap = attack_dir * (ex - x)
        lateral_gap = ey - y
        if 18.0 < forward_gap <= 85.0 and abs(lateral_gap) <= 42.0:
            front_count += 1
            if forward_gap < nearest_gap:
                nearest_gap = forward_gap
                nearest_lateral = lateral_gap

    if progress >= 18.0 and abs_y <= 28.0 and direct_lane_score >= 0.88:
        return "direct_shot", attack_dir * GOAL_X_CM, 0.0, 0.0, 0.0

    if abs_y >= 52.0 and progress >= 18.0:
        target_y = side * GOAL_POST_Y_CM
        if progress >= 42.0:
            return "sideline_shot", attack_dir * GOAL_X_CM, target_y, attack_dir * 32.0, -side * 14.0
        return "sideline_drive", attack_dir * GOAL_X_CM, target_y, attack_dir * 52.0, side * 12.0

    if front_count >= 2 and progress >= 28.0:
        if abs(nearest_lateral) > 4.0:
            side = -1.0 if nearest_lateral > 0.0 else 1.0
        target_x = x + attack_dir * 42.0
        target_y = side * SIDE_WALL_Y_CM
        if progress >= 38.0:
            return "wall_shot", target_x, target_y, attack_dir * 28.0, side * 18.0
        return "wall_setup", target_x, target_y, attack_dir * 44.0, side * 26.0

    if front_count == 1 and direct_lane_score < -0.05 and progress >= 38.0:
        if abs(nearest_lateral) > 4.0:
            side = -1.0 if nearest_lateral > 0.0 else 1.0
        target_x = x + attack_dir * 42.0
        target_y = side * SIDE_WALL_Y_CM
        return "wall_shot", target_x, target_y, attack_dir * 28.0, side * 18.0

    return "", 0.0, 0.0, 0.0, 0.0


def choose_defence_cover(ball_field, attack_dir: float, field_short: float = 200.0):
    """Return (state, target_x, target_y) for covering shot families."""

    if not ball_field or len(ball_field) < 2:
        return "", 0.0, 0.0

    by = float(ball_field[1])
    attack_dir = 1.0 if float(attack_dir) >= 0 else -1.0
    guard_x = -attack_dir * 58.0
    max_y = min(float(field_short) / 2.0 - 32.0, 58.0)

    if abs(by) >= 45.0:
        return "cover_sideline_shot", guard_x, clamp(by * 0.85, -max_y, max_y)
    if abs(by) <= 18.0:
        return "cover_direct_shot", guard_x, clamp(by * 0.5, -22.0, 22.0)
    if by > 0.0:
        return "cover_wall_shot", guard_x, min(max_y, 36.0)
    return "cover_wall_shot", guard_x, max(-max_y, -36.0)
