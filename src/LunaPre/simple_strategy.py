"""Small if/else strategy choices shared by role modules."""

from __future__ import annotations

from LunaPre.strategy_tools import clamp


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
