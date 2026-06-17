"""Simple rule-based offence role.

The historical offence script was intentionally direct: import the robot
helpers, read the ball state, then choose one of three actions.  Keep that
shape here, while exposing non-blocking entry points for tests and runners.
"""

from __future__ import annotations

from dataclasses import dataclass
import os
import time

from BasicFuc import *  # noqa: F403


BALL_NOT_FOUND = [0xFFF, 0xFFF]
BALL_HELD = [0xDDD, 0xDDD]


@dataclass(frozen=True)
class OffenceConfig:
    loop_dt: float = float(os.environ.get("OFFENCE_DT", "0.03"))
    attack_direction: float = float(os.environ.get("OFFENCE_ATTACK_SIGN", "1"))
    ohmyback_y_cm: float = float(os.environ.get("OFFENCE_OHMYBACK_Y", "45"))
    slip_y_cm: float = float(os.environ.get("OFFENCE_SLIP_Y", "26"))
    quick_shot_x_cm: float = float(os.environ.get("OFFENCE_QUICK_SHOT_X", "70"))
    shoot_yaw_tolerance_deg: float = float(os.environ.get("OFFENCE_SHOOT_YAW_TOL", "12"))
    trick_cycle_s: float = float(os.environ.get("OFFENCE_TRICK_CYCLE", "1.8"))


def _home_search_target() -> list[float]:
    home = list(Settings.CheckPoints.Home)  # noqa: F405
    while len(home) < 3:
        home.append(0)
    return [home[0] + 20, home[1], 0]


def _angle_error_deg(current: float, target: float) -> float:
    return (float(target) - float(current) + 180.0) % 360.0 - 180.0


def _attack_dir(value: float) -> int:
    return 1 if value >= 0 else -1


def _robot_pose() -> list[float] | None:
    try:
        pose = UnitedPosition.FieldRobotPos()  # noqa: F405
    except Exception:
        return None
    if not pose or len(pose) < 3 or list(pose[:3]) == [0xFFF, 0xFFF, 0xFFF]:
        return None
    return [float(pose[0]), float(pose[1]), float(pose[2])]


def _field_enemies() -> list:
    try:
        enemies = UnitedPosition.FieldChassisPos()  # noqa: F405
    except Exception:
        return []
    return list(enemies) if isinstance(enemies, (list, tuple)) else []


def _shoot_if_ready(goal_angle: float, tolerance: float) -> bool:
    if TurnTo_with_ball(goal_angle % 360):  # noqa: F405
        yaw = float(chassis.GetYaw())  # noqa: F405
        if abs(_angle_error_deg(yaw, goal_angle)) <= tolerance:
            peripheral.ShootBall()  # noqa: F405
            chassis.stop()  # noqa: F405
            return True
    return False


def _stop_united_position() -> None:
    stop_func = getattr(UnitedPosition, "Stop", None)  # noqa: F405
    if stop_func is None:
        stop_func = getattr(UnitedPosition, "stop", None)  # noqa: F405
    if stop_func is not None:
        stop_func()


def _play_with_ball(cfg: OffenceConfig, trick_slot: int) -> str:
    pose = _robot_pose()
    goal_x = _attack_dir(cfg.attack_direction) * 110
    goal_angle = UnitedPosition.Local2Angle([goal_x, 0])  # noqa: F405

    if pose is None:
        if _shoot_if_ready(goal_angle, cfg.shoot_yaw_tolerance_deg):
            return "shoot"
        return "turn_to_goal"

    x, y, _ = pose
    progress = _attack_dir(cfg.attack_direction) * x

    if abs(y) >= cfg.ohmyback_y_cm:
        home = _home_search_target()
        if OhMyBackStep(HomePos=home, attack_dir=_attack_dir(cfg.attack_direction)):  # noqa: F405
            return "oh_my_back_shot"
        return "oh_my_back"

    if progress >= cfg.quick_shot_x_cm:
        if _shoot_if_ready(goal_angle, cfg.shoot_yaw_tolerance_deg):
            return "quick_shot"
        return "turn_to_goal"

    if abs(y) >= cfg.slip_y_cm and trick_slot % 2 == 0:
        enemies = _field_enemies()
        enemy = enemies[0][:2] if enemies and len(enemies[0]) >= 2 else [0, 0]
        Slipsideshot(enemy)  # noqa: F405
        return "slip_side"

    if trick_slot % 3 == 0:
        lockball.Lockballangle()  # noqa: F405
        return "belbatov_turn"

    peripheral.Dribble(0.8)  # noqa: F405
    chassis.FieldMoveVector(45 * _attack_dir(cfg.attack_direction), -y * 1.2, goal_angle)  # noqa: F405
    return "feint_drive"


def _offence_tick(cfg: OffenceConfig, trick_slot: int = 0) -> str:
    ball_position = UnitedPosition.RobotBallPos()  # noqa: F405

    if ball_position == BALL_NOT_FOUND:
        peripheral.Dribble(0.7)  # noqa: F405
        UnitedPosition.Pos2Pos(_home_search_target(), 4.5)  # noqa: F405
        return "search_ball"

    if ball_position == BALL_HELD:
        return _play_with_ball(cfg, trick_slot)

    logger.warning(1)  # noqa: F405
    peripheral.Dribble(0.7)  # noqa: F405
    lockball.LockBallSlip()  # noqa: F405
    return "lock_ball"


class RuleOffenceStrategy:
    """Compatibility wrapper for simulator and script runners."""

    def __init__(self, config: OffenceConfig | None = None):
        self.cfg = config or OffenceConfig()
        self.state = "boot"
        self._started_ts = time.time()

    def step(self) -> None:
        cycle = max(self.cfg.trick_cycle_s, self.cfg.loop_dt)
        trick_slot = int((time.time() - self._started_ts) / cycle)
        self.state = _offence_tick(self.cfg, trick_slot)

    def stop(self) -> None:
        OffenceStop()


_offence_strategy = None


def OffenceStep() -> None:
    """Run one non-blocking offence tick."""

    global _offence_strategy
    if _offence_strategy is None:
        _offence_strategy = RuleOffenceStrategy()
    _offence_strategy.step()


def OffenceStop() -> None:
    global _offence_strategy
    try:
        peripheral.Dribble(False)  # noqa: F405
    except Exception:
        pass
    try:
        chassis.stop()  # noqa: F405
    except Exception:
        pass
    try:
        _stop_united_position()
    except Exception:
        pass
    try:
        chassis.stop()  # noqa: F405
    except Exception:
        pass
    _offence_strategy = None


def main() -> None:
    try:
        while True:
            OffenceStep()
            time.sleep(_offence_strategy.cfg.loop_dt if _offence_strategy else OffenceConfig().loop_dt)
    finally:
        OffenceStop()


if __name__ == "__main__":
    main()
