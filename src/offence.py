"""Rule-based offence strategy.

This file is intentionally a stronger baseline, not a magic world champion:
it uses the components that already exist in the repo and keeps the loop
non-blocking so it can run on the real robot or in the Unity simulator.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
import time

from BasicFuc import lockball
from LunaPre import Settings, UnitedPosition, chassis, logger, peripheral


BALL_NOT_FOUND = [0xFFF, 0xFFF]
BALL_HELD = [0xDDD, 0xDDD]


def _angle_error_deg(current: float, target: float) -> float:
    """Signed shortest angle error in degrees."""

    return ((float(target) - float(current) + 180.0) % 360.0) - 180.0


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _valid_pose(pose: list | tuple | None) -> bool:
    return bool(pose) and len(pose) >= 3 and list(pose[:3]) != [0xFFF, 0xFFF, 0xFFF]


def _valid_xy(point: list | tuple | None) -> bool:
    return bool(point) and len(point) >= 2 and list(point[:2]) not in (BALL_NOT_FOUND, BALL_HELD)


@dataclass(frozen=True)
class OffenceConfig:
    loop_dt: float = float(os.environ.get("OFFENCE_DT", "0.03"))
    log_every_s: float = float(os.environ.get("OFFENCE_LOG_EVERY", "0.5"))
    shot_cooldown_s: float = float(os.environ.get("OFFENCE_SHOT_COOLDOWN", "0.8"))
    shoot_yaw_tolerance_deg: float = float(os.environ.get("OFFENCE_SHOOT_YAW_TOL", "8"))
    lane_block_width_cm: float = float(os.environ.get("OFFENCE_LANE_BLOCK_WIDTH", "24"))
    carry_x_cm: float = float(os.environ.get("OFFENCE_CARRY_X", "82"))
    deep_shoot_x_cm: float = float(os.environ.get("OFFENCE_DEEP_SHOOT_X", "58"))
    search_speed_gain: float = float(os.environ.get("OFFENCE_SEARCH_GAIN", "1.4"))
    carry_speed_gain: float = float(os.environ.get("OFFENCE_CARRY_GAIN", "2.2"))


class RuleOffenceStrategy:
    """Fast, conservative state machine for the current hardware stack."""

    def __init__(self, config: OffenceConfig | None = None):
        self.cfg = config or OffenceConfig()
        self.state = "boot"
        self._last_log_ts = 0.0
        self._last_logged_state = None
        self._last_shot_ts = -999.0
        self._search_idx = 0
        self._last_dribble = None
        self._enemy_cache = None

        self.field_long = float(getattr(Settings.Bounds, "Long", 260))
        self.field_short = float(getattr(Settings.Bounds, "Short", 200))
        self.goal_x = self.field_long / 2.0
        self.goal_y = 0.0
        self.home = list(getattr(Settings.CheckPoints, "Home", [-70, 0, 0]))
        self.search_points = self._build_search_points()

    def step(self) -> None:
        """Run one strategy tick."""

        self._enemy_cache = None

        if not UnitedPosition.RestrictedArea():
            self._set_state("avoid_wall")
            return

        pose = UnitedPosition.FieldRobotPos()
        ball = UnitedPosition.RobotBallPos()

        if ball == BALL_NOT_FOUND:
            self._search_ball(pose)
            return

        if ball == BALL_HELD:
            self._play_with_ball(pose)
            return

        self._approach_ball(ball)

    def _build_search_points(self) -> list[list[float]]:
        half_y = min(55.0, self.field_short / 2.0 - 35.0)
        home_x = float(self.home[0]) if self.home else -70.0
        return [
            [home_x, 0.0, 0.0],
            [-45.0, half_y, 0.0],
            [-15.0, 0.0, 0.0],
            [-45.0, -half_y, 0.0],
        ]

    def _search_ball(self, pose: list | tuple | None) -> None:
        self._set_state("search_ball")
        self._set_dribble(False)

        if not _valid_pose(pose):
            chassis.RobotTurn(70)
            return

        target = self.search_points[self._search_idx]
        if UnitedPosition.Pos2Pos(target, self.cfg.search_speed_gain):
            self._search_idx = (self._search_idx + 1) % len(self.search_points)

    def _approach_ball(self, ball: list | tuple) -> None:
        bx, by = float(ball[0]), float(ball[1])
        distance = math.hypot(bx, by)
        self._set_state("approach_ball")

        if distance > 70.0 and abs(by) < 45.0:
            self._set_dribble(0.35)
        else:
            self._set_dribble(0.65)

        lockball.LockBallSlip()

    def _play_with_ball(self, pose: list | tuple | None) -> None:
        self._set_dribble(0.75)

        if not _valid_pose(pose):
            self._set_state("secure_ball_no_pose")
            self._face_goal_and_shoot_if_ready(0.0)
            return

        x, y, _ = float(pose[0]), float(pose[1]), float(pose[2])
        aim_y = self._best_goal_aim_y(pose)
        goal_yaw = UnitedPosition.Pos2Angle([x, y], [self.goal_x, aim_y]) % 360

        if self._can_shoot(pose, [self.goal_x, aim_y]):
            self._set_state("shoot_window")
            if self._face_goal_and_shoot_if_ready(goal_yaw):
                return

        carry_target = self._select_carry_target(pose)
        self._set_state("carry_to_lane")
        UnitedPosition.Pos2Pos(
            [carry_target[0], carry_target[1], goal_yaw],
            self.cfg.carry_speed_gain,
        )

    def _can_shoot(self, pose: list | tuple, aim: list[float]) -> bool:
        if time.time() - self._last_shot_ts < self.cfg.shot_cooldown_s:
            return False

        x = float(pose[0])
        y = float(pose[1])
        attack_ready = x >= self.cfg.deep_shoot_x_cm or (x >= 20.0 and abs(y) <= 32.0)
        if not attack_ready:
            return False

        return self._lane_is_open([x, y], aim)

    def _face_goal_and_shoot_if_ready(self, goal_yaw: float) -> bool:
        lockball.LockBallSlip()
        yaw = float(chassis.GetYaw())
        err = _angle_error_deg(yaw, goal_yaw)

        if abs(err) <= self.cfg.shoot_yaw_tolerance_deg:
            chassis.stop()
            peripheral.ShootBall()
            self._set_dribble(False)
            self._last_shot_ts = time.time()
            self._set_state("shot")
            return True

        turn_gain = _clamp(abs(err) / 45.0, 0.35, 1.0)
        chassis.FieldMoveVector(0, 0, goal_yaw, KpW=turn_gain)
        return False

    def _select_carry_target(self, pose: list | tuple) -> list[float]:
        x, y = float(pose[0]), float(pose[1])
        lane_candidates = [-48.0, -24.0, 0.0, 24.0, 48.0]
        max_y = self.field_short / 2.0 - 28.0

        best_y = min(lane_candidates, key=lambda lane_y: self._lane_cost([x, y], [self.goal_x, lane_y]))
        best_y = _clamp(best_y, -max_y, max_y)

        target_x = max(x + 22.0, self.cfg.carry_x_cm)
        target_x = min(target_x, self.goal_x - 18.0)
        return [target_x, best_y]

    def _best_goal_aim_y(self, pose: list | tuple) -> float:
        """Pick a near-post/far-post aim point based on open lanes."""

        goal_half = min(float(getattr(Settings.Vision, "GoalWidth", 60.0)) / 2.0, 38.0)
        candidates = [-goal_half * 0.65, 0.0, goal_half * 0.65]
        start = [float(pose[0]), float(pose[1])]
        return max(candidates, key=lambda gy: self._lane_score(start, [self.goal_x, gy]))

    def _lane_is_open(self, start: list[float], end: list[float]) -> bool:
        return self._lane_score(start, end) > 0.0

    def _lane_score(self, start: list[float], end: list[float]) -> float:
        enemies = self._field_enemies()
        if not enemies:
            return 1.0

        sx, sy = start
        ex, ey = end
        vx, vy = ex - sx, ey - sy
        denom = vx * vx + vy * vy
        if denom <= 1e-6:
            return 0.0

        score = 1.0
        for enemy in enemies:
            if len(enemy) < 2:
                continue
            px, py = float(enemy[0]), float(enemy[1])
            t = ((px - sx) * vx + (py - sy) * vy) / denom
            if not 0.05 <= t <= 0.98:
                continue
            closest_x = sx + t * vx
            closest_y = sy + t * vy
            distance = math.hypot(px - closest_x, py - closest_y)
            if distance < self.cfg.lane_block_width_cm:
                score -= 1.0
            else:
                score -= max(0.0, (self.cfg.lane_block_width_cm * 1.8 - distance) / 100.0)
        return score

    def _lane_cost(self, start: list[float], end: list[float]) -> float:
        sx, sy = start
        ex, ey = end
        distance_cost = math.hypot(ex - sx, ey - sy) / 100.0
        block_bonus = -2.0 * self._lane_score(start, end)
        side_cost = abs(ey) / 120.0
        return distance_cost + block_bonus + side_cost

    def _field_enemies(self) -> list:
        if self._enemy_cache is not None:
            return self._enemy_cache

        try:
            enemies = UnitedPosition.FieldChassisPos()
            if isinstance(enemies, (list, tuple)):
                self._enemy_cache = list(enemies)
                return self._enemy_cache
        except Exception:
            pass
        self._enemy_cache = []
        return []

    def _set_state(self, state: str) -> None:
        self.state = state
        now = time.time()
        if state != self._last_logged_state or now - self._last_log_ts >= self.cfg.log_every_s:
            logger.info("[offence] state={}", state)
            self._last_logged_state = state
            self._last_log_ts = now

    def _set_dribble(self, status) -> None:
        if self._last_dribble == status:
            return
        peripheral.Dribble(status)
        self._last_dribble = status

    def stop(self) -> None:
        try:
            peripheral.Dribble(False)
        except Exception:
            pass
        try:
            chassis.stop()
        except Exception:
            pass
        try:
            UnitedPosition.Stop()
        except Exception:
            pass


def main() -> None:
    strategy = RuleOffenceStrategy()
    try:
        while True:
            strategy.step()
            time.sleep(strategy.cfg.loop_dt)
    finally:
        strategy.stop()


if __name__ == "__main__":
    main()
