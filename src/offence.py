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
    boundary_recovery_speed_gain: float = float(os.environ.get("OFFENCE_BOUNDARY_RECOVERY_GAIN", "1.2"))
    # 1 => attack +X, -1 => attack -X, 0 => auto from SIM robot id.
    attack_direction: float = float(os.environ.get("OFFENCE_ATTACK_SIGN", "0"))


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
        self.attack_dir = self._resolve_attack_direction()
        self.goal_x = self.attack_dir * self.field_long / 2.0
        self.goal_y = 0.0
        self.home = self._resolve_home()
        self.search_points = self._build_search_points()

    def _resolve_attack_direction(self) -> float:
        if self.cfg.attack_direction > 0:
            return 1.0
        if self.cfg.attack_direction < 0:
            return -1.0

        try:
            bit_mode = str(getattr(Settings.RoboInfo, "Bit", "")).upper()
            robot_id = int(
                os.environ.get(
                    "XEL_ROBOT_ID",
                    str(getattr(getattr(Settings, "Sim", object()), "RobotId", getattr(Settings.RoboInfo, "number", 1))),
                )
            )
            if bit_mode == "SIM" and robot_id >= 3:
                return -1.0
        except Exception:
            pass
        return 1.0

    def _resolve_home(self) -> list[float]:
        home = list(getattr(Settings.CheckPoints, "Home", [-70, 0, 0]))
        while len(home) < 3:
            home.append(0.0)

        # Default config is written for +X attack. In SIM, enemy-side robots
        # attack -X, so mirror default own-half waypoints.
        if self.attack_dir < 0 and float(home[0]) * self.attack_dir > 0:
            home[0] = -float(home[0])
            home[2] = (float(home[2]) + 180.0) % 360.0

        safe_home = [float(home[0]), float(home[1]), float(home[2])]
        try:
            projected = UnitedPosition.ProjectSafePose(safe_home)
            if projected is not None:
                safe_home = projected
        except Exception:
            pass
        return [float(safe_home[0]), float(safe_home[1]), float(safe_home[2])]

    def step(self) -> None:
        """Run one strategy tick."""

        self._enemy_cache = None

        pose = UnitedPosition.FieldRobotPos()
        safe_target = self._project_recovery_pose(pose)
        if safe_target is not None:
            self._set_state("avoid_wall")
            self._set_dribble(False)
            UnitedPosition.Pos2Pos(safe_target, self.cfg.boundary_recovery_speed_gain)
            return

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
        search_yaw = 0.0 if self.attack_dir > 0 else 180.0
        return [
            [home_x, 0.0, search_yaw],
            [-45.0 * self.attack_dir, half_y, search_yaw],
            [-15.0 * self.attack_dir, 0.0, search_yaw],
            [-45.0 * self.attack_dir, -half_y, search_yaw],
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
        self._set_state("approach_ball")
        # LockBallSlip owns dribbler power while acquiring the ball. Setting it
        # here as well makes the IO command oscillate between two values.
        lockball.LockBallSlip()

    def _play_with_ball(self, pose: list | tuple | None) -> None:
        self._set_dribble(0.75)

        if not _valid_pose(pose):
            self._set_state("secure_ball_no_pose")
            fallback_goal_yaw = 0.0 if self.attack_dir > 0 else 180.0
            self._face_goal_and_shoot_if_ready(fallback_goal_yaw)
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
        progress = self.attack_dir * x
        attack_ready = progress >= self.cfg.deep_shoot_x_cm or (progress >= 20.0 and abs(y) <= 32.0)
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
        safety = self._restricted_params()
        clearance = float(safety.get("clearance", 5.0))
        x_guard = max(1.0, float(safety.get("x_limit", self.field_long / 2.0)) - clearance)
        y_guard = max(1.0, float(safety.get("y_limit", self.field_short / 2.0)) - clearance)
        goal_box_x_min = float(safety.get("goal_box_x_min", x_guard + 1.0))
        goal_box_y_half = float(safety.get("goal_box_y_half", 0.0))

        side_lane = min(y_guard, goal_box_y_half + max(5.0, clearance))
        lane_candidates = [-side_lane, -24.0, 0.0, 24.0, side_lane]
        max_y = min(self.field_short / 2.0 - 28.0, y_guard)

        progress = self.attack_dir * x
        target_progress = max(progress + 22.0, self.cfg.carry_x_cm)
        target_progress = min(target_progress, self.field_long / 2.0 - 18.0, x_guard)
        target_x = self.attack_dir * target_progress

        if abs(target_x) > goal_box_x_min:
            legal_lanes = [
                lane_y
                for lane_y in lane_candidates
                if abs(lane_y) >= goal_box_y_half + clearance and abs(lane_y) <= max_y
            ]
            if legal_lanes:
                lane_candidates = legal_lanes
            else:
                target_x = math.copysign(max(0.0, goal_box_x_min - clearance), target_x)

        best_y = min(lane_candidates, key=lambda lane_y: self._lane_cost([x, y], [target_x, lane_y]))
        best_y = _clamp(best_y, -max_y, max_y)

        # If we are still in the center lane, do not command a diagonal that
        # cuts through the goal box. First move sideways just before the box,
        # then advance down the side lane on later ticks.
        if abs(target_x) > goal_box_x_min and abs(y) < goal_box_y_half + clearance:
            pre_box_x = math.copysign(max(0.0, goal_box_x_min - clearance), target_x)
            if target_x >= 0:
                target_x = min(target_x, pre_box_x)
            else:
                target_x = max(target_x, pre_box_x)

        safe_target = self._project_strategy_target([target_x, best_y, float(pose[2])])
        return [safe_target[0], safe_target[1]]

    def _restricted_params(self) -> dict:
        try:
            return dict(UnitedPosition._restricted_area_params())
        except Exception:
            return {
                "x_limit": min(90.0, self.field_long / 2.0 - 40.0),
                "y_limit": min(70.0, self.field_short / 2.0 - 30.0),
                "goal_box_x_min": 65.0,
                "goal_box_y_half": 50.0,
                "clearance": 5.0,
            }

    def _project_recovery_pose(self, pose: list | tuple | None):
        if not _valid_pose(pose):
            return None
        try:
            return UnitedPosition.ProjectSafePose(pose)
        except Exception:
            return None

    def _project_strategy_target(self, target: list[float]) -> list[float]:
        try:
            projected = UnitedPosition.ProjectSafePose(target)
        except Exception:
            projected = None
        return projected if projected is not None else target

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
