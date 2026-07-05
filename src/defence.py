"""Rule-based defensive role.

The old defence entrypoint was a blocking loop. This version keeps
one `step()` call as one control tick, so
multi-robot runners can sample traces and stop cleanly.

v5 fix: avoid_wall caches recovery target and exits on arrival or stuck timeout.
"""

from __future__ import annotations

from dataclasses import dataclass
import time

from BasicFuc import lockball
from LunaPre import Settings, UnitedPosition, chassis, logger, peripheral
from LunaPre.strategy_tools import (
    BALL_HELD,
    BALL_NOT_FOUND,
    angle_error_deg,
    clamp,
    distance,
    safe_stop,
    valid_pose,
)


def choose_defence_cover(ball_field, attack_dir: float, field_short: float = 200.0):
    """Pick the goalie cover point for direct, wall, and sideline shots."""

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


@dataclass(frozen=True)
class DefenceConfig:
    loop_dt: float = 0.03
    log_every_s: float = 0.7
    guard_x_cm: float = 58.0
    guard_y_cm: float = 0.0
    intercept_offset_cm: float = 34.0
    chase_distance_cm: float = 88.0
    chase_progress_cm: float = 10.0
    chase_retreat_cm: float = -20.0
    hold_speed_gain: float = 1.25
    intercept_speed_gain: float = 2.05
    recovery_speed_gain: float = 1.2
    clear_cooldown_s: float = 0.8
    attack_direction: float = 0.0
    stuck_timeout_s: float = 1.5
    stuck_move_cm: float = 3.0
    wall_margin_cm: float = 30.0


class RuleDefenceStrategy:
    """Conservative defender: guard own box, challenge danger balls, clear if held."""

    def __init__(self, config: DefenceConfig | None = None):
        self.cfg = config or DefenceConfig()
        self.state = "boot"
        self._last_log_ts = 0.0
        self._last_logged_state = None
        self._last_dribble = None
        self._last_clear_ts = -999.0

        # v5: avoid_wall recovery state
        self._recovery_target: list[float] | None = None
        self._recovery_arrived = False
        self._recovery_start_ts: float | None = None
        self._recovery_start_pose: list[float] | None = None

        self.field_long = float(getattr(Settings.Bounds, "Long", 260))
        self.field_short = float(getattr(Settings.Bounds, "Short", 200))
        self.attack_dir = self._resolve_attack_direction()
        self.own_goal_x = -self.attack_dir * self.field_long / 2.0
        self.clear_x = self.attack_dir * (self.field_long / 2.0 - 20.0)
        self.home = self._project_target(
            [
                -self.attack_dir * abs(self.cfg.guard_x_cm),
                self.cfg.guard_y_cm,
                0.0 if self.attack_dir > 0 else 180.0,
            ]
        )

    def _resolve_attack_direction(self) -> float:
        if self.cfg.attack_direction > 0:
            return 1.0
        if self.cfg.attack_direction < 0:
            return -1.0

        try:
            bit_mode = str(getattr(Settings.RoboInfo, "Bit", "")).upper()
            robot_id = int(getattr(getattr(Settings, "Sim", object()), "RobotId", getattr(Settings.RoboInfo, "number", 1)))
            if bit_mode == "SIM" and robot_id >= 3:
                return -1.0
        except Exception:
            pass
        return 1.0

    def step(self) -> None:
        pose = UnitedPosition.FieldRobotPose()

        # 1. Boundary recovery has priority over all ball behavior.
        # Keep the selected recovery target stable until arrival or stuck timeout.
        if self._recovery_target is not None and not self._recovery_arrived:
            if self._recovery_is_stuck(pose):
                self._reset_recovery()
                self._hold_home()
                return

            arrived = UnitedPosition.Pos2Pos(self._recovery_target, self.cfg.recovery_speed_gain)
            if arrived:
                self._reset_recovery(arrived=True)
            self._set_state("avoid_wall")
            self._set_dribble(False)
            return

        safe_target = self._project_recovery_pose(pose)
        if safe_target is not None:
            self._recovery_target = safe_target
            self._recovery_arrived = False
            self._recovery_start_ts = time.time()
            self._recovery_start_pose = list(pose) if valid_pose(pose) else None
            self._set_state("avoid_wall")
            self._set_dribble(False)
            UnitedPosition.Pos2Pos(safe_target, self.cfg.recovery_speed_gain)
            return

        # 2. Ball state decides the defensive mode.
        ball = UnitedPosition.RobotBallPos()
        if ball == BALL_NOT_FOUND:
            self._hold_home()
            return

        if ball == BALL_HELD:
            self._clear_ball(pose)
            return

        # 3. Visible loose ball: challenge danger, otherwise intercept lane or hold.
        ball_field = UnitedPosition.FieldBallPos()
        if not valid_pose(pose) or ball_field in [BALL_NOT_FOUND, BALL_HELD]:
            self._hold_home()
            return

        ball_distance = distance(ball, [0, 0])
        ball_progress = self.attack_dir * float(ball_field[0])
        enemies = self._field_enemies()
        contested_ball = self._enemy_near_ball(ball_field, enemies)
        central_ball = abs(float(ball_field[1])) <= min(self.field_short / 2.0 - 40.0, 58.0)
        deep_guard = abs(self.cfg.guard_x_cm) >= 68.0

        if self._own_goal_mouth_dead_ball(ball_field):
            target = self._goal_mouth_dead_ball_cover_target(ball_field)
            self._set_state("cover_goal_mouth_dead_ball")
            self._set_dribble(False)
            UnitedPosition.Pos2Pos(target, self.cfg.hold_speed_gain)
            return

        if self._wide_own_goal_dead_ball(ball_field):
            target = self._wide_dead_ball_cover_target(ball_field)
            self._set_state("cover_wide_dead_ball")
            self._set_dribble(False)
            UnitedPosition.Pos2Pos(target, self.cfg.hold_speed_gain)
            return

        if deep_guard:
            state, target = self._deep_guard_cover_target(ball_field)
            if state:
                self._set_state(state)
                self._set_dribble(False)
                UnitedPosition.Pos2Pos(target, self.cfg.hold_speed_gain)
                return

        if contested_ball and ball_distance <= self.cfg.chase_distance_cm * 1.75:
            target = self._pressure_target(ball_field, contested=True)
            self._set_state("block_contested_ball")
            self._set_dribble(False)
            UnitedPosition.Pos2Pos(target, self.cfg.intercept_speed_gain)
            return

        # Close loose balls should be challenged instead of waiting on the
        # goal line; otherwise the attacker gets repeated free direct shots.
        if ball_distance <= min(self.cfg.chase_distance_cm, 42.0) and (central_ball or not contested_ball):
            self._set_state("challenge_ball")
            lockball.LockBallSlip()
            return

        # If the ball is still too far to grab, step forward onto the
        # goal-ball lane. This makes 1v1 defence apply pressure at midfield.
        if ball_distance <= self.cfg.chase_distance_cm * 1.55 and ball_progress >= self.cfg.chase_retreat_cm:
            target = self._pressure_target(ball_field)
            self._set_state("press_ball")
            self._set_dribble(False)
            UnitedPosition.Pos2Pos(target, self.cfg.intercept_speed_gain)
            return

        # Ball near midfield and close enough: occupy the defensive lane.
        if ball_distance <= self.cfg.chase_distance_cm and ball_progress <= self.cfg.chase_progress_cm:
            target = self._intercept_target(ball_field)
            self._set_state("intercept_lane")
            self._set_dribble(False)
            UnitedPosition.Pos2Pos(target, self.cfg.intercept_speed_gain)
            return

        state, target = self._cover_shot_target(ball_field)
        if state:
            self._set_state(state)
            self._set_dribble(False)
            UnitedPosition.Pos2Pos(target, self.cfg.intercept_speed_gain)
            return

        self._hold_home()

    def _hold_home(self) -> None:
        self._set_state("hold_home")
        self._set_dribble(False)
        UnitedPosition.Pos2Pos(self.home, self.cfg.hold_speed_gain)

    def _clear_ball(self, pose: list | tuple | None) -> None:
        self._set_state("clear_ball")
        self._set_dribble(0.65)

        if not valid_pose(pose):
            lockball.LockBallSlip()
            return

        target_yaw = UnitedPosition.Pos2Angle([pose[0], pose[1]], [self.clear_x, 0.0]) % 360.0
        lockball.LockBallSlip()
        yaw = float(chassis.GetYaw())
        yaw_err = angle_error_deg(yaw, target_yaw)
        if abs(yaw_err) < float(getattr(Settings.Motion, "ErrorRange", 10)):
            chassis.RobotMoveVector(0.82, 0, 0)
            if time.time() - self._last_clear_ts >= self.cfg.clear_cooldown_s:
                peripheral.ShootBall()
                self._last_clear_ts = time.time()
        else:
            chassis.FieldMoveVector(0, 0, target_yaw, KpW=0.65)

    def _intercept_target(self, ball_field: list | tuple) -> list[float]:
        ball_progress = self.attack_dir * float(ball_field[0])
        guard_progress = -abs(self.cfg.guard_x_cm)
        target_progress = min(guard_progress, ball_progress - abs(self.cfg.intercept_offset_cm))
        target_progress = max(target_progress, -abs(self.cfg.guard_x_cm) - 8.0)

        max_y = min(self.field_short / 2.0 - 32.0, 58.0)
        target_y = clamp(float(ball_field[1]) * 0.75, -max_y, max_y)
        target_x = self.attack_dir * target_progress
        target_yaw = UnitedPosition.Pos2Angle([target_x, target_y], [float(ball_field[0]), float(ball_field[1])]) % 360
        return self._project_target([target_x, target_y, target_yaw])

    def _field_enemies(self) -> list:
        try:
            enemies = UnitedPosition.FieldChassisPos()
        except Exception:
            return []
        return enemies if isinstance(enemies, list) else []

    def _enemy_near_ball(self, ball_field: list | tuple, enemies: list) -> bool:
        for enemy in enemies or []:
            if not enemy or len(enemy) < 2:
                continue
            # Treat an attacker who is closing on the ball as contested before
            # body contact. Waiting for a 30 cm overlap made R3 press into the
            # same lane, then trigger pushing during 1v2 starts.
            if distance([float(enemy[0]), float(enemy[1])], ball_field) <= 48.0:
                return True
        return False

    def _wide_own_goal_dead_ball(self, ball_field: list | tuple) -> bool:
        own_goal_progress = -self.attack_dir * float(ball_field[0])
        wide_y = abs(float(ball_field[1])) >= min(self.field_short / 2.0 - 28.0, 64.0)
        deep_x = own_goal_progress >= self.field_long / 2.0 - 34.0
        return bool(deep_x and wide_y)

    def _own_goal_mouth_dead_ball(self, ball_field: list | tuple) -> bool:
        own_goal_progress = -self.attack_dir * float(ball_field[0])
        central_y = abs(float(ball_field[1])) <= 34.0
        deep_x = own_goal_progress >= self.field_long / 2.0 - 34.0
        return bool(deep_x and central_y)

    def _goal_mouth_dead_ball_cover_target(self, ball_field: list | tuple) -> list[float]:
        guard_x = -self.attack_dir * min(abs(self.cfg.guard_x_cm), 58.0)
        ball_y = float(ball_field[1])
        target_y = -30.0 if ball_y >= 0.0 else 30.0
        target_yaw = UnitedPosition.Pos2Angle([guard_x, target_y], [float(ball_field[0]), ball_y]) % 360
        return self._project_target([guard_x, target_y, target_yaw])

    def _wide_dead_ball_cover_target(self, ball_field: list | tuple) -> list[float]:
        guard_x = -self.attack_dir * min(abs(self.cfg.guard_x_cm), 58.0)
        target_y = clamp(float(ball_field[1]) * 0.28, -24.0, 24.0)
        target_yaw = UnitedPosition.Pos2Angle([guard_x, target_y], [float(ball_field[0]), float(ball_field[1])]) % 360
        return self._project_target([guard_x, target_y, target_yaw])

    def _pressure_target(self, ball_field: list | tuple, *, contested: bool = False) -> list[float]:
        ball_progress = self.attack_dir * float(ball_field[0])
        # Keep contested-ball defence on the goal side of the ball. With the
        # current ball friction, a close pressure target turns into a pushing
        # reset instead of a useful block.
        offset = max(62.0, abs(self.cfg.intercept_offset_cm) + 24.0)
        target_progress = ball_progress - offset
        target_progress = clamp(target_progress, -abs(self.cfg.guard_x_cm) - 16.0, abs(self.cfg.guard_x_cm) - 18.0)

        max_y = min(self.field_short / 2.0 - 34.0, 54.0)
        ball_y = float(ball_field[1])
        if contested and abs(ball_y) <= 28.0:
            # Do not stand directly behind a contested central ball: the goal-box
            # projection pulls the defender to x~=52, so a same-lane target turns
            # into a pushing call. Use the opposite shoulder to block without ramming.
            side = -1.0 if ball_y > 0.0 else 1.0
            target_y = side * 44.0
        else:
            target_y = clamp(ball_y * 0.70, -max_y, max_y)
        target_x = self.attack_dir * target_progress
        target_yaw = UnitedPosition.Pos2Angle([target_x, target_y], [float(ball_field[0]), float(ball_field[1])]) % 360
        return self._project_target([target_x, target_y, target_yaw])

    def _cover_shot_target(self, ball_field: list | tuple):
        state, target_x, target_y = choose_defence_cover(ball_field, self.attack_dir, self.field_short)
        if not state:
            return "", None
        target_yaw = UnitedPosition.Pos2Angle([target_x, target_y], [float(ball_field[0]), float(ball_field[1])]) % 360
        return state, self._project_target([target_x, target_y, target_yaw])

    def _deep_guard_cover_target(self, ball_field: list | tuple):
        state, target = self._cover_shot_target(ball_field)
        if not state or target is None:
            return state, target

        # Staying too deep inside the goal/penalty mouth makes ProjectSafePose
        # push the goalie back into the lane. Keep the layered guard outside
        # that restricted box, and separate it laterally from the ball carrier.
        guard_x = -self.attack_dir * min(abs(self.cfg.guard_x_cm), 58.0)
        max_y = min(self.field_short / 2.0 - 38.0, 46.0)
        ball_y = float(ball_field[1])
        if state == "cover_direct_shot":
            target_y = -34.0 if ball_y > 0.0 else 34.0
        else:
            target_y = clamp(float(target[1]) * 0.82, -max_y, max_y)
        target_yaw = UnitedPosition.Pos2Angle([guard_x, target_y], [float(ball_field[0]), float(ball_field[1])]) % 360
        return state, self._project_target([guard_x, target_y, target_yaw])

    def _recovery_is_stuck(self, pose: list | tuple | None) -> bool:
        if self._recovery_start_ts is None or self._recovery_start_pose is None:
            return False
        if time.time() - self._recovery_start_ts <= self.cfg.stuck_timeout_s:
            return False
        if not valid_pose(pose) or not valid_pose(self._recovery_start_pose):
            return False
        return distance(pose, self._recovery_start_pose) < self.cfg.stuck_move_cm

    def _reset_recovery(self, *, arrived: bool = False) -> None:
        self._recovery_target = None
        self._recovery_arrived = arrived
        self._recovery_start_ts = None
        self._recovery_start_pose = None

    def _project_recovery_pose(self, pose: list | tuple | None):
        if not valid_pose(pose):
            return None
        # Only trigger ProjectSafePose when truly near field boundary.
        # ProjectSafePose's goal-box check is too aggressive for defensive
        # positions (e.g. x=-67, y=50 triggers a false positive), so we
        # gate it with a simple boundary proximity check first.
        x, y = float(pose[0]), float(pose[1])
        half_long = self.field_long / 2.0
        half_short = self.field_short / 2.0
        margin = max(20.0, float(self.cfg.wall_margin_cm))
        near_boundary = abs(x) > (half_long - margin) or abs(y) > (half_short - margin)
        if not near_boundary:
            return None
        try:
            return UnitedPosition.ProjectSafePose(pose)
        except Exception:
            return None

    def _project_target(self, target: list[float]) -> list[float]:
        try:
            projected = UnitedPosition.ProjectSafePose(target)
        except Exception:
            projected = None
        return projected if projected is not None else target

    def _set_state(self, state: str) -> None:
        self.state = state
        now = time.time()
        if state != self._last_logged_state or now - self._last_log_ts >= self.cfg.log_every_s:
            logger.info("[defence] state={}", state)
            self._last_logged_state = state
            self._last_log_ts = now

    def _set_dribble(self, status) -> None:
        if self._last_dribble == status:
            return
        peripheral.Dribble(status)
        self._last_dribble = status

    def stop(self) -> None:
        safe_stop(chassis, peripheral, UnitedPosition)


_defence_strategy = None


def DefenceStep() -> None:
    """老式非阻塞防守入口：每轮调用一次。"""

    global _defence_strategy
    if _defence_strategy is None:
        _defence_strategy = RuleDefenceStrategy()
    _defence_strategy.step()


def DefenceStop() -> None:
    global _defence_strategy
    if _defence_strategy is not None:
        _defence_strategy.stop()
        _defence_strategy = None


def main() -> None:
    try:
        while True:
            DefenceStep()
            time.sleep(_defence_strategy.cfg.loop_dt if _defence_strategy else DefenceConfig().loop_dt)
    finally:
        DefenceStop()


if __name__ == "__main__":
    main()
