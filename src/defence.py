"""Rule-based defensive role.

The old defence entrypoint was a blocking loop. This version mirrors
`offence.RuleOffenceStrategy`: one `step()` call performs one control tick, so
multi-robot runners can sample traces and stop cleanly.
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


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _valid_pose(pose: list | tuple | None) -> bool:
    return bool(pose) and len(pose) >= 3 and list(pose[:3]) != [0xFFF, 0xFFF, 0xFFF]


@dataclass(frozen=True)
class DefenceConfig:
    loop_dt: float = float(os.environ.get("DEFENCE_DT", "0.03"))
    log_every_s: float = float(os.environ.get("DEFENCE_LOG_EVERY", "0.7"))
    guard_x_cm: float = float(os.environ.get("DEFENCE_GUARD_X", "58"))
    guard_y_cm: float = float(os.environ.get("DEFENCE_GUARD_Y", "0"))
    intercept_offset_cm: float = float(os.environ.get("DEFENCE_INTERCEPT_OFFSET", "28"))
    chase_distance_cm: float = float(os.environ.get("DEFENCE_CHASE_DISTANCE", "82"))
    chase_progress_cm: float = float(os.environ.get("DEFENCE_CHASE_PROGRESS", "18"))
    hold_speed_gain: float = float(os.environ.get("DEFENCE_HOLD_GAIN", "1.1"))
    intercept_speed_gain: float = float(os.environ.get("DEFENCE_INTERCEPT_GAIN", "1.7"))
    recovery_speed_gain: float = float(os.environ.get("DEFENCE_BOUNDARY_RECOVERY_GAIN", "1.2"))
    clear_cooldown_s: float = float(os.environ.get("DEFENCE_CLEAR_COOLDOWN", "0.8"))
    attack_direction: float = float(os.environ.get("DEFENCE_ATTACK_SIGN", os.environ.get("OFFENCE_ATTACK_SIGN", "0")))


class RuleDefenceStrategy:
    """Conservative defender: guard own box, challenge danger balls, clear if held."""

    def __init__(self, config: DefenceConfig | None = None):
        self.cfg = config or DefenceConfig()
        self.state = "boot"
        self._last_log_ts = 0.0
        self._last_logged_state = None
        self._last_dribble = None
        self._last_clear_ts = -999.0

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

    def step(self) -> None:
        pose = UnitedPosition.FieldRobotPos()
        safe_target = self._project_recovery_pose(pose)
        if safe_target is not None:
            self._set_state("avoid_wall")
            self._set_dribble(False)
            UnitedPosition.Pos2Pos(safe_target, self.cfg.recovery_speed_gain)
            return

        ball = UnitedPosition.RobotBallPos()
        if ball == BALL_NOT_FOUND:
            self._hold_home()
            return

        if ball == BALL_HELD:
            self._clear_ball(pose)
            return

        self._defend_visible_ball(pose, ball)

    def _hold_home(self) -> None:
        self._set_state("hold_home")
        self._set_dribble(False)
        UnitedPosition.Pos2Pos(self.home, self.cfg.hold_speed_gain)

    def _clear_ball(self, pose: list | tuple | None) -> None:
        self._set_state("clear_ball")
        self._set_dribble(0.65)

        if not _valid_pose(pose):
            lockball.LockBallSlip()
            return

        target_yaw = UnitedPosition.Pos2Angle([pose[0], pose[1]], [self.clear_x, 0.0]) % 360.0
        lockball.LockBallSlip()
        yaw = float(chassis.GetYaw())
        yaw_err = (target_yaw - yaw + 180.0) % 360.0 - 180.0
        if abs(yaw_err) < float(getattr(Settings.Motion, "ErrorRange", 10)):
            chassis.RobotMoveVector(90, 0, 0)
            if time.time() - self._last_clear_ts >= self.cfg.clear_cooldown_s:
                peripheral.ShootBall()
                self._last_clear_ts = time.time()
        else:
            chassis.FieldMoveVector(0, 0, target_yaw, KpW=0.65)

    def _defend_visible_ball(self, pose: list | tuple | None, ball: list | tuple) -> None:
        ball_field = UnitedPosition.FieldBallPos()
        if not _valid_pose(pose) or ball_field in (BALL_NOT_FOUND, BALL_HELD):
            self._hold_home()
            return

        bx_rel = float(ball[0])
        by_rel = float(ball[1])
        ball_distance = math.hypot(bx_rel, by_rel)
        ball_progress = self.attack_dir * float(ball_field[0])

        if ball_distance <= self.cfg.chase_distance_cm and ball_progress <= self.cfg.chase_progress_cm:
            self._set_state("challenge_ball")
            lockball.LockBallSlip()
            return

        target = self._intercept_target(ball_field)
        self._set_state("intercept_lane")
        self._set_dribble(False)
        UnitedPosition.Pos2Pos(target, self.cfg.intercept_speed_gain)

    def _intercept_target(self, ball_field: list | tuple) -> list[float]:
        ball_progress = self.attack_dir * float(ball_field[0])
        guard_progress = -abs(self.cfg.guard_x_cm)
        target_progress = min(guard_progress, ball_progress - abs(self.cfg.intercept_offset_cm))
        target_progress = max(target_progress, -abs(self.cfg.guard_x_cm) - 8.0)

        max_y = min(self.field_short / 2.0 - 32.0, 58.0)
        target_y = _clamp(float(ball_field[1]) * 0.75, -max_y, max_y)
        target_x = self.attack_dir * target_progress
        target_yaw = UnitedPosition.Pos2Angle([target_x, target_y], [float(ball_field[0]), float(ball_field[1])]) % 360
        return self._project_target([target_x, target_y, target_yaw])

    def _project_recovery_pose(self, pose: list | tuple | None):
        if not _valid_pose(pose):
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
    strategy = RuleDefenceStrategy()
    try:
        while True:
            strategy.step()
            time.sleep(strategy.cfg.loop_dt)
    finally:
        strategy.stop()


if __name__ == "__main__":
    main()
