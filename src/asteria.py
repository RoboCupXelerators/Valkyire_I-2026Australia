"""Asteria：面向 0.1.7e 的无参数双机协作策略。
每拍只做一次观察和一次动作选择。它复用现有接球、带球和安全位移，
采用 Hyperion 的连续评分，并让两台同队机器人确定性分工而不争抢自由球。
"""
from __future__ import annotations
import math
import os
import time
from BasicFuc import lockball
from LunaPre import Bits, UnitedPosition, chassis, peripheral
from LunaPre.strategy_tools import angle_error_deg, clamp, safe_stop, valid_pose
import rule_candidate_common as helper
class Asteria:
    """Keep all cross-frame and team-role state inside one robot process."""
    # 30 Hz matches the existing candidates' safe simulator control cadence.
    LOOP_DT_S = 0.03
    # Start final docking before a free-ball approach can stall near 22 cm from
    # the pose origin; that is a motion-deadzone observation, not robot size.
    CAPTURE_DISTANCE_CM = 25.0
    # Leave 10 cm of approach error so the command remains above Unity's motion deadzone.
    APPROACH_STANDOFF_CM = 10.0
    # Keep recovery targets reachable near the 0.1.7e goal-box boundary.
    APPROACH_X_LIMIT_CM, APPROACH_Y_LIMIT_CM = 85.0, 70.0
    # A full 6 m/s kick remains accurate from x=44 cm and fires well before the
    # x=65 cm goal-box guard, reducing the chance that a slow carrier is blocked.
    FINISH_PROGRESS_CM, FINISH_LATERAL_CM = 44.0, 22.0
    # At roughly 50 cm, 18 degrees misses the centre by 16 cm, still inside
    # the 30 cm goal half-width while avoiding an endless in-place aim loop.
    AIM_TOLERANCE_DEG, SHOT_COOLDOWN_S = 18.0, 0.8
    # Preserve roller contact for one 0.15 s window before releasing a level-4 (6 m/s) kick.
    KICK_HOLD_S, KICK_POWER_LEVEL = 0.15, 4
    # Begin centralisation at 30 cm, well inside the 60 cm simulated goal mouth.
    RECENTER_START_Y_CM = 30.0
    # A 0.30 score lead prevents a 20–29 cm shoulder lane from reversing a
    # correctly aimed carrier; the hard 30 cm boundary still forces recovery.
    RECENTER_SCORE_ADVANTAGE = 0.30
    # A 0.42 m/s recenter crosses the 30 cm lane boundary before a nearby
    # chassis can pin the carrier against the 80 cm safety envelope.
    CARRY_SPEED_MPS, RECENTER_SPEED_MPS = 0.58, 0.42
    APPROACH_KP_MPS_PER_CM, APPROACH_X_LIMIT_MPS, APPROACH_Y_LIMIT_MPS = 0.014, 0.54, 0.38
    # Two teammate-separated search shoulders include y=58 cm to reacquire edge balls.
    SEARCH_STAGE_S, SEARCH_POINTS = 2.4, ((-24.0, 58.0), (16.0, -58.0))
    # A free ball on our half belongs to the even defender; otherwise odd attacker owns it.
    OWN_HALF_OWNER_CM = 0.0
    # In the 8 cm centre strip, ownership holds for 4 s—long enough for an
    # approach/docking attempt—then swaps if kickoff is still stalled.
    CENTER_CONTEST_HALF_WIDTH_CM, CENTER_HANDOFF_S = 8.0, 4.0
    # Inside the 25 cm docking window local camera data is more reliable than
    # a teammate role; permit either robot to finish the pickup instead of wait.
    EMERGENCY_CAPTURE_DISTANCE_CM = CAPTURE_DISTANCE_CM
    # Non-owner stands 28 cm behind and 30 cm sideways from a ball, outside capture range.
    SUPPORT_STANDOFF_CM, SUPPORT_LANE_CM = 28.0, 30.0
    # Goal cover stays outside the box at x=54 cm and tracks no farther than y=28 cm.
    GOAL_GUARD_X_CM, GOAL_GUARD_Y_CM = 54.0, 28.0
    # Chassis within 38 cm of the ball creates pressure; 46 by 15 cm blocks a carry lane.
    PRESSURE_DISTANCE_CM, LANE_BLOCK_AHEAD_CM, LANE_BLOCK_HALF_WIDTH_CM = 38.0, 46.0, 15.0
    # At 12 cm depth one non-guard presses; beyond 38 cm the defender stays at goal.
    LOST_DEEP_THREAT_CM, LOST_PRESS_DEPTH_CM, PRESS_STANDOFF_CM, PRESS_LATERAL_CM = 38.0, 12.0, 14.0, 10.0
    def __init__(self) -> None:
        self.last_shot_at = -math.inf
        self.kick_hold_until = -math.inf
        self.kick_release_pending = False
        self.search_started_at = time.monotonic()
        self.last_state = "boot"
        self.last_role = "boot"
    @staticmethod
    def _robot_id() -> int:
        try:
            return int(os.environ.get("XEL_ROBOT_ID", "1"))
        except ValueError:
            return 1
    def _is_defender_slot(self) -> bool:
        return self._robot_id() % 2 == 0
    def _lane_side(self) -> float:
        return -1.0 if self._is_defender_slot() else 1.0
    @staticmethod
    def _goal_yaw(direction: float) -> float:
        return 0.0 if direction > 0.0 else 180.0
    def _set_state(self, state: str) -> str:
        self.last_state = state
        return state
    def _move_to(self, pose, target) -> None:
        safe_target = helper._safe_target(target)
        error_x, error_y = float(safe_target[0]) - float(pose[0]), float(safe_target[1]) - float(pose[1])
        chassis.FieldMoveVector(
            clamp(error_x * self.APPROACH_KP_MPS_PER_CM, -self.APPROACH_X_LIMIT_MPS, self.APPROACH_X_LIMIT_MPS),
            clamp(error_y * self.APPROACH_KP_MPS_PER_CM, -self.APPROACH_Y_LIMIT_MPS, self.APPROACH_Y_LIMIT_MPS),
            float(safe_target[2]), KpW=0.75,
        )
    def _opponents(self, pose) -> list[list[float]]:
        """Use 0.1.7e chassis feed, with FieldChassisPos as the camera fallback."""
        try:
            raw_positions = Bits.GetEnemyPos()
            heading = math.radians(float(pose[2]))
        except Exception:
            raw_positions = []
        opponents = []
        for point in raw_positions if isinstance(raw_positions, list) else []:
            try:
                rel_x, rel_y = float(point[0]), float(point[1])
            except (IndexError, TypeError, ValueError):
                continue
            if math.isfinite(rel_x) and math.isfinite(rel_y):
                opponents.append([float(pose[0]) + rel_x * math.cos(heading) - rel_y * math.sin(heading), float(pose[1]) + rel_x * math.sin(heading) + rel_y * math.cos(heading)])
        if opponents:
            self.last_opponents = opponents
            return opponents
        try:
            raw_positions = UnitedPosition.FieldChassisPos()
        except Exception:
            return []
        for point in raw_positions if isinstance(raw_positions, list) else []:
            try:
                x, y = float(point[0]), float(point[1])
            except (IndexError, TypeError, ValueError):
                continue
            if math.isfinite(x) and math.isfinite(y):
                opponents.append([x, y])
        return opponents or getattr(self, "last_opponents", [])
    @staticmethod
    def _nearest(point, opponents):
        return min(opponents, key=lambda item: math.hypot(item[0] - point[0], item[1] - point[1]), default=None)
    def _under_pressure(self, field_ball, opponents) -> bool:
        enemy = self._nearest(field_ball, opponents)
        return enemy is not None and math.hypot(enemy[0] - field_ball[0], enemy[1] - field_ball[1]) <= self.PRESSURE_DISTANCE_CM
    def _free_ball_owner(self, field_ball, direction: float) -> bool:
        """Both teammates reach the identical decision, so exactly one can lock a free ball."""
        progress = direction * float(field_ball[0])
        if abs(progress) <= self.CENTER_CONTEST_HALF_WIDTH_CM:
            attacker_turn = int(time.monotonic() / self.CENTER_HANDOFF_S) % 2 == 0
            return self._is_defender_slot() != attacker_turn
        return self._is_defender_slot() == (progress <= self.OWN_HALF_OWNER_CM)
    def _guard_target(self, focus, direction: float):
        return [-direction * self.GOAL_GUARD_X_CM, clamp(float(focus[1]) * 0.55, -self.GOAL_GUARD_Y_CM, self.GOAL_GUARD_Y_CM), self._goal_yaw(direction)]
    def _support(self, pose, field_ball, direction: float, opponents) -> str:
        own_half = direction * float(field_ball[0]) <= self.OWN_HALF_OWNER_CM
        peripheral.Dribble(False)
        if own_half or self._under_pressure(field_ball, opponents):
            self.last_role = "goal_guard"
            self._move_to(pose, self._guard_target(field_ball, direction))
            return self._set_state("free_ball_goal_guard")
        self.last_role = "support"
        self._move_to(pose, [
            clamp(float(field_ball[0]) - direction * self.SUPPORT_STANDOFF_CM, -self.APPROACH_X_LIMIT_CM, self.APPROACH_X_LIMIT_CM),
            clamp(float(field_ball[1]) + self._lane_side() * self.SUPPORT_LANE_CM, -self.APPROACH_Y_LIMIT_CM, self.APPROACH_Y_LIMIT_CM),
            self._goal_yaw(direction),
        ])
        return self._set_state("free_ball_support")
    def _search(self, pose, direction: float) -> str:
        stage = (int((time.monotonic() - self.search_started_at) / self.SEARCH_STAGE_S) + int(self._is_defender_slot())) % len(self.SEARCH_POINTS)
        x, y = self.SEARCH_POINTS[stage]
        if lockball.LockBallSlip():
            self.last_role = "search_capture"
            return self._set_state("search_capture")
        peripheral.Dribble(False)
        self._move_to(pose, [direction * x, self._lane_side() * y, self._goal_yaw(direction)])
        return self._set_state("search_ball")
    @staticmethod
    def _sim_field_ball(pose):
        if os.environ.get("XEL_SIM_BALL_STATUS_FALLBACK", "0").lower() not in {"1", "true", "yes"}: return None
        try:
            relative = Bits.GetBallPos()
            if not helper._valid_ball(relative): return None
            heading = math.radians(float(pose[2])); c, s = math.cos(heading), math.sin(heading)
            return [float(pose[0]) + float(relative[0]) * c - float(relative[1]) * s, float(pose[1]) + float(relative[0]) * s + float(relative[1]) * c]
        except Exception: return None
    def _lost_ball(self, pose, direction: float, opponents) -> str:
        """With no ball coordinate, choose one guard and one patrol from enemy chassis threat."""
        danger = max(opponents, key=lambda enemy: -direction * enemy[0], default=None)
        danger_depth = -direction * danger[0] if danger is not None else -math.inf
        guard_is_defender = danger_depth >= self.LOST_DEEP_THREAT_CM
        if danger is None:
            guard_is_defender = True
        if self._is_defender_slot() == guard_is_defender:
            peripheral.Dribble(False)
            self.last_role = "goal_guard"
            self._move_to(pose, self._guard_target(danger or [0.0, 0.0], direction))
            return self._set_state("lost_goal_guard")
        if danger is not None and danger_depth >= self.LOST_PRESS_DEPTH_CM:
            self.last_role = "chassis_press"; peripheral.Dribble(0.75)
            self._move_to(pose, [clamp(danger[0] - direction * self.PRESS_STANDOFF_CM, -self.APPROACH_X_LIMIT_CM, self.APPROACH_X_LIMIT_CM), clamp(danger[1] + self._lane_side() * self.PRESS_LATERAL_CM, -self.APPROACH_Y_LIMIT_CM, self.APPROACH_Y_LIMIT_CM), self._goal_yaw(direction)])
            return self._set_state("lost_chassis_press")
        self.last_role = "threat_patrol" if danger is not None else "search"
        return self._search(pose, direction)
    def _collect(self, pose, robot_ball, direction: float, opponents, field_ball=None) -> str:
        if field_ball is None:
            field_ball = UnitedPosition.FieldBallPos()
        if not helper._valid_ball(field_ball):
            return self._lost_ball(pose, direction, opponents)
        field_distance = math.hypot(float(field_ball[0]) - float(pose[0]), float(field_ball[1]) - float(pose[1]))
        if ((helper._valid_ball(robot_ball) and math.hypot(float(robot_ball[0]), float(robot_ball[1])) <= self.CAPTURE_DISTANCE_CM) or field_distance <= self.CAPTURE_DISTANCE_CM):
            peripheral.Dribble(0.82)
            lockball.LockBallSlip()
            self.last_role = "ball_owner"
            return self._set_state("lock_ball")
        enemy = self._nearest(field_ball, opponents)
        side = self._lane_side() if enemy is None or not self._under_pressure(field_ball, opponents) else (-1.0 if enemy[1] >= field_ball[1] else 1.0)
        peripheral.Dribble(0.82)
        self.last_role = "ball_owner"
        self._move_to(pose, [
            clamp(float(field_ball[0]) - direction * self.APPROACH_STANDOFF_CM, -self.APPROACH_X_LIMIT_CM, self.APPROACH_X_LIMIT_CM),
            clamp(float(field_ball[1]) + side * 8.0, -self.APPROACH_Y_LIMIT_CM, self.APPROACH_Y_LIMIT_CM),
            self._goal_yaw(direction),
        ])
        return self._set_state("approach_ball")
    def _lane_blocker(self, pose, direction: float, opponents):
        for enemy in opponents:
            ahead = direction * (enemy[0] - float(pose[0]))
            if 0.0 < ahead <= self.LANE_BLOCK_AHEAD_CM and abs(enemy[1] - float(pose[1])) <= self.LANE_BLOCK_HALF_WIDTH_CM:
                return enemy
        return None
    def _direct_score(self, pose, direction: float) -> float:
        progress = clamp(direction * float(pose[0]) / self.FINISH_PROGRESS_CM, 0.0, 1.0)
        alignment = 1.0 - abs(angle_error_deg(float(pose[2]), self._goal_yaw(direction))) / 180.0
        centrality = 1.0 - clamp(abs(float(pose[1])) / self.RECENTER_START_Y_CM, 0.0, 1.0)
        return 0.55 * alignment + 0.30 * progress + 0.15 * centrality
    def _attack(self, pose, direction: float, opponents) -> str:
        now, goal_yaw = time.monotonic(), self._goal_yaw(direction)
        progress, lateral = direction * float(pose[0]), abs(float(pose[1]))
        yaw_error = abs(angle_error_deg(float(pose[2]), goal_yaw))
        if progress >= self.FINISH_PROGRESS_CM and lateral <= self.FINISH_LATERAL_CM:
            peripheral.Dribble(0.90)
            if yaw_error > self.AIM_TOLERANCE_DEG:
                chassis.FieldMoveVector(direction * 0.16, clamp(-float(pose[1]) * 0.010, -0.12, 0.12), goal_yaw, KpW=0.78)
                return self._set_state("aim_goal_carry")
            if now - self.last_shot_at >= self.SHOT_COOLDOWN_S:
                chassis.stop()
                peripheral.ShootBall(self.KICK_POWER_LEVEL)
                self.last_shot_at, self.kick_hold_until, self.kick_release_pending = now, now + self.KICK_HOLD_S, True
                return self._set_state("shoot")
            return self._set_state("shot_cooldown")
        lane_score = 0.70 * clamp(lateral / self.RECENTER_START_Y_CM, 0.0, 1.0) + 0.30 * clamp(progress / self.FINISH_PROGRESS_CM, 0.0, 1.0)
        if lateral >= self.RECENTER_START_Y_CM or lane_score > self._direct_score(pose, direction) + self.RECENTER_SCORE_ADVANTAGE:
            peripheral.Dribble(0.86)
            chassis.FieldMoveVector(-direction * 0.10, clamp(-float(pose[1]) * 0.012, -self.RECENTER_SPEED_MPS, self.RECENTER_SPEED_MPS), goal_yaw, KpW=0.76)
            return self._set_state("recenter_ball")
        blocker = self._lane_blocker(pose, direction, opponents)
        if blocker is not None:
            peripheral.Dribble(0.86)
            dodge = -1.0 if blocker[1] >= float(pose[1]) else 1.0
            chassis.FieldMoveVector(direction * 0.20, dodge * 0.22, goal_yaw, KpW=0.78)
            return self._set_state("carry_avoid_chassis")
        peripheral.Dribble(0.88)
        chassis.FieldMoveVector(direction * self.CARRY_SPEED_MPS, clamp(-float(pose[1]) * 0.010, -0.16, 0.16), goal_yaw, KpW=0.78)
        return self._set_state("carry_centre")
    def step(self) -> str:
        """Execute one bounded observe → assign role → act tick."""
        pose = UnitedPosition.FieldRobotPose()
        if not valid_pose(pose):
            chassis.stop()
            peripheral.Dribble(False)
            return self._set_state("invalid_pose_stop")
        if not UnitedPosition.RestrictedArea():
            peripheral.Dribble(False)  # RestrictedArea already sent its safe recovery command.
            return self._set_state("restricted_recover")
        if time.monotonic() < self.kick_hold_until:
            peripheral.Dribble(0.90)
            return self._set_state("kick_hold")
        if self.kick_release_pending:
            peripheral.Dribble(False)
            self.kick_release_pending = False
            return self._set_state("kick_release")
        direction, robot_ball = helper.attack_direction(), UnitedPosition.RobotBallPos()
        ball_state, opponents = helper._ball_state(robot_ball), self._opponents(pose)
        if ball_state == "lost":
            sim_ball = self._sim_field_ball(pose)
            if helper._valid_ball(sim_ball) and self._free_ball_owner(sim_ball, direction):
                return self._collect(pose, robot_ball, direction, opponents, sim_ball)
            return self._lost_ball(pose, direction, opponents)
        if ball_state == "free":
            # A local dribbler-window observation is stronger than a delayed or
            # occluded field coordinate: do not let role courtesy discard a ball
            # already physically reachable by this robot.
            near_ball = helper._valid_ball(robot_ball) and math.hypot(float(robot_ball[0]), float(robot_ball[1])) <= self.EMERGENCY_CAPTURE_DISTANCE_CM
            if near_ball:
                peripheral.Dribble(0.82)
                lockball.LockBallSlip()
                self.last_role = "close_claim"
                return self._set_state("lock_ball")
            field_ball = UnitedPosition.FieldBallPos()
            if not helper._valid_ball(field_ball):
                return self._lost_ball(pose, direction, opponents)
            return self._collect(pose, robot_ball, direction, opponents) if self._free_ball_owner(field_ball, direction) else self._support(pose, field_ball, direction, opponents)
        self.last_role = "ball_carrier"
        return self._attack(pose, direction, opponents)
controller = Asteria()
def step() -> str:
    """Public one-tick entrypoint; robot identity comes from the shared simulator runtime."""
    return controller.step()
def main() -> None:
    try:
        while True:
            step()
            time.sleep(Asteria.LOOP_DT_S)
    except KeyboardInterrupt:
        pass
    finally:
        safe_stop(chassis, peripheral, UnitedPosition)
if __name__ == "__main__":
    main()
