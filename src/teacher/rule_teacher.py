"""Pure rule teacher for behaviour cloning and regression debugging."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import os
from typing import Any

from mlp.contracts import (
    DecisionPacketV1,
    DeployObservationV1,
    DribbleMode,
    FaceMode,
    IntentId,
    ShootIntent,
)
from LunaPre.strategy_tools import angle_error_deg as _angle_error_deg
from LunaPre.strategy_tools import clamp as _clamp


@dataclass(frozen=True)
class TeacherConfig:
    field_long: float = 260.0
    field_short: float = 200.0
    goal_width: float = 90.0
    lane_block_width_cm: float = float(os.environ.get("OFFENCE_LANE_BLOCK_WIDTH", "24"))
    carry_x_cm: float = float(os.environ.get("OFFENCE_CARRY_X", "82"))
    deep_shoot_x_cm: float = float(os.environ.get("OFFENCE_DEEP_SHOOT_X", "58"))
    shoot_yaw_tolerance_deg: float = float(os.environ.get("OFFENCE_SHOOT_YAW_TOL", "8"))
    search_speed_gain: float = float(os.environ.get("OFFENCE_SEARCH_GAIN", "1.4"))
    carry_speed_gain: float = float(os.environ.get("OFFENCE_CARRY_GAIN", "2.2"))
    boundary_recovery_speed_gain: float = float(os.environ.get("OFFENCE_BOUNDARY_RECOVERY_GAIN", "1.2"))
    x_limit: float = 90.0
    y_limit: float = 70.0
    goal_box_width: float = 25.0
    goal_box_y_half: float = 50.0
    clearance: float = 5.0
    recovery_clearance: float = 16.0


@dataclass
class TeacherDecision:
    packet: DecisionPacketV1
    state: str
    reasons: list[str] = field(default_factory=list)
    safety: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "packet": self.packet.to_dict(),
            "state": self.state,
            "reasons": list(self.reasons),
            "safety": dict(self.safety),
        }


class RuleTeacher:
    """Rule-based high-level policy that never touches hardware directly."""

    def __init__(self, config: TeacherConfig | None = None):
        self.config = config or self._load_config()
        self.search_idx = 0
        self.last_state = "boot"

    def decide(self, obs: DeployObservationV1) -> TeacherDecision:
        reasons: list[str] = []

        safe_target = self.project_safe_pose([obs.self_x, obs.self_y, obs.self_yaw])
        if safe_target is not None:
            reasons.append("pose_outside_safe_region")
            packet = DecisionPacketV1(
                intent_id=int(IntentId.AVOID_WALL),
                target_x=safe_target[0],
                target_y=safe_target[1],
                target_yaw=safe_target[2],
                face_mode=int(FaceMode.FACE_KEEP),
                move_gain=self.config.boundary_recovery_speed_gain,
                dribble_mode=int(DribbleMode.OFF),
                shoot_intent=int(ShootIntent.NONE),
                confidence=1.0,
            )
            return self._decision("avoid_wall", packet, reasons, obs)

        if obs.has_ball:
            return self._decide_with_ball(obs, reasons)

        if not obs.ball_visible:
            return self._decide_search(obs, reasons)

        return self._decide_approach(obs, reasons)

    def project_safe_pose(self, pose: list[float] | tuple[float, ...] | None) -> list[float] | None:
        """检查位姿是否安全，返回修正后的目标位置
        
        只在真正出界时才触发修正，避免在禁区边缘反复震荡
        """
        if not pose or len(pose) < 3:
            return None

        x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
        cfg = self.config
        
        # 边界限制
        guard_x = max(0.0, cfg.x_limit - cfg.clearance)
        guard_y = max(0.0, cfg.y_limit - cfg.clearance)
        recover_x = max(0.0, cfg.x_limit - cfg.recovery_clearance)
        recover_y = max(0.0, cfg.y_limit - cfg.recovery_clearance)
        
        # 禁区边界
        goal_box_x_min = max(0.0, cfg.x_limit - cfg.goal_box_width)

        target_x = x
        target_y = y
        corrected = False

        # X 方向出界检测
        if x > guard_x:
            target_x = recover_x
            corrected = True
        elif x < -guard_x:
            target_x = -recover_x
            corrected = True

        # Y 方向出界检测
        if y > guard_y:
            target_y = recover_y
            corrected = True
        elif y < -guard_y:
            target_y = -recover_y
            corrected = True

        # 禁区检测：只有在真正进入禁区时才修正
        # 禁区条件：X 超过禁区线 且 Y 在禁区范围内
        in_goal_box = abs(x) > goal_box_x_min and abs(y) < cfg.goal_box_y_half
        
        if in_goal_box:
            # 禁区修正策略：优先后退（X方向），不侧移到更远的地方
            sign_x = 1.0 if x >= 0 else -1.0
            
            # 往后退到禁区线外面
            retreat_x = sign_x * max(0.0, goal_box_x_min - cfg.clearance)
            target_x = retreat_x
            
            # Y 方向：保持当前位置或往中间走（不往外走！）
            # 只有当 Y 在禁区边缘时才需要往中间移动
            if abs(y) > cfg.goal_box_y_half - cfg.clearance:
                # 接近禁区 Y 边缘，往中间移动一点
                target_y = y * 0.8  # 收缩到 80%，保持方向但靠近中间
                target_y = _clamp(target_y, -cfg.goal_box_y_half, cfg.goal_box_y_half)
            
            corrected = True

        if not corrected:
            return None
        return [target_x, target_y, yaw]

    def path_is_safe(self, start: list[float], end: list[float], samples: int = 10) -> bool:
        for idx in range(1, max(2, samples) + 1):
            ratio = idx / float(max(2, samples))
            pose = [
                float(start[0]) + (float(end[0]) - float(start[0])) * ratio,
                float(start[1]) + (float(end[1]) - float(start[1])) * ratio,
                float(start[2]) if len(start) > 2 else 0.0,
            ]
            if self.project_safe_pose(pose) is not None:
                return False
        return True

    def _decide_search(self, obs: DeployObservationV1, reasons: list[str]) -> TeacherDecision:
        reasons.append("ball_not_visible")
        target = self._search_points()[self.search_idx]
        if abs(obs.self_x - target[0]) < 10 and abs(obs.self_y - target[1]) < 10:
            self.search_idx = (self.search_idx + 1) % len(self._search_points())
            target = self._search_points()[self.search_idx]
            reasons.append("advance_search_point")

        packet = DecisionPacketV1(
            intent_id=int(IntentId.SEARCH_BALL),
            target_x=target[0],
            target_y=target[1],
            target_yaw=target[2],
            face_mode=int(FaceMode.FACE_KEEP),
            move_gain=self.config.search_speed_gain,
            dribble_mode=int(DribbleMode.OFF),
            shoot_intent=int(ShootIntent.NONE),
        )
        return self._decision("search_ball", packet, reasons, obs)

    def _decide_approach(self, obs: DeployObservationV1, reasons: list[str]) -> TeacherDecision:
        distance = math.hypot(obs.ball_rel_x, obs.ball_rel_y)
        reasons.append("ball_visible")
        reasons.append(f"ball_distance={distance:.1f}")
        dribble = DribbleMode.WEAK if distance > 70.0 and abs(obs.ball_rel_y) < 45.0 else DribbleMode.STRONG
        packet = DecisionPacketV1(
            intent_id=int(IntentId.APPROACH_BALL),
            face_mode=int(FaceMode.FACE_BALL),
            move_gain=1.0,
            dribble_mode=int(dribble),
            shoot_intent=int(ShootIntent.NONE),
        )
        return self._decision("approach_ball", packet, reasons, obs)

    def _decide_with_ball(self, obs: DeployObservationV1, reasons: list[str]) -> TeacherDecision:
        aim_y = self._best_goal_aim_y(obs)
        goal_yaw = self._pos_to_angle([obs.self_x, obs.self_y], [self.config.field_long / 2.0, aim_y]) % 360.0
        yaw_err = abs(_angle_error_deg(obs.self_yaw, goal_yaw))
        reasons.append("has_ball")
        reasons.append(f"goal_yaw={goal_yaw:.1f}")

        if self._can_shoot(obs, [self.config.field_long / 2.0, aim_y]):
            reasons.append("shoot_window_open")
            if yaw_err <= self.config.shoot_yaw_tolerance_deg:
                packet = DecisionPacketV1(
                    intent_id=int(IntentId.SHOOT),
                    target_x=obs.self_x,
                    target_y=obs.self_y,
                    target_yaw=goal_yaw,
                    face_mode=int(FaceMode.FACE_GOAL),
                    move_gain=0.0,
                    dribble_mode=int(DribbleMode.STRONG),
                    shoot_intent=int(ShootIntent.TRY_SHOOT),
                )
                return self._decision("shoot", packet, reasons, obs)
            reasons.append("need_face_goal")

        target = self._select_carry_target(obs)
        packet = DecisionPacketV1(
            intent_id=int(IntentId.CARRY_TO_POSE),
            target_x=target[0],
            target_y=target[1],
            target_yaw=goal_yaw,
            face_mode=int(FaceMode.FACE_GOAL),
            move_gain=self.config.carry_speed_gain,
            dribble_mode=int(DribbleMode.STRONG),
            shoot_intent=int(ShootIntent.NONE),
        )
        return self._decision("carry_to_lane", packet, reasons, obs)

    def _select_carry_target(self, obs: DeployObservationV1) -> list[float]:
        cfg = self.config
        x, y = float(obs.self_x), float(obs.self_y)
        x_guard = max(1.0, cfg.x_limit - cfg.clearance)
        y_guard = max(1.0, cfg.y_limit - cfg.clearance)
        goal_box_x_min = max(0.0, cfg.x_limit - cfg.goal_box_width)
        max_y = min(cfg.field_short / 2.0 - 28.0, y_guard)

        # 优先往中间移动，以便满足射门条件
        # 如果 Y 位置偏离中间太多，强制往中间走
        center_threshold = 30.0  # 超过这个距离就优先往中间走
        
        # 目标 X: 向前推进
        target_x = max(x + 22.0, cfg.carry_x_cm)
        target_x = min(target_x, cfg.field_long / 2.0 - 18.0, x_guard)
        
        # 如果在禁区附近，限制 X
        if abs(target_x) > goal_box_x_min:
            target_x = math.copysign(max(0.0, goal_box_x_min - cfg.clearance), target_x)

        # 目标 Y: 优先往中间走
        if abs(y) > center_threshold:
            # 偏离中间太多，强制往中间移动
            # 选择一个介于当前位置和中间的目标
            target_y = y * 0.3  # 往中间收缩到 30%
            target_y = _clamp(target_y, -max_y, max_y)
        else:
            # 已经接近中间，选择最佳射门通道
            lane_candidates = [-24.0, 0.0, 24.0]
            best_y = min(lane_candidates, key=lambda lane_y: self._lane_cost(obs, [target_x, lane_y]))
            target_y = _clamp(best_y, -max_y, max_y)

        # 检查安全
        safe_target = self.project_safe_pose([target_x, target_y, obs.self_yaw])
        if safe_target is not None:
            return [safe_target[0], safe_target[1]]
        return [target_x, target_y]

    def _decision(
        self,
        state: str,
        packet: DecisionPacketV1,
        reasons: list[str],
        obs: DeployObservationV1,
    ) -> TeacherDecision:
        self.last_state = state
        target = [packet.target_x, packet.target_y, packet.target_yaw]
        target_projection = None
        checks_target = packet.intent_id in (
            int(IntentId.AVOID_WALL),
            int(IntentId.CARRY_TO_POSE),
            int(IntentId.SEARCH_BALL),
        )
        checks_path = packet.intent_id in (
            int(IntentId.CARRY_TO_POSE),
            int(IntentId.SEARCH_BALL),
        )
        if checks_target:
            target_projection = self.project_safe_pose(target)

        safety = {
            "target_legal": target_projection is None,
            "target_projection": target_projection,
            "path_legal": True
            if not checks_path
            else self.path_is_safe([obs.self_x, obs.self_y, obs.self_yaw], target),
        }
        return TeacherDecision(packet=packet, state=state, reasons=list(reasons), safety=safety)

    def _search_points(self) -> list[list[float]]:
        half_y = min(55.0, self.config.field_short / 2.0 - 35.0)
        return [
            [-70.0, 0.0, 0.0],
            [-45.0, half_y, 0.0],
            [-15.0, 0.0, 0.0],
            [-45.0, -half_y, 0.0],
        ]

    def _best_goal_aim_y(self, obs: DeployObservationV1) -> float:
        goal_half = min(self.config.goal_width / 2.0, 38.0)
        candidates = [-goal_half * 0.65, 0.0, goal_half * 0.65]
        return max(candidates, key=lambda gy: self._lane_score(obs, [self.config.field_long / 2.0, gy]))

    def _can_shoot(self, obs: DeployObservationV1, aim: list[float]) -> bool:
        attack_ready = obs.self_x >= self.config.deep_shoot_x_cm or (obs.self_x >= 20.0 and abs(obs.self_y) <= 32.0)
        return attack_ready and self._lane_score(obs, aim) > 0.0

    def _lane_score(self, obs: DeployObservationV1, end: list[float]) -> float:
        sx, sy = float(obs.self_x), float(obs.self_y)
        ex, ey = float(end[0]), float(end[1])
        vx, vy = ex - sx, ey - sy
        denom = vx * vx + vy * vy
        if denom <= 1e-6:
            return 0.0

        score = 1.0
        for enemy in obs.enemies:
            if not getattr(enemy, "visible", False):
                continue
            px = sx + float(enemy.rel_x)
            py = sy + float(enemy.rel_y)
            t = ((px - sx) * vx + (py - sy) * vy) / denom
            if not 0.05 <= t <= 0.98:
                continue
            closest_x = sx + t * vx
            closest_y = sy + t * vy
            distance = math.hypot(px - closest_x, py - closest_y)
            if distance < self.config.lane_block_width_cm:
                score -= 1.0
            else:
                score -= max(0.0, (self.config.lane_block_width_cm * 1.8 - distance) / 100.0)
        return score

    def _lane_cost(self, obs: DeployObservationV1, end: list[float]) -> float:
        distance_cost = math.hypot(float(end[0]) - obs.self_x, float(end[1]) - obs.self_y) / 100.0
        block_bonus = -2.0 * self._lane_score(obs, end)
        side_cost = abs(float(end[1])) / 120.0
        return distance_cost + block_bonus + side_cost

    @staticmethod
    def _pos_to_angle(start: list[float], end: list[float]) -> float:
        return math.degrees(math.atan2(float(end[1]) - float(start[1]), float(end[0]) - float(start[0])))

    @classmethod
    def _load_config(cls) -> TeacherConfig:
        try:
            from LunaPre import Settings

            safety_cfg = getattr(getattr(Settings, "Motion", object()), "RestrictedArea", None)
            return TeacherConfig(
                field_long=float(getattr(Settings.Bounds, "Long", 260)),
                field_short=float(getattr(Settings.Bounds, "Short", 200)),
                goal_width=float(getattr(Settings.Vision, "GoalWidth", 90.0)),
                x_limit=float(getattr(safety_cfg, "XLimit", 90.0)),
                y_limit=float(getattr(safety_cfg, "YLimit", 70.0)),
                goal_box_width=float(getattr(safety_cfg, "GoalBoxWidth", 25.0)),
                goal_box_y_half=float(getattr(safety_cfg, "GoalBoxYHalf", 50.0)),
                clearance=float(getattr(safety_cfg, "Clearance", 5.0)),
                recovery_clearance=float(getattr(safety_cfg, "RecoveryClearance", 16.0)),
            )
        except Exception:
            return TeacherConfig()
