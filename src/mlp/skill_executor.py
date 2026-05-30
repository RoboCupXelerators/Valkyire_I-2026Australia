"""High-level decision packet executor.

This module is the first M2 skeleton for the roadmap:
- policy outputs `DecisionPacketV1`
- executor translates it into existing chassis / peripheral / skill calls

The implementation is intentionally conservative:
- reuse current non-blocking helpers where possible
- prefer safe fallbacks over aggressive movement
- keep legacy low-level action control path intact
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .config import MLPConfig
from .contracts import DecisionPacketV1, DribbleMode, FaceMode, IntentId, ShootIntent


@dataclass
class SkillExecutionResult:
    executed: bool
    status: str = ""


class SkillExecutor:
    """Translate `DecisionPacketV1` into the existing control stack."""

    def __init__(self, config: Optional[MLPConfig] = None):
        self.config = config or MLPConfig()

    def execute(self, packet: DecisionPacketV1) -> SkillExecutionResult:
        """Execute a high-level decision packet using existing skills."""
        try:
            from LunaPre import UnitedPosition, chassis, peripheral
            from BasicFuc import lockball
        except Exception as e:
            return SkillExecutionResult(False, f"runtime_import_failed: {e}")

        self._apply_dribble_mode(peripheral, packet.dribble_mode)
        intent = packet.intent_id

        if packet.shoot_intent == ShootIntent.TRY_SHOOT and intent != IntentId.SHOOT:
            try:
                peripheral.ShootBall()
            except Exception:
                pass

        if intent == IntentId.SEARCH_BALL:
            chassis.stop()
            return SkillExecutionResult(True, "search_ball_stop")

        if intent in (IntentId.APPROACH_BALL, IntentId.SECURE_BALL):
            lockball.LockBallSlip()
            return SkillExecutionResult(True, "lockball")

        if intent in (
            IntentId.CARRY_TO_POSE,
            IntentId.BLOCK_GOAL,
            IntentId.MARK_ENEMY,
            IntentId.SUPPORT_MATE,
            IntentId.RETURN_HOME,
        ):
            target_yaw = self._resolve_target_yaw(UnitedPosition, packet)
            UnitedPosition.Pos2Pos(
                [float(packet.target_x), float(packet.target_y), float(target_yaw)],
                speed_gain=self._resolve_move_gain(packet.move_gain),
            )
            return SkillExecutionResult(True, "pos2pos")

        if intent == IntentId.AVOID_WALL:
            if abs(float(packet.target_x)) > 1e-6 or abs(float(packet.target_y)) > 1e-6:
                target_yaw = self._resolve_target_yaw(UnitedPosition, packet)
                UnitedPosition.Pos2Pos(
                    [float(packet.target_x), float(packet.target_y), float(target_yaw)],
                    speed_gain=self._resolve_move_gain(packet.move_gain),
                )
                return SkillExecutionResult(True, "avoid_wall_pos2pos")
            chassis.stop()
            return SkillExecutionResult(True, "avoid_wall_stop")

        if intent == IntentId.SHOOT:
            try:
                peripheral.ShootBall()
            except Exception:
                return SkillExecutionResult(False, "shoot_failed")
            return SkillExecutionResult(True, "shoot")

        chassis.stop()
        return SkillExecutionResult(False, f"unknown_intent:{intent}")

    def _apply_dribble_mode(self, peripheral, dribble_mode: int) -> None:
        if dribble_mode == DribbleMode.OFF:
            peripheral.Dribble(False)
        elif dribble_mode == DribbleMode.WEAK:
            peripheral.Dribble(0.35)
        elif dribble_mode == DribbleMode.STRONG:
            peripheral.Dribble(0.8)
        else:
            peripheral.Dribble(False)

    def _resolve_move_gain(self, move_gain: float) -> float:
        """Clamp move_gain into a practical Pos2Pos gain range."""
        try:
            g = float(move_gain)
        except Exception:
            return 1.0
        return max(0.1, min(3.0, g))

    def _resolve_target_yaw(self, UnitedPosition, packet: DecisionPacketV1) -> float:
        """Resolve target yaw according to face_mode."""
        if packet.face_mode == FaceMode.FACE_KEEP:
            try:
                return float(UnitedPosition.FieldRobotPos()[2])
            except Exception:
                return float(packet.target_yaw)

        if packet.face_mode == FaceMode.FACE_TARGET:
            try:
                current = UnitedPosition.FieldRobotPos()
                return float(UnitedPosition.Pos2Angle(current, [packet.target_x, packet.target_y]))
            except Exception:
                return float(packet.target_yaw)

        if packet.face_mode == FaceMode.FACE_BALL:
            try:
                ball = UnitedPosition.FieldBallPos()
                current = UnitedPosition.FieldRobotPos()
                if isinstance(ball, list) and len(ball) >= 2 and ball not in ([0xFFF, 0xFFF], [0xDDD, 0xDDD]):
                    return float(UnitedPosition.Pos2Angle(current, ball))
            except Exception:
                pass
            return float(packet.target_yaw)

        if packet.face_mode == FaceMode.FACE_GOAL:
            return float(packet.target_yaw)

        return float(packet.target_yaw)

