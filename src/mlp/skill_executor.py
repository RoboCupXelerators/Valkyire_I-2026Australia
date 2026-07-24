"""Translate high-level policy decisions into the existing control stack."""

from __future__ import annotations

from dataclasses import dataclass

from .contracts import DecisionPacket, DribbleMode, FaceMode, IntentId, ShootIntent


@dataclass(frozen=True)
class SkillExecutionResult:
    executed: bool
    status: str


class SkillExecutor:
    def safe_stop(self) -> SkillExecutionResult:
        try:
            from LunaPre import UnitedPosition, chassis, peripheral
            from LunaPre.strategy_tools import safe_stop
            safe_stop(chassis, peripheral, UnitedPosition)
            return SkillExecutionResult(True, "safe_stop")
        except Exception as exc:
            return SkillExecutionResult(False, f"safe_stop_failed:{exc}")

    def execute(self, packet: DecisionPacket) -> SkillExecutionResult:
        if not packet.control_enabled:
            return self.safe_stop()
        try:
            from LunaPre import UnitedPosition, chassis, peripheral
            from BasicFuc import lockball
        except Exception as exc:
            return SkillExecutionResult(False, f"runtime_import_failed:{exc}")

        self._set_dribbler(peripheral, packet.dribble_mode)
        intent = IntentId(packet.intent_id)
        if intent == IntentId.SEARCH_BALL:
            chassis.stop()
            return SkillExecutionResult(True, "search_stop")
        if intent in (IntentId.APPROACH_BALL, IntentId.SECURE_BALL):
            lockball.LockBallSlip()
            return SkillExecutionResult(True, "lock_ball")
        if intent == IntentId.SHOOT or packet.shoot_intent == int(ShootIntent.TRY_SHOOT):
            peripheral.ShootBall()
            return SkillExecutionResult(True, "shoot")
        if intent == IntentId.AVOID_WALL and abs(packet.target_x) + abs(packet.target_y) < 1e-6:
            chassis.stop()
            return SkillExecutionResult(True, "avoid_wall_stop")

        UnitedPosition.Pos2Pos(
            [float(packet.target_x), float(packet.target_y), self._target_yaw(UnitedPosition, packet)],
            speed_gain=max(0.1, min(3.0, float(packet.move_gain))),
        )
        return SkillExecutionResult(True, "pos2pos")

    @staticmethod
    def _set_dribbler(peripheral, mode: int) -> None:
        if mode == int(DribbleMode.WEAK):
            peripheral.Dribble(0.35)
        elif mode == int(DribbleMode.STRONG):
            peripheral.Dribble(0.8)
        else:
            peripheral.Dribble(False)

    @staticmethod
    def _target_yaw(position, packet: DecisionPacket) -> float:
        if packet.face_mode == int(FaceMode.FACE_KEEP):
            try:
                return float(position.FieldRobotPose()[2])
            except Exception:
                pass
        if packet.face_mode == int(FaceMode.FACE_TARGET):
            try:
                return float(position.Pos2Angle(position.FieldRobotPose(), [packet.target_x, packet.target_y]))
            except Exception:
                pass
        return float(packet.target_yaw)

