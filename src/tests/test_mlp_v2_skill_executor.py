import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from mlp.contracts import DecisionPacket, IntentId, ShootIntent
from mlp.skill_executor import SkillExecutor


class FakePeripheral:
    def __init__(self):
        self.shots = 0

    def Dribble(self, _value):
        pass

    def ShootBall(self):
        self.shots += 1


class FakeChassis:
    def __init__(self):
        self.stops = 0

    def stop(self):
        self.stops += 1


class SkillExecutorTests(unittest.TestCase):
    def runtime_modules(self, ball_value):
        held = object()
        position = types.SimpleNamespace(
            RobotBallPos=lambda: [8.5, 0.0] if ball_value == "held" else [20.0, 0.0],
            BallStatus=lambda ball=None: ball_value,
        )
        chassis = FakeChassis()
        peripheral = FakePeripheral()
        luna = types.ModuleType("LunaPre")
        luna.__path__ = []
        luna.UnitedPosition = position
        luna.chassis = chassis
        luna.peripheral = peripheral
        tools = types.ModuleType("LunaPre.strategy_tools")
        tools.BALL_HELD = held
        basic = types.ModuleType("BasicFuc")
        basic.lockball = types.SimpleNamespace(LockBallSlip=lambda: True)
        return {"LunaPre": luna, "LunaPre.strategy_tools": tools, "BasicFuc": basic}, chassis, peripheral

    def test_shoot_requires_possession(self):
        modules, chassis, peripheral = self.runtime_modules("free")
        packet = DecisionPacket(intent_id=int(IntentId.SHOOT), shoot_intent=int(ShootIntent.TRY_SHOOT))
        with patch.dict(sys.modules, modules):
            result = SkillExecutor().execute(packet)
        self.assertEqual(result.status, "shoot_blocked_no_possession")
        self.assertEqual(peripheral.shots, 0)
        self.assertEqual(chassis.stops, 1)

    def test_shoot_is_edge_limited_while_possession_remains_true(self):
        modules, chassis, peripheral = self.runtime_modules("held")
        times = iter((1.0, 1.1, 1.3))
        executor = SkillExecutor(shoot_cooldown_s=0.25, clock=lambda: next(times))
        packet = DecisionPacket(intent_id=int(IntentId.SHOOT), shoot_intent=int(ShootIntent.TRY_SHOOT))
        with patch.dict(sys.modules, modules):
            statuses = [executor.execute(packet).status for _ in range(3)]
        self.assertEqual(statuses, ["shoot", "shoot_cooldown", "shoot"])
        self.assertEqual(peripheral.shots, 2)
        self.assertEqual(chassis.stops, 1)


if __name__ == "__main__":
    unittest.main()
