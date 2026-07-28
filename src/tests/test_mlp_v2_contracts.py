import json
import sys
import time
import unittest
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from mlp.config import RuntimeConfig
from mlp.adapters import deployable_referee_state, observation_from_lunapre
from mlp.bundle import create_manifest, load_manifest
from mlp.contracts import (
    CONTROL_INTERFACE_DTYPE,
    CONTROL_INTERFACE_VERSION,
    CONTROL_INTERFACE_WIDTH,
    FEATURE_DIM,
    POLICY_ARCHITECTURE_ID,
    POLICY_MODULE_NAMES,
    FeatureSpec,
    PolicyObservation,
    RefereeState,
)
from mlp.features import FeatureEncoder
from mlp.runtime import ControlGate


class MlpV2ContractTests(unittest.TestCase):
    def test_feature_contract_is_exactly_49(self):
        encoded = FeatureEncoder().encode(PolicyObservation())
        self.assertEqual(FEATURE_DIM, 49)
        self.assertEqual(len(encoded), 49)
        self.assertEqual(len(FeatureSpec().names), 49)

    def test_platform_configs_are_complete_and_portable(self):
        paths = sorted((ROOT / "config" / "mlp").glob("*.json"))
        self.assertEqual(len(paths), 6)
        for path in paths:
            config = RuntimeConfig.load(path)
            self.assertFalse(Path(config.bundle_path).is_absolute(), path.name)
            self.assertEqual(config.control_period_ms, 30)
            self.assertFalse(config.training_enabled, path.name)

    def test_referee_adapter_never_exposes_privileged_detail(self):
        detailed = SimpleNamespace(
            phase="PENALIZED", control_enabled=False, penalty_active=True,
            penalty_reason="PUSHING", penalty_remaining_s=42.0,
        )
        self.assertEqual(
            deployable_referee_state(detailed),
            (int(RefereeState.STOP), False),
        )
        play = SimpleNamespace(phase="PLAY", control_enabled=True)
        self.assertEqual(
            deployable_referee_state(play),
            (int(RefereeState.PLAY), True),
        )

    def test_lunapre_observation_freezes_unavailable_match_fields(self):
        referee = SimpleNamespace(
            phase="PENALIZED", control_enabled=False, penalty_active=True,
            penalty_reason="PUSHING", penalty_remaining_s=42.0,
        )
        position = SimpleNamespace(
            FieldRobotPose=lambda: [0.0, 0.0, 0.0],
            RobotBallPos=lambda: [-1, -1],
            RobotChassisPos=lambda: [],
            DirectDistance=lambda: [100.0] * 4,
            RobotVelocity=lambda: [0.0, 0.0, 0.0, True],
            RefereeState=lambda: referee,
        )
        lunapre = ModuleType("LunaPre")
        lunapre.UnitedPosition = position
        lunapre.ensure_runtime = lambda: None
        strategy_tools = ModuleType("LunaPre.strategy_tools")
        strategy_tools.BALL_HELD = [0, 0]
        strategy_tools.BALL_NOT_FOUND = [-1, -1]
        with patch.dict(sys.modules, {"LunaPre": lunapre, "LunaPre.strategy_tools": strategy_tools}):
            observation = observation_from_lunapre()

        self.assertEqual(observation.referee_state, int(RefereeState.STOP))
        self.assertFalse(observation.control_enabled)
        self.assertFalse(observation.penalty_active)
        self.assertEqual(observation.penalty_remaining_s, 0.0)
        self.assertEqual(observation.score_diff, 0)
        self.assertEqual(observation.match_remaining_s, 300.0)
        encoded = dict(zip(FeatureSpec().names, FeatureEncoder().encode(observation)))
        self.assertEqual(encoded["penalty_active"], 0.0)
        self.assertEqual(encoded["penalty_remaining"], 0.0)
        self.assertEqual(encoded["score_diff"], 0.0)
        self.assertEqual(encoded["match_remaining"], 1.0)

    def test_referee_gate_precedes_inference(self):
        gate = ControlGate(120)
        observation = PolicyObservation(control_enabled=False)
        self.assertFalse(gate.evaluate(observation).enabled)
        observation = PolicyObservation(penalty_active=True)
        self.assertFalse(gate.evaluate(observation).enabled)
        observation = PolicyObservation(referee_state=int(RefereeState.STOP))
        self.assertFalse(gate.evaluate(observation).enabled)

    def test_stale_observation_is_stopped(self):
        now = 1000
        observation = PolicyObservation(observed_at_ms=now - 121)
        self.assertEqual(ControlGate(120).evaluate(observation, now).reason, "stale_observation")

    def test_bundle_contract_hash_is_enforced_without_torch(self):
        import tempfile

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "manifest.json"
            manifest = create_manifest()
            path.write_text(json.dumps(manifest.__dict__), encoding="utf-8")
            self.assertEqual(load_manifest(directory).contract_hash, manifest.contract_hash)
            values = dict(manifest.__dict__)
            values["contract_hash"] = "wrong"
            path.write_text(json.dumps(values), encoding="utf-8")
            with self.assertRaises(ValueError):
                load_manifest(directory)

    def test_bundle_declares_control_interface_contract(self):
        manifest = create_manifest()
        self.assertEqual(manifest.architecture_id, POLICY_ARCHITECTURE_ID)
        self.assertEqual(manifest.control_interface_version, CONTROL_INTERFACE_VERSION)
        self.assertEqual(manifest.control_interface_width, CONTROL_INTERFACE_WIDTH)
        self.assertEqual(manifest.control_interface_dtype, CONTROL_INTERFACE_DTYPE)
        self.assertEqual(tuple(manifest.module_names), POLICY_MODULE_NAMES)


if __name__ == "__main__":
    unittest.main()
