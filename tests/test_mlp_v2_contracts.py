import json
import sys
import time
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from mlp.config import RuntimeConfig
from mlp.bundle import create_manifest, load_manifest
from mlp.contracts import FEATURE_DIM, FeatureSpec, PolicyObservation, RefereeState
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


if __name__ == "__main__":
    unittest.main()
