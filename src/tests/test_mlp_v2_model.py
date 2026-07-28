import importlib.util
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


@unittest.skipUnless(importlib.util.find_spec("torch"), "torch is not installed in the local code-only environment")
class MlpV2ModelTests(unittest.TestCase):
    def test_all_heads_and_log_prob_shapes(self):
        import torch
        from mlp.model import PolicyNetwork

        model = PolicyNetwork()
        features = torch.zeros(3, 49)
        roles = torch.tensor([0, 1, 1])
        action, log_prob, entropy, value = model.sample(features, roles)
        self.assertEqual(action["continuous"].shape, (3, 4))
        self.assertEqual(log_prob.shape, (3,))
        self.assertEqual(entropy.shape, (3,))
        self.assertEqual(value.shape, (3,))
        self.assertEqual(model.encode_control_interface(features, roles).shape, (3, 256))
        evaluated, _, _ = model.evaluate_actions(features, roles, action)
        self.assertTrue(torch.allclose(log_prob, evaluated, atol=1e-5))

    def test_legacy_checkpoint_migration_preserves_policy_behavior(self):
        import torch
        from torch import nn
        from torch.distributions import Categorical, Normal
        from mlp.model import PolicyNetwork, load_compatible_policy_state_dict

        class LegacyHead(nn.Module):
            def __init__(self, output_dim):
                super().__init__()
                self.layers = nn.Sequential(nn.Linear(256, 128), nn.SiLU(), nn.Linear(128, output_dim))

            def forward(self, features):
                return self.layers(features)

        class LegacyPolicyNetwork(nn.Module):
            def __init__(self):
                super().__init__()
                self.role_embedding = nn.Embedding(2, 8)
                self.trunk = nn.Sequential(
                    nn.Linear(57, 256), nn.LayerNorm(256), nn.SiLU(),
                    nn.Linear(256, 256), nn.LayerNorm(256), nn.SiLU(),
                )
                self.continuous_head = LegacyHead(4)
                self.intent_head = LegacyHead(10)
                self.face_head = LegacyHead(4)
                self.dribble_head = LegacyHead(3)
                self.shoot_head = LegacyHead(2)
                self.value_head = LegacyHead(1)
                self.log_std = nn.Parameter(torch.full((4,), -0.5))

            def forward(self, features, role):
                encoded = self.trunk(torch.cat((features, self.role_embedding(role.long().reshape(-1))), dim=-1))
                return {
                    "continuous_mean": self.continuous_head(encoded),
                    "continuous_log_std": self.log_std.expand(features.shape[0], -1),
                    "intent_logits": self.intent_head(encoded),
                    "face_logits": self.face_head(encoded),
                    "dribble_logits": self.dribble_head(encoded),
                    "shoot_logits": self.shoot_head(encoded),
                    "value": self.value_head(encoded).squeeze(-1),
                }

            def sample(self, features, role, deterministic=False):
                outputs = self(features, role)
                continuous_dist = Normal(outputs["continuous_mean"], outputs["continuous_log_std"].exp())
                continuous = torch.tanh(outputs["continuous_mean"]) if deterministic else torch.tanh(continuous_dist.rsample())
                raw = torch.atanh(continuous.clamp(-0.999999, 0.999999))
                log_prob = (continuous_dist.log_prob(raw) - torch.log(1.0 - continuous.square() + 1e-6)).sum(dim=-1)
                actions = {"continuous": continuous}
                for name in ("intent", "face", "dribble", "shoot"):
                    logits = outputs[f"{name}_logits"]
                    dist = Categorical(logits=logits)
                    choice = logits.argmax(dim=-1) if deterministic else dist.sample()
                    actions[name] = choice
                    log_prob = log_prob + dist.log_prob(choice)
                return actions, log_prob

        torch.manual_seed(7)
        legacy = LegacyPolicyNetwork().eval()
        modular = PolicyNetwork().eval()
        migrated = load_compatible_policy_state_dict(modular, legacy.state_dict())
        self.assertTrue(migrated)
        features = torch.linspace(-1.0, 1.0, 98).reshape(2, 49)
        roles = torch.tensor([0, 1])
        legacy_outputs = legacy(features, roles)
        modular_outputs = modular(features, roles)
        for key in legacy_outputs:
            self.assertTrue(torch.equal(legacy_outputs[key], modular_outputs[key]), key)
        torch.manual_seed(11)
        legacy_actions, legacy_log_prob = legacy.sample(features, roles)
        torch.manual_seed(11)
        modular_actions, modular_log_prob, _, _ = modular.sample(features, roles)
        self.assertTrue(torch.equal(legacy_actions["continuous"], modular_actions["continuous"]))
        for name in ("intent", "face", "dribble", "shoot"):
            self.assertTrue(torch.equal(legacy_actions[name], modular_actions[name]), name)
        self.assertTrue(torch.equal(legacy_log_prob, modular_log_prob))
        deterministic_actions, _, _, _ = modular.sample(features, roles, deterministic=True)
        legacy_deterministic, _ = legacy.sample(features, roles, deterministic=True)
        self.assertTrue(torch.equal(legacy_deterministic["continuous"], deterministic_actions["continuous"]))
        for name in ("intent", "face", "dribble", "shoot"):
            self.assertTrue(torch.equal(legacy_deterministic[name], deterministic_actions[name]), name)

    @unittest.skipUnless(
        importlib.util.find_spec("onnx") and importlib.util.find_spec("onnxruntime"),
        "onnx and onnxruntime are not installed",
    )
    def test_onnx_and_pytorch_deterministic_packets_match(self):
        import tempfile
        import numpy as np
        import onnxruntime as ort
        import torch
        from mlp.bundle import save_onnx_bundle
        from mlp.config import RuntimeConfig
        from mlp.contracts import PolicyObservation
        from mlp.model import OnnxPolicyWrapper, PolicyNetwork
        from mlp.runtime import PolicyRuntime

        model = PolicyNetwork().eval()
        with tempfile.TemporaryDirectory() as directory:
            save_onnx_bundle(model, directory)
            base = dict(
                platform="test", device="cpu", bundle_path=directory,
                sim_command_port=20001, sim_status_port=20002, scene_control_port=20100,
            )
            torch_runtime = PolicyRuntime(RuntimeConfig(model_format="pytorch", **base), model=model)
            onnx_runtime = PolicyRuntime(RuntimeConfig(model_format="onnx", **base))
            observation = PolicyObservation()
            features = torch.tensor([torch_runtime.encoder.encode(observation)], dtype=torch.float32)
            role = torch.tensor([observation.role], dtype=torch.long)
            with torch.inference_mode():
                torch_outputs = OnnxPolicyWrapper(model)(features, role)
            session = ort.InferenceSession(str(Path(directory) / "policy.onnx"), providers=["CPUExecutionProvider"])
            onnx_outputs = session.run(None, {"features": features.numpy(), "role": role.numpy()})
            for torch_output, onnx_output in zip(torch_outputs, onnx_outputs):
                np.testing.assert_allclose(torch_output.detach().numpy(), onnx_output, rtol=0.0, atol=1e-5)

            torch_packet = torch_runtime.infer(observation).to_dict()
            onnx_packet = onnx_runtime.infer(observation).to_dict()
            for key in ("target_x", "target_y", "target_yaw", "move_gain", "confidence"):
                # Packet values are decoded into physical units. The strict
                # 1e-5 bound above applies to exported tensor outputs.
                self.assertAlmostEqual(torch_packet[key], onnx_packet[key], delta=5e-5)
            for key in ("intent_id", "face_mode", "dribble_mode", "shoot_intent"):
                self.assertEqual(torch_packet[key], onnx_packet[key])


if __name__ == "__main__":
    unittest.main()
