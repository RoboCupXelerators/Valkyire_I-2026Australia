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
        evaluated, _, _ = model.evaluate_actions(features, roles, action)
        self.assertTrue(torch.allclose(log_prob, evaluated, atol=1e-5))

    @unittest.skipUnless(
        importlib.util.find_spec("onnx") and importlib.util.find_spec("onnxruntime"),
        "onnx and onnxruntime are not installed",
    )
    def test_onnx_and_pytorch_deterministic_packets_match(self):
        import tempfile
        import torch
        from mlp.bundle import save_onnx_bundle
        from mlp.config import RuntimeConfig
        from mlp.contracts import PolicyObservation
        from mlp.model import PolicyNetwork
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
            torch_packet = torch_runtime.infer(observation).to_dict()
            onnx_packet = onnx_runtime.infer(observation).to_dict()
            for key in ("target_x", "target_y", "target_yaw", "move_gain", "confidence"):
                self.assertAlmostEqual(torch_packet[key], onnx_packet[key], places=5)
            for key in ("intent_id", "face_mode", "dribble_mode", "shoot_intent"):
                self.assertEqual(torch_packet[key], onnx_packet[key])


if __name__ == "__main__":
    unittest.main()
