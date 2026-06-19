import os
import sys

import numpy as np
import torch
import torch.nn as nn

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from mlp.config import MLPConfig
from mlp.contracts import DecisionPacketV1, DeployObservationV1
from mlp.inference import MLPInference
from mlp.model import create_model
from mlp.state_encoder import StateEncoder


def test_legacy_action_output_shape():
    config = MLPConfig(output_mode="legacy_action")
    model = create_model(config)

    x = torch.zeros((2, config.state_dim), dtype=torch.float32)
    action, log_prob, value = model.get_action(x, deterministic=True)

    assert action.shape == (2, 5)
    assert log_prob.shape == (2, 1)
    assert value.shape == (2, 1)


def test_decision_packet_output_shape_and_decode():
    config = MLPConfig(output_mode="decision_packet")
    model = create_model(config)

    x = torch.zeros((2, config.state_dim), dtype=torch.float32)
    action, log_prob, value = model.get_action(x, deterministic=True)

    assert config.action_dim == config.decision_continuous_dim + 4
    assert action.shape == (2, config.action_dim)
    assert log_prob.shape == (2, 1)
    assert value.shape == (2, 1)

    packet = StateEncoder(config).decode_decision_packet(action[0].detach().numpy())
    assert isinstance(packet, DecisionPacketV1)
    assert 0 <= packet.intent_id < config.decision_intent_classes
    assert 0 <= packet.face_mode < config.decision_face_classes
    assert 0 <= packet.dribble_mode < config.decision_dribble_classes
    assert 0 <= packet.shoot_intent < config.decision_shoot_classes
    assert -config.field_length / 2 <= packet.target_x <= config.field_length / 2
    assert -config.field_length / 2 <= packet.target_y <= config.field_length / 2
    assert 0.0 <= packet.move_gain <= 1.0
    assert 50 <= packet.decision_horizon_ms <= 500
    assert 0.0 <= packet.confidence <= 1.0


def test_decision_packet_decode_padding_is_safe():
    config = MLPConfig(output_mode="decision_packet")
    packet = StateEncoder(config).decode_decision_packet(np.zeros(2, dtype=np.float32))

    assert isinstance(packet, DecisionPacketV1)
    assert packet.intent_id == 0
    assert packet.face_mode == 0
    assert packet.dribble_mode == 0
    assert packet.shoot_intent == 0


def test_decision_packet_inference_does_not_smooth_classes(tmp_path):
    config = MLPConfig(output_mode="decision_packet", action_smoothing=0.9)
    model_path = tmp_path / "packet_model.pt"
    torch.save(create_model(config).state_dict(), model_path)

    inference = MLPInference(str(model_path), config=config, device="cpu")

    class StubModel(nn.Module):
        def __init__(self, actions):
            super().__init__()
            self.actions = list(actions)

        def get_action(self, _x, deterministic=True):
            action = self.actions.pop(0)
            return action.unsqueeze(0), torch.zeros((1, 1)), torch.zeros((1, 1))

    first = torch.tensor([0, 0, 0, 0, 0, 0, 9, 3, 2, 1], dtype=torch.float32)
    second = torch.tensor([0, 0, 0, 0, 0, 0, 1, 0, 0, 0], dtype=torch.float32)
    inference.model = StubModel([first, second])

    obs = DeployObservationV1()
    p1 = inference.infer_packet(obs)
    p2 = inference.infer_packet(obs)

    assert p1.intent_id == 9
    assert p1.face_mode == 3
    assert p1.dribble_mode == 2
    assert p1.shoot_intent == 1
    assert p2.intent_id == 1
    assert p2.face_mode == 0
    assert p2.dribble_mode == 0
    assert p2.shoot_intent == 0
