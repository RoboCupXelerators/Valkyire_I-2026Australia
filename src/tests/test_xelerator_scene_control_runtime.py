from utils.XeleratorSceneControl import XeleratorSceneControl


class CaptureClient(XeleratorSceneControl):
    def __init__(self):
        self.calls = []

    def request(self, cmd, **payload):
        self.calls.append((cmd, payload))
        return {"ok": True, "cmd": cmd, **payload}


def test_runtime_robot_tuning_fields_are_encoded():
    client = CaptureClient()
    result = client.set_robot_tuning(
        robot_id=3,
        motor_torque_nm=0.75,
        dribbler_surface_speed_m_s=2.4,
        dribbler_friction=0.65,
        visual_model_id="custom-a",
    )
    assert result["robot_id"] == 3
    assert result["motor_torque_nm"] == 0.75
    assert result["dribbler_surface_speed_m_s"] == 2.4
    assert result["dribbler_friction"] == 0.65
    assert result["visual_model_id"] == "custom-a"


def test_runtime_configuration_commands_are_explicit():
    client = CaptureClient()
    assert client.set_sim_mode("match")["mode"] == "match"
    assert client.configure_match(timed=False, duration_s=120)["timed"] is False
    assert client.reload_config()["cmd"] == "reload_config"
    assert client.refresh_models()["cmd"] == "refresh_models"
