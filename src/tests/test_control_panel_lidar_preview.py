from types import SimpleNamespace

import control_panel
from control_panel import ControlPanelApp, _wall_segments_from_direct_distance_mm


def _app(*, bits, positions):
    app = ControlPanelApp.__new__(ControlPanelApp)
    app.bits = bits
    app._get_positions_facade = lambda: positions
    return app


def test_bits_preview_returns_robot_frame_wall_segments_from_direct_distances():
    positions = SimpleNamespace(DirectDistance=lambda: [1200, 800, 500, 300])
    app = _app(bits=object(), positions=positions)

    assert app.get_lidar_point_cloud() == {
        "preview_mode": "walls",
        "point_cloud_cm": [],
        "wall_segments_cm": [
            [[120.0, -30.0], [120.0, 80.0]],
            [[120.0, 80.0], [-50.0, 80.0]],
            [[-50.0, 80.0], [-50.0, -30.0]],
            [[-50.0, -30.0], [120.0, -30.0]],
        ],
    }


def test_non_bits_preview_returns_native_point_cloud_and_no_walls(monkeypatch):
    lidar = SimpleNamespace(running=True, disabled=False, last_point_cloud=[(1.25, -0.5)])
    monkeypatch.setitem(
        control_panel.sys.modules,
        "LunaPre.runtime",
        SimpleNamespace(lidar=lidar),
    )
    app = _app(bits=None, positions=None)

    assert app.get_lidar_point_cloud() == {
        "preview_mode": "point_cloud",
        "point_cloud_cm": [[125.0, -50.0]],
        "wall_segments_cm": [],
    }


def test_explicit_ros2_source_keeps_point_preview_with_bits_runtime(monkeypatch):
    lidar = SimpleNamespace(running=True, disabled=False, last_point_cloud=[(1.25, -0.5)])
    monkeypatch.setitem(
        control_panel.sys.modules,
        "LunaPre.runtime",
        SimpleNamespace(
            sensor=SimpleNamespace(Source="ros2", last_source="ros2"),
            lidar=lidar,
        ),
    )
    app = _app(bits=object(), positions=None)
    monkeypatch.setattr(app, "get_lidar_status", lambda: {"preview_mode": "point_cloud"})

    assert app.get_lidar_point_cloud() == {
        "preview_mode": "point_cloud",
        "point_cloud_cm": [[125.0, -50.0]],
        "wall_segments_cm": [],
    }


def test_wall_segments_reject_disabled_or_incomplete_distance_data():
    assert _wall_segments_from_direct_distance_mm([4095, 4095, 4095, 4095]) == []
    assert _wall_segments_from_direct_distance_mm([100, 200, 300]) == []


def test_pose_state_reads_zero_argument_field_robot_pose_once():
    calls = []

    def get_pose(*args, **kwargs):
        calls.append((args, kwargs))
        return [12.5, -4.0, 90.0]

    app = _app(bits=object(), positions=SimpleNamespace(FieldRobotPose=get_pose))
    result = {"robot_pose_valid": False}

    app._fill_pose_state(result)

    assert calls == [((), {})]
    assert result == {
        "robot_x_cm": 12.5,
        "robot_y_cm": -4.0,
        "robot_yaw_deg": 90.0,
        "robot_pose_valid": True,
    }
