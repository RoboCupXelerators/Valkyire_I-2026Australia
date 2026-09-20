from __future__ import annotations

import os
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from vision_utils.HybridVision import HybridVision  # noqa: E402
from vision_utils.hybrid.fusion import FusionEngine  # noqa: E402
from vision_utils.hybrid.ego_motion import PoseHistory, field_to_robot, robot_to_field  # noqa: E402
from vision_utils.hybrid.profiles import load_hybrid_config  # noqa: E402
from vision_utils.hybrid.scheduler import AdaptiveFrameDistributor  # noqa: E402
from vision_utils.hybrid.types import Detection  # noqa: E402


def _det(label: str, ts: float, pos=(10.0, 0.0), conf=0.8, cam=0, source="yolo") -> Detection:
    cls_id = {"ball": 0, "blue_goal": 1, "yellow_goal": 2, "chassis": 3}.get(label, -1)
    return Detection(
        camera_index=cam,
        label=label,
        confidence=conf,
        bbox_xyxy=(100, 100, 140, 140),
        timestamp=ts,
        source=source,
        cls_id=cls_id,
        position_cm=pos,
    )


def test_hybrid_config_caps_yolo_to_capture_but_preserves_output_rate():
    settings = SimpleNamespace(
        Vision=SimpleNamespace(Profile="orin", CaptureFps=30, OutputHz=90, YoloHz=60, TrackerHz=15),
        VisionVals=SimpleNamespace(),
    )

    cfg = load_hybrid_config(settings)

    assert cfg.capture_fps == 30
    assert cfg.yolo_hz == 30
    assert cfg.tracker_hz == 30
    assert cfg.output_hz == 90


def test_hybrid_config_defaults_yolo_to_30_for_30fps_camera():
    settings = SimpleNamespace(
        Vision=SimpleNamespace(Profile="orin", CaptureFps=30),
        VisionVals=SimpleNamespace(),
    )

    cfg = load_hybrid_config(settings)

    assert cfg.yolo_hz == 30
    assert cfg.output_hz == 60


def test_hybrid_frame_gate_accepts_only_new_timestamps():
    assert HybridVision._is_new_frame(10.1, 10.0)
    assert not HybridVision._is_new_frame(10.0, 10.0)
    assert not HybridVision._is_new_frame(9.9, 10.0)


def test_adaptive_frame_distributor_focuses_camera_after_ball_observation():
    distributor = AdaptiveFrameDistributor(
        camera_count=4,
        pump_hz=20.0,
        min_hz=4.0,
        focus_hz=20.0,
        focus_hold_ms=250.0,
        preferred_camera_index=2,
    )
    frames = [object(), object(), object(), object()]
    timestamps = [1.0, 1.0, 1.0, 1.0]

    selected, _ = distributor.select(frames, timestamps, now=100.0)
    assert all(frame is not None for frame in selected)

    distributor.observe(2, 0.9, now=100.01)
    timestamps = [2.0, 2.0, 2.0, 2.0]
    selected, _ = distributor.select(frames, timestamps, now=100.06)

    assert selected[2] is frames[2]
    assert [i for i, frame in enumerate(selected) if frame is not None] == [2]


def test_adaptive_frame_distributor_keeps_background_camera_due():
    distributor = AdaptiveFrameDistributor(
        camera_count=4,
        pump_hz=20.0,
        min_hz=4.0,
        focus_hz=20.0,
        focus_hold_ms=100.0,
        max_frames_per_tick=1,
    )
    frames = [object(), object(), object(), object()]
    timestamps = [1.0, 1.0, 1.0, 1.0]
    distributor.select(frames, timestamps, now=100.0)
    distributor.observe(2, 0.9, now=100.01)

    seen = set()
    for step in range(1, 9):
        timestamps = [100.0 + step] * 4
        selected, _ = distributor.select(frames, timestamps, now=100.05 * step + 100.0)
        seen.update(i for i, frame in enumerate(selected) if frame is not None)

    assert 2 in seen
    assert len(seen) >= 2


def test_fusion_replaces_chassis_snapshot_instead_of_accumulating():
    fusion = FusionEngine(lost_ms=200.0, predict_ms=40.0)

    fusion.update_yolo([_det("chassis", 1.0, pos=(10.0, 0.0), conf=0.7)])
    first = fusion.output(1.05).chassis_positions_cm
    assert len(first) == 1
    assert first[0][0] == 10.0

    fusion.update_yolo([_det("chassis", 1.1, pos=(80.0, 0.0), conf=0.8)])
    second = fusion.output(1.12).chassis_positions_cm
    assert len(second) == 1
    assert second[0][0] == 80.0


def test_fusion_dedupes_nearby_chassis_positions():
    fusion = FusionEngine(lost_ms=200.0, predict_ms=40.0)

    fusion.update_yolo(
        [
            _det("chassis", 1.0, pos=(20.0, 10.0), conf=0.9, cam=0),
            _det("chassis", 1.0, pos=(24.0, 12.0), conf=0.7, cam=1),
        ]
    )

    chassis = fusion.output(1.02).chassis_positions_cm
    assert chassis == [[20.0, 10.0, 0.9]]


def test_prediction_output_does_not_advance_kalman_time():
    fusion = FusionEngine(lost_ms=200.0, predict_ms=40.0)
    fusion.update_blob([_det("ball", 1.0, pos=(0.0, 0.0), conf=0.9, source="blob")])
    fusion.update_blob([_det("ball", 1.033, pos=(3.0, 0.0), conf=0.9, source="blob")])
    last_filter_ts = fusion.kalman.last_ts

    out = fusion.output(1.05)

    assert out.ball_source == "prediction"
    assert fusion.kalman.last_ts == last_filter_ts


def test_blob_prefers_continuous_ball_over_far_high_confidence_candidate():
    fusion = FusionEngine(lost_ms=200.0, predict_ms=40.0)
    fusion.update_blob([_det("ball", 1.0, pos=(20.0, 0.0), conf=0.85, source="blob", cam=0)])

    fusion.update_blob(
        [
            _det("ball", 1.033, pos=(22.0, 0.0), conf=0.55, source="blob", cam=0),
            _det("ball", 1.033, pos=(120.0, 0.0), conf=0.90, source="blob", cam=1),
        ]
    )

    out = fusion.output(1.034)
    assert out.ball_pos_cm[0] < 35.0
    assert fusion.track.camera_index == 0


def test_same_frame_yolo_correction_does_not_create_velocity_spike():
    fusion = FusionEngine(lost_ms=200.0, predict_ms=40.0)
    fusion.update_blob([_det("ball", 1.0, pos=(20.0, 0.0), conf=0.85, source="blob")])

    fusion.update_yolo([_det("ball", 1.0, pos=(23.0, 0.0), conf=0.9, source="yolo")])

    vx, vy = fusion.kalman.velocity
    assert abs(vx) < 1.0
    assert abs(vy) < 1.0


def test_pose_history_interpolates_translation_and_wrapped_yaw():
    history = PoseHistory(max_age_s=1.0)
    history.add((0.0, 0.0, 350.0), 1.0)
    history.add((10.0, 20.0, 10.0), 2.0)

    pose = history.resolve(1.5)

    assert pose is not None
    assert (pose.x_cm, pose.y_cm) == (5.0, 10.0)
    assert pose.yaw_deg == pytest.approx(0.0)


def test_hybrid_ego_motion_keeps_static_ball_stable_in_field_frame():
    vision = object.__new__(HybridVision)
    vision._pose_history = PoseHistory(max_age_s=1.0)
    vision._fusion_uses_field_frame = False
    vision.fusion = FusionEngine(lost_ms=200.0, predict_ms=40.0)

    vision.UpdateRobotPose((0.0, 0.0, 0.0), 1.0)
    first = vision._pose_corrected_detections([_det("ball", 1.0, pos=(100.0, 0.0))])
    vision.fusion.update_yolo(first)

    vision.UpdateRobotPose((10.0, 0.0, 0.0), 2.0)
    second = vision._pose_corrected_detections([_det("ball", 2.0, pos=(90.0, 0.0))])
    vision.fusion.update_yolo(second)

    assert first[0].position_cm == pytest.approx((100.0, 0.0))
    assert second[0].position_cm == pytest.approx((100.0, 0.0))
    output = vision._output_in_robot_frame(vision.fusion.output(2.0), 2.0)
    assert output.ball_pos_cm == pytest.approx((90.0, 0.0), abs=1.0)
    assert abs(vision.fusion.kalman.velocity[0]) < 1.0


def test_robot_field_transform_roundtrip_with_rotation():
    history = PoseHistory(max_age_s=1.0)
    history.add((10.0, 20.0, 90.0), 1.0)
    pose = history.resolve(1.0)

    field_point = robot_to_field((100.0, 25.0), pose)
    robot_point = field_to_robot(field_point, pose)

    assert field_point == pytest.approx((-15.0, 120.0))
    assert robot_point == pytest.approx((100.0, 25.0))
