from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from vision_utils.modern_calibration import ModernCalibration, ModernCalibrationError, model_bbox_to_raw


ROOT = Path(__file__).resolve().parents[2]


def test_nano_v2_calibration_round_trips_ground_plane_coordinates():
    calibration = ModernCalibration.load(ROOT / "Tools" / "Calibration_Nano_07" / "CalibrationData0.npz")

    px = calibration.cm_to_pixel(15.0, -10.0)
    cm = calibration.pixel_to_cm(*px, frame_size=(640, 480))

    assert calibration.image_size == (640, 480)
    assert cm == pytest.approx((15.0, -10.0), abs=0.15)


def test_projection_rejects_a_frame_from_a_different_raw_pixel_space():
    calibration = ModernCalibration.load(ROOT / "Tools" / "Calibration_Nano_07" / "CalibrationData0.npz")

    with pytest.raises(ModernCalibrationError, match="expects raw 640x480, got 640x640"):
        calibration.pixel_to_cm(320.0, 320.0, frame_size=(640, 640))


def test_model_bbox_is_restored_to_raw_camera_pixels_before_projection():
    assert model_bbox_to_raw((100.0, 200.0, 120.0, 220.0), (640, 480)) == pytest.approx(
        (100.0, 120.0, 120.0, 140.0)
    )
    assert model_bbox_to_raw((100.0, 200.0, 120.0, 220.0), (640, 640)) == pytest.approx(
        (100.0, 200.0, 120.0, 220.0)
    )


def test_legacy_matrix_calibration_is_rejected(tmp_path):
    path = tmp_path / "CalibrationData0.npz"
    np.savez(path, matrix=np.eye(3), p2c=np.array([0.1, 0.0, 0.0]))

    with pytest.raises(ModernCalibrationError, match="not pixel2cm_v2"):
        ModernCalibration.load(path)
