import numpy as np

from vision_utils.pose_calibration import (
    apply_pose_affine,
    apply_pose_homography,
    camera_plane_affine_from_robot_affine,
    camera_plane_homography_from_robot_homography,
    compose_pose_affine_into_homography,
    compose_pose_homography_into_homography,
    fit_pose_affine,
    fit_pose_homography,
)


def test_fit_pose_affine_recovers_planar_transform():
    source = np.array([[-50.0, -10.0], [-30.0, -10.0], [-50.0, 15.0], [-30.0, 15.0]])
    expected = np.array([[1.8, 0.2, 3.0], [-0.1, 1.5, -4.0]])
    target = apply_pose_affine(source, expected)

    fit = fit_pose_affine(source, target, ransac_threshold_cm=0.01)

    assert fit.inlier_count == len(source)
    assert np.allclose(fit.matrix_cm, expected, atol=1e-5)
    assert fit.median_error_cm < 1e-5


def test_composed_homography_preserves_raw_projection():
    homography = np.array([[1.2, 0.1, 12.0], [0.0, 0.9, 8.0], [0.0, 0.0, 1.0]])
    affine = np.array([[1.7, 0.2, 4.0], [-0.1, 1.3, -2.0]])
    candidate = compose_pose_affine_into_homography(homography, affine)
    source_cm = np.array([12.0, -8.0])
    target_cm = apply_pose_affine([source_cm], affine)[0]
    source_mm = np.array([source_cm[0] * 10.0, source_cm[1] * 10.0, 1.0])
    target_mm = np.array([target_cm[0] * 10.0, target_cm[1] * 10.0, 1.0])

    source_raw = homography @ source_mm
    target_raw = candidate @ target_mm
    source_raw /= source_raw[2]
    target_raw /= target_raw[2]

    assert np.allclose(source_raw, target_raw)


def test_back_camera_plane_affine_matches_robot_frame_correction():
    robot_affine = np.array([[1.8, 0.2, 3.0], [-0.1, 1.5, -4.0]])
    plane_affine = camera_plane_affine_from_robot_affine(robot_affine, 0)
    plane = np.array([[6.0, -9.0]])
    robot = np.array([[-plane[0, 1], plane[0, 0]]])
    expected_robot = apply_pose_affine(robot, robot_affine)
    corrected_plane = apply_pose_affine(plane, plane_affine)
    corrected_robot = np.array([[-corrected_plane[0, 1], corrected_plane[0, 0]]])

    assert np.allclose(corrected_robot, expected_robot)


def test_pose_homography_and_camera_plane_conversion_preserve_projection():
    source = np.array([[-50.0, -10.0], [-30.0, -10.0], [-50.0, 15.0], [-30.0, 15.0], [-40.0, 5.0]])
    expected = np.array([[1.3, 0.1, 4.0], [-0.05, 1.1, -2.0], [0.001, -0.002, 1.0]])
    target = apply_pose_homography(source, expected)
    fit = fit_pose_homography(source, target, ransac_threshold_cm=0.01)

    assert fit.inlier_count == len(source)
    assert fit.median_error_cm < 1e-5

    homography = np.array([[1.2, 0.1, 12.0], [0.0, 0.9, 8.0], [0.0, 0.0, 1.0]])
    plane_transform = camera_plane_homography_from_robot_homography(fit.matrix_cm, 0)
    candidate = compose_pose_homography_into_homography(homography, plane_transform)
    plane_cm = np.array([[6.0, -9.0]])
    robot_cm = np.array([[-plane_cm[0, 1], plane_cm[0, 0]]])
    target_robot_cm = apply_pose_homography(robot_cm, fit.matrix_cm)
    target_plane_cm = np.array([[target_robot_cm[0, 1], -target_robot_cm[0, 0]]])
    source_mm = np.array([plane_cm[0, 0] * 10.0, plane_cm[0, 1] * 10.0, 1.0])
    target_mm = np.array([target_plane_cm[0, 0] * 10.0, target_plane_cm[0, 1] * 10.0, 1.0])

    source_raw = homography @ source_mm
    target_raw = candidate @ target_mm
    source_raw /= source_raw[2]
    target_raw /= target_raw[2]

    assert np.allclose(source_raw, target_raw)
