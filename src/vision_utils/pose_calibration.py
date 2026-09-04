"""Fit pose-referenced camera calibration updates from chassis observations."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class PoseAffineFit:
    matrix_cm: np.ndarray
    inlier_mask: np.ndarray
    residual_cm: np.ndarray

    @property
    def inlier_count(self) -> int:
        return int(np.count_nonzero(self.inlier_mask))

    @property
    def median_error_cm(self) -> float:
        values = self.residual_cm[self.inlier_mask]
        return float(np.median(values)) if values.size else float("inf")

    @property
    def p90_error_cm(self) -> float:
        values = self.residual_cm[self.inlier_mask]
        return float(np.quantile(values, 0.9)) if values.size else float("inf")


@dataclass(frozen=True)
class PoseHomographyFit:
    matrix_cm: np.ndarray
    inlier_mask: np.ndarray
    residual_cm: np.ndarray

    @property
    def inlier_count(self) -> int:
        return int(np.count_nonzero(self.inlier_mask))

    @property
    def median_error_cm(self) -> float:
        values = self.residual_cm[self.inlier_mask]
        return float(np.median(values)) if values.size else float("inf")

    @property
    def p90_error_cm(self) -> float:
        values = self.residual_cm[self.inlier_mask]
        return float(np.quantile(values, 0.9)) if values.size else float("inf")


def fit_pose_affine(
    source_cm,
    target_cm,
    *,
    ransac_threshold_cm: float = 3.0,
) -> PoseAffineFit:
    """Fit target coordinates from camera-projected coordinates in centimetres."""

    source = np.asarray(source_cm, dtype=np.float32).reshape(-1, 2)
    target = np.asarray(target_cm, dtype=np.float32).reshape(-1, 2)
    if source.shape != target.shape or source.shape[0] < 3:
        raise ValueError("at least three paired 2D observations are required")
    if not np.all(np.isfinite(source)) or not np.all(np.isfinite(target)):
        raise ValueError("pose calibration observations must be finite")

    matrix, inlier_mask = cv2.estimateAffine2D(
        source,
        target,
        method=cv2.RANSAC,
        ransacReprojThreshold=float(ransac_threshold_cm),
        maxIters=10_000,
        confidence=0.99,
        refineIters=20,
    )
    if matrix is None or inlier_mask is None:
        raise ValueError("observations do not define a stable affine calibration")

    matrix = np.asarray(matrix, dtype=np.float64)
    mask = np.asarray(inlier_mask, dtype=bool).reshape(-1)
    predicted = apply_pose_affine(source, matrix)
    residual = np.linalg.norm(target.astype(np.float64) - predicted, axis=1)
    if int(np.count_nonzero(mask)) < 3:
        raise ValueError("pose calibration has fewer than three RANSAC inliers")
    return PoseAffineFit(matrix_cm=matrix, inlier_mask=mask, residual_cm=residual)


def apply_pose_affine(points_cm, matrix_cm) -> np.ndarray:
    points = np.asarray(points_cm, dtype=np.float64).reshape(-1, 2)
    matrix = np.asarray(matrix_cm, dtype=np.float64).reshape(2, 3)
    homogeneous = np.column_stack((points, np.ones(len(points), dtype=np.float64)))
    return homogeneous @ matrix.T


def fit_pose_homography(
    source_cm,
    target_cm,
    *,
    ransac_threshold_cm: float = 3.0,
) -> PoseHomographyFit:
    """Fit a projective ground-plane correction from paired 2D observations."""

    source = np.asarray(source_cm, dtype=np.float32).reshape(-1, 2)
    target = np.asarray(target_cm, dtype=np.float32).reshape(-1, 2)
    if source.shape != target.shape or source.shape[0] < 4:
        raise ValueError("at least four paired 2D observations are required")
    if not np.all(np.isfinite(source)) or not np.all(np.isfinite(target)):
        raise ValueError("pose calibration observations must be finite")

    matrix, inlier_mask = cv2.findHomography(
        source,
        target,
        method=cv2.RANSAC,
        ransacReprojThreshold=float(ransac_threshold_cm),
        maxIters=10_000,
        confidence=0.99,
    )
    if matrix is None or inlier_mask is None:
        raise ValueError("observations do not define a stable homography")
    matrix = np.asarray(matrix, dtype=np.float64)
    mask = np.asarray(inlier_mask, dtype=bool).reshape(-1)
    predicted = apply_pose_homography(source, matrix)
    residual = np.linalg.norm(target.astype(np.float64) - predicted, axis=1)
    if int(np.count_nonzero(mask)) < 4:
        raise ValueError("pose calibration has fewer than four RANSAC inliers")
    return PoseHomographyFit(matrix_cm=matrix, inlier_mask=mask, residual_cm=residual)


def apply_pose_homography(points_cm, matrix_cm) -> np.ndarray:
    points = np.asarray(points_cm, dtype=np.float64).reshape(-1, 2)
    matrix = np.asarray(matrix_cm, dtype=np.float64).reshape(3, 3)
    homogeneous = np.column_stack((points, np.ones(len(points), dtype=np.float64))) @ matrix.T
    return homogeneous[:, :2] / homogeneous[:, 2:3]


def compose_pose_affine_into_homography(homography_world_to_raw, matrix_cm) -> np.ndarray:
    """Embed a centimetre-plane affine correction into a world-to-raw homography."""

    homography = np.asarray(homography_world_to_raw, dtype=np.float64).reshape(3, 3)
    affine = np.asarray(matrix_cm, dtype=np.float64).reshape(2, 3)
    transform_mm = np.eye(3, dtype=np.float64)
    transform_mm[:2, :2] = affine[:, :2]
    transform_mm[:2, 2] = affine[:, 2] * 10.0
    if abs(float(np.linalg.det(transform_mm))) < 1e-12:
        raise ValueError("pose affine is singular")
    return homography @ np.linalg.inv(transform_mm)


def camera_plane_affine_from_robot_affine(matrix_cm, camera_port: int) -> np.ndarray:
    """Convert a robot-frame affine correction to the camera plane of one port."""

    rotations = {
        0: np.array([[0.0, -1.0], [1.0, 0.0]]),
        2: np.eye(2),
        4: np.array([[0.0, 1.0], [-1.0, 0.0]]),
        6: np.array([[-1.0, 0.0], [0.0, -1.0]]),
    }
    try:
        robot_from_plane = rotations[int(camera_port)]
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"unsupported camera port {camera_port}") from exc
    robot_affine = np.asarray(matrix_cm, dtype=np.float64).reshape(2, 3)
    plane_from_robot = robot_from_plane.T
    plane_affine = np.empty((2, 3), dtype=np.float64)
    plane_affine[:, :2] = plane_from_robot @ robot_affine[:, :2] @ robot_from_plane
    plane_affine[:, 2] = plane_from_robot @ robot_affine[:, 2]
    return plane_affine


def camera_plane_homography_from_robot_homography(matrix_cm, camera_port: int) -> np.ndarray:
    """Convert a robot-frame homography to the corresponding camera plane."""

    rotations = {
        0: np.array([[0.0, -1.0], [1.0, 0.0]]),
        2: np.eye(2),
        4: np.array([[0.0, 1.0], [-1.0, 0.0]]),
        6: np.array([[-1.0, 0.0], [0.0, -1.0]]),
    }
    try:
        robot_from_plane = rotations[int(camera_port)]
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"unsupported camera port {camera_port}") from exc
    rotation = np.eye(3, dtype=np.float64)
    rotation[:2, :2] = robot_from_plane
    robot_homography = np.asarray(matrix_cm, dtype=np.float64).reshape(3, 3)
    return np.linalg.inv(rotation) @ robot_homography @ rotation


def compose_pose_homography_into_homography(homography_world_to_raw, matrix_cm) -> np.ndarray:
    """Embed a centimetre-plane projective correction into a world-to-raw homography."""

    homography = np.asarray(homography_world_to_raw, dtype=np.float64).reshape(3, 3)
    transform_cm = np.asarray(matrix_cm, dtype=np.float64).reshape(3, 3)
    scale = np.diag((10.0, 10.0, 1.0))
    transform_mm = scale @ transform_cm @ np.linalg.inv(scale)
    if abs(float(np.linalg.det(transform_mm))) < 1e-12:
        raise ValueError("pose homography is singular")
    return homography @ np.linalg.inv(transform_mm)


__all__ = [
    "PoseAffineFit",
    "PoseHomographyFit",
    "apply_pose_affine",
    "apply_pose_homography",
    "camera_plane_affine_from_robot_affine",
    "camera_plane_homography_from_robot_homography",
    "compose_pose_affine_into_homography",
    "compose_pose_homography_into_homography",
    "fit_pose_affine",
    "fit_pose_homography",
]
