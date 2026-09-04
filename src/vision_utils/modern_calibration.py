"""Modern camera-to-ground-plane calibration primitives.

The active vision stack accepts only the ``pixel2cm_v2`` calibration format.
Its homography maps the calibrated camera's raw pixels to the ground plane in
millimetres, with the robot centre as the origin.  Model tensors are commonly
letterboxed to 640x640, so detections must be returned to raw camera pixels
before projection.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


class ModernCalibrationError(RuntimeError):
    """A calibration cannot safely project a camera observation."""


def _text(data, key: str) -> str:
    if key not in data.files:
        return ""
    values = np.asarray(data[key]).reshape(-1)
    return str(values[0]) if values.size else ""


def _bool(data, key: str, default: bool = False) -> bool:
    if key not in data.files:
        return default
    values = np.asarray(data[key]).reshape(-1)
    if not values.size:
        return default
    value = values[0]
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


@dataclass(frozen=True)
class ModernCalibration:
    path: Path
    homography_world_to_raw: np.ndarray
    image_size: tuple[int, int]
    camera_matrix: np.ndarray
    distortion: np.ndarray
    undistort_points: bool

    @classmethod
    def load(cls, path: str | Path) -> "ModernCalibration":
        calibration_path = Path(path)
        try:
            data = np.load(str(calibration_path), allow_pickle=False)
        except Exception as exc:
            raise ModernCalibrationError(f"cannot load calibration {calibration_path}: {exc}") from exc

        try:
            required = {"version", "format", "origin", "coordinate_convention", "homography_space", "image_size", "K", "dist", "H"}
            missing = sorted(required.difference(data.files))
            if missing:
                raise ModernCalibrationError(f"{calibration_path} is not pixel2cm_v2; missing {', '.join(missing)}")
            if int(np.asarray(data["version"]).reshape(-1)[0]) < 2 or _text(data, "format") != "pixel2cm_v2":
                raise ModernCalibrationError(f"{calibration_path} is not a pixel2cm_v2 calibration")
            if _text(data, "origin") != "robot_center" or _text(data, "coordinate_convention") != "robot_center_v2":
                raise ModernCalibrationError(f"{calibration_path} does not use robot_center_v2 coordinates")
            if _text(data, "homography_space") != "raw_pixels":
                raise ModernCalibrationError(f"{calibration_path} homography is not defined in raw pixel space")

            image_size = np.asarray(data["image_size"], dtype=np.int64).reshape(-1)
            if image_size.size < 2 or int(image_size[0]) <= 0 or int(image_size[1]) <= 0:
                raise ModernCalibrationError(f"{calibration_path} has invalid image_size")
            homography = np.asarray(data["H"], dtype=np.float64)
            camera_matrix = np.asarray(data["K"], dtype=np.float64)
            distortion = np.asarray(data["dist"], dtype=np.float64)
            if homography.shape != (3, 3) or not np.all(np.isfinite(homography)):
                raise ModernCalibrationError(f"{calibration_path} has invalid homography")
            if camera_matrix.shape != (3, 3) or not np.all(np.isfinite(camera_matrix)):
                raise ModernCalibrationError(f"{calibration_path} has invalid camera matrix")
            try:
                np.linalg.inv(homography)
            except np.linalg.LinAlgError as exc:
                raise ModernCalibrationError(f"{calibration_path} has a singular homography") from exc
            return cls(
                path=calibration_path,
                homography_world_to_raw=homography,
                image_size=(int(image_size[0]), int(image_size[1])),
                camera_matrix=camera_matrix,
                distortion=distortion,
                undistort_points=_bool(data, "undistort_points", False),
            )
        finally:
            data.close()

    def _validate_frame_size(self, frame_size: tuple[int, int] | None) -> None:
        if frame_size is None:
            return
        observed = (int(frame_size[0]), int(frame_size[1]))
        if observed != self.image_size:
            raise ModernCalibrationError(
                f"calibration {self.path.name} expects raw {self.image_size[0]}x{self.image_size[1]}, got {observed[0]}x{observed[1]}"
            )

    @staticmethod
    def _apply_homography(x: float, y: float, matrix: np.ndarray) -> tuple[float, float]:
        point = matrix @ np.array([float(x), float(y), 1.0], dtype=np.float64)
        if abs(float(point[2])) < 1e-12:
            raise ModernCalibrationError("homography mapped observation to infinity")
        point /= point[2]
        return float(point[0]), float(point[1])

    def pixel_to_cm(self, x: float, y: float, *, frame_size: tuple[int, int] | None = None) -> tuple[float, float]:
        self._validate_frame_size(frame_size)
        raw_x, raw_y = float(x), float(y)
        if self.undistort_points:
            point = np.array([[[raw_x, raw_y]]], dtype=np.float64)
            undistorted = cv2.undistortPoints(point, self.camera_matrix, self.distortion, P=self.camera_matrix)
            raw_x, raw_y = float(undistorted[0, 0, 0]), float(undistorted[0, 0, 1])
        world_x_mm, world_y_mm = self._apply_homography(raw_x, raw_y, np.linalg.inv(self.homography_world_to_raw))
        return round(world_x_mm / 10.0, 2), round(world_y_mm / 10.0, 2)

    def cm_to_pixel(self, x_cm: float, y_cm: float) -> tuple[int, int]:
        raw_x, raw_y = self._apply_homography(float(x_cm) * 10.0, float(y_cm) * 10.0, self.homography_world_to_raw)
        return int(round(raw_x)), int(round(raw_y))


def model_bbox_to_raw(
    bbox_xyxy: tuple[float, float, float, float],
    source_size: tuple[int, int],
    model_size: tuple[int, int] = (640, 640),
) -> tuple[float, float, float, float]:
    """Undo the aspect-preserving letterbox used by all detector backends."""

    source_width, source_height = (float(source_size[0]), float(source_size[1]))
    model_width, model_height = (float(model_size[0]), float(model_size[1]))
    if source_width <= 0 or source_height <= 0:
        raise ModernCalibrationError(f"invalid source image size {source_size}")
    scale = min(model_width / source_width, model_height / source_height)
    pad_x = (model_width - source_width * scale) / 2.0
    pad_y = (model_height - source_height * scale) / 2.0
    x1, y1, x2, y2 = bbox_xyxy
    raw = (
        (float(x1) - pad_x) / scale,
        (float(y1) - pad_y) / scale,
        (float(x2) - pad_x) / scale,
        (float(y2) - pad_y) / scale,
    )
    return (
        max(0.0, min(source_width, raw[0])),
        max(0.0, min(source_height, raw[1])),
        max(0.0, min(source_width, raw[2])),
        max(0.0, min(source_height, raw[3])),
    )
