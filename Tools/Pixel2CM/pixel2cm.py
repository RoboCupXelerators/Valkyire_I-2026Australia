"""Pixel-to-centimeter conversion helpers.

This module now supports two calibration formats:

1) Legacy format (old `CalibrationDataX.npz`)
   - keys: `matrix`, `p2c`
   - uses a perspective warp + linear scale/offset

2) Modern format (new web calibrator)
   - keys: `version`, `K`, `dist`, `H`, `rvec`, `tvec`, `board_size`, `square_size_mm`
   - converts raw pixel -> undistorted pixel -> board plane coordinates in cm
   - CM2Pixel uses the stored pose for accurate inverse projection

The modern format is recommended because it corrects lens distortion before
mapping to the chessboard plane, which is the main cause of long-distance
drift / warping in the legacy workflow.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import cv2
import numpy as np

APP_DIR = Path(__file__).parent
DATA_DIR = APP_DIR / "Calibration"


def _as_float_array(value: Any, dtype=np.float64) -> np.ndarray:
    arr = np.asarray(value, dtype=dtype)
    return arr


class Pixel2CM:
    def __init__(self):
        self.HomePath = Path.home()

        # default camera ports used by the rest of the project
        self.CamPorts = [0, 2, 4, 6]

        self._calibrations: list[dict[str, Any]] = []
        self._legacy_mode: list[bool] = []

        for i in range(4):
            path = DATA_DIR / f"CalibrationData{self.CamPorts[i]}.npz"
            self._calibrations.append(self._load_calibration(path))
            self._legacy_mode.append(self._calibrations[-1].get("mode") == "legacy")

        # Backward-compatible attributes (legacy callers may inspect them)
        self.PerspectiveMatrices = []
        self.P2CK = []
        self.P2CHB = []
        self.P2CVB = []
        for calib in self._calibrations:
            if calib.get("mode") == "legacy":
                self.PerspectiveMatrices.append(calib["matrix"])
                self.P2CK.append(float(calib["p2c"][0]))
                self.P2CHB.append(float(calib["p2c"][1]))
                self.P2CVB.append(float(calib["p2c"][2]))
            else:
                # Modern format: populate reasonable defaults so older callers do not crash.
                self.PerspectiveMatrices.append(np.asarray(calib.get("H", np.eye(3)), dtype=np.float64))
                self.P2CK.append(1.0)
                self.P2CHB.append(0.0)
                self.P2CVB.append(0.0)

    def _load_calibration(self, path: Path) -> dict[str, Any]:
        if not path.exists():
            return {"mode": "missing", "matrix": np.eye(3, dtype=np.float64), "p2c": np.array([1.0, 0.0, 0.0])}

        data = np.load(path, allow_pickle=True)
        keys = set(data.files)

        # Modern calibration format
        if {"K", "dist", "H"}.issubset(keys) or ("version" in keys and int(np.atleast_1d(data["version"])[0]) >= 2):
            calib = {
                "mode": "modern",
                "version": int(np.atleast_1d(data["version"])[0]) if "version" in keys else 2,
                "K": _as_float_array(data["K"]),
                "dist": _as_float_array(data["dist"]),
                "H": _as_float_array(data["H"]),
                "rvec": _as_float_array(data["rvec"]) if "rvec" in keys else None,
                "tvec": _as_float_array(data["tvec"]) if "tvec" in keys else None,
                "board_size": tuple(np.asarray(data["board_size"]).astype(int).tolist()) if "board_size" in keys else None,
                "square_size_mm": float(np.atleast_1d(data["square_size_mm"])[0]) if "square_size_mm" in keys else 15.0,
                "origin": str(np.atleast_1d(data["origin"])[0]) if "origin" in keys else "bottom_left_inner_corner",
                "image_size": tuple(np.asarray(data["image_size"]).astype(int).tolist()) if "image_size" in keys else None,
            }
            return calib

        # Legacy calibration format
        if {"matrix", "p2c"}.issubset(keys):
            return {
                "mode": "legacy",
                "matrix": _as_float_array(data["matrix"]),
                "p2c": _as_float_array(data["p2c"]),
            }

        # Unknown file: keep the program alive with identity fallback.
        return {
            "mode": "unknown",
            "matrix": np.eye(3, dtype=np.float64),
            "p2c": np.array([1.0, 0.0, 0.0], dtype=np.float64),
        }

    @staticmethod
    def _apply_homography(X: float, Y: float, Matrix: np.ndarray):
        pt = np.array([X, Y, 1.0], dtype=np.float64)
        res = Matrix @ pt
        if abs(float(res[2])) < 1e-12:
            return float(X), float(Y)
        res = res / res[2]
        return float(res[0]), float(res[1])

    @staticmethod
    def _undistort_point(x: float, y: float, K: np.ndarray, dist: np.ndarray) -> tuple[float, float]:
        pt = np.array([[[x, y]]], dtype=np.float64)
        und = cv2.undistortPoints(pt, K, dist, P=K)
        return float(und[0, 0, 0]), float(und[0, 0, 1])

    def _pixel_to_world_cm_modern(self, X: float, Y: float, calib: dict[str, Any]):
        K = calib["K"]
        dist = calib["dist"]
        H = calib["H"]

        u, v = self._undistort_point(float(X), float(Y), K, dist)
        H_inv = np.linalg.inv(H)
        wx, wy = self._apply_homography(u, v, H_inv)

        # H is expressed in mm on the chessboard plane.
        return float(wx) / 10.0, float(wy) / 10.0

    def _world_cm_to_pixel_modern(self, X: float, Y: float, calib: dict[str, Any]):
        K = calib["K"]
        dist = calib["dist"]
        rvec = calib.get("rvec")
        tvec = calib.get("tvec")
        square_mm = float(calib.get("square_size_mm", 15.0))

        # For modern calibration, prefer exact projection through pose.
        # If pose is unavailable, fall back to homography inversion.
        if rvec is not None and tvec is not None:
            obj = np.array([[[float(X) * 10.0, float(Y) * 10.0, 0.0]]], dtype=np.float64)
            img, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
            return int(round(float(img[0, 0, 0]))), int(round(float(img[0, 0, 1])))

        H = calib["H"]
        pt = np.array([float(X) * 10.0, float(Y) * 10.0, 1.0], dtype=np.float64)
        img = H @ pt
        if abs(float(img[2])) < 1e-12:
            return int(round(float(X))), int(round(float(Y)))
        img = img / img[2]
        return int(round(float(img[0]))), int(round(float(img[1])))

    def Pixel2CM(self, X, Y, CamIndex):
        calib = self._calibrations[CamIndex]

        if calib.get("mode") == "modern":
            return self._pixel_to_world_cm_modern(X, Y, calib)

        P2CK = self.P2CK[CamIndex]
        P2CHB = self.P2CHB[CamIndex]
        P2CVB = self.P2CVB[CamIndex]

        X, Y = self._apply_homography(X, Y, self.PerspectiveMatrices[CamIndex])
        X = float(P2CK * X + P2CHB)
        Y = float(-P2CK * Y + P2CVB)
        return X, Y

    def CM2Pixel(self, X, Y, CamIndex):
        calib = self._calibrations[CamIndex]

        if calib.get("mode") == "modern":
            return self._world_cm_to_pixel_modern(X, Y, calib)

        P2CK = self.P2CK[CamIndex]
        P2CHB = self.P2CHB[CamIndex]
        P2CVB = self.P2CVB[CamIndex]

        X = (float(X) - P2CHB) / P2CK
        Y = -(float(Y) - P2CVB) / P2CK
        X, Y = self._apply_homography(X, Y, np.linalg.inv(self.PerspectiveMatrices[CamIndex]))
        return int(round(X)), int(round(Y))


if __name__ == "__main__":
    # Quick self-test / manual use
    converter = Pixel2CM()
    print(converter.Pixel2CM(400, 400, 0))