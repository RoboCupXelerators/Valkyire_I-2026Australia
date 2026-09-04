"""Create a camera calibration candidate from synchronized chassis-pose logs."""

from __future__ import annotations

import argparse
import json
from bisect import bisect_left
from pathlib import Path

import numpy as np

from vision_utils.modern_calibration import ModernCalibration, model_bbox_to_raw
from vision_utils.pose_calibration import (
    apply_pose_homography,
    camera_plane_homography_from_robot_homography,
    compose_pose_homography_into_homography,
    fit_pose_homography,
)


def _read_jsonl(path: Path) -> list[dict]:
    text = path.read_text(encoding="utf-8").replace("\\n", "\n")
    return [json.loads(line) for line in text.splitlines() if line.strip()]


def _phase_is_stable(phase: str, mode: str) -> bool:
    if mode == "hold":
        return phase.startswith("hold_")
    if mode == "grid":
        return phase in {"row0_start", "row1_start", "row1_return", "row0_final"} or phase.startswith("row1_hold_")
    return False


def _camera_slot(camera_port: int, cam_ports: list[int]) -> int:
    try:
        return cam_ports.index(int(camera_port))
    except ValueError as exc:
        raise ValueError(f"camera port {camera_port} is not in {cam_ports}") from exc


def _to_robot_frame(plane_x_cm: float, plane_y_cm: float, camera_port: int) -> tuple[float, float]:
    if camera_port == 4:
        return plane_y_cm, -plane_x_cm
    if camera_port == 2:
        return plane_x_cm, plane_y_cm
    if camera_port == 0:
        return -plane_y_cm, plane_x_cm
    if camera_port == 6:
        return -plane_x_cm, -plane_y_cm
    raise ValueError(f"unsupported camera port {camera_port}")


def _nearest_pose(rows: list[dict], timestamps: list[float], timestamp: float) -> dict | None:
    index = bisect_left(timestamps, timestamp)
    candidates = rows[max(0, index - 1):index + 1]
    if not candidates:
        return None
    return min(candidates, key=lambda row: abs(float(row["wall_time"]) - timestamp))


def _collect_pairs(
    local_rows: list[dict],
    remote_rows: list[dict],
    projector: ModernCalibration,
    *,
    camera_port: int,
    source_size: tuple[int, int],
    phase_mode: str,
    max_time_skew_s: float,
    min_confidence: float,
) -> tuple[np.ndarray, np.ndarray]:
    timestamps = [float(row["wall_time"]) for row in remote_rows]
    slot = _camera_slot(camera_port, [0, 2, 4, 6])
    source_points = []
    target_points = []
    for row in local_rows:
        bboxes = row.get("bboxes")
        if not isinstance(bboxes, list) or slot >= len(bboxes):
            continue
        bbox = bboxes[slot]
        if not isinstance(bbox, (list, tuple)) or len(bbox) < 6 or float(bbox[5]) < min_confidence:
            continue
        timestamp = float(row["wall_time"])
        pose_row = _nearest_pose(remote_rows, timestamps, timestamp)
        if pose_row is None or abs(float(pose_row["wall_time"]) - timestamp) > max_time_skew_s:
            continue
        if not pose_row.get("accepted") or not _phase_is_stable(str(pose_row.get("phase", "")), phase_mode):
            continue
        pose = pose_row.get("filtered_pose")
        if not isinstance(pose, (list, tuple)) or len(pose) < 2:
            continue
        _, x1, y1, x2, y2, _confidence = bbox[:6]
        raw_x1, raw_y1, raw_x2, raw_y2 = model_bbox_to_raw((x1, y1, x2, y2), source_size)
        plane_x_cm, plane_y_cm = projector.pixel_to_cm(
            (raw_x1 + raw_x2) * 0.5,
            raw_y2,
            frame_size=source_size,
        )
        source_points.append(_to_robot_frame(plane_x_cm, plane_y_cm, camera_port))
        target_points.append((float(pose[0]), float(pose[1])))
    return np.asarray(source_points, dtype=np.float64), np.asarray(target_points, dtype=np.float64)


def _transform_metadata(data: dict[str, np.ndarray], matrix_cm: np.ndarray) -> None:
    points = data.get("frame_world_points_cm")
    if points is not None and np.asarray(points).ndim == 2 and np.asarray(points).shape[1] == 2:
        transformed = apply_pose_homography(points, matrix_cm)
        data["frame_world_points_cm"] = transformed.astype(np.float32)
        data["frame_world_extent_cm"] = np.array(
            [
                np.min(transformed[:, 0]),
                np.max(transformed[:, 0]),
                np.min(transformed[:, 1]),
                np.max(transformed[:, 1]),
            ],
            dtype=np.float32,
        )
    origin = data.get("origin_offset_cm")
    if origin is not None and np.asarray(origin).shape == (2,):
        transformed_origin = apply_pose_homography([origin], matrix_cm)[0]
        data["origin_offset_cm"] = transformed_origin.astype(np.float32)
        data["origin_offset_mm"] = (transformed_origin * 10.0).astype(np.float32)


def _write_candidate(base_path: Path, output_path: Path, fit, camera_port: int) -> None:
    source = np.load(base_path, allow_pickle=False)
    try:
        data = {name: np.asarray(source[name]) for name in source.files}
    finally:
        source.close()
    plane_homography = camera_plane_homography_from_robot_homography(fit.matrix_cm, camera_port)
    homography = compose_pose_homography_into_homography(data["H"], plane_homography)
    data["H"] = homography
    data["matrix"] = np.linalg.inv(homography)
    _transform_metadata(data, plane_homography)
    data["samples_used"] = np.array([fit.inlier_count], dtype=np.int32)
    data["rmse"] = np.array([fit.median_error_cm], dtype=np.float32)
    data["pose_fit_homography_cm"] = np.asarray(fit.matrix_cm, dtype=np.float64)
    data["pose_fit_plane_homography_cm"] = np.asarray(plane_homography, dtype=np.float64)
    data["pose_fit_inlier_count"] = np.array([fit.inlier_count], dtype=np.int32)
    data["pose_fit_median_error_cm"] = np.array([fit.median_error_cm], dtype=np.float32)
    data["pose_fit_p90_error_cm"] = np.array([fit.p90_error_cm], dtype=np.float32)
    data["pose_fit_camera_port"] = np.array([camera_port], dtype=np.int32)
    data["pose_fit_source"] = np.array(["synchronized_remote_pose"])
    data["frame_world_warning"] = np.array(["Pose-refined from synchronized chassis observations; validate before competition use."])
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(output_path, **data)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-calibration", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--local-log", type=Path, action="append", required=True)
    parser.add_argument("--remote-log", type=Path, action="append", required=True)
    parser.add_argument("--phase-mode", choices=("hold", "grid"), action="append", required=True)
    parser.add_argument("--camera-port", type=int, default=0)
    parser.add_argument("--source-width", type=int, default=640)
    parser.add_argument("--source-height", type=int, default=480)
    parser.add_argument("--max-time-skew-s", type=float, default=0.2)
    parser.add_argument("--min-confidence", type=float, default=0.5)
    args = parser.parse_args()
    if not (len(args.local_log) == len(args.remote_log) == len(args.phase_mode)):
        raise SystemExit("--local-log, --remote-log, and --phase-mode counts must match")

    projector = ModernCalibration.load(args.base_calibration)
    source_sets = []
    target_sets = []
    for local_path, remote_path, phase_mode in zip(args.local_log, args.remote_log, args.phase_mode):
        source, target = _collect_pairs(
            _read_jsonl(local_path),
            _read_jsonl(remote_path),
            projector,
            camera_port=args.camera_port,
            source_size=(args.source_width, args.source_height),
            phase_mode=phase_mode,
            max_time_skew_s=args.max_time_skew_s,
            min_confidence=args.min_confidence,
        )
        if len(source):
            source_sets.append(source)
            target_sets.append(target)
    if not source_sets:
        raise SystemExit("no synchronized stable chassis observations were found")

    fit = fit_pose_homography(np.vstack(source_sets), np.vstack(target_sets))
    _write_candidate(args.base_calibration, args.output, fit, args.camera_port)
    print(json.dumps({
        "output": str(args.output),
        "homography_cm": fit.matrix_cm.tolist(),
        "inlier_count": fit.inlier_count,
        "median_error_cm": round(fit.median_error_cm, 3),
        "p90_error_cm": round(fit.p90_error_cm, 3),
    }))


if __name__ == "__main__":
    main()
