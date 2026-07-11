from __future__ import annotations

import math
from typing import Callable, Optional

from .roi import RoiState, grow_around_bbox
from .types import Detection
try:
    from vision_utils.ball_geometry import reconstruct_ball_contact_point  # type: ignore
except Exception:  # pragma: no cover
    from ..ball_geometry import reconstruct_ball_contact_point  # type: ignore


RobotFrameFn = Callable[[float, float, int], tuple[float, float]]
PixelToCmFn = Callable[[float, float, int], tuple[float, float]]
BBoxTo640Fn = Callable[[tuple[int, int, int, int], tuple[int, ...]], Optional[tuple[int, int, int, int]]]


class BlobTracker:
    def __init__(
        self,
        cv2_module,
        np_module,
        pixel_to_cm: PixelToCmFn,
        to_robot_frame: RobotFrameFn,
        raw_bbox_to_640: BBoxTo640Fn,
        roi_scale: float = 2.0,
        lost_ms: float = 120.0,
        min_area: float = 80.0,
    ):
        self.cv2 = cv2_module
        self.np = np_module
        self.pixel_to_cm = pixel_to_cm
        self.to_robot_frame = to_robot_frame
        self.raw_bbox_to_640 = raw_bbox_to_640
        self.roi_scale = float(roi_scale)
        self.lost_seconds = float(lost_ms) / 1000.0
        self.min_area = float(min_area)
        self.threshold = (5, 15, 128, 255, 150, 255)
        self.min_fill_ratio = 0.35
        self.max_aspect_ratio = 1.9
        self.edge_margin_px = 3
        self._rois = [RoiState() for _ in range(4)]

    def reset_roi(self, detection: Detection) -> None:
        cam = int(detection.camera_index)
        if not (0 <= cam < len(self._rois)):
            return
        if detection.raw_bbox_xyxy is None:
            return
        x1, y1, x2, y2 = detection.raw_bbox_xyxy or detection.bbox_xyxy
        if detection.raw_frame_size is not None:
            frame_width, frame_height = detection.raw_frame_size
        else:
            frame_width, frame_height = x2 + 1, y2 + 1
        frame_width = max(1, int(frame_width))
        frame_height = max(1, int(frame_height))
        self._rois[cam].rect = grow_around_bbox((x1, y1, x2, y2), frame_width, frame_height, self.roi_scale)
        self._rois[cam].last_ts = float(detection.timestamp)
        self._rois[cam].lost_count = 0

    def _find_blobs(self, frame, roi: Optional[tuple[int, int, int, int]]):
        cv2 = self.cv2
        np = self.np
        offset_x = 0
        offset_y = 0
        src = frame
        if roi is not None:
            x, y, w, h = roi
            src = frame[y : y + h, x : x + w]
            offset_x = x
            offset_y = y
            if src is None or src.size == 0:
                src = frame
                offset_x = 0
                offset_y = 0

        hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
        lower = np.array((self.threshold[0], self.threshold[2], self.threshold[4]))
        upper = np.array((self.threshold[1], self.threshold[3], self.threshold[5]))
        mask = cv2.inRange(hsv, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blobs = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.min_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            x += offset_x
            y += offset_y
            blobs.append((x, y, w, h, area))
        return blobs

    def _shape_confidence(self, blob, frame_shape) -> float:
        x, y, w, h, area = blob
        bbox_area = float(max(1, w * h))
        if area < self.min_area or bbox_area < self.min_area:
            return 0.0
        aspect = max(w, h) / max(1, min(w, h))
        # Allow moderately wide boxes (bottom occlusion by dribbler). Extreme
        # aspect ratios are still rejected as non-ball noise.
        if aspect > max(self.max_aspect_ratio, 2.4):
            return 0.0
        fill = float(area) / bbox_area
        if fill < self.min_fill_ratio:
            return 0.0
        frame_h, frame_w = frame_shape[:2]
        edge = self.edge_margin_px
        if x <= edge or y <= edge or (x + w) >= frame_w - edge:
            return 0.0

        aspect_score = max(0.0, 1.0 - (aspect - 1.0) / max(0.01, self.max_aspect_ratio - 1.0))
        fill_score = min(1.0, fill / 0.75)
        area_score = min(1.0, math.sqrt(area / max(1.0, self.min_area * 6.0)))
        return float(max(0.0, min(1.0, 0.45 * fill_score + 0.35 * aspect_score + 0.20 * area_score)))

    def track_frame(self, camera_index: int, frame, timestamp: float) -> Optional[Detection]:
        if frame is None:
            return None
        cam = int(camera_index)
        roi_state = self._rois[cam] if 0 <= cam < len(self._rois) else RoiState()
        if roi_state.rect is not None and (float(timestamp) - roi_state.last_ts) > self.lost_seconds:
            roi_state.clear()

        blobs = self._find_blobs(frame, roi_state.rect)
        if not blobs and roi_state.rect is not None:
            roi_state.lost_count += 1
            blobs = self._find_blobs(frame, None)

        best = None
        for blob in blobs:
            x, y, w, h, area = blob
            px_x = float(x + w * 0.5)
            bottom_y = float(y + h)
            contact_x, contact_y, geom = reconstruct_ball_contact_point(px_x, bottom_y, w, h)
            conf = self._shape_confidence(blob, frame.shape)
            # Bottom-occluded wide boxes are still valid balls near the dribbler.
            if conf <= 0.0:
                if geom.get("occlusion") == "bottom" and geom.get("reconstructed"):
                    conf = 0.35
                else:
                    continue
            elif geom.get("reconstructed"):
                conf = max(0.05, conf * 0.95)
            plane_x, plane_y = self.pixel_to_cm(contact_x, contact_y, cam)
            robot_x, robot_y = self.to_robot_frame(float(plane_x), float(plane_y), cam)
            if robot_x < -10.0 or abs(robot_y) > 180.0:
                continue
            dist = float(math.hypot(robot_x, robot_y))
            if dist > 320.0:
                continue
            score = (conf, -dist, area)
            if best is None or score > best[0]:
                best = (score, blob, robot_x, robot_y, conf)

        if best is None:
            if 0 <= cam < len(self._rois):
                self._rois[cam].lost_count += 1
            return None

        _, blob, robot_x, robot_y, conf = best
        x, y, w, h, _area = blob
        raw_bbox = (int(x), int(y), int(x + w), int(y + h))
        bbox_640 = self.raw_bbox_to_640((int(x), int(y), int(w), int(h)), frame.shape)
        if bbox_640 is None:
            return None

        if 0 <= cam < len(self._rois):
            frame_h, frame_w = frame.shape[:2]
            extra = min(80.0, max(0.0, math.hypot(robot_x, robot_y) * 0.08))
            self._rois[cam].rect = grow_around_bbox(raw_bbox, int(frame_w), int(frame_h), self.roi_scale, extra_px=extra)
            self._rois[cam].last_ts = float(timestamp)
            self._rois[cam].lost_count = 0

        return Detection(
            camera_index=cam,
            label="ball",
            confidence=conf,
            bbox_xyxy=tuple(int(v) for v in bbox_640),
            timestamp=float(timestamp),
            source="blob",
            cls_id=0,
            position_cm=(round(float(robot_x), 2), round(float(robot_y), 2)),
            raw_bbox_xyxy=raw_bbox,
        )
