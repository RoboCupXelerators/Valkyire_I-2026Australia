from __future__ import annotations

import math
from typing import Iterable, Optional

from .kalman import BallKalman
from .types import Detection, TrackState, VisionOutput


class FusionEngine:
    def __init__(self, lost_ms: float = 120.0, predict_ms: float = 50.0):
        self.lost_seconds = float(lost_ms) / 1000.0
        self.predict_seconds = float(predict_ms) / 1000.0
        self.kalman = BallKalman()
        self.track = TrackState()
        self.last_ball_bbox: Optional[tuple[int, int, int, int, int, float]] = None
        self.ball_bboxes = [None, None, None, None]
        self.chassis_positions = []
        self.chassis_bboxes = [None, None, None, None]
        self.goal_bboxes = [None, None, None, None]
        self.blue_goal_pos = None
        self.yellow_goal_pos = None

    @staticmethod
    def _best_detection(detections: Iterable[Detection], label: str, source: str | None = None) -> Optional[Detection]:
        best = None
        for det in detections:
            if det.label != label:
                continue
            if source is not None and det.source != source:
                continue
            if best is None or det.confidence > best.confidence:
                best = det
        return best

    def update_yolo(self, detections: list[Detection]) -> None:
        for det in detections:
            cam = int(det.camera_index)
            if det.label == "ball":
                if 0 <= cam < 4:
                    self.ball_bboxes[cam] = det.as_bbox_tuple()
                if self.last_ball_bbox is None or det.confidence >= self.last_ball_bbox[5]:
                    self.last_ball_bbox = det.as_bbox_tuple()
            elif det.label == "chassis":
                if 0 <= cam < 4:
                    self.chassis_bboxes[cam] = det.as_bbox_tuple()
                if det.position_cm is not None:
                    x, y = det.position_cm
                    self.chassis_positions.append((round(x, 2), round(y, 2), float(det.confidence)))
                    self.chassis_positions = self.chassis_positions[-8:]
            elif det.label in ("blue_goal", "yellow_goal"):
                if 0 <= cam < 4:
                    self.goal_bboxes[cam] = det.as_goal_bbox_tuple()
                if det.position_cm is not None:
                    item = [round(det.position_cm[0], 2), round(det.position_cm[1], 2), float(det.confidence)]
                    if det.label == "blue_goal":
                        self.blue_goal_pos = item
                    else:
                        self.yellow_goal_pos = item

        yolo_ball = self._best_detection(detections, "ball", source="yolo")
        if yolo_ball is not None and yolo_ball.position_cm is not None:
            x, y = yolo_ball.position_cm
            self.kalman.update(x, y, yolo_ball.timestamp, confidence=max(0.6, yolo_ball.confidence))
            self.track = TrackState(
                position_cm=(float(x), float(y)),
                velocity_cm_s=self.kalman.velocity,
                confidence=max(0.0, min(1.0, yolo_ball.confidence)),
                source="yolo",
                last_observed_ts=float(yolo_ball.timestamp),
                last_output_ts=float(yolo_ball.timestamp),
                camera_index=int(yolo_ball.camera_index),
            )

    def update_blob(self, detections: list[Detection]) -> None:
        if not detections:
            return
        for det in detections:
            cam = int(det.camera_index)
            if 0 <= cam < 4:
                self.ball_bboxes[cam] = det.as_bbox_tuple()
            if self.last_ball_bbox is None or det.confidence >= self.last_ball_bbox[5]:
                self.last_ball_bbox = det.as_bbox_tuple()

        blob = self._best_detection(detections, "ball", source="blob")
        if blob is None or blob.position_cm is None:
            return
        x, y = blob.position_cm
        if self.track.position_cm is not None and self.track.last_observed_ts > 0:
            dt = max(1e-3, float(blob.timestamp) - float(self.track.last_observed_ts))
            jump = math.hypot(float(x) - self.track.position_cm[0], float(y) - self.track.position_cm[1])
            allowed = 35.0 + 450.0 * dt
            if jump > allowed and blob.confidence < 0.75:
                return

        fx, fy = self.kalman.update(float(x), float(y), blob.timestamp, confidence=max(0.2, blob.confidence))
        self.track = TrackState(
            position_cm=(fx, fy),
            velocity_cm_s=self.kalman.velocity,
            confidence=max(0.0, min(1.0, blob.confidence * 0.85)),
            source="blob",
            last_observed_ts=float(blob.timestamp),
            last_output_ts=float(blob.timestamp),
            camera_index=int(blob.camera_index),
        )

    def output(self, timestamp: float) -> VisionOutput:
        ts = float(timestamp)
        if self.track.last_observed_ts <= 0.0 or self.track.position_cm is None:
            return VisionOutput(timestamp=ts, chassis_positions_cm=list(self.chassis_positions), chassis_bboxes=list(self.chassis_bboxes), goal_bboxes=list(self.goal_bboxes))

        age = max(0.0, ts - self.track.last_observed_ts)
        if age <= self.predict_seconds:
            pos = self.track.position_cm
            conf = max(0.0, min(1.0, self.track.confidence))
            source = self.track.source
        elif age <= self.lost_seconds:
            pos = self.kalman.predict(ts)
            decay = max(0.0, 1.0 - (age - self.predict_seconds) / max(1e-3, self.lost_seconds - self.predict_seconds))
            conf = max(0.05, self.track.confidence * 0.55 * decay)
            source = "prediction"
        else:
            pos = (0.0, 0.0)
            conf = 0.0
            source = "prediction"

        # Keep overlay boxes longer than the trusted ball-position window.
        # The position still drops after `lost_seconds`; this only prevents a
        # low-rate YOLO tick from disappearing between control-panel polls.
        keep_boxes = age <= max(self.lost_seconds, 1.0)
        return VisionOutput(
            ball_pos_cm=(round(float(pos[0]), 2), round(float(pos[1]), 2)) if conf > 0.0 else (0.0, 0.0),
            ball_confidence=round(float(conf), 3),
            ball_source=source,
            ball_bbox=self.last_ball_bbox if keep_boxes else None,
            ball_bboxes=list(self.ball_bboxes) if keep_boxes else [None, None, None, None],
            chassis_positions_cm=list(self.chassis_positions),
            chassis_bboxes=list(self.chassis_bboxes),
            goal_bboxes=list(self.goal_bboxes),
            blue_goal_pos_cm=self.blue_goal_pos,
            yellow_goal_pos_cm=self.yellow_goal_pos,
            timestamp=ts,
        )
