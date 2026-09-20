from __future__ import annotations

import math
from typing import Iterable, Optional

from .kalman import BallKalman
from .types import Detection, TrackState, VisionOutput


class FusionEngine:
    def __init__(self, lost_ms: float = 120.0, predict_ms: float = 50.0):
        self.lost_seconds = float(lost_ms) / 1000.0
        self.predict_seconds = float(predict_ms) / 1000.0
        self.box_hold_seconds = max(0.30, self.lost_seconds * 2.5)
        self.object_hold_seconds = max(0.45, self.lost_seconds * 3.0)
        self.same_frame_seconds = 0.006
        self.stationary_speed_cm_s = 35.0
        self.max_ball_speed_cm_s = 420.0
        self.camera_switch_penalty_cm = 14.0
        self.kalman = BallKalman()
        self.track = TrackState()
        self.last_ball_bbox: Optional[tuple[int, int, int, int, int, float]] = None
        self.last_ball_bbox_ts = 0.0
        self.ball_bboxes = [None, None, None, None]
        self.ball_bbox_ts = [0.0, 0.0, 0.0, 0.0]
        self.chassis_positions = []
        self.chassis_positions_ts = 0.0
        self.chassis_bboxes = [None, None, None, None]
        self.chassis_bbox_ts = [0.0, 0.0, 0.0, 0.0]
        self.goal_bboxes = [None, None, None, None]
        self.goal_bbox_ts = [0.0, 0.0, 0.0, 0.0]
        self.blue_goal_pos = None
        self.blue_goal_ts = 0.0
        self.yellow_goal_pos = None
        self.yellow_goal_ts = 0.0

    def reset_spatial_state(self) -> None:
        """Drop coordinate-bearing state when the tracking reference frame changes."""

        self.kalman = BallKalman()
        self.track = TrackState()
        self.chassis_positions = []
        self.chassis_positions_ts = 0.0
        self.blue_goal_pos = None
        self.blue_goal_ts = 0.0
        self.yellow_goal_pos = None
        self.yellow_goal_ts = 0.0

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

    @staticmethod
    def _dedupe_positions(items: list[tuple[float, float, float]], merge_distance_cm: float = 18.0) -> list[list[float]]:
        merged: list[list[float]] = []
        for x, y, conf in sorted(items, key=lambda item: item[2], reverse=True):
            if any(math.hypot(float(x) - float(old[0]), float(y) - float(old[1])) <= merge_distance_cm for old in merged):
                continue
            merged.append([round(float(x), 2), round(float(y), 2), round(float(conf), 3)])
        return merged

    @staticmethod
    def _recent_values(values: list, timestamps: list[float], now: float, hold_seconds: float) -> list:
        recent = []
        for value, ts in zip(values, timestamps):
            if value is not None and ts > 0.0 and (now - float(ts)) <= hold_seconds:
                recent.append(value)
            else:
                recent.append(None)
        return recent

    def _recent_ball_bboxes(self, timestamp: float) -> list:
        return self._recent_values(self.ball_bboxes, self.ball_bbox_ts, timestamp, self.box_hold_seconds)

    def _recent_chassis_bboxes(self, timestamp: float) -> list:
        return self._recent_values(self.chassis_bboxes, self.chassis_bbox_ts, timestamp, self.object_hold_seconds)

    def _recent_goal_bboxes(self, timestamp: float) -> list:
        return self._recent_values(self.goal_bboxes, self.goal_bbox_ts, timestamp, self.object_hold_seconds)

    def _best_recent_ball_bbox(self, timestamp: float):
        recent = [bbox for bbox in self._recent_ball_bboxes(timestamp) if bbox is not None]
        if not recent:
            return None
        return max(recent, key=lambda bbox: float(bbox[5]))

    def _stabilize_position(self, x: float, y: float, source: str, confidence: float) -> tuple[float, float]:
        previous = self.track.position_cm
        if previous is None or self.track.last_observed_ts <= 0.0:
            return float(x), float(y)

        dx = float(x) - float(previous[0])
        dy = float(y) - float(previous[1])
        jump = math.hypot(dx, dy)
        deadband = 1.8 if source == "yolo" else 2.5
        if jump <= deadband:
            return float(previous[0]), float(previous[1])

        conf = max(0.0, min(1.0, float(confidence)))
        if source == "yolo":
            alpha = 0.35 + 0.25 * conf
        else:
            alpha = 0.20 + 0.22 * conf
        if jump >= 25.0:
            alpha = max(alpha, 0.65)
        alpha = max(0.18, min(0.75, alpha))
        return float(previous[0]) + dx * alpha, float(previous[1]) + dy * alpha

    def _choose_ball_detection(self, detections: list[Detection], source: str) -> Optional[Detection]:
        candidates = [
            det
            for det in detections
            if det.label == "ball" and det.source == source and det.position_cm is not None
        ]
        if not candidates:
            return None

        if self.track.position_cm is None or self.track.last_observed_ts <= 0.0:
            return max(candidates, key=lambda det: float(det.confidence))

        best = None
        best_score = float("inf")
        for det in candidates:
            x, y = det.position_cm
            ref_x, ref_y = self.kalman.forecast(float(det.timestamp))
            jump = math.hypot(float(x) - ref_x, float(y) - ref_y)
            dt = max(0.0, float(det.timestamp) - float(self.track.last_observed_ts))
            age = max(0.0, float(det.timestamp) - float(self.track.last_observed_ts))
            allowed = 16.0 + 360.0 * dt
            if age <= self.lost_seconds and jump > allowed * 2.0 and det.confidence < 0.98:
                continue
            if age <= self.lost_seconds and jump > allowed and det.confidence < 0.92:
                continue
            score = jump - float(det.confidence) * 10.0
            if int(det.camera_index) != int(self.track.camera_index):
                score += self.camera_switch_penalty_cm
            if source == "blob":
                score += max(0.0, 0.55 - float(det.confidence)) * 25.0
            if score < best_score:
                best = det
                best_score = score
        return best

    def _condition_velocity(
        self,
        previous_pos: tuple[float, float] | None,
        smoothed_pos: tuple[float, float],
        timestamp: float,
        previous_ts: float,
        source: str,
        confidence: float,
    ) -> None:
        vx, vy = self.kalman.velocity
        if previous_pos is not None and previous_ts > 0.0:
            dt = float(timestamp) - float(previous_ts)
            if dt > 1e-3:
                svx = (float(smoothed_pos[0]) - float(previous_pos[0])) / dt
                svy = (float(smoothed_pos[1]) - float(previous_pos[1])) / dt
                vx = vx * 0.25 + svx * 0.75
                vy = vy * 0.25 + svy * 0.75
                smooth_speed = math.hypot(svx, svy)
                if smooth_speed < self.stationary_speed_cm_s:
                    vx, vy = 0.0, 0.0

        speed = math.hypot(float(vx), float(vy))
        if speed < self.stationary_speed_cm_s:
            vx, vy = 0.0, 0.0
        elif speed > self.max_ball_speed_cm_s:
            scale = self.max_ball_speed_cm_s / max(1e-6, speed)
            vx *= scale
            vy *= scale

        if source == "blob" and confidence < 0.55:
            vx *= 0.5
            vy *= 0.5
        self.kalman.set_velocity(vx, vy)

    def _apply_ball_measurement(self, det: Detection, source: str, confidence_scale: float = 1.0) -> None:
        if det.position_cm is None:
            return
        x, y = det.position_cm
        ts = float(det.timestamp)
        previous_pos = self.track.position_cm
        previous_ts = float(self.track.last_observed_ts)
        if previous_ts > 0.0 and ts < previous_ts - self.same_frame_seconds:
            return

        confidence = max(0.0, min(1.0, float(det.confidence) * float(confidence_scale)))
        same_frame = previous_ts > 0.0 and ts <= previous_ts + self.same_frame_seconds
        if same_frame and source == "blob" and self.track.source == "yolo":
            return
        if same_frame and self.kalman.initialized:
            sx, sy = self._stabilize_position(float(x), float(y), source, det.confidence)
            self.kalman.set_position(sx, sy)
        else:
            confidence_floor = 0.6 if source == "yolo" else 0.2
            fx, fy = self.kalman.update(float(x), float(y), ts, confidence=max(confidence_floor, confidence))
            sx, sy = self._stabilize_position(fx, fy, source, det.confidence)
            self.kalman.set_position(sx, sy)
            self._condition_velocity(previous_pos, (sx, sy), ts, previous_ts, source, det.confidence)

        self.track = TrackState(
            position_cm=(sx, sy),
            velocity_cm_s=self.kalman.velocity,
            confidence=confidence,
            source=source,
            last_observed_ts=max(ts, previous_ts),
            last_output_ts=max(ts, previous_ts),
            camera_index=int(det.camera_index),
        )

    def update_yolo(self, detections: list[Detection]) -> None:
        chassis_positions: list[tuple[float, float, float]] = []
        chassis_seen = False
        batch_ts = max((float(det.timestamp) for det in detections), default=0.0)

        for det in detections:
            cam = int(det.camera_index)
            if det.label == "ball":
                if 0 <= cam < 4:
                    self.ball_bboxes[cam] = det.as_bbox_tuple()
                    self.ball_bbox_ts[cam] = float(det.timestamp)
                if self.last_ball_bbox is None or det.confidence >= self.last_ball_bbox[5] or float(det.timestamp) > self.last_ball_bbox_ts:
                    self.last_ball_bbox = det.as_bbox_tuple()
                    self.last_ball_bbox_ts = float(det.timestamp)
            elif det.label == "chassis":
                chassis_seen = True
                if 0 <= cam < 4:
                    self.chassis_bboxes[cam] = det.as_bbox_tuple()
                    self.chassis_bbox_ts[cam] = float(det.timestamp)
                if det.position_cm is not None:
                    x, y = det.position_cm
                    chassis_positions.append((float(x), float(y), float(det.confidence)))
            elif det.label in ("blue_goal", "yellow_goal"):
                if 0 <= cam < 4:
                    self.goal_bboxes[cam] = det.as_goal_bbox_tuple()
                    self.goal_bbox_ts[cam] = float(det.timestamp)
                if det.position_cm is not None:
                    item = [round(det.position_cm[0], 2), round(det.position_cm[1], 2), float(det.confidence)]
                    if det.label == "blue_goal":
                        self.blue_goal_pos = item
                        self.blue_goal_ts = float(det.timestamp)
                    else:
                        self.yellow_goal_pos = item
                        self.yellow_goal_ts = float(det.timestamp)

        if chassis_seen:
            self.chassis_positions = self._dedupe_positions(chassis_positions)
            self.chassis_positions_ts = batch_ts
        elif detections and batch_ts > 0.0:
            self.chassis_positions = []
            self.chassis_positions_ts = batch_ts

        yolo_ball = self._choose_ball_detection(detections, "yolo")
        if yolo_ball is not None:
            self._apply_ball_measurement(yolo_ball, "yolo", confidence_scale=1.0)

    def update_blob(self, detections: list[Detection]) -> None:
        if not detections:
            return
        for det in detections:
            cam = int(det.camera_index)
            if 0 <= cam < 4:
                self.ball_bboxes[cam] = det.as_bbox_tuple()
                self.ball_bbox_ts[cam] = float(det.timestamp)
            if self.last_ball_bbox is None or det.confidence >= self.last_ball_bbox[5] or float(det.timestamp) > self.last_ball_bbox_ts:
                self.last_ball_bbox = det.as_bbox_tuple()
                self.last_ball_bbox_ts = float(det.timestamp)

        blob = self._choose_ball_detection(detections, "blob")
        if blob is None or blob.position_cm is None:
            return
        self._apply_ball_measurement(blob, "blob", confidence_scale=0.85)

    def output(self, timestamp: float) -> VisionOutput:
        ts = float(timestamp)
        ball_bboxes = self._recent_ball_bboxes(ts)
        ball_bbox = self._best_recent_ball_bbox(ts)
        chassis_bboxes = self._recent_chassis_bboxes(ts)
        goal_bboxes = self._recent_goal_bboxes(ts)
        chassis_positions = (
            list(self.chassis_positions)
            if self.chassis_positions_ts > 0.0 and (ts - self.chassis_positions_ts) <= self.object_hold_seconds
            else []
        )
        blue_goal = self.blue_goal_pos if self.blue_goal_ts > 0.0 and (ts - self.blue_goal_ts) <= self.object_hold_seconds else None
        yellow_goal = self.yellow_goal_pos if self.yellow_goal_ts > 0.0 and (ts - self.yellow_goal_ts) <= self.object_hold_seconds else None

        if self.track.last_observed_ts <= 0.0 or self.track.position_cm is None:
            return VisionOutput(
                timestamp=ts,
                ball_bbox=ball_bbox,
                ball_bboxes=ball_bboxes,
                chassis_positions_cm=chassis_positions,
                chassis_bboxes=chassis_bboxes,
                goal_bboxes=goal_bboxes,
                blue_goal_pos_cm=blue_goal,
                yellow_goal_pos_cm=yellow_goal,
            )

        age = max(0.0, ts - self.track.last_observed_ts)
        if age <= self.lost_seconds:
            speed = math.hypot(*self.kalman.velocity)
            if age <= 0.002:
                pos = self.track.position_cm
                source = self.track.source
            elif speed < self.stationary_speed_cm_s:
                pos = self.track.position_cm
                source = "prediction"
            else:
                pos = self.kalman.forecast(ts)
                source = "prediction"
            if age <= self.predict_seconds:
                conf = max(0.0, min(1.0, self.track.confidence))
            else:
                decay = max(0.0, 1.0 - (age - self.predict_seconds) / max(1e-3, self.lost_seconds - self.predict_seconds))
                conf = max(0.05, self.track.confidence * 0.55 * decay)
        else:
            pos = (0.0, 0.0)
            conf = 0.0
            source = "prediction"

        return VisionOutput(
            ball_pos_cm=(round(float(pos[0]), 2), round(float(pos[1]), 2)) if conf > 0.0 else (0.0, 0.0),
            ball_confidence=round(float(conf), 3),
            ball_source=source,
            ball_bbox=ball_bbox,
            ball_bboxes=ball_bboxes,
            chassis_positions_cm=chassis_positions,
            chassis_bboxes=chassis_bboxes,
            goal_bboxes=goal_bboxes,
            blue_goal_pos_cm=blue_goal,
            yellow_goal_pos_cm=yellow_goal,
            timestamp=ts,
        )
