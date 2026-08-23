from __future__ import annotations

from typing import Callable, Optional

from .types import Detection
try:
    from vision_utils.ball_geometry import contact_point_from_bbox_xyxy  # type: ignore
except Exception:  # pragma: no cover
    from ..ball_geometry import contact_point_from_bbox_xyxy  # type: ignore


PixelToCmFn = Callable[[float, float, int], tuple[float, float]]
RobotFrameFn = Callable[[float, float, int], tuple[float, float]]


class YoloDetector:
    def __init__(
        self,
        model_path: str,
        engine_path: str,
        input_size: int = 640,
        prefer_engine: bool = True,
        device=None,
        enabled: bool = True,
        fixed_batch_size: int | None = None,
        pixel_to_cm: PixelToCmFn | None = None,
        to_robot_frame: RobotFrameFn | None = None,
    ):
        self.model_path = model_path
        self.engine_path = engine_path
        self.input_size = int(input_size)
        self.prefer_engine = bool(prefer_engine)
        self.device = device
        self.enabled = bool(enabled)
        self.fixed_batch_size = max(1, int(fixed_batch_size)) if fixed_batch_size else None
        self._partial_batch_supported: bool | None = None
        self.partial_batch_padded_count = 0
        self.pixel_to_cm = pixel_to_cm
        self.to_robot_frame = to_robot_frame
        self.backend = "disabled"
        self.error = ""
        self._yolo = None
        self._names = None
        self._cls_ball = 0
        self._cls_blue_goal = 1
        self._cls_yellow_goal = 2
        self._cls_chassis = 3
        if self.enabled:
            self._init_backend()

    def _init_backend(self) -> None:
        try:
            import os
            from ultralytics import YOLO  # type: ignore

            path = None
            if self.prefer_engine and self.engine_path and os.path.exists(self.engine_path):
                path = self.engine_path
                self.backend = "ultralytics-tensorrt"
            elif self.model_path and os.path.exists(self.model_path):
                path = self.model_path
                self.backend = "ultralytics-pytorch"
            else:
                self.backend = "disabled"
                self.error = f"model not found: pt={self.model_path}, engine={self.engine_path}"
                return

            self._yolo = YOLO(path, task="detect")
            self._init_class_mapping()
        except Exception as exc:
            self._yolo = None
            self.backend = "disabled"
            self.error = str(exc)

    def _init_class_mapping(self) -> None:
        try:
            names = getattr(self._yolo, "names", None)
            if not isinstance(names, dict):
                return
            self._names = names
            inv = {str(v).lower().replace(" ", ""): int(k) for k, v in names.items()}
            self._cls_ball = inv.get("ball", inv.get("sportsball", self._cls_ball))
            self._cls_blue_goal = inv.get("bluegoal", self._cls_blue_goal)
            self._cls_yellow_goal = inv.get("yellowgoal", self._cls_yellow_goal)
            self._cls_chassis = inv.get("chassis", inv.get("car", self._cls_chassis))
        except Exception:
            pass

    def available(self) -> bool:
        return self._yolo is not None

    @staticmethod
    def _resize_letterbox(cv2, np, frame, size: int):
        height, width = frame.shape[:2]
        scale = min(size / width, size / height)
        nw = int(width * scale)
        nh = int(height * scale)
        resized = cv2.resize(frame, (nw, nh))
        out = np.zeros((size, size, 3), dtype=np.uint8)
        y0 = (size - nh) // 2
        x0 = (size - nw) // 2
        out[y0 : y0 + nh, x0 : x0 + nw] = resized
        return out

    @staticmethod
    def _bbox_640_to_raw(bbox, frame_shape, size: int) -> tuple[int, int, int, int]:
        x1, y1, x2, y2 = bbox
        frame_h, frame_w = frame_shape[:2]
        scale = min(size / frame_w, size / frame_h)
        pad_x = (size - frame_w * scale) / 2.0
        pad_y = (size - frame_h * scale) / 2.0
        rx1 = int(max(0, min(frame_w - 1, round((float(x1) - pad_x) / scale))))
        ry1 = int(max(0, min(frame_h - 1, round((float(y1) - pad_y) / scale))))
        rx2 = int(max(0, min(frame_w, round((float(x2) - pad_x) / scale))))
        ry2 = int(max(0, min(frame_h, round((float(y2) - pad_y) / scale))))
        return rx1, ry1, rx2, ry2

    def detect_batch(self, frames, timestamps, cv2_module, np_module) -> list[Detection]:
        if self._yolo is None:
            return []
        imgs = []
        cam_map = []
        for cam, frame in enumerate(frames):
            if frame is None:
                continue
            imgs.append(self._resize_letterbox(cv2_module, np_module, frame, self.input_size))
            cam_map.append(cam)
        if not imgs:
            return []

        inference_imgs = imgs
        result_camera_map = list(cam_map)
        partial_batch = bool(
            self.fixed_batch_size
            and len(imgs) < self.fixed_batch_size
            and self.backend == "ultralytics-tensorrt"
        )

        def pad_to_fixed_batch():
            padded_imgs = list(imgs)
            padded_map = list(cam_map)
            while len(padded_imgs) < int(self.fixed_batch_size or 1):
                padded_imgs.append(imgs[-1])
                padded_map.append(None)
            self.partial_batch_padded_count += 1
            return padded_imgs, padded_map

        if partial_batch and self._partial_batch_supported is False:
            inference_imgs, result_camera_map = pad_to_fixed_batch()

        try:
            results = list(
                self._yolo.predict(
                    inference_imgs,
                    verbose=False,
                    conf=0.25,
                    device=self.device,
                    imgsz=self.input_size,
                )
            )
            if partial_batch and inference_imgs is imgs:
                self._partial_batch_supported = True
        except Exception:
            if partial_batch and inference_imgs is imgs:
                self._partial_batch_supported = False
                inference_imgs, result_camera_map = pad_to_fixed_batch()
                results = list(
                    self._yolo.predict(
                        inference_imgs,
                        verbose=False,
                        conf=0.25,
                        device=self.device,
                        imgsz=self.input_size,
                    )
                )
            else:
                results = []
                result_camera_map = []
                for img, cam in zip(imgs, cam_map):
                    part = list(
                        self._yolo.predict(
                            img,
                            verbose=False,
                            conf=0.25,
                            device=self.device,
                            imgsz=self.input_size,
                        )
                    )
                    results.extend(part)
                    result_camera_map.extend([cam] * len(part))

        detections: list[Detection] = []
        for local_index, result in enumerate(results):
            if local_index >= len(result_camera_map) or result_camera_map[local_index] is None:
                continue
            cam = int(result_camera_map[local_index])
            ts = float(timestamps[cam]) if cam < len(timestamps) else 0.0
            boxes = getattr(result, "boxes", None)
            if boxes is None or len(boxes) == 0:
                continue
            xyxy = boxes.xyxy.detach().cpu().numpy()
            confs = boxes.conf.detach().cpu().numpy()
            classes = boxes.cls.detach().cpu().numpy().astype(int)
            for (x1, y1, x2, y2), conf, cls_id in zip(xyxy, confs, classes):
                cls_i = int(cls_id)
                label = None
                if cls_i == self._cls_ball:
                    label = "ball"
                elif cls_i == self._cls_chassis:
                    label = "chassis"
                elif cls_i == self._cls_blue_goal:
                    label = "blue_goal"
                elif cls_i == self._cls_yellow_goal:
                    label = "yellow_goal"
                if label is None:
                    continue
                bbox = (int(x1), int(y1), int(x2), int(y2))
                raw_frame = frames[cam]
                raw_bbox = None
                raw_size = None
                if raw_frame is not None:
                    raw_bbox = self._bbox_640_to_raw(bbox, raw_frame.shape, self.input_size)
                    raw_size = (int(raw_frame.shape[1]), int(raw_frame.shape[0]))
                anchor_bbox = raw_bbox if raw_bbox is not None else bbox
                anchor_x1, anchor_y1, anchor_x2, anchor_y2 = anchor_bbox
                bottom_y = float(anchor_y2)
                center_x = (float(anchor_x1) + float(anchor_x2)) * 0.5
                contact_x, contact_y = center_x, bottom_y
                det_conf = float(conf)
                if label == "ball":
                    contact_x, contact_y, geom = contact_point_from_bbox_xyxy(
                        anchor_x1,
                        anchor_y1,
                        anchor_x2,
                        anchor_y2,
                    )
                    if geom.get("reconstructed"):
                        det_conf = max(0.05, det_conf * 0.92)
                pos = None
                if self.pixel_to_cm is not None and self.to_robot_frame is not None:
                    try:
                        plane_x, plane_y = self.pixel_to_cm(contact_x, contact_y, cam, frame_size=raw_size)
                        pos = self.to_robot_frame(float(plane_x), float(plane_y), cam)
                    except Exception:
                        pos = None
                detections.append(
                    Detection(
                        camera_index=cam,
                        label=label,
                        confidence=det_conf,
                        bbox_xyxy=bbox,
                        timestamp=ts,
                        source="yolo",
                        cls_id=cls_i,
                        position_cm=pos,
                        raw_bbox_xyxy=raw_bbox,
                        raw_frame_size=raw_size,
                    )
                )
        return detections
