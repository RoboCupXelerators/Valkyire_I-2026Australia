from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(slots=True)
class HybridConfig:
    mode: str = "hybrid"
    profile: str = "orin"
    capture_fps: int = 60
    output_hz: float = 60.0
    yolo_hz: float = 20.0
    tracker_hz: float = 60.0
    yolo_enabled: bool = True
    yolo_model_size: str = "v8s"
    yolo_input_size: int = 640
    yolo_batch: int = 4
    tracker_roi_scale: float = 2.0
    tracker_lost_ms: float = 120.0
    tracker_predict_ms: float = 50.0
    min_ball_area: float = 80.0
    camera_slots: int = 4


def _value(section: Any, name: str, default: Any) -> Any:
    return getattr(section, name, default) if section is not None else default


def load_hybrid_config(settings: Any, overrides: dict | None = None) -> HybridConfig:
    vision = getattr(settings, "Vision", None)
    vision_vals = getattr(settings, "VisionVals", None)
    profile = str(_value(vision, "Profile", "orin")).lower()

    if profile == "rpi5":
        defaults = HybridConfig(profile="rpi5", capture_fps=30, output_hz=30.0, yolo_hz=2.0, tracker_hz=30.0, yolo_enabled=False)
    elif profile == "cuda":
        defaults = HybridConfig(profile="cuda", capture_fps=60, output_hz=60.0, yolo_hz=12.0, tracker_hz=60.0, yolo_enabled=True)
    else:
        defaults = HybridConfig(profile="orin")

    cam_fps = _value(vision_vals, "CamFps", defaults.capture_fps)
    cfg = HybridConfig(
        mode=str(_value(vision, "Mode", defaults.mode)).lower(),
        profile=profile,
        capture_fps=int(_value(vision, "CaptureFps", cam_fps)),
        output_hz=float(_value(vision, "OutputHz", defaults.output_hz)),
        yolo_hz=float(_value(vision, "YoloHz", defaults.yolo_hz)),
        tracker_hz=float(_value(vision, "TrackerHz", defaults.tracker_hz)),
        yolo_enabled=bool(_value(vision, "YoloEnabled", defaults.yolo_enabled)),
        yolo_model_size=str(_value(vision, "YoloModelSize", defaults.yolo_model_size)),
        yolo_input_size=int(_value(vision, "YoloInputSize", defaults.yolo_input_size)),
        yolo_batch=int(_value(vision, "YoloBatch", defaults.yolo_batch)),
        tracker_roi_scale=float(_value(vision, "TrackerRoiScale", defaults.tracker_roi_scale)),
        tracker_lost_ms=float(_value(vision, "TrackerLostMs", defaults.tracker_lost_ms)),
        tracker_predict_ms=float(_value(vision, "TrackerPredictMs", defaults.tracker_predict_ms)),
        min_ball_area=float(_value(vision, "MinBallBBoxArea", _value(vision_vals, "BlobMinBallArea", defaults.min_ball_area))),
        camera_slots=4,
    )

    if overrides:
        for key, value in overrides.items():
            if hasattr(cfg, key) and value is not None:
                setattr(cfg, key, value)
    cfg.output_hz = max(1.0, float(cfg.output_hz))
    cfg.yolo_hz = max(0.0, float(cfg.yolo_hz))
    cfg.tracker_hz = max(1.0, float(cfg.tracker_hz))
    cfg.yolo_batch = max(1, int(cfg.yolo_batch))
    return cfg
