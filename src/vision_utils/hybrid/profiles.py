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
    adaptive_frame_distribution: bool = True
    camera_min_yolo_hz: float = 4.0
    camera_focus_yolo_hz: float = 20.0
    camera_focus_hold_ms: float = 250.0
    yolo_max_frames_per_tick: int = 4


def _value(section: Any, name: str, default: Any) -> Any:
    if section is None:
        return default
    if isinstance(section, dict):
        return section.get(name, default)
    return getattr(section, name, default)


def load_hybrid_config(settings: Any, overrides: dict | None = None) -> HybridConfig:
    vision = getattr(settings, "Vision", None)
    vision_vals = getattr(settings, "VisionVals", None)
    profile = str(_value(vision, "Profile", "orin")).lower()

    if profile == "rpi5":
        defaults = HybridConfig(
            profile="rpi5",
            capture_fps=30,
            output_hz=30.0,
            yolo_hz=2.0,
            tracker_hz=30.0,
            yolo_enabled=False,
            camera_min_yolo_hz=1.0,
            camera_focus_yolo_hz=2.0,
        )
    elif profile == "cuda":
        defaults = HybridConfig(
            profile="cuda",
            capture_fps=60,
            output_hz=60.0,
            yolo_hz=12.0,
            tracker_hz=60.0,
            yolo_enabled=True,
            camera_min_yolo_hz=3.0,
            camera_focus_yolo_hz=12.0,
        )
    else:
        defaults = HybridConfig(profile="orin")

    cam_fps = _value(vision, "CamFps", _value(vision_vals, "CamFps", defaults.capture_fps))
    capture_fps = int(_value(vision, "CaptureFps", cam_fps))
    explicit_yolo_hz = _value(vision, "YoloHz", None)
    if explicit_yolo_hz is None and profile not in ("rpi5",) and capture_fps <= 30:
        yolo_hz = float(capture_fps)
    else:
        yolo_hz = float(_value(vision, "YoloHz", defaults.yolo_hz))

    cfg = HybridConfig(
        mode=str(_value(vision, "Mode", defaults.mode)).lower(),
        profile=profile,
        capture_fps=capture_fps,
        output_hz=float(_value(vision, "OutputHz", defaults.output_hz)),
        yolo_hz=yolo_hz,
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
        adaptive_frame_distribution=bool(
            _value(vision, "AdaptiveFrameDistribution", defaults.adaptive_frame_distribution)
        ),
        camera_min_yolo_hz=float(_value(vision, "CameraMinYoloHz", defaults.camera_min_yolo_hz)),
        camera_focus_yolo_hz=float(_value(vision, "CameraFocusYoloHz", defaults.camera_focus_yolo_hz)),
        camera_focus_hold_ms=float(_value(vision, "CameraFocusHoldMs", defaults.camera_focus_hold_ms)),
        yolo_max_frames_per_tick=int(_value(vision, "YoloMaxFramesPerTick", defaults.yolo_max_frames_per_tick)),
    )

    if overrides:
        for key, value in overrides.items():
            if hasattr(cfg, key) and value is not None:
                setattr(cfg, key, value)
    cfg.capture_fps = max(1, int(cfg.capture_fps))
    cfg.output_hz = max(1.0, float(cfg.output_hz))
    cfg.yolo_hz = min(float(cfg.capture_fps), max(0.0, float(cfg.yolo_hz)))
    cfg.tracker_hz = max(float(cfg.capture_fps), float(cfg.tracker_hz), 1.0)
    cfg.yolo_batch = max(1, int(cfg.yolo_batch))
    cfg.camera_min_yolo_hz = min(cfg.yolo_hz, max(0.0, float(cfg.camera_min_yolo_hz)))
    cfg.camera_focus_yolo_hz = min(cfg.yolo_hz, max(cfg.camera_min_yolo_hz, float(cfg.camera_focus_yolo_hz)))
    cfg.camera_focus_hold_ms = max(0.0, float(cfg.camera_focus_hold_ms))
    cfg.yolo_max_frames_per_tick = max(1, min(cfg.camera_slots, int(cfg.yolo_max_frames_per_tick)))
    return cfg
