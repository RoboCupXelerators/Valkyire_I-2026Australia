"""Demo pipeline for match-video policy dataset generation.

The demo does not require a real video. It creates a short synthetic match,
pretends YOLO detected the ball/robots, lets a mock Gemma/Qwen label short
segments, and exports the same .npz shape consumed by mlp.behavior_cloning.
"""

from __future__ import annotations

import argparse
import json
import math
import random
import sys
import types
from collections import Counter
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

# Import only the lightweight MLP contract modules. The package __init__ imports
# torch-backed model code, which is not needed for dataset generation.
mlp_pkg = types.ModuleType("mlp")
mlp_pkg.__path__ = [str(SRC_ROOT / "mlp")]
sys.modules.setdefault("mlp", mlp_pkg)

from mlp.config import MLPConfig  # noqa: E402
from mlp.contracts import (  # noqa: E402
    DecisionPacketV1,
    DeployObservationV1,
    DribbleMode,
    EntityObservationV1,
    FaceMode,
    IntentId,
    ShootIntent,
)
from mlp.state_encoder import StateEncoder  # noqa: E402


FIELD_X_MIN = -130.0
FIELD_X_MAX = 130.0
FIELD_Y_MIN = -100.0
FIELD_Y_MAX = 100.0
FRAME_W = 1280
FRAME_H = 720


@dataclass
class SimFrame:
    frame: int
    time_sec: float
    self_x: float
    self_y: float
    self_yaw: float
    ball_x: float
    ball_y: float
    ball_visible: bool
    has_ball: bool
    enemy_x: float
    enemy_y: float
    phase: str


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def angle_to(src_x: float, src_y: float, dst_x: float, dst_y: float) -> float:
    return math.degrees(math.atan2(dst_y - src_y, dst_x - src_x)) % 360.0


def angle_error(current: float, target: float) -> float:
    return (target - current + 180.0) % 360.0 - 180.0


def field_to_robot(dx: float, dy: float, yaw_deg: float) -> tuple[float, float]:
    yaw = math.radians(yaw_deg)
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    return cos_y * dx + sin_y * dy, -sin_y * dx + cos_y * dy


class FieldProjector:
    """Simple field <-> pixel mapper for the demo.

    Real video should replace this with homography from calibration.
    """

    def __init__(self, width: int = FRAME_W, height: int = FRAME_H):
        self.width = width
        self.height = height

    def field_to_pixel(self, x: float, y: float) -> tuple[float, float]:
        px = (x - FIELD_X_MIN) / (FIELD_X_MAX - FIELD_X_MIN) * self.width
        py = self.height - (y - FIELD_Y_MIN) / (FIELD_Y_MAX - FIELD_Y_MIN) * self.height
        return px, py

    def pixel_to_field(self, px: float, py: float) -> tuple[float, float]:
        x = FIELD_X_MIN + (px / self.width) * (FIELD_X_MAX - FIELD_X_MIN)
        y = FIELD_Y_MIN + ((self.height - py) / self.height) * (FIELD_Y_MAX - FIELD_Y_MIN)
        return x, y


class SyntheticMatch:
    """Create a tiny match timeline with search, approach, carry, shoot, avoid."""

    def __init__(self, frames: int, fps: float):
        self.frames = frames
        self.fps = fps

    def frames_iter(self) -> Iterable[SimFrame]:
        for idx in range(self.frames):
            ratio = idx / max(1, self.frames - 1)
            time_sec = idx / self.fps

            if ratio < 0.18:
                local = ratio / 0.18
                self_x = lerp(-95.0, -70.0, local)
                self_y = -22.0 + math.sin(local * math.pi * 2.0) * 18.0
                ball_x, ball_y = -18.0, 18.0
                ball_visible = False
                has_ball = False
                self_yaw = lerp(30.0, 210.0, local) % 360.0
                phase = "search"
            elif ratio < 0.50:
                local = (ratio - 0.18) / 0.32
                ball_x, ball_y = -12.0, 12.0
                self_x = lerp(-70.0, -22.0, local)
                self_y = lerp(-22.0, 8.0, local)
                ball_visible = True
                has_ball = False
                self_yaw = angle_to(self_x, self_y, ball_x, ball_y)
                phase = "approach"
            elif ratio < 0.78:
                local = (ratio - 0.50) / 0.28
                self_x = lerp(-22.0, 58.0, local)
                self_y = lerp(8.0, -5.0, local)
                self_yaw = angle_to(self_x, self_y, FIELD_X_MAX, 0.0)
                fx, fy = math.cos(math.radians(self_yaw)), math.sin(math.radians(self_yaw))
                ball_x = self_x + fx * 6.0
                ball_y = self_y + fy * 6.0
                ball_visible = True
                has_ball = True
                phase = "carry"
            elif ratio < 0.90:
                local = (ratio - 0.78) / 0.12
                self_x = lerp(58.0, 68.0, local)
                self_y = lerp(-5.0, -2.0, local)
                self_yaw = angle_to(self_x, self_y, FIELD_X_MAX, 0.0)
                if local < 0.55:
                    fx = math.cos(math.radians(self_yaw))
                    fy = math.sin(math.radians(self_yaw))
                    ball_x = self_x + fx * 6.0
                    ball_y = self_y + fy * 6.0
                    has_ball = True
                else:
                    ball_x = lerp(self_x + 8.0, 122.0, (local - 0.55) / 0.45)
                    ball_y = lerp(self_y, 0.0, (local - 0.55) / 0.45)
                    has_ball = False
                ball_visible = True
                phase = "shoot"
            else:
                local = (ratio - 0.90) / 0.10
                self_x = lerp(98.0, 74.0, local)
                self_y = lerp(78.0, 58.0, local)
                ball_x, ball_y = 32.0, 12.0
                ball_visible = True
                has_ball = False
                self_yaw = angle_to(self_x, self_y, 74.0, 58.0)
                phase = "avoid_wall"

            enemy_x = clamp(self_x + 35.0 + 10.0 * math.sin(idx * 0.08), FIELD_X_MIN, FIELD_X_MAX)
            enemy_y = clamp(self_y + 30.0 * math.cos(idx * 0.06), FIELD_Y_MIN, FIELD_Y_MAX)

            yield SimFrame(
                frame=idx,
                time_sec=time_sec,
                self_x=self_x,
                self_y=self_y,
                self_yaw=self_yaw,
                ball_x=ball_x,
                ball_y=ball_y,
                ball_visible=ball_visible,
                has_ball=has_ball,
                enemy_x=enemy_x,
                enemy_y=enemy_y,
                phase=phase,
            )


class MockYoloDetector:
    """Stand-in for local YOLO.

    Output shape mirrors what the real video tool should persist before
    tracking and coordinate conversion.
    """

    def __init__(self, projector: FieldProjector, rng: random.Random):
        self.projector = projector
        self.rng = rng

    def detect(self, frame: SimFrame) -> list[dict[str, Any]]:
        rows = [
            self._object("robot_blue", "self", frame.self_x, frame.self_y, 54, 48, 0.94),
            self._object("robot_yellow", "enemy_0", frame.enemy_x, frame.enemy_y, 54, 48, 0.89),
        ]
        if frame.ball_visible or frame.has_ball:
            rows.append(self._object("ball", "ball", frame.ball_x, frame.ball_y, 22, 22, 0.91))
        return rows

    def _object(
        self,
        class_name: str,
        object_id: str,
        x: float,
        y: float,
        width: float,
        height: float,
        confidence: float,
    ) -> dict[str, Any]:
        px, py = self.projector.field_to_pixel(x, y)
        px += self.rng.uniform(-2.2, 2.2)
        py += self.rng.uniform(-2.2, 2.2)
        return {
            "class": class_name,
            "object_id": object_id,
            "confidence": round(confidence + self.rng.uniform(-0.04, 0.04), 3),
            "bbox_xyxy": [
                round(px - width / 2.0, 2),
                round(py - height / 2.0, 2),
                round(px + width / 2.0, 2),
                round(py + height / 2.0, 2),
            ],
            "center_px": [round(px, 2), round(py, 2)],
        }


def detections_to_tracklet(
    frame: SimFrame,
    detections: list[dict[str, Any]],
    projector: FieldProjector,
) -> dict[str, Any]:
    objects = []
    for det in detections:
        px, py = det["center_px"]
        x, y = projector.pixel_to_field(float(px), float(py))
        objects.append(
            {
                "track_id": det["object_id"],
                "class": det["class"],
                "confidence": det["confidence"],
                "field_x": round(x, 2),
                "field_y": round(y, 2),
            }
        )
    return {
        "frame": frame.frame,
        "time_sec": round(frame.time_sec, 3),
        "phase_hint": frame.phase,
        "self_yaw": round(frame.self_yaw, 2),
        "objects": objects,
    }


def tracklet_to_observation(
    current: dict[str, Any],
    previous: dict[str, Any] | None,
    fps: float,
) -> DeployObservationV1:
    objects = {obj["track_id"]: obj for obj in current["objects"]}
    self_obj = objects["self"]
    self_x = float(self_obj["field_x"])
    self_y = float(self_obj["field_y"])
    self_yaw = float(current["self_yaw"])

    prev_self = None
    if previous is not None:
        prev_objects = {obj["track_id"]: obj for obj in previous["objects"]}
        prev_self = prev_objects.get("self")

    if prev_self:
        self_vx = (self_x - float(prev_self["field_x"])) * fps
        self_vy = (self_y - float(prev_self["field_y"])) * fps
    else:
        self_vx = 0.0
        self_vy = 0.0

    ball_obj = objects.get("ball")
    ball_visible = ball_obj is not None
    ball_rel_x = 0.0
    ball_rel_y = 0.0
    has_ball = False
    if ball_obj is not None:
        dx = float(ball_obj["field_x"]) - self_x
        dy = float(ball_obj["field_y"]) - self_y
        ball_rel_x, ball_rel_y = field_to_robot(dx, dy, self_yaw)
        has_ball = math.hypot(dx, dy) <= 9.5

    enemies = []
    for obj in current["objects"]:
        if obj["track_id"].startswith("enemy"):
            dx = float(obj["field_x"]) - self_x
            dy = float(obj["field_y"]) - self_y
            rel_x, rel_y = field_to_robot(dx, dy, self_yaw)
            enemies.append(
                EntityObservationV1(
                    rel_x=rel_x,
                    rel_y=rel_y,
                    visible=True,
                    age_ms=0,
                    confidence=float(obj["confidence"]),
                )
            )

    return DeployObservationV1(
        self_x=self_x,
        self_y=self_y,
        self_yaw=self_yaw,
        self_vx=self_vx,
        self_vy=self_vy,
        self_wz=0.0,
        velocity_valid=previous is not None,
        ball_rel_x=ball_rel_x,
        ball_rel_y=ball_rel_y,
        ball_visible=ball_visible,
        ball_age_ms=0 if ball_visible else 999,
        has_ball=has_ball,
        wall_front=max(0.0, FIELD_X_MAX - self_x),
        wall_left=max(0.0, FIELD_Y_MAX - self_y),
        wall_back=max(0.0, self_x - FIELD_X_MIN),
        wall_right=max(0.0, self_y - FIELD_Y_MIN),
        wall_valid=True,
        enemies=enemies,
        max_enemies=4,
        role=2,
    )


class MockSegmentLabeler:
    """Stand-in for Gemma/Qwen segment-level intent labeling."""

    def __init__(self, model_name: str):
        self.model_name = model_name

    def label_frames(self, observations: list[DeployObservationV1]) -> list[dict[str, Any]]:
        frame_intents = [self._intent_for_obs(obs) for obs in observations]
        segments: list[dict[str, Any]] = []
        start = 0
        for idx in range(1, len(frame_intents) + 1):
            if idx == len(frame_intents) or frame_intents[idx] != frame_intents[start]:
                intent = frame_intents[start]
                confidence = self._confidence(intent, observations[start:idx])
                segments.append(
                    {
                        "start_frame": start,
                        "end_frame": idx - 1,
                        "intent": IntentId(intent).name,
                        "intent_id": int(intent),
                        "confidence": confidence,
                        "model": self.model_name,
                        "reason": self._reason(intent),
                    }
                )
                start = idx
        return segments

    def _intent_for_obs(self, obs: DeployObservationV1) -> IntentId:
        if abs(obs.self_x) > 88.0 or abs(obs.self_y) > 68.0:
            return IntentId.AVOID_WALL
        if obs.has_ball:
            goal_yaw = angle_to(obs.self_x, obs.self_y, FIELD_X_MAX, 0.0)
            if obs.self_x > 55.0 and abs(obs.self_y) < 28.0 and abs(angle_error(obs.self_yaw, goal_yaw)) < 14.0:
                return IntentId.SHOOT
            return IntentId.CARRY_TO_POSE
        if not obs.ball_visible:
            return IntentId.SEARCH_BALL
        return IntentId.APPROACH_BALL

    def _confidence(self, intent: IntentId, obs_slice: list[DeployObservationV1]) -> float:
        base = {
            IntentId.SEARCH_BALL: 0.86,
            IntentId.APPROACH_BALL: 0.88,
            IntentId.CARRY_TO_POSE: 0.84,
            IntentId.SHOOT: 0.82,
            IntentId.AVOID_WALL: 0.93,
        }.get(intent, 0.75)
        if len(obs_slice) < 5:
            base -= 0.08
        return round(clamp(base, 0.0, 1.0), 3)

    def _reason(self, intent: IntentId) -> str:
        return {
            IntentId.SEARCH_BALL: "ball is not visible across the segment",
            IntentId.APPROACH_BALL: "visible ball and no possession",
            IntentId.CARRY_TO_POSE: "possession detected, moving toward shooting lane",
            IntentId.SHOOT: "possession near goal with heading aligned",
            IntentId.AVOID_WALL: "robot is outside the preferred safe region",
        }.get(intent, "fallback label")


def segment_lookup(segments: list[dict[str, Any]], frame: int) -> dict[str, Any]:
    for segment in segments:
        if int(segment["start_frame"]) <= frame <= int(segment["end_frame"]):
            return segment
    raise KeyError(f"frame {frame} not covered by segments")


def decision_for_label(segment: dict[str, Any], obs: DeployObservationV1) -> DecisionPacketV1:
    intent = IntentId(int(segment["intent_id"]))
    goal_yaw = angle_to(obs.self_x, obs.self_y, FIELD_X_MAX, 0.0)
    confidence = float(segment["confidence"])

    if intent == IntentId.SEARCH_BALL:
        target_x = -40.0 if obs.self_x < -55.0 else 10.0
        target_y = -45.0 if obs.self_y > 0.0 else 45.0
        return DecisionPacketV1(
            intent_id=int(intent),
            target_x=target_x,
            target_y=target_y,
            target_yaw=obs.self_yaw,
            face_mode=int(FaceMode.FACE_KEEP),
            move_gain=1.3,
            confidence=confidence,
            decision_horizon_ms=400,
        )

    if intent == IntentId.APPROACH_BALL:
        yaw = angle_to(0.0, 0.0, obs.ball_rel_x, obs.ball_rel_y)
        return DecisionPacketV1(
            intent_id=int(intent),
            target_x=obs.self_x,
            target_y=obs.self_y,
            target_yaw=yaw,
            face_mode=int(FaceMode.FACE_BALL),
            move_gain=1.0,
            dribble_mode=int(DribbleMode.WEAK),
            confidence=confidence,
            decision_horizon_ms=180,
        )

    if intent == IntentId.CARRY_TO_POSE:
        return DecisionPacketV1(
            intent_id=int(intent),
            target_x=82.0,
            target_y=clamp(obs.self_y * 0.45, -25.0, 25.0),
            target_yaw=goal_yaw,
            face_mode=int(FaceMode.FACE_GOAL),
            move_gain=1.8,
            dribble_mode=int(DribbleMode.STRONG),
            confidence=confidence,
            decision_horizon_ms=220,
        )

    if intent == IntentId.SHOOT:
        return DecisionPacketV1(
            intent_id=int(intent),
            target_x=obs.self_x,
            target_y=obs.self_y,
            target_yaw=goal_yaw,
            face_mode=int(FaceMode.FACE_GOAL),
            move_gain=0.0,
            dribble_mode=int(DribbleMode.STRONG),
            shoot_intent=int(ShootIntent.TRY_SHOOT),
            confidence=confidence,
            decision_horizon_ms=120,
        )

    if intent == IntentId.AVOID_WALL:
        return DecisionPacketV1(
            intent_id=int(intent),
            target_x=clamp(obs.self_x, -74.0, 74.0),
            target_y=clamp(obs.self_y, -58.0, 58.0),
            target_yaw=obs.self_yaw,
            face_mode=int(FaceMode.FACE_KEEP),
            move_gain=1.1,
            dribble_mode=int(DribbleMode.OFF),
            confidence=confidence,
            decision_horizon_ms=300,
        )

    return DecisionPacketV1(intent_id=int(IntentId.RETURN_HOME), target_x=0.0, target_y=0.0, confidence=0.5)


def encode_action(packet: DecisionPacketV1, obs: DeployObservationV1, config: MLPConfig) -> np.ndarray:
    vx = 0.0
    vy = 0.0
    wz = 0.0
    kick = 0.0
    dribble = 0.0

    intent = IntentId(packet.intent_id)
    if intent in (IntentId.SEARCH_BALL, IntentId.CARRY_TO_POSE, IntentId.AVOID_WALL):
        dx = float(packet.target_x) - float(obs.self_x)
        dy = float(packet.target_y) - float(obs.self_y)
        dist = math.hypot(dx, dy)
        if dist > 1e-6:
            gain = clamp(float(packet.move_gain) / 2.2, 0.0, 1.0)
            vx = (dx / dist) * gain
            vy = (dy / dist) * gain
    elif intent == IntentId.APPROACH_BALL:
        dist = math.hypot(float(obs.ball_rel_x), float(obs.ball_rel_y))
        if dist > 1e-6:
            gain = min(1.0, dist / 120.0)
            vx = (float(obs.ball_rel_x) / dist) * gain
            vy = (float(obs.ball_rel_y) / dist) * gain
    elif intent == IntentId.SHOOT:
        kick = 1.0

    yaw_error = angle_error(float(obs.self_yaw), float(packet.target_yaw))
    wz = clamp(yaw_error / max(config.max_angular_speed, 1e-6), -1.0, 1.0)

    dribble_mode = DribbleMode(packet.dribble_mode)
    if dribble_mode == DribbleMode.WEAK:
        dribble = 0.35
    elif dribble_mode == DribbleMode.STRONG:
        dribble = 0.8

    return np.asarray([vx, vy, wz, kick, dribble], dtype=np.float32)


def write_jsonl(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8") as f:
        for row in rows:
            f.write(json.dumps(row, ensure_ascii=False) + "\n")


def render_report(path: Path, timeline: list[dict[str, Any]], summary: dict[str, Any]) -> None:
    payload = json.dumps({"timeline": timeline, "summary": summary}, ensure_ascii=False)
    html = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Video Policy Dataset Demo</title>
  <style>
    :root {{
      color-scheme: light;
      --bg: #f7f8fb;
      --panel: #ffffff;
      --ink: #15202b;
      --muted: #667085;
      --line: #d7dde8;
      --accent: #0f766e;
      --orange: #d97706;
      --red: #dc2626;
      --blue: #2563eb;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      font-family: Inter, "Segoe UI", Arial, sans-serif;
      background: var(--bg);
      color: var(--ink);
    }}
    header {{
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 20px;
      padding: 16px 22px;
      border-bottom: 1px solid var(--line);
      background: var(--panel);
    }}
    h1 {{ margin: 0; font-size: 18px; font-weight: 700; }}
    .meta {{ color: var(--muted); font-size: 13px; }}
    main {{
      display: grid;
      grid-template-columns: minmax(460px, 1fr) 360px;
      gap: 16px;
      padding: 16px;
    }}
    .surface {{
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      overflow: hidden;
    }}
    .toolbar {{
      display: flex;
      align-items: center;
      gap: 10px;
      padding: 12px;
      border-bottom: 1px solid var(--line);
    }}
    button {{
      height: 34px;
      min-width: 38px;
      border: 1px solid var(--line);
      background: #fff;
      border-radius: 6px;
      cursor: pointer;
      color: var(--ink);
      font-weight: 600;
    }}
    button:hover {{ border-color: var(--accent); color: var(--accent); }}
    input[type="range"] {{ flex: 1; }}
    canvas {{
      display: block;
      width: 100%;
      aspect-ratio: 1.3 / 1;
      background: #166534;
    }}
    aside {{ display: flex; flex-direction: column; gap: 16px; }}
    .section {{ padding: 14px; }}
    .section h2 {{ margin: 0 0 10px; font-size: 14px; }}
    .kv {{ display: grid; grid-template-columns: 120px 1fr; gap: 8px; font-size: 13px; }}
    .key {{ color: var(--muted); }}
    .pill {{
      display: inline-flex;
      align-items: center;
      height: 24px;
      padding: 0 8px;
      border-radius: 999px;
      background: #e6fffb;
      color: #0f766e;
      font-size: 12px;
      font-weight: 700;
    }}
    pre {{
      max-height: 260px;
      overflow: auto;
      padding: 12px;
      margin: 0;
      background: #0f172a;
      color: #dbeafe;
      font-size: 12px;
      line-height: 1.45;
    }}
    @media (max-width: 900px) {{
      main {{ grid-template-columns: 1fr; }}
    }}
  </style>
</head>
<body>
  <header>
    <div>
      <h1>Video Policy Dataset Demo</h1>
      <div class="meta">Mock YOLO -> tracklets -> mock Gemma/Qwen labels -> transitions.npz</div>
    </div>
    <div class="meta" id="shape"></div>
  </header>
  <main>
    <section class="surface">
      <div class="toolbar">
        <button id="prev" title="Previous frame">&lt;</button>
        <button id="play" title="Play or pause">Play</button>
        <button id="next" title="Next frame">&gt;</button>
        <input id="scrub" type="range" min="0" max="0" value="0" />
        <span class="meta" id="frameLabel"></span>
      </div>
      <canvas id="field" width="960" height="740"></canvas>
    </section>
    <aside>
      <section class="surface section">
        <h2>Current Label</h2>
        <div class="kv">
          <div class="key">Intent</div><div><span class="pill" id="intent"></span></div>
          <div class="key">Confidence</div><div id="confidence"></div>
          <div class="key">Reason</div><div id="reason"></div>
          <div class="key">Phase Hint</div><div id="phase"></div>
        </div>
      </section>
      <section class="surface section">
        <h2>Dataset Summary</h2>
        <div class="kv" id="summary"></div>
      </section>
      <section class="surface">
        <pre id="json"></pre>
      </section>
    </aside>
  </main>
  <script>
    const data = {payload};
    const timeline = data.timeline;
    const summary = data.summary;
    const canvas = document.getElementById('field');
    const ctx = canvas.getContext('2d');
    const scrub = document.getElementById('scrub');
    let idx = 0;
    let timer = null;
    scrub.max = String(timeline.length - 1);
    document.getElementById('shape').textContent = `${{summary.state_shape.join('x')}} states, ${{summary.action_shape.join('x')}} actions`;

    function sx(x) {{ return ((x + 130) / 260) * canvas.width; }}
    function sy(y) {{ return canvas.height - ((y + 100) / 200) * canvas.height; }}

    function drawField() {{
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = '#166534';
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      ctx.strokeStyle = 'rgba(255,255,255,.8)';
      ctx.lineWidth = 3;
      ctx.strokeRect(sx(-130), sy(100), sx(130)-sx(-130), sy(-100)-sy(100));
      ctx.beginPath();
      ctx.moveTo(sx(0), sy(-100));
      ctx.lineTo(sx(0), sy(100));
      ctx.stroke();
      ctx.beginPath();
      ctx.arc(sx(0), sy(0), 72, 0, Math.PI * 2);
      ctx.stroke();
      ctx.strokeRect(sx(105), sy(45), sx(130)-sx(105), sy(-45)-sy(45));
      ctx.strokeRect(sx(-130), sy(45), sx(-105)-sx(-130), sy(-45)-sy(45));
    }}

    function drawPoint(x, y, color, label, radius=10) {{
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.arc(sx(x), sy(y), radius, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = '#fff';
      ctx.font = '14px Segoe UI';
      ctx.fillText(label, sx(x) + 12, sy(y) - 12);
    }}

    function draw() {{
      const row = timeline[idx];
      drawField();
      ctx.strokeStyle = 'rgba(255,255,255,.35)';
      ctx.lineWidth = 2;
      ctx.beginPath();
      for (let i = 0; i <= idx; i++) {{
        const obs = timeline[i].observation;
        const x = sx(obs.self_x);
        const y = sy(obs.self_y);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }}
      ctx.stroke();

      const obs = row.observation;
      const yaw = obs.self_yaw * Math.PI / 180;
      drawPoint(obs.self_x, obs.self_y, '#2563eb', 'self', 13);
      ctx.strokeStyle = '#bfdbfe';
      ctx.lineWidth = 4;
      ctx.beginPath();
      ctx.moveTo(sx(obs.self_x), sy(obs.self_y));
      ctx.lineTo(sx(obs.self_x + Math.cos(yaw) * 18), sy(obs.self_y + Math.sin(yaw) * 18));
      ctx.stroke();

      if (row.ball_field) drawPoint(row.ball_field.x, row.ball_field.y, '#f97316', 'ball', 8);
      if (row.enemy_field) drawPoint(row.enemy_field.x, row.enemy_field.y, '#dc2626', 'enemy', 11);
      if (row.decision) {{
        drawPoint(row.decision.target_x, row.decision.target_y, '#0f766e', 'target', 7);
      }}

      document.getElementById('frameLabel').textContent = `Frame ${{row.frame}} / ${{timeline.length - 1}}`;
      document.getElementById('intent').textContent = row.label.intent;
      document.getElementById('confidence').textContent = row.label.confidence.toFixed(3);
      document.getElementById('reason').textContent = row.label.reason;
      document.getElementById('phase').textContent = row.phase_hint;
      document.getElementById('json').textContent = JSON.stringify(row, null, 2);
      scrub.value = String(idx);
    }}

    function setIndex(next) {{
      idx = Math.max(0, Math.min(timeline.length - 1, next));
      draw();
    }}

    document.getElementById('prev').onclick = () => setIndex(idx - 1);
    document.getElementById('next').onclick = () => setIndex(idx + 1);
    document.getElementById('play').onclick = () => {{
      if (timer) {{
        clearInterval(timer);
        timer = null;
        document.getElementById('play').textContent = 'Play';
      }} else {{
        timer = setInterval(() => setIndex((idx + 1) % timeline.length), 120);
        document.getElementById('play').textContent = 'Pause';
      }}
    }};
    scrub.oninput = (event) => setIndex(Number(event.target.value));

    const summaryEl = document.getElementById('summary');
    const summaryRows = [
      ['Frames', summary.frames],
      ['Labels', JSON.stringify(summary.intents)],
      ['NPZ', summary.npz],
      ['Detections', summary.detections],
    ];
    summaryEl.innerHTML = summaryRows.map(([k, v]) => `<div class="key">${{k}}</div><div>${{v}}</div>`).join('');
    draw();
  </script>
</body>
</html>
"""
    path.write_text(html, encoding="utf-8")


def build_demo(args: argparse.Namespace) -> dict[str, Any]:
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    projector = FieldProjector()
    rng = random.Random(args.seed)
    detector = MockYoloDetector(projector, rng)
    labeler = MockSegmentLabeler(args.labeler)
    encoder = StateEncoder(MLPConfig())
    config = MLPConfig()

    detections_rows = []
    tracklet_rows = []
    observations: list[DeployObservationV1] = []
    timeline_rows = []

    previous_tracklet = None
    for frame in SyntheticMatch(args.frames, args.fps).frames_iter():
        detections = detector.detect(frame)
        tracklet = detections_to_tracklet(frame, detections, projector)
        obs = tracklet_to_observation(tracklet, previous_tracklet, args.fps)

        detections_rows.append(
            {
                "frame": frame.frame,
                "time_sec": round(frame.time_sec, 3),
                "detections": detections,
            }
        )
        tracklet_rows.append(tracklet)
        observations.append(obs)
        previous_tracklet = tracklet

    labels = labeler.label_frames(observations)

    states = []
    actions = []
    rewards = []
    dones = []
    intent_counts: Counter[str] = Counter()

    for frame_idx, (tracklet, obs) in enumerate(zip(tracklet_rows, observations)):
        label = segment_lookup(labels, frame_idx)
        packet = decision_for_label(label, obs)
        action = encode_action(packet, obs, config)
        states.append(encoder.encode(obs))
        actions.append(action)
        rewards.append(1.0 if IntentId(packet.intent_id) == IntentId.SHOOT else 0.0)
        dones.append(False)
        intent_counts[IntentId(packet.intent_id).name] += 1

        object_map = {obj["track_id"]: obj for obj in tracklet["objects"]}
        ball = object_map.get("ball")
        enemy = object_map.get("enemy_0")
        timeline_rows.append(
            {
                "frame": frame_idx,
                "time_sec": tracklet["time_sec"],
                "phase_hint": tracklet["phase_hint"],
                "observation": obs.to_dict(),
                "label": label,
                "decision": packet.to_dict(),
                "action": action.tolist(),
                "ball_field": {"x": ball["field_x"], "y": ball["field_y"]} if ball else None,
                "enemy_field": {"x": enemy["field_x"], "y": enemy["field_y"]} if enemy else None,
            }
        )

    detections_path = output_dir / f"detections_{timestamp}.jsonl"
    tracklets_path = output_dir / f"tracklets_{timestamp}.jsonl"
    labels_path = output_dir / f"labels_{timestamp}.jsonl"
    transitions_path = output_dir / f"transitions_{timestamp}.npz"
    summary_path = output_dir / f"summary_{timestamp}.json"
    report_path = output_dir / f"report_{timestamp}.html"

    write_jsonl(detections_path, detections_rows)
    write_jsonl(tracklets_path, tracklet_rows)
    write_jsonl(labels_path, labels)
    np.savez_compressed(
        transitions_path,
        states=np.asarray(states, dtype=np.float32),
        actions=np.asarray(actions, dtype=np.float32),
        rewards=np.asarray(rewards, dtype=np.float32),
        dones=np.asarray(dones, dtype=np.bool_),
    )

    summary = {
        "frames": args.frames,
        "fps": args.fps,
        "seed": args.seed,
        "labeler": args.labeler,
        "detections": str(detections_path),
        "tracklets": str(tracklets_path),
        "labels": str(labels_path),
        "npz": str(transitions_path),
        "summary": str(summary_path),
        "report": str(report_path),
        "state_shape": list(np.asarray(states).shape),
        "action_shape": list(np.asarray(actions).shape),
        "intents": dict(sorted(intent_counts.items())),
    }
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    render_report(report_path, timeline_rows, summary)
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate a video policy dataset demo")
    parser.add_argument("--frames", type=int, default=240, help="number of synthetic frames")
    parser.add_argument("--fps", type=float, default=15.0, help="synthetic video FPS")
    parser.add_argument("--seed", type=int, default=20260616, help="random seed")
    parser.add_argument("--labeler", default="mock-qwen3", help="name recorded for the mock VLM labeler")
    parser.add_argument("--output-dir", default="Records/video_policy_dataset_demo", help="output directory")
    args = parser.parse_args()

    if args.frames < 30:
        raise ValueError("--frames should be at least 30 for useful segments")
    if args.fps <= 0:
        raise ValueError("--fps must be positive")

    summary = build_demo(args)
    print("[video-demo] wrote detections:", summary["detections"])
    print("[video-demo] wrote tracklets:", summary["tracklets"])
    print("[video-demo] wrote labels:", summary["labels"])
    print("[video-demo] wrote npz:", summary["npz"])
    print("[video-demo] wrote report:", summary["report"])
    print("[video-demo] states=", summary["state_shape"], "actions=", summary["action_shape"])
    print("[video-demo] intents=", summary["intents"])


if __name__ == "__main__":
    main()
