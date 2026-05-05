"""Vision Dashboard Server

This script exposes a tiny HTTP server for visualizing `src/Vision.py` outputs
in a web dashboard (auto-driving style HUD).

Design goals
------------
- Zero extra python dependencies (uses stdlib http.server)
- Read-only: does not change existing vision pipeline
- Works with all backends selected by `vision_utils.Detect_Method`

Endpoints 
---------
- GET /               : dashboard HTML
- GET /static/*       : static assets (JS/CSS)
- GET /api/state      : latest positions & bboxes as JSON
- GET /api/cam/{i}.jpg: camera i latest frame as JPEG (i=0..3)

Notes
-----
- Camera frames are pulled from `vision.Frames[i]` (BGR ndarray) and encoded
  as JPEG on request.
- Ball bbox (xyxy) is available only on TunaIntelligence backend; other
  backends will return null.
"""

from __future__ import annotations

import argparse
import json
import os
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import cv2


def _repo_root() -> Path:
    # scripts/vision_dashboard.py -> repo root
    return Path(__file__).resolve().parents[1]


def _json_safe(obj):
    """Recursively convert common numpy / custom scalar types to JSON-safe values."""
    if obj is None or isinstance(obj, (str, int, float, bool)):
        return obj

    # numpy scalars / other scalar-like objects
    item = getattr(obj, "item", None)
    if callable(item):
        try:
            return _json_safe(item())
        except Exception:
            pass

    # numpy arrays / array-like objects
    tolist = getattr(obj, "tolist", None)
    if callable(tolist):
        try:
            return _json_safe(tolist())
        except Exception:
            pass

    if isinstance(obj, dict):
        return {str(k): _json_safe(v) for k, v in obj.items()}

    if isinstance(obj, (list, tuple, set)):
        return [_json_safe(v) for v in obj]

    # Fallback: make it printable instead of crashing the dashboard endpoint.
    return str(obj)


def _bbox_at_cam(bbox, cam_index: int):
    """Return bbox tuple for a specific camera, accepting either per-cam lists or a single bbox."""
    if bbox is None:
        return None
    if isinstance(bbox, (list, tuple)) and len(bbox) == 4 and all(
        (item is None or isinstance(item, (list, tuple))) for item in bbox
    ):
        if cam_index < 0 or cam_index >= len(bbox):
            return None
        return bbox[cam_index]
    if isinstance(bbox, (list, tuple)) and len(bbox) >= 6:
        try:
            if int(bbox[0]) == int(cam_index):
                return bbox
        except Exception:
            return None
    return None


def _draw_bbox_label(img, bbox, color, label: str, scale_x: float = 1.0, scale_y: float = 1.0):
    if bbox is None:
        return
    try:
        if len(bbox) < 6:
            return
        # Support both:
        #   (cam, x1, y1, x2, y2, conf)
        #   (cam, cls, x1, y1, x2, y2, conf)
        if len(bbox) >= 7:
            x1_i, y1_i, x2_i, y2_i, conf_i = 2, 3, 4, 5, 6
        else:
            x1_i, y1_i, x2_i, y2_i, conf_i = 1, 2, 3, 4, 5
        x1 = int(round(float(bbox[x1_i]) * scale_x))
        y1 = int(round(float(bbox[y1_i]) * scale_y))
        x2 = int(round(float(bbox[x2_i]) * scale_x))
        y2 = int(round(float(bbox[y2_i]) * scale_y))
        conf = float(bbox[conf_i])
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        text = f"{label} {conf:.2f}"
        text_y = max(15, y1 - 5)
        cv2.putText(img, text, (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    except Exception:
        pass


def _map_bbox_640_to_image(bbox, src_w: int, src_h: int, dst_w: int, dst_h: int):
    """Map a bbox from 640x640 detector space into the displayed image space.

    The detector uses letterbox preprocessing, so we first undo the padding/scale
    from 640x640 back to the source frame, then scale to the final display size.
    Returns a bbox tuple with coordinates in the destination image space.
    """
    if bbox is None:
        return None
    try:
        if len(bbox) < 6:
            return None
        prefix = list(bbox[:2]) if len(bbox) >= 7 else [bbox[0]]
        x1 = float(bbox[2 if len(bbox) >= 7 else 1])
        y1 = float(bbox[3 if len(bbox) >= 7 else 2])
        x2 = float(bbox[4 if len(bbox) >= 7 else 3])
        y2 = float(bbox[5 if len(bbox) >= 7 else 4])
        conf = float(bbox[6 if len(bbox) >= 7 else 5])

        src_w = max(1, int(src_w))
        src_h = max(1, int(src_h))
        dst_w = max(1, int(dst_w))
        dst_h = max(1, int(dst_h))

        # Inverse of the letterbox used to resize source -> 640x640.
        scale = min(640.0 / src_w, 640.0 / src_h)
        new_w = src_w * scale
        new_h = src_h * scale
        pad_x = (640.0 - new_w) / 2.0
        pad_y = (640.0 - new_h) / 2.0

        def to_src(xp: float, yp: float) -> tuple[float, float]:
            return (xp - pad_x) / scale, (yp - pad_y) / scale

        sx1, sy1 = to_src(x1, y1)
        sx2, sy2 = to_src(x2, y2)

        # Clip to source frame first.
        sx1 = max(0.0, min(float(src_w - 1), sx1))
        sy1 = max(0.0, min(float(src_h - 1), sy1))
        sx2 = max(0.0, min(float(src_w - 1), sx2))
        sy2 = max(0.0, min(float(src_h - 1), sy2))

        # Then scale to the displayed image size.
        dx1 = sx1 * (dst_w / float(src_w))
        dy1 = sy1 * (dst_h / float(src_h))
        dx2 = sx2 * (dst_w / float(src_w))
        dy2 = sy2 * (dst_h / float(src_h))

        if len(bbox) >= 7:
            return (int(prefix[0]), int(prefix[1]), int(round(dx1)), int(round(dy1)), int(round(dx2)), int(round(dy2)), conf)
        return (int(prefix[0]), int(round(dx1)), int(round(dy1)), int(round(dx2)), int(round(dy2)), conf)
    except Exception:
        return None


class _App:
    def __init__(self, vision, web_dir: Path, jpeg_quality: int = 75):
        self.vision = vision
        self.web_dir = web_dir
        self.jpeg_quality = int(jpeg_quality)
        self.start_ts = time.time()

    def state_dict(self) -> dict:
        vision = self.vision
        ball_pos = None
        chassis_pos = None
        try:
            ball_pos = vision.GetBallPos()
        except Exception:
            ball_pos = [0, 0]
        try:
            chassis_pos = vision.GetChassisPos()
        except Exception:
            chassis_pos = []

        # Optional bbox cache (TunaIntelligence)
        ball_bbox = None
        ball_bboxes = None
        try:
            ball_bbox = vision.GetBallBBox()
        except Exception:
            ball_bbox = None
        try:
            ball_bboxes = vision.GetBallBBoxes()
        except Exception:
            ball_bboxes = [None, None, None, None]

        chassis_bbox = None
        chassis_bboxes = None
        try:
            chassis_bbox = vision.GetChassisBBox()
        except Exception:
            chassis_bbox = None
        try:
            chassis_bboxes = vision.GetChassisBBoxes()
        except Exception:
            chassis_bboxes = [None, None, None, None]

        goal_bbox = None
        goal_bboxes = None
        try:
            goal_bbox = vision.GetGoalBBox()
        except Exception:
            goal_bbox = None
        try:
            goal_bboxes = vision.GetGoalBBoxes()
        except Exception:
            goal_bboxes = [None, None, None, None]

        # Optional goals (Blue/Yellow), list format: [blue_goal, yellow_goal]
        blue_goal = None
        yellow_goal = None
        try:
            goals = vision.GetGoalsPos()
            if isinstance(goals, (list, tuple)):
                if len(goals) >= 1:
                    blue_goal = goals[0]
                if len(goals) >= 2:
                    yellow_goal = goals[1]
            elif isinstance(goals, dict):
                # backward compatibility
                blue_goal = goals.get("blue_goal")
                yellow_goal = goals.get("yellow_goal")
        except Exception:
            blue_goal = None
            yellow_goal = None

        # Provide some basic counters if present
        counters = {}
        for k in ("ReadCamTF", "PreProcessTF", "InferTF", "PostProcessTF"):
            if hasattr(vision, k):
                try:
                    counters[k] = int(getattr(vision, k))
                except Exception:
                    pass

        return {
            "ts": time.time(),
            "uptime_s": round(time.time() - self.start_ts, 3),
            "ball_pos_cm": ball_pos,
            "chassis_list_cm": chassis_pos,
            "blue_goal_cm": blue_goal,
            "yellow_goal_cm": yellow_goal,
            "ball_bbox_640": ball_bbox,  # (cam,x1,y1,x2,y2,conf) | null
            "ball_bboxes_640": ball_bboxes,  # list len=4
            "chassis_bbox_640": chassis_bbox,
            "chassis_bboxes_640": chassis_bboxes,
            "goal_bbox_640": goal_bbox,
            "goal_bboxes_640": goal_bboxes,
            "counters": counters,
        }

    def state_dict_json_safe(self) -> dict:
        return _json_safe(self.state_dict())

    def cam_jpeg(self, cam_index: int, resize_to: int | None = None) -> tuple[bytes | None, float | None]:
        """Get camera frame as JPEG with processing time.

        Returns:
            tuple: (jpeg_bytes, processing_time_ms) or (None, None) on error
        """
        start_ts = time.time()
        try:
            frames = getattr(self.vision, "Frames", None)
            if frames is None or cam_index < 0 or cam_index >= len(frames):
                return None, None
            fr = frames[cam_index]
            if fr is None:
                return None, None
            img = fr
            if resize_to is not None:
                # keep aspect ratio, set long side=resize_to
                h, w = img.shape[:2]
                if max(h, w) > int(resize_to):
                    if h >= w:
                        nh = int(resize_to)
                        nw = int(w * (nh / h))
                    else:
                        nw = int(resize_to)
                        nh = int(h * (nw / w))
                    img = cv2.resize(img, (nw, nh))

            # 不在后端画框，画框由前端完成

            ok, buf = cv2.imencode(
                ".jpg",
                img,
                [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)],
            )
            if not ok:
                return None, None
            processing_time_ms = (time.time() - start_ts) * 1000.0
            return bytes(buf), processing_time_ms
        except Exception:
            return None, None


def _guess_content_type(p: Path) -> str:
    s = p.suffix.lower()
    if s == ".html":
        return "text/html; charset=utf-8"
    if s == ".js":
        return "application/javascript; charset=utf-8"
    if s == ".css":
        return "text/css; charset=utf-8"
    if s == ".svg":
        return "image/svg+xml"
    if s in (".png", ".jpg", ".jpeg", ".gif"):
        return f"image/{s.lstrip('.').replace('jpeg', 'jpg')}"
    return "application/octet-stream"


def make_handler(app: _App):
    web_dir = app.web_dir

    class Handler(BaseHTTPRequestHandler):
        server_version = "VisionDashboard/0.1"

        def log_message(self, fmt: str, *args):
            # Keep it concise; integrate with existing logger if needed.
            try:
                print("[dash] " + (fmt % args))
            except Exception:
                pass

        def _send(self, status: int, content_type: str, body: bytes):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)
            path = u.path

            if path == "/" or path == "":
                p = web_dir / "index.html"
                body = p.read_bytes()
                return self._send(200, "text/html; charset=utf-8", body)

            if path.startswith("/static/"):
                rel = path[len("/static/") :]
                p = (web_dir / "static" / rel).resolve()
                # Prevent path traversal
                if not str(p).startswith(str((web_dir / "static").resolve())):
                    return self._send(403, "text/plain; charset=utf-8", b"forbidden")
                if not p.exists() or not p.is_file():
                    return self._send(404, "text/plain; charset=utf-8", b"not found")
                return self._send(200, _guess_content_type(p), p.read_bytes())

            if path == "/api/state":
                payload = json.dumps(app.state_dict_json_safe(), ensure_ascii=False).encode("utf-8")
                return self._send(200, "application/json; charset=utf-8", payload)

            if path.startswith("/api/cam/") and path.endswith(".jpg"):
                # /api/cam/0.jpg or /api/cam/0.jpg?size=640
                try:
                    name = path.split("/")[-1]
                    cam_i = int(name.split(".")[0])
                except Exception:
                    return self._send(400, "text/plain; charset=utf-8", b"bad cam index")
                q = u.query or ""
                resize_to = None
                if "size=" in q:
                    try:
                        resize_to = int(q.split("size=")[-1].split("&")[0])
                    except Exception:
                        resize_to = None
                jpg, processing_time_ms = app.cam_jpeg(cam_i, resize_to=resize_to)
                if jpg is None:
                    return self._send(204, "text/plain; charset=utf-8", b"")
                # Send processing time in response header for latency display
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(jpg)))
                if processing_time_ms is not None:
                    self.send_header("X-Processing-Time-Ms", f"{processing_time_ms:.1f}")
                self.end_headers()
                self.wfile.write(jpg)
                return

            return self._send(404, "text/plain; charset=utf-8", b"not found")

    return Handler


def main():
    ap = argparse.ArgumentParser(description="Web dashboard for src/Vision.py")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--jpeg-quality", type=int, default=75)
    ap.add_argument(
        "--web-dir",
        default=str(_repo_root() / "web" / "vision_dashboard"),
        help="Directory containing index.html and static/",
    )
    args = ap.parse_args()

    # Import vision from repo
    # Allow running from repo root or anywhere.
    import sys

    root = str(_repo_root())
    if root not in sys.path:
        sys.path.insert(0, root)

    from src.Vision import UnitedVision  # noqa

    vision = UnitedVision()

    web_dir = Path(args.web_dir).resolve()
    if not (web_dir / "index.html").exists():
        raise FileNotFoundError(f"dashboard index.html not found in: {web_dir}")

    app = _App(vision=vision, web_dir=web_dir, jpeg_quality=args.jpeg_quality)
    handler = make_handler(app)
    httpd = ThreadingHTTPServer((args.host, int(args.port)), handler)
    print(f"Vision dashboard: http://{args.host}:{args.port} (web_dir={web_dir})")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            httpd.server_close()
        except Exception:
            pass
        try:
            vision.close(join_timeout=1.0)
        except Exception:
            pass


if __name__ == "__main__":
    main()
