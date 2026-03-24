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

        # Optional goals (Blue/Yellow)
        goals = []
        try:
            goals = vision.GetGoalsPos()
        except Exception:
            goals = []

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
            "goals_list_cm": goals,
            "ball_bbox_640": ball_bbox,  # (cam,x1,y1,x2,y2,conf) | null
            "ball_bboxes_640": ball_bboxes,  # list len=4
            "chassis_bbox_640": chassis_bbox,
            "chassis_bboxes_640": chassis_bboxes,
            "counters": counters,
        }

    def cam_jpeg(self, cam_index: int, resize_to: int | None = None) -> bytes | None:
        try:
            frames = getattr(self.vision, "Frames", None)
            if frames is None or cam_index < 0 or cam_index >= len(frames):
                return None
            fr = frames[cam_index]
            if fr is None:
                return None
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
            ok, buf = cv2.imencode(
                ".jpg",
                img,
                [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)],
            )
            if not ok:
                return None
            return bytes(buf)
        except Exception:
            return None


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
                payload = json.dumps(app.state_dict(), ensure_ascii=False).encode("utf-8")
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
                jpg = app.cam_jpeg(cam_i, resize_to=resize_to)
                if jpg is None:
                    return self._send(204, "text/plain; charset=utf-8", b"")
                return self._send(200, "image/jpeg", jpg)

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
