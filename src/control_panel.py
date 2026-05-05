"""Xelerator Control Panel Server

统一控制面板后端，整合视觉监控和比赛记录/回放功能。

功能:
- 视觉监控 (继承自 vision_dashboard)
- 比赛记录管理
- 回放控制
- 系统设置

启动方法:
    python control_panel.py --port 8080
"""

from __future__ import annotations

import argparse
import io
import json
import os
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import cv2


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _json_safe(obj):
    """Recursively convert common numpy / custom scalar types to JSON-safe values."""
    if obj is None or isinstance(obj, (str, int, float, bool)):
        return obj
    item = getattr(obj, "item", None)
    if callable(item):
        try:
            return _json_safe(item())
        except Exception:
            pass
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
    return str(obj)


class ControlPanelApp:
    """控制面板应用状态"""
    
    def __init__(self, vision=None, web_dir: Path = None, jpeg_quality: int = 75):
        self.vision = vision
        self.web_dir = web_dir or _repo_root() / "web" / "control_panel"
        self.jpeg_quality = int(jpeg_quality)
        self.start_ts = time.time()
        
        # Recording state
        self._is_recording = False
        self._recording_session_id = None
        self._recording_frame_count = 0
        self._recording_start_ts = None
    
    def get_state_dict(self) -> dict:
        """获取视觉状态"""
        if not self.vision:
            return {"ts": time.time(), "uptime_s": time.time() - self.start_ts}
        
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
        
        return {
            "ts": time.time(),
            "uptime_s": round(time.time() - self.start_ts, 3),
            "ball_pos_cm": ball_pos,
            "chassis_list_cm": chassis_pos,
        }
    
    def get_cam_jpeg(self, cam_index: int, resize_to: int = None) -> bytes:
        """获取摄像头 JPEG 图像"""
        if not self.vision:
            return None
        try:
            frames = getattr(self.vision, "Frames", None)
            if frames is None or cam_index < 0 or cam_index >= len(frames):
                return None
            fr = frames[cam_index]
            if fr is None:
                return None
            img = fr
            if resize_to is not None:
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
    
    def get_recording_status(self) -> dict:
        """获取记录状态"""
        return {
            "is_recording": self._is_recording,
            "session_id": self._recording_session_id,
            "frame_count": self._recording_frame_count,
            "duration": time.time() - self._recording_start_ts if self._recording_start_ts else 0,
        }
    
    def start_recording(self, session_id: str = None, freq: int = 5, description: str = "") -> dict:
        """开始记录"""
        if self._is_recording:
            return {"success": False, "error": "Already recording"}
        
        try:
            from ReasonData.data import SetupDatabase, MatchSession
            SetupDatabase()
            
            if session_id is None:
                session_id = time.strftime("%Y%m%d_%H%M%S", time.localtime())
            
            # Create session in database
            try:
                MatchSession.create_session(session_id, description=description)
            except Exception:
                pass  # Session may already exist
            
            self._is_recording = True
            self._recording_session_id = session_id
            self._recording_frame_count = 0
            self._recording_start_ts = time.time()
            
            return {"success": True, "session_id": session_id}
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def stop_recording(self) -> dict:
        """停止记录"""
        if not self._is_recording:
            return {"success": False, "error": "Not recording"}
        
        try:
            from ReasonData.data import MatchSession
            session = MatchSession.get_or_none(MatchSession.SessionId == self._recording_session_id)
            if session:
                session.end_session()
        except Exception:
            pass
        
        result = {
            "success": True,
            "session_id": self._recording_session_id,
            "frame_count": self._recording_frame_count,
        }
        
        self._is_recording = False
        self._recording_session_id = None
        self._recording_frame_count = 0
        self._recording_start_ts = None
        
        return result
    
    def list_sessions(self) -> list:
        """列出所有会话"""
        try:
            from ReasonData.data import MatchSession, Positions
            
            sessions = MatchSession.select().order_by(MatchSession.StartTime.desc())
            result = []
            
            for s in sessions:
                frame_count = Positions.select().where(Positions.SessionId == s.SessionId).count()
                duration = 0
                if s.EndTime and s.StartTime:
                    duration = s.EndTime - s.StartTime
                
                result.append({
                    "session_id": s.SessionId,
                    "start_time": s.StartTime,
                    "end_time": s.EndTime,
                    "duration": duration,
                    "frame_count": frame_count,
                    "description": s.Description,
                })
            
            return result
        except Exception as e:
            print(f"[ControlPanel] Error listing sessions: {e}")
            return []
    
    def get_session_data(self, session_id: str) -> dict:
        """获取会话数据"""
        try:
            from ReasonData.data import ExportSessionToDict
            return ExportSessionToDict(session_id)
        except Exception as e:
            return {"error": str(e)}
    
    def delete_session(self, session_id: str) -> dict:
        """删除会话"""
        try:
            from ReasonData.data import MatchSession, Positions, Outputs
            
            # Delete positions
            Positions.delete().where(Positions.SessionId == session_id).execute()
            Outputs.delete().where(Outputs.SessionId == session_id).execute()
            
            # Delete session
            session = MatchSession.get_or_none(MatchSession.SessionId == session_id)
            if session:
                session.delete_instance()
            
            return {"success": True}
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def export_session(self, session_id: str) -> bytes:
        """导出会话为 JSON"""
        data = self.get_session_data(session_id)
        return json.dumps(data, ensure_ascii=False, indent=2).encode('utf-8')
    
    def get_config(self) -> dict:
        """获取配置"""
        try:
            from ReasonData.data import GetConfig
            return GetConfig()
        except Exception as e:
            print(f"[ControlPanel] Error getting config: {e}")
            return {}
    
    def save_config(self, config: dict) -> dict:
        """保存配置"""
        try:
            from ReasonData.data import SaveConfig
            SaveConfig(config)
            return {"success": True}
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def get_system_info(self) -> dict:
        """获取系统信息"""
        import socket
        
        # Get local IP
        local_ip = "--"
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
            s.close()
        except Exception:
            pass
        
        return {
            "local_ip": local_ip,
            "hostname": socket.gethostname(),
        }


def make_handler(app: ControlPanelApp):
    web_dir = app.web_dir
    
    class Handler(BaseHTTPRequestHandler):
        server_version = "ControlPanel/1.0"
        
        def log_message(self, fmt: str, *args):
            try:
                print("[panel] " + (fmt % args))
            except Exception:
                pass
        
        def _send(self, status: int, content_type: str, body: bytes):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        
        def _send_json(self, data: dict, status: int = 200):
            body = json.dumps(_json_safe(data), ensure_ascii=False).encode('utf-8')
            self._send(status, "application/json; charset=utf-8", body)
        
        def do_OPTIONS(self):
            """Handle CORS preflight"""
            self.send_response(200)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, DELETE, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.end_headers()
        
        def do_GET(self):
            u = urlparse(self.path)
            path = u.path
            
            # Static files
            if path == "/" or path == "":
                p = web_dir / "index.html"
                body = p.read_bytes()
                return self._send(200, "text/html; charset=utf-8", body)
            
            if path.startswith("/static/"):
                rel = path[len("/static/"):]
                p = (web_dir / "static" / rel).resolve()
                if not str(p).startswith(str((web_dir / "static").resolve())):
                    return self._send(403, "text/plain; charset=utf-8", b"forbidden")
                if not p.exists() or not p.is_file():
                    return self._send(404, "text/plain; charset=utf-8", b"not found")
                
                ct = "application/octet-stream"
                if p.suffix == ".css":
                    ct = "text/css; charset=utf-8"
                elif p.suffix == ".js":
                    ct = "application/javascript; charset=utf-8"
                return self._send(200, ct, p.read_bytes())
            
            # API endpoints
            if path == "/api/state":
                return self._send_json(app.get_state_dict())
            
            if path.startswith("/api/cam/") and path.endswith(".jpg"):
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
                
                jpg = app.get_cam_jpeg(cam_i, resize_to=resize_to)
                if jpg is None:
                    return self._send(204, "text/plain; charset=utf-8", b"")
                return self._send(200, "image/jpeg", jpg)
            
            if path == "/api/sessions":
                return self._send_json(app.list_sessions())
            
            if path == "/api/recording/status":
                return self._send_json(app.get_recording_status())
            
            if path.startswith("/api/sessions/") and path.endswith("/data"):
                session_id = path.split("/")[-2]
                return self._send_json(app.get_session_data(session_id))
            
            if path.startswith("/api/sessions/") and path.endswith("/export"):
                session_id = path.split("/")[-2]
                data = app.export_session(session_id)
                return self._send(200, "application/json", data)
            
            if path == "/api/config":
                return self._send_json(app.get_config())
            
            if path == "/api/system-info":
                return self._send_json(app.get_system_info())
            
            return self._send(404, "text/plain; charset=utf-8", b"not found")
        
        def do_POST(self):
            u = urlparse(self.path)
            path = u.path
            
            # Read JSON body
            content_length = int(self.headers.get("Content-Length", 0))
            body_data = {}
            if content_length > 0:
                try:
                    body = self.rfile.read(content_length)
                    body_data = json.loads(body.decode('utf-8'))
                except Exception:
                    pass
            
            if path == "/api/recording/start":
                session_id = body_data.get("sessionId")
                freq = body_data.get("freq", 5)
                desc = body_data.get("description", "")
                result = app.start_recording(session_id, freq, desc)
                return self._send_json(result)
            
            if path == "/api/recording/stop":
                result = app.stop_recording()
                return self._send_json(result)
            
            if path == "/api/config":
                result = app.save_config(body_data)
                return self._send_json(result)
            
            return self._send(404, "text/plain; charset=utf-8", b"not found")
        
        def do_DELETE(self):
            u = urlparse(self.path)
            path = u.path
            
            if path.startswith("/api/sessions/"):
                session_id = path.split("/")[-1]
                result = app.delete_session(session_id)
                return self._send_json(result)
            
            return self._send(404, "text/plain; charset=utf-8", b"not found")
    
    return Handler


def main():
    ap = argparse.ArgumentParser(description="Xelerator Control Panel Server")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--jpeg-quality", type=int, default=75)
    ap.add_argument(
        "--web-dir",
        default=str(_repo_root() / "web" / "control_panel"),
        help="Directory containing web UI files"
    )
    ap.add_argument("--no-vision", action="store_true", help="Disable vision module")
    args = ap.parse_args()
    
    import sys
    root = str(_repo_root())
    if root not in sys.path:
        sys.path.insert(0, root)
    
    # Initialize vision
    vision = None
    if not args.no_vision:
        try:
            from src.Vision import UnitedVision
            vision = UnitedVision()
            print("[ControlPanel] Vision module initialized")
        except Exception as e:
            print(f"[ControlPanel] Warning: Could not initialize vision: {e}")
    
    web_dir = Path(args.web_dir).resolve()
    if not (web_dir / "index.html").exists():
        raise FileNotFoundError(f"Web UI not found in: {web_dir}")
    
    app = ControlPanelApp(
        vision=vision,
        web_dir=web_dir,
        jpeg_quality=args.jpeg_quality
    )
    
    handler = make_handler(app)
    httpd = ThreadingHTTPServer((args.host, int(args.port)), handler)
    
    print(f"\n{'='*60}")
    print(f"  Xelerator Control Panel")
    print(f"  http://{args.host}:{args.port}")
    print(f"{'='*60}\n")
    
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[ControlPanel] Shutting down...")
    finally:
        try:
            httpd.server_close()
        except Exception:
            pass
        if vision:
            try:
                vision.close(join_timeout=1.0)
            except Exception:
                pass


if __name__ == "__main__":
    main()