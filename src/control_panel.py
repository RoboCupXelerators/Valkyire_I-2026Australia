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
    """Map a bbox from 640x640 detector space into the displayed image space."""
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

        def to_src(xp: float, yp: float) -> tuple:
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
        result = {
            "ts": time.time(),
            "uptime_s": round(time.time() - self.start_ts, 3),
            "ball_pos_cm": [0, 0],
            "chassis_list_cm": [],
            "robot_x_cm": 0,
            "robot_y_cm": 0,
            "robot_yaw_deg": 0,
            "has_ball": False,
            "ball_bboxes_640": [None, None, None, None],
            "chassis_bboxes_640": [None, None, None, None],
            "goal_bboxes_640": [None, None, None, None],
        }
        
        # Try to get robot state from global state module
        try:
            from robot_state import get_robot_state
            robot_state = get_robot_state()
            result["robot_x_cm"] = robot_state.get("x_cm", 0)
            result["robot_y_cm"] = robot_state.get("y_cm", 0)
            result["robot_yaw_deg"] = robot_state.get("yaw_deg", 0)
            result["has_ball"] = robot_state.get("has_ball", False)
        except Exception:
            pass
        
        # Try to get data from vision module
        if self.vision:
            try:
                ball_pos = self.vision.GetBallPos()
                if ball_pos:
                    result["ball_pos_cm"] = ball_pos
            except Exception as e:
                pass
            
            try:
                chassis_pos = self.vision.GetChassisPos()
                if chassis_pos:
                    result["chassis_list_cm"] = chassis_pos
            except Exception as e:
                pass
            
            # Get bboxes for drawing
            try:
                ball_bboxes = self.vision.GetBallBBoxes()
                if ball_bboxes:
                    result["ball_bboxes_640"] = ball_bboxes
            except Exception:
                pass
            
            try:
                chassis_bboxes = self.vision.GetChassisBBoxes()
                if chassis_bboxes:
                    result["chassis_bboxes_640"] = chassis_bboxes
            except Exception:
                pass
            
            try:
                goal_bboxes = self.vision.GetGoalBBoxes()
                if goal_bboxes:
                    result["goal_bboxes_640"] = goal_bboxes
            except Exception:
                pass
        
        # Try to get data from simulator bridge if available
        if hasattr(self, 'sim_bridge') and self.sim_bridge:
            try:
                state = self.sim_bridge.get_robot_state()
                if state:
                    result["robot_x_cm"] = state.get('x_cm', 0)
                    result["robot_y_cm"] = state.get('y_cm', 0)
                    result["ball_pos_cm"] = [state.get('ball_x_cm', 0), state.get('ball_y_cm', 0)]
            except Exception as e:
                pass
        
        return result
    
    def get_cam_jpeg(self, cam_index: int, resize_to: int = None) -> tuple[bytes | None, float | None]:
        """获取摄像头 JPEG 图像和处理时间
        
        Returns:
            tuple: (jpeg_bytes, processing_time_ms) 或 (None, None)
        """
        start_ts = time.time()
        if not self.vision:
            return None, None
        try:
            frames = getattr(self.vision, "Frames", None)
            if frames is None or cam_index < 0 or cam_index >= len(frames):
                return None, None
            fr = frames[cam_index]
            if fr is None:
                return None, None
            img = fr  # 不修改图像，画框由前端完成
            
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
                return None, None
            processing_time_ms = (time.time() - start_ts) * 1000.0
            return bytes(buf), processing_time_ms
        except Exception:
            return None, None
    
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
        import json
        try:
            # Direct path to config file
            config_file = _repo_root() / "src" / "ReasonData" / "data" / "config.json"
            print(f"[ControlPanel] Config file path: {config_file}")
            print(f"[ControlPanel] Config file exists: {config_file.exists()}")
            
            if config_file.exists():
                with open(config_file, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                print(f"[ControlPanel] Config loaded: RoboInfo={config.get('RoboInfo')}")
                return config
            else:
                print(f"[ControlPanel] Config file not found")
                return {}
        except Exception as e:
            print(f"[ControlPanel] Error getting config: {e}")
            import traceback
            traceback.print_exc()
            return {}
    
    def save_config(self, config: dict) -> dict:
        """保存配置到 JSON 文件"""
        import json
        try:
            # 直接写入配置文件，不依赖 peewee
            config_file = _repo_root() / "src" / "ReasonData" / "data" / "config.json"
            config_file.parent.mkdir(parents=True, exist_ok=True)
            with open(config_file, 'w', encoding='utf-8') as f:
                json.dump(config, f, ensure_ascii=False, indent=4)
            return {"success": True}
        except Exception as e:
            print(f"[ControlPanel] Error saving config: {e}")
            import traceback
            traceback.print_exc()
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
    
    def list_files(self, directory: str, extensions: list = None) -> list:
        """列出目录下的文件"""
        import os
        from pathlib import Path
        
        try:
            # Resolve directory path
            repo_root = _repo_root()
            
            # Handle different directory types
            if directory == "models":
                # Check YOLO_Models/models directory
                models_dir = repo_root.parent / "YOLO_Models" / "models"
                if not models_dir.exists():
                    models_dir = repo_root / "models"
                if not models_dir.exists():
                    models_dir = Path("/models")
                search_dir = models_dir
            elif directory == "hailo_models":
                # Use the same directory as models for Hailo models (.hef files)
                possible_dirs = [
                    Path("/usr/share/hailo-models"),
                    repo_root.parent / "YOLO_Models" / "models",  # Same as models
                    repo_root / "models",
                    repo_root / "hailo_models",
                ]
                search_dir = None
                for d in possible_dirs:
                    if d.exists():
                        search_dir = d
                        break
                if search_dir is None:
                    # Fall back to models directory
                    search_dir = repo_root.parent / "YOLO_Models" / "models"
                    if not search_dir.exists():
                        search_dir = repo_root / "models"
            elif directory == "calibration":
                search_dir = repo_root.parent / "YOLO_Models" / "calibration"
                if not search_dir.exists():
                    search_dir = repo_root / "Calibration"
            elif directory == "records":
                search_dir = repo_root / "Records"
            elif directory.startswith("/"):
                search_dir = Path(directory)
            else:
                search_dir = repo_root / directory
            
            if not search_dir.exists():
                return []
            
            files = []
            for f in search_dir.iterdir():
                if f.is_file():
                    if extensions is None or f.suffix.lower() in extensions:
                        files.append(f.name)
            
            return sorted(files)
        except Exception as e:
            print(f"[ControlPanel] Error listing files: {e}")
            return []
    
    def get_dynamic_options(self) -> dict:
        """获取动态选项（用于下拉框）"""
        return {
            "hailo_models": self.list_files("hailo_models", [".hef"]),
            "tuna_models": self.list_files("models", [".pt", ".onnx"]),
            "tuna_engines": self.list_files("models", [".engine", ".trt"]),
            "calibration_files": self.list_files("calibration", [".npz", ".yaml", ".yml"]),
        }
    
    def run_ota_update(self) -> dict:
        """运行 OTA 更新"""
        import subprocess
        import threading
        
        def run_ota():
            try:
                repo_root = _repo_root()
                ota_script = repo_root / "src" / "utils" / "ota_updater.py"
                
                if not ota_script.exists():
                    self.ota_log = "OTA 脚本不存在: " + str(ota_script)
                    return
                
                self.ota_log = "开始 OTA 更新...\n"
                
                # Run OTA script
                result = subprocess.run(
                    ["python", str(ota_script)],
                    cwd=str(repo_root),
                    capture_output=True,
                    text=True,
                    timeout=300  # 5 minutes timeout
                )
                
                self.ota_log += result.stdout
                if result.stderr:
                    self.ota_log += "\n错误:\n" + result.stderr
                
                if result.returncode == 0:
                    self.ota_log += "\n\nOTA 更新完成！"
                else:
                    self.ota_log += f"\n\nOTA 更新失败 (退出码: {result.returncode})"
                    
            except subprocess.TimeoutExpired:
                self.ota_log += "\n\nOTA 更新超时（超过5分钟）"
            except Exception as e:
                self.ota_log = f"OTA 更新出错: {e}"
        
        # Initialize log
        self.ota_log = "正在启动 OTA 更新..."
        
        # Run in background thread
        thread = threading.Thread(target=run_ota, daemon=True)
        thread.start()
        
        return {"success": True, "message": "OTA 更新已启动"}
    
    def get_ota_log(self) -> str:
        """获取 OTA 日志"""
        return getattr(self, 'ota_log', '暂无日志')


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
                
                jpg, processing_time_ms = app.get_cam_jpeg(cam_i, resize_to=resize_to)
                if jpg is None:
                    return self._send(204, "text/plain; charset=utf-8", b"")
                # Send processing time in response header for latency display
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Content-Length", str(len(jpg)))
                if processing_time_ms is not None:
                    self.send_header("X-Processing-Time-Ms", f"{processing_time_ms:.1f}")
                self.end_headers()
                self.wfile.write(jpg)
                return
            
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
            
            if path == "/api/dynamic-options":
                return self._send_json(app.get_dynamic_options())
            
            if path == "/api/ota/log":
                return self._send_json({"log": app.get_ota_log()})
            
            if path.startswith("/api/files/"):
                directory = path[len("/api/files/"):]
                extensions = None
                q = u.query or ""
                if "ext=" in q:
                    try:
                        ext_str = q.split("ext=")[-1].split("&")[0]
                        extensions = ext_str.split(",")
                    except Exception:
                        pass
                return self._send_json(app.list_files(directory, extensions))
            
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
            
            if path == "/api/ota/start":
                result = app.run_ota_update()
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
    ap.add_argument("--no-chassis", action="store_true", help="Disable chassis module (no yaw data)")
    args = ap.parse_args()
    
    import sys
    root = str(_repo_root())
    if root not in sys.path:
        sys.path.insert(0, root)
    
    # Initialize chassis module for yaw data
    if not args.no_chassis:
        try:
            # Import LunaPre to initialize chassis and Bits
            # This will start the state update thread in chassis.py
            from src.LunaPre import chassis, compass, Bits
            print("[ControlPanel] Chassis module initialized, yaw data available")
        except Exception as e:
            print(f"[ControlPanel] Warning: Could not initialize chassis: {e}")
    
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
        # Clean up chassis/lidar resources
        try:
            from src.LunaPre import lidar, UnitedPosition
            if lidar:
                try:
                    lidar.stop()
                    print("[ControlPanel] Lidar stopped")
                except Exception:
                    pass
            if UnitedPosition:
                try:
                    UnitedPosition.Stop()
                    print("[ControlPanel] UnitedPosition stopped")
                except Exception:
                    pass
        except Exception:
            pass


if __name__ == "__main__":
    main()