"""Xelerator Control Panel Server

统一控制面板后端，整合视觉监控、比赛记录/回放、摄像头标定和鸟瞰图预览功能。

功能:
- 视觉监控
- 比赛记录管理
- 回放控制
- 系统设置
- 摄像头标定
- 鸟瞰图预览

启动方法:
    python control_panel.py --port 8080

"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import cv2
import numpy as np

BALL_NOT_FOUND = [0xFFF, 0xFFF]  # matches LunaPre.strategy_tools.BALL_NOT_FOUND; avoid importing LunaPre at server import time
POSE_NOT_FOUND = [BALL_NOT_FOUND[0], BALL_NOT_FOUND[0], BALL_NOT_FOUND[0]]
LIDAR_DISABLED_DISTS = [BALL_NOT_FOUND[0], BALL_NOT_FOUND[0], BALL_NOT_FOUND[0], BALL_NOT_FOUND[0]]
DEFAULT_CALIB_ORIGIN_X_CM = 7.5
DEFAULT_CALIB_ORIGIN_Y_CM = 16.5
DM_CHASSIS_MAX_LINEAR_MPS = 2.29
DM_CONTROL_LEASE_TIMEOUT_S = 1.0


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _calibration_dir() -> Path:
    return _repo_root() / "src" / "ReasonData" / "data" / "Calibration"


def _calibration_file_info(path: Path) -> dict:
    file_info = {
        "name": path.name,
        "path": str(path),
        "size": f"{path.stat().st_size / 1024:.1f} KB",
        "camera_index": None,
        "board_size": None,
        "square_size_mm": None,
        "origin_offset_cm": None,
        "rmse": None,
    }

    try:
        data = np.load(path, allow_pickle=True)
        if "camera_index" in data.files:
            file_info["camera_index"] = int(data["camera_index"][0])
        if "board_size" in data.files:
            file_info["board_size"] = tuple(data["board_size"].tolist())
        if "square_size_mm" in data.files:
            file_info["square_size_mm"] = float(data["square_size_mm"][0])
        if "origin_offset_cm" in data.files:
            file_info["origin_offset_cm"] = np.asarray(data["origin_offset_cm"], dtype=np.float64).tolist()
        if "rmse" in data.files:
            file_info["rmse"] = float(data["rmse"][0])
    except Exception:
        pass

    return file_info


def _npz_text(data, key: str, default: str = "") -> str:
    if key not in data.files:
        return default
    try:
        return str(np.atleast_1d(data[key])[0])
    except Exception:
        return default


def _npz_bool(data, key: str, default: bool = False) -> bool:
    if key not in data.files:
        return default
    try:
        value = np.atleast_1d(data[key])[0]
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)
    except Exception:
        return default


def _modern_pose_from_npz(data) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None]:
    H = np.asarray(data["H"], dtype=np.float64)
    rvec = np.asarray(data["rvec"], dtype=np.float64) if "rvec" in data.files else None
    tvec = np.asarray(data["tvec"], dtype=np.float64) if "tvec" in data.files else None
    if rvec is not None and rvec.size == 0:
        rvec = None
    if tvec is not None and tvec.size == 0:
        tvec = None

    # Files saved before robot_center_v2 used Y = board_y - origin_y.  The
    # intended camera-local convention is Y = board_y + origin_y so objects in
    # front of the front camera become positive robot X after _to_robot_frame().
    if (
        _npz_text(data, "origin") == "robot_center"
        and _npz_text(data, "coordinate_convention") != "robot_center_v2"
        and "origin_offset_cm" in data.files
    ):
        offset = np.asarray(data["origin_offset_cm"], dtype=np.float64).reshape(-1)
        if offset.size >= 2:
            dy_mm = 2.0 * float(offset[1]) * 10.0
            new_to_old = np.array(
                [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, -dy_mm],
                    [0.0, 0.0, 1.0],
                ],
                dtype=np.float64,
            )
            H = H @ new_to_old
            if rvec is not None and tvec is not None:
                R, _ = cv2.Rodrigues(rvec)
                d = np.array([[0.0], [-dy_mm], [0.0]], dtype=np.float64)
                tvec = np.asarray(tvec, dtype=np.float64).reshape(3, 1) + R @ d

    return H, rvec, tvec


def _calibration_plane_diagnostic(
    H: np.ndarray,
    K: np.ndarray | None,
    dist: np.ndarray | None,
    image_size,
    undistort_points: bool = True,
) -> dict:
    """Estimate the visible calibrated plane range from the current homography."""
    try:
        size = np.asarray(image_size, dtype=np.float64).reshape(-1)
        if size.size < 2:
            raise ValueError("image_size is missing width/height")
        w = float(size[0])
        h = float(size[1])
        if w <= 1.0 or h <= 1.0:
            raise ValueError(f"invalid image_size={tuple(size[:2])}")

        pts = np.array(
            [
                [[0.0, 0.0]],
                [[w - 1.0, 0.0]],
                [[w - 1.0, h - 1.0]],
                [[0.0, h - 1.0]],
                [[w * 0.5, h * 0.5]],
                [[w * 0.5, h - 1.0]],
            ],
            dtype=np.float64,
        )
        if undistort_points and K is not None and dist is not None and np.asarray(dist).size:
            camera_matrix = np.asarray(K, dtype=np.float64)
            pts = cv2.undistortPoints(
                pts,
                camera_matrix,
                np.asarray(dist, dtype=np.float64),
                P=camera_matrix,
            )

        world_mm = cv2.perspectiveTransform(pts, np.linalg.inv(np.asarray(H, dtype=np.float64))).reshape(-1, 2)
        if not np.all(np.isfinite(world_mm)):
            raise ValueError("non-finite projected plane coordinates")

        corners_cm = world_mm[:4] / 10.0
        all_cm = world_mm / 10.0
        x_min = float(np.min(corners_cm[:, 0]))
        x_max = float(np.max(corners_cm[:, 0]))
        y_min = float(np.min(corners_cm[:, 1]))
        y_max = float(np.max(corners_cm[:, 1]))
        x_span = x_max - x_min
        y_span = y_max - y_min

        warning = ""
        if y_max < 60.0 or y_span < 30.0:
            warning = (
                f"警告: 当前整幅图在标定平面上只覆盖 Y={y_min:.1f}..{y_max:.1f}cm，"
                "远距离会被明显压缩；最后一张样本请把棋盘格平放在实际测量/地面参考平面上。"
            )
        elif y_max < 100.0:
            warning = (
                f"提示: 当前前向最大约 {y_max:.1f}cm；如果要测 100cm，"
                "请确认最后一张样本所在平面与被测目标落点平面一致。"
            )

        return {
            "extent_cm": {
                "x_min": x_min,
                "x_max": x_max,
                "y_min": y_min,
                "y_max": y_max,
                "x_span": float(x_span),
                "y_span": float(y_span),
            },
            "points_cm": {
                "top_left": all_cm[0].tolist(),
                "top_right": all_cm[1].tolist(),
                "bottom_right": all_cm[2].tolist(),
                "bottom_left": all_cm[3].tolist(),
                "center": all_cm[4].tolist(),
                "bottom_center": all_cm[5].tolist(),
            },
            "warning": warning,
        }
    except Exception as e:
        return {"error": str(e)}


_SRC_DIR = _repo_root() / "src"
if str(_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(_SRC_DIR))


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


class CalibrationSession:
    """摄像头标定会话状态"""

    def __init__(self):
        self.lock = threading.Lock()
        self.running = False
        self.cap = None
        self.cam_index = 0
        self.width = 640
        self.height = 480
        self.board_w = 9
        self.board_h = 9
        self.square_mm = 15.0
        self.origin_x_cm = DEFAULT_CALIB_ORIGIN_X_CM
        self.origin_y_cm = DEFAULT_CALIB_ORIGIN_Y_CM

        self.latest_frame = None
        self.latest_corners = None
        self.latest_found = False
        self.latest_size = (640, 480)

        self.samples = []
        self.result = None
        self.status = "Idle"
        self.error = None


class BirdEyeSession:
    """鸟瞰图预览会话状态"""

    def __init__(self):
        self.lock = threading.Lock()
        self.running = False
        self.cap = None
        self.cam_index = 0
        self.width = 640
        self.height = 480
        self.calib_file = None
        self.pixels_per_mm = 4.0
        self.grid_mm = 10.0
        self.preview_scale = 0.4
        self.use_mjpg = False

        self.calib_data = None
        self.status = "Idle"
        self.error = None


class CameraResourceManager:
    """摄像头资源管理器 - 协调 Vision 模块和工具页的摄像头访问"""

    def __init__(self):
        self.lock = threading.Lock()
        # 记录哪些摄像头正在被 Vision 模块使用
        self.vision_active_cams = set()
        # 记录哪些摄像头正在被工具页使用
        self.tools_active_cams = set()
        # Vision 模块是否已启动
        self.vision_running = False

    def register_vision_cams(self, cam_indices: list):
        """注册 Vision 模块使用的摄像头"""
        with self.lock:
            self.vision_active_cams = set(cam_indices)
            self.vision_running = True
            print(f"[CameraManager] Vision registered cameras: {cam_indices}")

    def unregister_vision_cams(self):
        """注销 Vision 模块的摄像头"""
        with self.lock:
            self.vision_active_cams.clear()
            self.vision_running = False
            print(f"[CameraManager] Vision unregistered all cameras")

    def can_tool_use_cam(self, cam_index: int) -> bool:
        """检查工具页是否可以使用指定摄像头"""
        with self.lock:
            # 如果 Vision 正在运行且使用该摄像头，工具页应该从 Vision 获取画面
            if self.vision_running and cam_index in self.vision_active_cams:
                return False  # 不能直接访问，应该从 Vision 获取
            return True  # 可以直接访问

    def register_tool_cam(self, cam_index: int) -> bool:
        """注册工具页使用的摄像头"""
        with self.lock:
            if cam_index in self.vision_active_cams and self.vision_running:
                print(f"[CameraManager] Tool cannot use cam {cam_index} - Vision is using it")
                return False
            self.tools_active_cams.add(cam_index)
            print(f"[CameraManager] Tool registered camera: {cam_index}")
            return True

    def unregister_tool_cam(self, cam_index: int):
        """注销工具页的摄像头"""
        with self.lock:
            self.tools_active_cams.discard(cam_index)
            print(f"[CameraManager] Tool unregistered camera: {cam_index}")

    def get_status(self) -> dict:
        """获取资源状态"""
        with self.lock:
            return {
                "vision_running": self.vision_running,
                "vision_cams": list(self.vision_active_cams),
                "tools_cams": list(self.tools_active_cams),
            }


class ControlPanelApp:
    """控制面板应用状态"""

    def __init__(
        self,
        vision=None,
        web_dir: Path = None,
        jpeg_quality: int = 75,
        bits=None,
        allow_runtime_init: bool = True,
    ):
        self.vision = vision
        self.web_dir = web_dir or _repo_root() / "web" / "control_panel"
        self.jpeg_quality = int(jpeg_quality)
        self.start_ts = time.time()

        # Bits 对象（用于模拟器环境下获取位姿和球位置）
        self.bits = bits
        self._allow_runtime_init = bool(allow_runtime_init)
        self._last_lidar_status = None

        # 检测是否为模拟器环境
        self._is_simulator = self._detect_simulator_mode()

        # 键盘控制状态
        self._control_lock = threading.Lock()
        self._control_enabled = False
        self._control_speed = 0.5  # 默认移动速度，m/s
        self._control_turn_speed = 120.0  # 默认转向速度，deg/s
        self._dribble_active = False  # 盘球状态
        self._control_runtime_error = None
        self._control_profile = "normal"
        self._control_target = (0.0, 0.0, 0.0)
        self._control_last_lease = 0.0
        self._control_disable_reason = "not_enabled"
        self._control_worker_stop = threading.Event()
        self._control_worker_thread = None

        # 摄像头资源管理器
        self.camera_manager = CameraResourceManager()

        # Recording state
        self._is_recording = False
        self._recording_session_id = None
        self._recording_frame_count = 0
        self._recording_start_ts = None

        # Calibration state
        self.calib_session = CalibrationSession()
        self.calib_dir = _calibration_dir()
        self.calib_dir.mkdir(parents=True, exist_ok=True)

        # BirdEye state
        self.birdeye_session = BirdEyeSession()

        # Vision startup diagnostics. The server can start before cameras are
        # ready, then attach vision in the background.
        self._vision_status_lock = threading.Lock()
        self._vision_status = {
            "state": "ready" if self.vision is not None else "not_started",
            "backend": None,
            "error": None,
        }
        self._vision_attach_lock = threading.Lock()
        self._vision_attaching = False

        # 注册 Vision 模块的摄像头
        if self.vision is not None:
            self._register_vision_cameras()

    def _detect_simulator_mode(self) -> bool:
        """检测是否运行在模拟器环境下"""
        # 检查 Bits 类型
        if self.bits is not None:
            bits_type = type(self.bits).__name__
            if bits_type == "XeleratorSimBits":
                return True

        # 检查 vision 类型
        if self.vision is not None:
            vision_type = type(self.vision).__name__
            if vision_type == "XeleratorVision":
                return True

        # 检查环境变量
        import os
        if os.environ.get("ROBOCUPX_BITS", "").upper() == "SIM":
            return True

        # 检查配置
        try:
            from ReasonData import Settings
            if str(getattr(Settings.RoboInfo, "Bit", "")).upper() == "SIM":
                return True
        except Exception:
            pass

        return False

    def _register_vision_cameras(self):
        """从 Vision 模块获取并注册摄像头索引"""
        try:
            cam_ports = getattr(self.vision, 'CamPorts', None)
            if cam_ports:
                self.camera_manager.register_vision_cams(cam_ports)
            else:
                # 从配置获取默认摄像头索引
                config = self.get_config()
                vision_vals = config.get("VisionVals", {})
                default_cams = [
                    vision_vals.get("Back", 0),
                    vision_vals.get("Left", 2),
                    vision_vals.get("Front", 4),
                    vision_vals.get("Right", 6),
                ]
                self.camera_manager.register_vision_cams(default_cams)
        except Exception as e:
            print(f"[ControlPanel] Error registering vision cameras: {e}")

    # ==================== 视觉状态 ====================

    def set_vision_status(self, state: str, backend: str = None, error: str = None):
        with self._vision_status_lock:
            self._vision_status = {
                "state": state,
                "backend": backend,
                "error": error,
            }

    def get_vision_status(self) -> dict:
        with self._vision_status_lock:
            status = dict(self._vision_status)
        status["available"] = self.vision is not None
        status["type"] = type(self.vision).__name__ if self.vision is not None else None
        status["detect_backend"] = None
        if self.vision is not None:
            try:
                mro = type(self.vision).__mro__
                if len(mro) > 1:
                    status["detect_backend"] = mro[1].__name__
            except Exception:
                pass
        return status

    def get_lidar_status(self) -> dict:
        """Return lidar diagnostics without initializing the LunaPre runtime."""
        status = {
            "available": False,
            "runtime_initialized": False,
            "state": "not_initialized",
            "backend": None,
            "driver": None,
            "running": False,
            "disabled": None,
            "dist_mm": list(LIDAR_DISABLED_DISTS),
            "point_count": 0,
            "raw_point_count": 0,
            "scan_point_count": 0,
            "packet_point_count": 0,
            "line_count": 0,
            "message": "LunaPre runtime not initialized",
        }

        runtime = sys.modules.get("LunaPre.runtime")
        if runtime is None:
            return status

        status["runtime_initialized"] = bool(getattr(runtime, "_runtime_initialized", False))
        lidar = getattr(runtime, "lidar", None)
        if lidar is None:
            if status["runtime_initialized"]:
                status["state"] = "missing"
                status["message"] = "Runtime initialized but lidar object is missing"
            return status

        status["available"] = True
        status["backend"] = getattr(lidar, "backend", None)
        status["driver"] = type(getattr(lidar, "driver", None)).__name__ if getattr(lidar, "driver", None) else None
        status["running"] = bool(getattr(lidar, "running", False))
        status["disabled"] = bool(getattr(lidar, "disabled", False))
        status["point_count"] = int(getattr(lidar, "last_scan_point_count", 0) or 0)
        status["raw_point_count"] = int(getattr(lidar, "last_raw_scan_count", 0) or 0)
        driver = getattr(lidar, "driver", None)
        status["scan_point_count"] = int(
            getattr(driver, "last_scan_point_count", 0)
            or status["raw_point_count"]
            or 0
        )
        status["packet_point_count"] = int(
            getattr(driver, "last_packet_point_count", 0) or 0
        )
        status["line_count"] = int(getattr(lidar, "last_line_count", 0) or 0)

        try:
            positions = self._get_positions_facade()
            if positions is not None:
                snapshot = getattr(positions, "DirectDistanceSnapshot", None)
                if callable(snapshot):
                    dists = list(snapshot())
                else:
                    dists = list(lidar.GetDists())
            else:
                dists = list(lidar.GetDists())
            status["dist_mm"] = dists
        except Exception as e:
            status["state"] = "error"
            status["message"] = str(e)
            return status

        if status["disabled"] or status["dist_mm"] == LIDAR_DISABLED_DISTS:
            status["state"] = "disabled"
            status["message"] = "Lidar driver is disabled"
        elif status["dist_mm"] == [0, 0, 0, 0]:
            status["state"] = "no_data"
            status["message"] = "Lidar object exists but no scan has been normalized yet"
        else:
            status["state"] = "ok"
            status["message"] = "Receiving lidar distance data"

        self._last_lidar_status = status
        return status

    def _get_runtime_module(self):
        runtime = sys.modules.get("LunaPre.runtime")
        if runtime is not None and getattr(runtime, "_runtime_initialized", False):
            return runtime
        return None

    def _get_positions_facade(self):
        """Return an already-loaded LunaPre.UnitedPosition without starting vision/runtime."""
        runtime = self._get_runtime_module()
        if runtime is None:
            return None
        try:
            import LunaPre
            luna = LunaPre
        except Exception:
            luna = sys.modules.get("LunaPre")
        if luna is None:
            return None
        return getattr(luna, "UnitedPosition", None)

    def _fill_state_from_positions(self, result: dict) -> None:
        """Fill lidar-derived pose only; detection data must come from app.vision."""
        positions = self._get_positions_facade()
        if positions is None:
            return

        try:
            pose = positions.FieldRobotPose()
            if pose and len(pose) >= 3 and list(pose) != POSE_NOT_FOUND:
                result["robot_x_cm"] = round(float(pose[0]), 2)
                result["robot_y_cm"] = round(float(pose[1]), 2)
                result["robot_yaw_deg"] = round(float(pose[2]), 2)
        except Exception:
            pass

    def _call_provider(self, provider, *names):
        if provider is None:
            return None
        for name in names:
            method = getattr(provider, name, None)
            if callable(method):
                try:
                    return method()
                except Exception:
                    return None
        return None

    def _fill_pose_state(self, result: dict) -> None:
        """Fill robot pose through the Position facade; never start another runtime."""
        positions = self._get_positions_facade()
        if positions is not None:
            try:
                snapshot = getattr(positions, "DirectDistanceSnapshot", None)
                dists = list(snapshot()) if callable(snapshot) else list(LIDAR_DISABLED_DISTS)
                if dists != LIDAR_DISABLED_DISTS and dists != [0, 0, 0, 0]:
                    try:
                        pose = positions.FieldRobotPose(distance=dists)
                    except TypeError:
                        pose = positions.FieldRobotPose()
                    if pose and len(pose) >= 3 and list(pose) != POSE_NOT_FOUND:
                        result["robot_x_cm"] = round(float(pose[0]), 2)
                        result["robot_y_cm"] = round(float(pose[1]), 2)
                        result["robot_yaw_deg"] = round(float(pose[2]), 2)
                        result["robot_pose_valid"] = True
                        return
            except Exception:
                pass

        pose = self._call_provider(self.bits, "GetPose")
        if pose and len(pose) >= 3:
            try:
                result["robot_x_cm"] = round(float(pose[0]), 2)
                result["robot_y_cm"] = round(float(pose[1]), 2)
                result["robot_yaw_deg"] = round(float(pose[2]), 2)
                result["robot_pose_valid"] = True
                return
            except Exception:
                pass

        chassis, _ = self._get_chassis_and_peripheral(initialize=False)
        pose = self._call_provider(chassis, "GetPose")
        if pose and len(pose) >= 3:
            try:
                result["robot_x_cm"] = round(float(pose[0]), 2)
                result["robot_y_cm"] = round(float(pose[1]), 2)
                result["robot_yaw_deg"] = round(float(pose[2]), 2)
                result["robot_pose_valid"] = True
                return
            except Exception:
                pass

        candidate = self._call_provider(self.bits, "GetHeadingCandidate")
        if isinstance(candidate, dict):
            try:
                heading = float(candidate["heading_deg"])
                if math.isfinite(heading):
                    result["robot_heading_candidate_deg"] = round(heading, 2)
                    result["robot_heading_available"] = True
                    result["robot_heading_source"] = candidate.get("source")
                    result["robot_heading_selected_axis"] = candidate.get("selected_axis")
                    result["robot_heading_axis_mapping_verified"] = bool(
                        candidate.get("axis_mapping_verified", False))
                    result["robot_direction_defined"] = bool(
                        candidate.get("robot_direction_defined", False))
                    result["robot_heading_valid_for_motion"] = bool(
                        candidate.get("valid_for_motion", False))
                    return
            except (KeyError, TypeError, ValueError):
                pass

        yaw = self._call_provider(self.bits, "GetYaw")
        if yaw is None:
            yaw = self._call_provider(chassis, "GetYaw")
        if yaw is not None:
            try:
                result["robot_yaw_deg"] = round(float(yaw), 2)
                result["robot_pose_valid"] = True
                result["robot_heading_available"] = True
                result["robot_heading_source"] = "MOTION_SAFE_GET_YAW"
                result["robot_heading_axis_mapping_verified"] = True
                result["robot_heading_valid_for_motion"] = True
            except Exception:
                pass

    def _normalize_ball_observation(self, ball_pos):
        if ball_pos is None:
            return list(BALL_NOT_FOUND)
        tolist = getattr(ball_pos, "tolist", None)
        if callable(tolist):
            try:
                ball_pos = tolist()
            except Exception:
                return list(BALL_NOT_FOUND)
        if not isinstance(ball_pos, (list, tuple)) or len(ball_pos) < 2:
            return list(BALL_NOT_FOUND)
        try:
            x = float(ball_pos[0])
            y = float(ball_pos[1])
        except Exception:
            return list(BALL_NOT_FOUND)
        if not (math.isfinite(x) and math.isfinite(y)):
            return list(BALL_NOT_FOUND)
        if [int(x), int(y)] == BALL_NOT_FOUND:
            return list(BALL_NOT_FOUND)
        if [int(x), int(y)] == [0xDDD, 0xDDD]:
            return [0xDDD, 0xDDD]
        if x == 0 and y == 0:
            return list(BALL_NOT_FOUND)
        return [round(x, 2), round(y, 2)]

    def _fill_state_from_vision_provider(self, result: dict) -> None:
        """Fill live detection state from the single vision object owned by the panel."""
        vision = self.vision
        if vision is None:
            return

        ball_pos = self._call_provider(vision, "GetBallPos", "get_ball_pos")
        normalized_ball = self._normalize_ball_observation(ball_pos)
        result["ball_pos_cm"] = normalized_ball

        ball_pos_raw = self._call_provider(vision, "GetBallPosRaw", "get_ball_pos_raw")
        if ball_pos_raw is None:
            ball_pos_raw = getattr(vision, "BallPosRaw", None)
        result["ball_pos_raw_cm"] = self._normalize_ball_observation(ball_pos_raw)

        ball_pixel = self._call_provider(vision, "GetBallPixelPoint", "get_ball_pixel_point")
        if ball_pixel is not None:
            result["ball_pixel_point"] = _json_safe(ball_pixel)

        has_ball = self._call_provider(self.bits, "HasBall")
        result["has_ball"] = bool(has_ball) or normalized_ball == [0xDDD, 0xDDD]

        chassis_pos = self._call_provider(vision, "GetChassisPos", "get_chassis_pos")
        if chassis_pos is not None:
            result["chassis_list_cm"] = chassis_pos

        goals = self._call_provider(vision, "GetGoalsPos", "get_goals_pos")
        if isinstance(goals, dict):
            result["blue_goal_pos_cm"] = goals.get("blue_goal")
            result["yellow_goal_pos_cm"] = goals.get("yellow_goal")
        elif isinstance(goals, (list, tuple)):
            if len(goals) > 0:
                result["blue_goal_pos_cm"] = goals[0]
            if len(goals) > 1:
                result["yellow_goal_pos_cm"] = goals[1]

    def _fill_state_from_runtime_fallback(self, result: dict) -> None:
        """Fallback for panels started without a usable Positions facade."""
        try:
            if result["robot_x_cm"] == 0 and result["robot_y_cm"] == 0:
                chassis, _ = self._get_chassis_and_peripheral()
                if chassis:
                    if hasattr(chassis, 'GetPose') and callable(getattr(chassis, 'GetPose')):
                        try:
                            xp, yp, yawp = chassis.GetPose()
                            result["robot_x_cm"] = round(float(xp), 2)
                            result["robot_y_cm"] = round(float(yp), 2)
                            result["robot_yaw_deg"] = round(float(yawp), 2)
                        except Exception:
                            pass
                    else:
                        try:
                            yawp = chassis.GetYaw() if hasattr(chassis, 'GetYaw') else None
                            if yawp is not None:
                                result["robot_yaw_deg"] = round(float(yawp), 2)
                        except Exception:
                            pass
        except Exception:
            pass

        try:
            if self.bits is not None and hasattr(self.bits, 'GetYaw') and callable(getattr(self.bits, 'GetYaw')):
                yawp = self.bits.GetYaw()
                if yawp is not None:
                    result["robot_yaw_deg"] = round(float(yawp), 2)
        except Exception:
            pass

    def _fill_vision_debug_state(self, result: dict) -> None:
        """Read vision-only diagnostics that Positions intentionally does not own."""
        if self.vision is None:
            return

        for method_name, key in (
            ("GetBallBBoxes", "ball_bboxes_640"),
            ("GetChassisBBoxes", "chassis_bboxes_640"),
            ("GetGoalBBoxes", "goal_bboxes_640"),
        ):
            try:
                method = getattr(self.vision, method_name, None)
                if callable(method):
                    value = method()
                    if value:
                        result[key] = value
            except Exception:
                pass

        try:
            result["perf"]["preprocess_tf"] = int(getattr(self.vision, 'PreProcessTF', 0))
            result["perf"]["inference_tf"] = int(getattr(self.vision, 'InferTF', 0))
            result["perf"]["postprocess_tf"] = int(getattr(self.vision, 'PostProcessTF', 0))
            result["perf"]["camera_tf"] = int(getattr(self.vision, 'ReadCamTF', 0))
            for attr, key in (
                ("PreProcessMs", "preprocess_ms"),
                ("InferMs", "inference_ms"),
                ("InferBatchMs", "inference_batch_ms"),
                ("PostProcessMs", "postprocess_ms"),
            ):
                value = getattr(self.vision, attr, None)
                if value is not None:
                    value = float(value)
                    if value > 0:
                        result["perf"][key] = round(value, 3)
            result["perf"]["inference_batch_size"] = int(getattr(self.vision, "InferBatchSize", 0) or 0)
            hybrid_perf_getter = getattr(self.vision, "GetHybridPerf", None)
            if callable(hybrid_perf_getter):
                hybrid_perf = hybrid_perf_getter() or {}
                for key in (
                    "blob_ms",
                    "fusion_ms",
                    "output_hz",
                    "yolo_hz",
                    "tracker_hz",
                    "vision_confidence",
                    "yolo_backend",
                    "yolo_error",
                ):
                    if key in hybrid_perf:
                        result["perf"][key] = hybrid_perf[key]
        except Exception as e:
            print(f"[ControlPanel] Error getting performance metrics: {e}")

    def scan_lidar_usb(self) -> dict:
        """Scan common Linux serial device paths for lidar USB diagnostics."""
        configured_port = None
        configured_type = None
        try:
            config_file = _repo_root() / "src" / "ReasonData" / "data" / "config.json"
            if config_file.exists():
                with config_file.open("r", encoding="utf-8") as f:
                    config = json.load(f)
                configured_port = config.get("Ports", {}).get("LidarPort")
                configured_type = config.get("RoboInfo", {}).get("LidarType")
        except Exception:
            pass

        candidates = []
        patterns = [
            "/dev/ldlidar",
            "/dev/ttyCH343USB*",
            "/dev/sllidar",
            "/dev/rplidar",
            "/dev/ttyUSB*",
            "/dev/ttyACM*",
            "/dev/serial/by-id/*",
            "/dev/serial/by-path/*",
        ]
        seen = set()

        for pattern in patterns:
            for path_str in sorted(glob.glob(pattern)):
                if path_str in seen:
                    continue
                seen.add(path_str)
                info = {"path": path_str, "exists": os.path.exists(path_str)}
                try:
                    resolved = os.path.realpath(path_str)
                    info["resolved"] = resolved
                    info["readable"] = os.access(resolved, os.R_OK)
                    info["writable"] = os.access(resolved, os.W_OK)
                except Exception:
                    info["resolved"] = None
                    info["readable"] = False
                    info["writable"] = False
                candidates.append(info)

        likely = [
            item for item in candidates
            if any(token in item["path"].lower() for token in ("sllidar", "rplidar", "silicon", "cp210", "ttych343", "ttyusb", "ttyacm"))
            or any(token in str(item.get("resolved", "")).lower() for token in ("ttych343", "ttyusb", "ttyacm"))
        ]

        return {
            "configured_port": configured_port or "/dev/sllidar",
            "configured_type": configured_type,
            "candidates": candidates,
            "likely": likely,
            "message": "No serial lidar-like devices found" if not likely else f"Found {len(likely)} lidar-like serial path(s)",
        }

    def get_state_dict(self) -> dict:
        """获取视觉状态"""
        result = {
            "ts": time.time(),
            "uptime_s": round(time.time() - self.start_ts, 3),
            "ball_pos_cm": list(BALL_NOT_FOUND),
            "ball_pos_raw_cm": list(BALL_NOT_FOUND),
            "ball_pixel_point": None,
            "chassis_list_cm": [],
            "blue_goal_pos_cm": None,
            "yellow_goal_pos_cm": None,
            "robot_x_cm": 0,
            "robot_y_cm": 0,
            "robot_yaw_deg": 0,
            "robot_pose_valid": False,
            "robot_heading_candidate_deg": None,
            "robot_heading_available": False,
            "robot_heading_source": None,
            "robot_heading_selected_axis": None,
            "robot_heading_axis_mapping_verified": False,
            "robot_direction_defined": False,
            "robot_heading_valid_for_motion": False,
            "has_ball": False,
            "ball_bboxes_640": [None, None, None, None],
            "chassis_bboxes_640": [None, None, None, None],
            "goal_bboxes_640": [None, None, None, None],
            "is_simulator": self._is_simulator,
            "vision_status": self.get_vision_status(),
            "lidar_status": self.get_lidar_status(),
            # 性能指标 - 返回帧计数器，前端计算延时
            "perf": {
                "preprocess_tf": 0,  # 预处理帧计数
                "inference_tf": 0,   # 推理帧计数
                "postprocess_tf": 0, # 后处理帧计数
                "camera_tf": 0,      # 摄像头帧计数
                "preprocess_ms": None,
                "inference_ms": None,
                "inference_batch_ms": None,
                "inference_batch_size": 0,
                "postprocess_ms": None,
                "blob_ms": None,
                "fusion_ms": None,
                "output_hz": None,
                "yolo_hz": None,
                "tracker_hz": None,
                "vision_confidence": None,
                "yolo_backend": None,
                "yolo_error": None,
            },
        }

        self._fill_pose_state(result)
        self._fill_state_from_vision_provider(result)
        self._fill_vision_debug_state(result)

        return result

    def get_cam_jpeg(self, cam_index: int, resize_to: int = None) -> tuple:
        """获取摄像头 JPEG 图像和处理时间

        cam_index: 摄像头端口号（如 0, 2, 4, 6）或 Frames 列表索引（0-3）
        """
        start_ts = time.time()
        if not self.vision:
            return None, None
        try:
            frames = getattr(self.vision, "Frames", None)
            if frames is None:
                return None, None

            # 获取 CamPorts 映射
            cam_ports = getattr(self.vision, 'CamPorts', [0, 2, 4, 6])

            # 如果 cam_index 在 CamPorts 中，转换为 Frames 索引
            if cam_index in cam_ports:
                frame_idx = cam_ports.index(cam_index)
            elif 0 <= cam_index < len(frames):
                # 如果 cam_index 是 Frames 索引（0-3）
                frame_idx = cam_index
            else:
                return None, None

            if frame_idx < 0 or frame_idx >= len(frames):
                return None, None

            fr = frames[frame_idx]
            if fr is None:
                return None, None
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

            ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)])
            if not ok:
                return None, None
            processing_time_ms = (time.time() - start_ts) * 1000.0
            return bytes(buf), processing_time_ms
        except Exception:
            return None, None

    # ==================== 记录功能 ====================

    def get_recording_status(self) -> dict:
        return {
            "is_recording": self._is_recording,
            "session_id": self._recording_session_id,
            "frame_count": self._recording_frame_count,
            "duration": time.time() - self._recording_start_ts if self._recording_start_ts else 0,
        }

    def start_recording(self, session_id: str = None, freq: int = 5, description: str = "") -> dict:
        if self._is_recording:
            return {"success": False, "error": "Already recording"}

        try:
            from ReasonData.data import SetupDatabase, MatchSession
            SetupDatabase()

            if session_id is None:
                session_id = time.strftime("%Y%m%d_%H%M%S", time.localtime())

            try:
                MatchSession.create_session(session_id, description=description)
            except Exception:
                pass

            self._is_recording = True
            self._recording_session_id = session_id
            self._recording_frame_count = 0
            self._recording_start_ts = time.time()

            return {"success": True, "session_id": session_id}
        except Exception as e:
            return {"success": False, "error": str(e)}

    def stop_recording(self) -> dict:
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
        try:
            from ReasonData.data import ExportSessionToDict
            return ExportSessionToDict(session_id)
        except Exception as e:
            return {"error": str(e)}

    def delete_session(self, session_id: str) -> dict:
        try:
            from ReasonData.data import MatchSession, Positions, Outputs

            Positions.delete().where(Positions.SessionId == session_id).execute()
            Outputs.delete().where(Outputs.SessionId == session_id).execute()

            session = MatchSession.get_or_none(MatchSession.SessionId == session_id)
            if session:
                session.delete_instance()

            return {"success": True}
        except Exception as e:
            return {"success": False, "error": str(e)}

    def export_session(self, session_id: str) -> bytes:
        data = self.get_session_data(session_id)
        return json.dumps(data, ensure_ascii=False, indent=2).encode('utf-8')

    # ==================== 配置管理 ====================

    def get_config(self) -> dict:
        try:
            config_file = _repo_root() / "src" / "ReasonData" / "data" / "config.json"
            if config_file.exists():
                with open(config_file, 'r', encoding='utf-8') as f:
                    return json.load(f)
            return {}
        except Exception as e:
            print(f"[ControlPanel] Error getting config: {e}")
            return {}

    def get_config_schema(self) -> dict:
        """Return config schema with metadata for dynamic form generation.

        DEPRECATED: Use get_config_template() instead, which auto-generates
        from config.py DEFAULT_CONFIG.
        """
        return self.get_config_template()

    def get_config_template(self) -> dict:
        """Auto-generate config template from config.py DEFAULT_CONFIG.

        This ensures the control panel stays synchronized with the actual
        config structure without manual maintenance.
        """
        # Import DEFAULT_CONFIG from config.py
        try:
            from ReasonData.config import Preference
            default_config = Preference.DEFAULT_CONFIG
        except Exception as e:
            print(f"[ControlPanel] Error loading DEFAULT_CONFIG: {e}")
            # Fallback to reading from config.json
            default_config = self.get_config()

        # Define field metadata (type, options, labels, etc.)
        # This is the only part that needs maintenance when adding new fields
        field_metadata = {
            # RoboInfo
            ("RoboInfo", "number"): {"type": "int", "label": "机器人编号", "min": 1, "max": 99, "tab": "robot", "icon": "🤖"},
            ("RoboInfo", "type"): {"type": "select", "label": "机器人类型", "options": ["OP", "DP"], "option_labels": {"OP": "OP (进攻)", "DP": "DP (防守)"}, "tab": "robot"},
            ("RoboInfo", "Bit"): {"type": "select", "label": "Bits 类型", "options": ["SIM", "AB", "IL", "3Q"], "option_labels": {"SIM": "SIM (模拟器)", "AB": "AB (Arisu Bits)", "IL": "IL (IceLoong Bits)", "3Q": "3Q (Bits)"}, "tab": "robot"},
            ("RoboInfo", "Rec"): {"type": "select", "label": "识别类型", "options": ["AI", "TI", "XV", "BB", "SY", "HV"], "option_labels": {"AI": "AI (Arisu Intelligence)", "TI": "TI (Tuna Intelligence)", "XV": "XV (Xelerator Vision)", "BB": "BB (Blob Based)", "SY": "SY (Sim Yolo)", "HV": "HV (Hybrid Vision)"}, "tab": "robot"},
            ("RoboInfo", "LidarBackend"): {"type": "select", "label": "雷达后端", "options": ["rplidar", "ros2", "ldlidar"], "option_labels": {"rplidar": "rplidar (原生驱动)", "ros2": "ros2 (ROS2 节点)", "ldlidar": "ldlidar (LDROBOT)"}, "tab": "robot"},
            ("RoboInfo", "LidarType"): {"type": "select", "label": "雷达型号", "options": ["s1", "s2", "ld06", "ld07"], "option_labels": {"s1": "s1 (Slamtec S1)", "s2": "s2 (Slamtec S2)", "ld06": "ld06 (LDROBOT LD06)", "ld07": "ld07 (LDROBOT LD07)"}, "tab": "robot"},
            # Ports
            ("Ports", "LidarID"): {"type": "int", "label": "雷达 ID", "min": 1, "max": 255, "tab": "robot"},
            ("Ports", "LidarPort"): {"type": "string", "label": "雷达端口", "tab": "robot"},
            ("Ports", "IceLoongPort"): {"type": "string", "label": "IceLoong 端口", "tab": "robot"},
            ("Ports", "ElecMagnet"): {"type": "int", "label": "电磁铁端口", "min": 0, "max": 15, "tab": "robot"},
            ("Ports", "Dribble"): {"type": "int", "label": "运球电机端口", "min": 0, "max": 15, "tab": "robot"},
            # Wheels - group together for horizontal layout
            ("Ports", "LeftFront"): {"type": "int", "label": "左前轮", "min": 0, "max": 15, "tab": "robot", "group": "wheels", "group_label": "轮子端口"},
            ("Ports", "LeftBack"): {"type": "int", "label": "左后轮", "min": 0, "max": 15, "tab": "robot", "group": "wheels"},
            ("Ports", "RightBack"): {"type": "int", "label": "右后轮", "min": 0, "max": 15, "tab": "robot", "group": "wheels"},
            ("Ports", "RightFront"): {"type": "int", "label": "右前轮", "min": 0, "max": 15, "tab": "robot", "group": "wheels"},
            # Vision
            ("Vision", "CamPorts"): {"type": "list[int]", "label": "摄像头端口", "description": "按顺序: 后, 左, 前, 右", "item_labels": ["后", "左", "前", "右"], "tab": "vision", "icon": "📷"},
            ("Vision", "ModelName"): {"type": "string", "label": "模型名称", "description": "不含后缀，各推理库自动选择所需格式", "tab": "vision"},
            ("Vision", "CalibrationFolder"): {"type": "string", "label": "标定文件夹", "tab": "vision"},
            ("Vision", "BallDiameter"): {"type": "float", "label": "球直径 (CM)", "min": 1.0, "max": 20.0, "step": 0.1, "tab": "vision"},
            ("Vision", "ChassisWidth"): {"type": "float", "label": "底盘宽度 (CM)", "min": 5.0, "max": 30.0, "step": 0.5, "tab": "vision"},
            ("Vision", "ChassisHeight"): {"type": "float", "label": "底盘高度 (CM)", "min": 5.0, "max": 30.0, "step": 0.5, "tab": "vision"},
            ("Vision", "GoalWidth"): {"type": "float", "label": "球门宽度 (CM)", "min": 30.0, "max": 200.0, "tab": "vision"},
            ("Vision", "GoalHeight"): {"type": "float", "label": "球门高度 (CM)", "min": 5.0, "max": 30.0, "tab": "vision"},
            ("Vision", "MinBallBBoxArea"): {"type": "int", "label": "最小球检测面积", "description": "bbox 面积阈值", "min": 10, "max": 500, "tab": "vision"},
            ("Vision", "BallOffsetXCM"): {"type": "float", "label": "球 X 校正 (CM)", "description": "加到球最终 X 坐标", "min": -50.0, "max": 50.0, "step": 0.5, "tab": "vision"},
            ("Vision", "BallOffsetYCM"): {"type": "float", "label": "球 Y 校正 (CM)", "description": "加到球最终 Y 坐标", "min": -50.0, "max": 50.0, "step": 0.5, "tab": "vision"},
            ("Vision", "Record"): {"type": "bool", "label": "启用录像", "tab": "vision"},
            ("Vision", "RecordDir"): {"type": "string", "label": "录像目录", "tab": "vision"},
            # Field
            ("Field", "Short"): {"type": "int", "label": "场地短边 (CM)", "min": 100, "max": 300, "tab": "data", "icon": "🏟️"},
            ("Field", "Long"): {"type": "int", "label": "场地长边 (CM)", "min": 100, "max": 400, "tab": "data"},
            ("Field", "Home"): {"type": "list[float]", "label": "回位点", "description": "[X, Y, θ°]", "item_labels": ["X (CM)", "Y (CM)", "θ (度)"], "tab": "data"},
            # Chassis
            ("Chassis", "WheelRadiusM"): {"type": "float", "label": "轮子半径 (M)", "min": 0.01, "max": 0.1, "step": 0.001, "tab": "robot", "icon": "⚙️"},
            ("Chassis", "RobotRadiusM"): {"type": "float", "label": "机器人半径 (M)", "description": "机器人中心到轮子的半径", "min": 0.05, "max": 0.2, "step": 0.001, "tab": "robot"},
            ("Chassis", "MaxWheelRPM"): {"type": "int", "label": "最大轮速 (RPM)", "min": 100, "max": 1000, "tab": "robot"},
            # Motion
            ("Motion", "CatchVal"): {"type": "list[float]", "label": "抱球位置", "description": "机器人中心到球心的距离和偏移", "item_labels": ["距离 (CM)", "偏移 (CM)"], "tab": "data", "icon": "🏃"},
            ("Motion", "ErrorRange"): {"type": "int", "label": "到达误差范围 (CM)", "min": 1, "max": 50, "tab": "data"},
            # Motion.RestrictedArea (nested)
            ("Motion.RestrictedArea", "XLimit"): {"type": "float", "label": "X 限制 (CM)", "tab": "data"},
            ("Motion.RestrictedArea", "YLimit"): {"type": "float", "label": "Y 限制 (CM)", "tab": "data"},
            ("Motion.RestrictedArea", "GoalBoxWidth"): {"type": "float", "label": "球门区宽度 (CM)", "tab": "data"},
            ("Motion.RestrictedArea", "GoalBoxYHalf"): {"type": "float", "label": "球门区 Y 半宽 (CM)", "tab": "data"},
            ("Motion.RestrictedArea", "Clearance"): {"type": "float", "label": "安全间隙 (CM)", "tab": "data"},
            ("Motion.RestrictedArea", "RecoveryClearance"): {"type": "float", "label": "恢复间隙 (CM)", "tab": "data"},
            ("Motion.RestrictedArea", "RecoveryExtra"): {"type": "float", "label": "恢复额外间隙 (CM)", "tab": "data"},
            # Debug
            ("Debug", "FullLog"): {"type": "bool", "label": "完整日志", "description": "启用详细日志输出", "tab": "data", "icon": "🔧"},
            ("Debug", "Beta"): {"type": "bool", "label": "Beta 功能", "description": "启用实验性功能", "tab": "data"},
            ("Debug", "RecordMotion"): {"type": "bool", "label": "启用动作记录", "description": "用于回放分析", "tab": "data"},
            ("Debug", "RecordInterval"): {"type": "int", "label": "记录间隔 (帧)", "min": 1, "max": 100, "tab": "data"},
            ("Debug", "RecordSessionId"): {"type": "string", "label": "会话 ID", "description": "可选，留空自动生成", "nullable": True, "tab": "data"},
            ("Debug", "RecordPos2Pos"): {"type": "bool", "label": "记录 Pos2Pos", "tab": "data"},
            ("Debug", "Pos2PosRecordDir"): {"type": "string", "label": "Pos2Pos 目录", "tab": "data"},
        }

        # Build sections from DEFAULT_CONFIG
        sections = {}
        section_order = 0

        for section_key, section_data in default_config.items():
            if not isinstance(section_data, dict):
                continue

            section_meta = {"label": section_key, "fields": {}}
            section_order += 1
            field_order = 0

            for field_key, field_value in section_data.items():
                field_order += 1
                meta_key = (section_key, field_key)

                if meta_key in field_metadata:
                    # Use provided metadata
                    field_info = dict(field_metadata[meta_key])
                    field_info["default"] = field_value
                    field_info["order"] = field_order
                    section_meta["fields"][field_key] = field_info

                    # Propagate tab/icon to section
                    if "tab" in field_info and "tab" not in section_meta:
                        section_meta["tab"] = field_info["tab"]
                    if "icon" in field_info and "icon" not in section_meta:
                        section_meta["icon"] = field_info["icon"]
                else:
                    # Auto-detect type from value
                    field_type = self._detect_field_type(field_value)
                    section_meta["fields"][field_key] = {
                        "type": field_type,
                        "label": field_key,
                        "default": field_value,
                        "order": field_order,
                    }

            # Set default tab if not specified
            if "tab" not in section_meta:
                section_meta["tab"] = "data"

            sections[section_key] = section_meta

        return {
            "$schema": "config_template_v1",
            "sections": sections,
            "_meta": {
                "version": "1.0",
                "generated_from": "ReasonData/config.py DEFAULT_CONFIG",
                "last_updated": time.strftime("%Y-%m-%d", time.localtime()),
            }
        }

    def _detect_field_type(self, value) -> str:
        """Auto-detect field type from default value."""
        if isinstance(value, bool):
            return "bool"
        elif isinstance(value, int):
            return "int"
        elif isinstance(value, float):
            return "float"
        elif isinstance(value, (list, tuple)):
            if len(value) == 0:
                return "list"
            first_item = value[0]
            if isinstance(first_item, int):
                return "list[int]"
            elif isinstance(first_item, float):
                return "list[float]"
            else:
                return "list"
        elif isinstance(value, dict):
            return "object"
        elif value is None:
            return "string"
        else:
            return "string"

    def save_config(self, config: dict) -> dict:
        try:
            config_file = _repo_root() / "src" / "ReasonData" / "data" / "config.json"
            config_file.parent.mkdir(parents=True, exist_ok=True)
            with open(config_file, 'w', encoding='utf-8') as f:
                json.dump(config, f, ensure_ascii=False, indent=4)
            return {"success": True}
        except Exception as e:
            return {"success": False, "error": str(e)}

    def get_system_info(self) -> dict:
        import socket

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
            "is_simulator": self._is_simulator,
        }

    def get_simulator_state(self) -> dict:
        """获取模拟器状态 - 使用 Positions 的语义口径。"""
        if not self._is_simulator:
            return {
                "available": False,
                "message": "Not running in simulator mode"
            }

        result = {
            "available": True,
            "robot_pose": {"x_cm": 0, "y_cm": 0, "yaw_deg": 0},
            "ball_pos": list(BALL_NOT_FOUND),
            "has_ball": False,
            "enemy_positions": [],
        }

        positions = self._get_positions_facade()
        if positions is not None:
            try:
                x, y, yaw = positions.FieldRobotPose()
                if [x, y, yaw] != POSE_NOT_FOUND:
                    result["robot_pose"] = {"x_cm": round(float(x), 2), "y_cm": round(float(y), 2), "yaw_deg": round(float(yaw), 2)}
            except Exception:
                pass

            try:
                ball_pos = positions.RobotBallPos()
                result["ball_pos"] = ball_pos
                result["has_ball"] = positions.BallState(ball_pos) == "held"
            except Exception:
                pass

            try:
                result["enemy_positions"] = positions.FieldChassisPos()
            except Exception:
                pass

        return result

    def list_files(self, directory: str, extensions: list = None) -> list:
        try:
            repo_root = _repo_root()

            if directory == "models":
                models_dir = repo_root.parent / "YOLO_Models" / "models"
                if not models_dir.exists():
                    models_dir = repo_root / "models"
                search_dir = models_dir
            elif directory == "hailo_models":
                possible_dirs = [
                    Path("/usr/share/hailo-models"),
                    repo_root.parent / "YOLO_Models" / "models",
                    repo_root / "models",
                ]
                search_dir = None
                for d in possible_dirs:
                    if d.exists():
                        search_dir = d
                        break
                if search_dir is None:
                    search_dir = repo_root / "models"
            elif directory == "calibration":
                search_dir = self.calib_dir
            elif directory == "records":
                search_dir = repo_root / "Records"
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
        return {
            "hailo_models": self.list_files("hailo_models", [".hef"]),
            "tuna_models": self.list_files("models", [".pt", ".onnx"]),
            "tuna_engines": self.list_files("models", [".engine", ".trt"]),
            "calibration_files": self.list_files("calibration", [".npz"]),
        }

    def get_calibration_files(self) -> list:
        if not self.calib_dir.exists():
            return []
        return [_calibration_file_info(f) for f in sorted(self.calib_dir.glob("CalibrationData*.npz"))]

    # ==================== 摄像头标定 ====================

    def calib_list_cameras(self) -> list:
        """列出可用摄像头 - 从 Vision 模块获取 CamPorts"""
        # 模拟器环境下，检查 XeleratorVision 的摄像头槽位
        if self._is_simulator:
            # XeleratorVision 使用默认的 4 个摄像头槽位
            # 从配置获取摄像头槽位
            config = self.get_config()
            vision_cfg = config.get("Vision", {})
            cam_ports = vision_cfg.get("CamPorts", [0, 2, 4, 6])
            if cam_ports:
                print(f"[ControlPanel] Simulator cameras from config: {cam_ports}")
                return list(cam_ports)

            # 尝试从 XeleratorVision 获取摄像头槽位
            if self.vision:
                slots = getattr(self.vision, '_camera_slot_defaults', None)
                if slots is not None:
                    try:
                        slots = slots()
                        if slots:
                            print(f"[ControlPanel] Simulator cameras from XeleratorVision: {slots}")
                            return list(slots)
                    except Exception:
                        pass

            # 默认槽位
            default_slots = [0, 2, 4, 6]
            print(f"[ControlPanel] Simulator default cameras: {default_slots}")
            return default_slots

        # 非模拟器环境：如果 Vision 模块可用，直接返回 CamPorts
        if self.vision:
            cam_ports = getattr(self.vision, 'CamPorts', None)
            if cam_ports:
                print(f"[ControlPanel] Cameras from Vision CamPorts: {cam_ports}")
                return list(cam_ports)

        # 如果 Vision 模块不可用，从配置获取
        config = self.get_config()
        vision_vals = config.get("VisionVals", {})

        config_cameras = [
            vision_vals.get("Back", 0),
            vision_vals.get("Left", 2),
            vision_vals.get("Front", 4),
            vision_vals.get("Right", 6),
        ]

        # 去重并排序
        cameras = sorted(set(config_cameras))
        print(f"[ControlPanel] Cameras from config: {cameras}")
        return cameras

    def _get_frame_from_vision(self, cam_index: int) -> np.ndarray | None:
        """从 Vision 模块获取指定摄像头的画面"""
        if not self.vision:
            return None

        frames = getattr(self.vision, "Frames", None)
        if frames is None:
            return None

        # 获取 CamPorts 映射（可能不存在于 XeleratorVision）
        cam_ports = getattr(self.vision, 'CamPorts', None)

        # 模拟器环境：XeleratorVision 没有 CamPorts，直接使用索引
        if cam_ports is None:
            # 直接使用 cam_index 作为 Frames 索引（0-3）
            if 0 <= cam_index < len(frames):
                frame = frames[cam_index]
                if frame is not None:
                    return frame.copy()
            return None

        # 非模拟器环境：cam_index 是摄像头端口号（如 0, 2, 4, 6）
        # 需要转换为 Frames 列表中的位置
        try:
            # 查找 cam_index 在 CamPorts 中的位置
            frame_idx = cam_ports.index(cam_index)
        except (ValueError, AttributeError):
            # 如果找不到，假设 cam_index 就是 Frames 的索引
            frame_idx = cam_index

        if frame_idx < 0 or frame_idx >= len(frames):
            return None

        frame = frames[frame_idx]
        if frame is not None:
            return frame.copy()
        return None

    def calib_start_stream(
        self,
        cam_index: int,
        width: int,
        height: int,
        board_w: int,
        board_h: int,
        square_mm: float,
        origin_x_cm: float = DEFAULT_CALIB_ORIGIN_X_CM,
        origin_y_cm: float = DEFAULT_CALIB_ORIGIN_Y_CM,
    ) -> dict:
        """启动标定摄像头流 - 完全依赖 Vision 模块获取画面"""
        with self.calib_session.lock:
            if self.calib_session.running:
                return {"success": False, "error": "Stream already running"}

            self.calib_session.cam_index = int(cam_index)
            self.calib_session.width = int(width)
            self.calib_session.height = int(height)
            self.calib_session.board_w = int(board_w)
            self.calib_session.board_h = int(board_h)
            self.calib_session.square_mm = float(square_mm)
            self.calib_session.origin_x_cm = float(origin_x_cm)
            self.calib_session.origin_y_cm = float(origin_y_cm)

            # 检查 Vision 模块是否可用
            if not self.vision:
                return {"success": False, "error": "Vision module not available. Please start vision first."}

            frames = getattr(self.vision, "Frames", None)
            if frames is None:
                return {"success": False, "error": "Vision module not initialized. Frames not available."}

            # 验证摄像头索引
            cam_ports = getattr(self.vision, 'CamPorts', None)
            if cam_ports is not None:
                # 非模拟器环境：验证索引是否在 CamPorts 中
                if cam_index not in cam_ports:
                    return {"success": False, "error": f"Camera {cam_index} not in Vision CamPorts {cam_ports}"}
                print(f"[Calibration] Using Vision module for camera {cam_index} (CamPorts: {cam_ports})")
            else:
                # 模拟器环境：验证索引是否在 Frames 范围内
                if cam_index < 0 or cam_index >= len(frames):
                    return {"success": False, "error": f"Camera index {cam_index} out of range (0-{len(frames)-1})"}
                print(f"[Calibration] Using Vision module for camera slot {cam_index} (simulator mode)")

            # 不打开摄像头，完全依赖 Vision 模块
            self.calib_session.cap = None
            self.calib_session.running = True
            self.calib_session.status = "Streaming (Vision)"
            self.calib_session.samples = []
            self.calib_session.result = None

            # Start capture thread
            threading.Thread(target=self._calib_capture_loop, daemon=True).start()

            return {"success": True, "message": f"Started camera {cam_index} via Vision module"}

    def _calib_capture_loop(self):
        """标定摄像头捕获循环 - 完全依赖 Vision 模块获取画面"""
        while self.calib_session.running:
            try:
                frame = self._get_frame_from_vision(self.calib_session.cam_index)

                if frame is None:
                    time.sleep(0.03)
                    continue

                # Detect chessboard
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                found, corners = cv2.findChessboardCornersSB(gray, (self.calib_session.board_w, self.calib_session.board_h), None)

                with self.calib_session.lock:
                    self.calib_session.latest_frame = frame
                    self.calib_session.latest_corners = corners.copy() if found and corners is not None else None
                    self.calib_session.latest_found = found
                    self.calib_session.latest_size = (frame.shape[1], frame.shape[0])

            except Exception as e:
                print(f"[Calibration] Capture error: {e}")
                time.sleep(0.1)

    def calib_stop_stream(self) -> dict:
        """停止标定摄像头流"""
        with self.calib_session.lock:
            self.calib_session.running = False
            if self.calib_session.cap:
                self.calib_session.cap.release()
                self.calib_session.cap = None
            self.calib_session.status = "Stopped"

        return {"success": True, "message": "Stream stopped"}

    def calib_get_frame(self) -> bytes:
        """获取标定帧（带棋盘格标注）"""
        with self.calib_session.lock:
            if self.calib_session.latest_frame is None:
                # Return placeholder
                img = np.zeros((self.calib_session.height, self.calib_session.width, 3), dtype=np.uint8)
                cv2.putText(img, "No frame", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            else:
                img = self.calib_session.latest_frame.copy()

                # Draw chessboard corners
                if self.calib_session.latest_corners is not None:
                    cv2.drawChessboardCorners(img, (self.calib_session.board_w, self.calib_session.board_h),
                                               self.calib_session.latest_corners, self.calib_session.latest_found)

                # Draw info
                info = [
                    f"Camera={self.calib_session.cam_index}",
                    f"Board={self.calib_session.board_w}x{self.calib_session.board_h}",
                    f"Origin=({self.calib_session.origin_x_cm:.1f},{self.calib_session.origin_y_cm:.1f})cm",
                    f"Found={self.calib_session.latest_found}",
                    f"Samples={len(self.calib_session.samples)}",
                ]
                y = 30
                for line in info:
                    cv2.putText(img, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
                    cv2.putText(img, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1, cv2.LINE_AA)
                    y += 25

        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if ok:
            return bytes(buf)
        return b""

    def calib_capture_sample(self) -> dict:
        """捕获标定样本"""
        with self.calib_session.lock:
            if self.calib_session.latest_frame is None:
                return {"success": False, "error": "No frame available"}
            if self.calib_session.latest_corners is None:
                return {"success": False, "error": "No chessboard detected"}

            area = 0.0
            try:
                corners = self.calib_session.latest_corners
                board_w = self.calib_session.board_w
                board_h = self.calib_session.board_h
                tl = corners[0, 0]
                tr = corners[board_w - 1, 0]
                br = corners[-1, 0]
                bl = corners[board_w * (board_h - 1), 0]
                quad = np.array([tl, tr, br, bl], dtype=np.float32)
                area = float(abs(cv2.contourArea(quad.reshape(-1, 1, 2))))
            except Exception:
                area = 0.0

            sample = {
                "frame": self.calib_session.latest_frame.copy(),
                "corners": self.calib_session.latest_corners.copy(),
                "image_size": self.calib_session.latest_size,
                "area": area,
            }
            self.calib_session.samples.append(sample)

            return {"success": True, "message": f"Captured sample #{len(self.calib_session.samples)}"}

    def calib_clear_samples(self) -> dict:
        """清除所有样本"""
        with self.calib_session.lock:
            self.calib_session.samples = []
            self.calib_session.result = None
        return {"success": True, "message": "Samples cleared"}

    def calib_calibrate(self) -> dict:
        """执行标定"""
        with self.calib_session.lock:
            samples = list(self.calib_session.samples)

        if len(samples) < 3:
            return {"success": False, "error": "Need at least 3 samples"}

        board_w = self.calib_session.board_w
        board_h = self.calib_session.board_h
        square_mm = self.calib_session.square_mm
        origin_x_mm = float(self.calib_session.origin_x_cm) * 10.0
        origin_y_mm = float(self.calib_session.origin_y_cm) * 10.0

        # Board coordinates are shifted so (0, 0) is the robot center.
        # X: left inner corner -> robot center.  Y: robot center -> bottom row.
        objp = np.zeros((board_w * board_h, 3), np.float32)
        for row in range(board_h):
            for col in range(board_w):
                idx = row * board_w + col
                objp[idx, 0] = col * square_mm - origin_x_mm
                objp[idx, 1] = (board_h - 1 - row) * square_mm + origin_y_mm

        objpoints = []
        imgpoints = []
        image_size = samples[0]["image_size"]

        for s in samples:
            if s["corners"] is not None:
                objpoints.append(objp)
                imgpoints.append(s["corners"])

        if len(objpoints) < 3:
            return {"success": False, "error": "Not enough valid samples"}

        try:
            rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, image_size, None, None)

            # Calculate RMSE
            sq_err = 0.0
            n_pts = 0
            for obj, img, rv, tv in zip(objpoints, imgpoints, rvecs, tvecs):
                proj, _ = cv2.projectPoints(obj, rv, tv, K, dist)
                err = proj.reshape(-1, 2) - img.reshape(-1, 2)
                sq_err += float(np.sum(err * err))
                n_pts += int(obj.shape[0])
            rmse = math.sqrt(sq_err / max(1, n_pts))

            # The final pose defines the robot coordinate plane. Use the last
            # captured valid sample so users can gather varied intrinsic samples
            # first, then place the board on the reference plane for the final shot.
            best = next((s for s in reversed(samples) if s["corners"] is not None), None)
            if best is None:
                return {"success": False, "error": "No valid pose sample"}

            # For ground-plane coordinates, a direct homography from the final
            # raw chessboard pixels to the robot plane is more stable than
            # estimating full camera intrinsics from a few coplanar samples.
            obj_xy = objp[:, :2].astype(np.float64)
            img_xy = best["corners"].reshape(-1, 2).astype(np.float64)
            img_to_world, mask = cv2.findHomography(img_xy, obj_xy, method=0)
            if img_to_world is None:
                return {"success": False, "error": "Homography estimation failed"}
            H = np.linalg.inv(img_to_world)

            reproj_img = cv2.perspectiveTransform(obj_xy.reshape(-1, 1, 2), H).reshape(-1, 2)
            h_err = reproj_img - img_xy
            h_rmse = math.sqrt(float(np.mean(np.sum(h_err * h_err, axis=1))))
            diagnostic = _calibration_plane_diagnostic(H, None, None, image_size, undistort_points=False)
            extent = diagnostic.get("extent_cm") or {}
            extent_msg = ""
            if extent:
                extent_msg = (
                    f", plane X={extent['x_min']:.1f}..{extent['x_max']:.1f}cm"
                    f", Y={extent['y_min']:.1f}..{extent['y_max']:.1f}cm"
                )
            message = f"标定完成, H_RMSE={h_rmse:.3f}px{extent_msg}"

            self.calib_session.result = {
                "K": K,
                "dist": dist,
                "rvec": None,
                "tvec": None,
                "H": H,
                "rmse": h_rmse,
                "camera_rmse": rmse,
                "board_size": (board_w, board_h),
                "square_size_mm": square_mm,
                "origin_offset_cm": (self.calib_session.origin_x_cm, self.calib_session.origin_y_cm),
                "image_size": image_size,
                "homography_space": "raw_pixels",
                "undistort_points": False,
                "diagnostic": diagnostic,
                "message": message,
            }

            return {
                "success": True,
                "message": message,
                "rmse": h_rmse,
                "diagnostic": diagnostic,
            }

        except Exception as e:
            return {"success": False, "error": str(e)}

    def calib_save(self, cam_index: int) -> dict:
        """保存标定结果"""
        if self.calib_session.result is None:
            return {"success": False, "error": "No calibration result"}

        result = self.calib_session.result

        # Legacy compatibility
        flip = np.diag([1.0, -1.0, 1.0])
        legacy_matrix = flip @ np.linalg.inv(result["H"])
        legacy_p2c = np.array([0.1, 0.0, 0.0], dtype=np.float64)

        out_path = self.calib_dir / f"CalibrationData{int(cam_index)}.npz"

        try:
            np.savez(
                out_path,
                version=np.array([2], dtype=np.int32),
                format="pixel2cm_v2",
                camera_index=np.array([int(cam_index)], dtype=np.int32),
                board_size=np.array(result["board_size"], dtype=np.int32),
                square_size_mm=np.array([result["square_size_mm"]], dtype=np.float32),
                origin=np.array(["robot_center"]),
                board_origin=np.array(["bottom_left_inner_corner"]),
                coordinate_convention=np.array(["robot_center_v2"]),
                homography_space=np.array([result.get("homography_space", "raw_pixels")]),
                undistort_points=np.array([bool(result.get("undistort_points", False))]),
                origin_offset_cm=np.array(result["origin_offset_cm"], dtype=np.float32),
                origin_offset_mm=np.array(result["origin_offset_cm"], dtype=np.float32) * 10.0,
                image_size=np.array(result["image_size"], dtype=np.int32),
                K=result["K"],
                dist=result["dist"],
                rvec=(
                    np.asarray(result["rvec"], dtype=np.float64)
                    if result.get("rvec") is not None
                    else np.empty((0, 1), dtype=np.float64)
                ),
                tvec=(
                    np.asarray(result["tvec"], dtype=np.float64)
                    if result.get("tvec") is not None
                    else np.empty((0, 1), dtype=np.float64)
                ),
                H=result["H"],
                rmse=np.array([result["rmse"]], dtype=np.float32),
                camera_rmse=np.array([result.get("camera_rmse", np.nan)], dtype=np.float32),
                frame_world_extent_cm=np.array(
                    [
                        result.get("diagnostic", {}).get("extent_cm", {}).get("x_min", np.nan),
                        result.get("diagnostic", {}).get("extent_cm", {}).get("x_max", np.nan),
                        result.get("diagnostic", {}).get("extent_cm", {}).get("y_min", np.nan),
                        result.get("diagnostic", {}).get("extent_cm", {}).get("y_max", np.nan),
                    ],
                    dtype=np.float32,
                ),
                frame_world_points_cm=np.array(
                    list(result.get("diagnostic", {}).get("points_cm", {}).values()),
                    dtype=np.float32,
                ),
                frame_world_warning=np.array([result.get("diagnostic", {}).get("warning", "")]),
                samples_used=np.array([len(self.calib_session.samples)], dtype=np.int32),
                matrix=legacy_matrix,
                p2c=legacy_p2c,
            )

            reload_result = self._reload_vision_calibration()
            suffix = ""
            if reload_result["reloaded"]:
                names = ", ".join(reload_result["targets"])
                suffix = f" | reloaded: {names}"
            elif reload_result["errors"]:
                suffix = f" | reload failed: {'; '.join(reload_result['errors'])}"
            else:
                suffix = " | no active vision to reload"

            return {"success": True, "message": f"Saved to {out_path}{suffix}"}
        except Exception as e:
            return {"success": False, "error": str(e)}

    def _reload_vision_calibration(self) -> dict:
        targets = []

        def _add_target(obj):
            if obj is None:
                return
            if any(obj is existing for existing in targets):
                return
            targets.append(obj)

        _add_target(self.vision)
        for module_name in ("LunaPre.runtime", "src.LunaPre.runtime"):
            runtime = sys.modules.get(module_name)
            if runtime is not None:
                _add_target(getattr(runtime, "vision", None))

        reloaded = []
        errors = []
        for target in targets:
            name = type(target).__name__
            try:
                reload_fn = getattr(target, "reload_calibration", None)
                if callable(reload_fn):
                    reload_fn()
                    reloaded.append(name)
                    continue

                init_fn = getattr(target, "_init_calibration", None)
                if callable(init_fn):
                    init_fn()
                    reloaded.append(name)
                    continue

                errors.append(f"{name}: reload_calibration unavailable")
            except Exception as e:
                errors.append(f"{name}: {e}")

        return {
            "reloaded": bool(reloaded),
            "targets": reloaded,
            "errors": errors,
        }

    def calib_get_status(self) -> dict:
        """获取标定状态"""
        with self.calib_session.lock:
            return {
                "running": self.calib_session.running,
                "status": self.calib_session.status,
                "cam_index": self.calib_session.cam_index,
                "board_size": (self.calib_session.board_w, self.calib_session.board_h),
                "origin_offset_cm": (self.calib_session.origin_x_cm, self.calib_session.origin_y_cm),
                "samples": len(self.calib_session.samples),
                "found": self.calib_session.latest_found,
                "has_result": self.calib_session.result is not None,
                "rmse": self.calib_session.result.get("rmse") if self.calib_session.result else None,
                "message": self.calib_session.result.get("message") if self.calib_session.result else None,
                "diagnostic": self.calib_session.result.get("diagnostic") if self.calib_session.result else None,
            }

    # ==================== 键盘控制 ====================

    def control_enable(self, enabled: bool) -> dict:
        """启用/禁用键盘控制"""
        with self._control_lock:
            if enabled:
                chassis, _ = self._get_chassis_and_peripheral(initialize=True)
                if not chassis:
                    self._control_enabled = False
                    return {
                        "success": False,
                        "enabled": False,
                        "error": self._control_runtime_error or "Control runtime unavailable",
                    }

                try:
                    set_profile = getattr(self.bits, "SetChassisProfile", None)
                    if callable(set_profile):
                        set_profile(self._control_profile)
                    arm = getattr(self.bits, "Arm", None)
                    if callable(arm):
                        arm()
                except Exception as e:
                    self._control_enabled = False
                    self._control_runtime_error = str(e)
                    return {"success": False, "enabled": False, "error": str(e)}

                self._control_target = (0.0, 0.0, 0.0)
                self._control_last_lease = time.monotonic()
                self._control_enabled = True
                self._control_disable_reason = None
                self._control_runtime_error = None
                self._ensure_control_worker()
                return {"success": True, "enabled": True}

            if not self._control_enabled:
                self._control_target = (0.0, 0.0, 0.0)
                self._control_runtime_error = None
                return {"success": True, "enabled": False}

            self._control_enabled = False
            self._control_target = (0.0, 0.0, 0.0)
            self._control_disable_reason = "user_disabled"
            try:
                self._control_disable_hardware()
            except Exception as e:
                self._control_runtime_error = str(e)
                return {"success": False, "enabled": False, "error": str(e)}
            self._control_runtime_error = None
            return {"success": True, "enabled": False}

    def control_set_speed(self, speed: float, turn_speed: float = None) -> dict:
        """设置移动速度和转向速度，单位分别为 m/s 和 deg/s。"""
        with self._control_lock:
            value = float(speed)
            if not math.isfinite(value):
                raise ValueError("speed must be finite")
            # Cached legacy pages send millimetres per second (for example
            # 100), while the current API uses SI values (for example 0.10).
            if abs(value) > DM_CHASSIS_MAX_LINEAR_MPS:
                value *= 0.001
            self._control_speed = max(0.0, min(DM_CHASSIS_MAX_LINEAR_MPS, value))
            if turn_speed is not None:
                self._control_turn_speed = max(0.0, min(360.0, float(turn_speed)))
        return {"success": True, "speed": self._control_speed, "turn_speed": self._control_turn_speed}

    def control_get_status(self) -> dict:
        """获取控制状态"""
        chassis, peripheral = self._get_chassis_and_peripheral(initialize=False)
        lease_age = max(0.0, time.monotonic() - self._control_last_lease)
        return {
            "enabled": self._control_enabled,
            "speed": self._control_speed,
            "turn_speed": self._control_turn_speed,
            "dribble_active": self._dribble_active,
            "runtime_initialized": self._is_control_runtime_initialized(),
            "chassis_available": bool(chassis),
            "peripheral_available": bool(peripheral),
            "backend": type(self.bits).__name__ if self.bits is not None else None,
            "profile": self._control_profile,
            "runtime_error": self._control_runtime_error,
            "disable_reason": self._control_disable_reason,
            "lease_age_ms": round(lease_age * 1000.0, 1),
            "lease_timeout_ms": round(DM_CONTROL_LEASE_TIMEOUT_S * 1000.0),
        }

    def control_renew_lease(self) -> dict:
        """Renew browser ownership without adding another serial transaction."""
        with self._control_lock:
            if not self._control_enabled:
                return {
                    "success": False,
                    "enabled": False,
                    "error": "Control not enabled",
                    "disable_reason": self._control_disable_reason,
                }
            self._control_last_lease = time.monotonic()
            return {"success": True, "enabled": True}

    def control_set_profile(self, profile: str) -> dict:
        """Select a C-board motion profile while the chassis is Disabled."""
        value = str(profile).strip().lower()
        allowed = {"safe", "normal", "competition_candidate"}
        if value not in allowed:
            return {"success": False, "error": f"Unknown control profile: {profile}"}
        with self._control_lock:
            if self._control_enabled:
                return {"success": False, "error": "Disable chassis before changing profile"}
            self._ensure_control_runtime()
            setter = getattr(self.bits, "SetChassisProfile", None)
            if not callable(setter):
                return {"success": False, "error": "DM2325 profile control unavailable"}
            try:
                setter(value)
            except Exception as e:
                self._control_runtime_error = str(e)
                return {"success": False, "error": str(e)}
            self._control_profile = value
            self._control_runtime_error = None
            return {"success": True, "profile": value}

    def control_get_telemetry(self) -> dict:
        """Return C-board motor and optional IMU feedback for the control page."""
        # Do not hold the control lock across CDC exchanges. The 30 ms
        # command worker must be able to refresh the C-board watchdog between
        # telemetry transactions while the chassis is moving.
        with self._control_lock:
            self._ensure_control_runtime()
            get_diagnostics = getattr(self.bits, "GetChassisDiagnostics", None)
            get_status = getattr(self.bits, "GetChassisTelemetry", None)
            # A moving chassis has a 100 ms board watchdog. Do not add another
            # CDC exchange to the telemetry poll while control is enabled.
            imu_method = "GetCachedIMUState" if self._control_enabled else "GetIMUState"
            get_imu_state = getattr(self.bits, imu_method, None)
            if not callable(get_diagnostics) or not callable(get_status):
                return {"success": False, "error": "DM2325 telemetry unavailable"}
        try:
            diagnostics = get_diagnostics()
            status = get_status()
        except Exception as e:
            self._control_runtime_error = str(e)
            return {"success": False, "error": str(e)}
        imu = {"available": False, "error": "IMU telemetry unavailable"}
        if callable(get_imu_state):
            try:
                imu = self._control_imu_telemetry(get_imu_state())
            except Exception as e:
                imu = {"available": False, "error": str(e)}
        with self._control_lock:
            status_by_wheel = {motor.wheel: motor for motor in status.motors}
            for wheel in diagnostics["wheels"]:
                motor = status_by_wheel.get(wheel["wheel"])
                if motor is None:
                    continue
                wheel.update({
                    "online": motor.online,
                    "measured_position_rad": motor.position_rad if motor.online else None,
                    "motor_temperature_c": motor.rotor_temperature_c if motor.online else None,
                    "driver_temperature_c": motor.temperature_c if motor.online else None,
                    "can_rx_count": motor.rx_count,
                    "dropped_frame_count": motor.stale_count,
                })
            diagnostics.update({
                "success": True,
                "runtime_state": status.runtime_state,
                "runtime_fault": status.runtime_fault,
                "can_esr": status.can_esr,
                "watchdog_state": "healthy" if status.runtime_fault == 0 else "fault",
                "imu": imu,
            })
            return diagnostics

    @staticmethod
    def _control_imu_telemetry(state) -> dict:
        """Convert the typed driver state into JSON-safe panel telemetry."""
        def finite(value):
            value = float(value)
            return value if math.isfinite(value) else None

        def vector(values):
            return [finite(value) for value in values] if len(values) == 3 else [None] * 3

        snapshot_age_ms = finite(state.age_ms)
        received = finite(getattr(state, "received_monotonic", time.monotonic()))
        if snapshot_age_ms is not None and received is not None:
            snapshot_age_ms += max(0.0, time.monotonic() - received) * 1000.0
        roll = finite(state.roll_rad)
        pitch = finite(state.pitch_rad)
        yaw = finite(state.yaw_rad)
        return {
            "available": True,
            "healthy": bool(
                state.healthy
                and snapshot_age_ms is not None
                and snapshot_age_ms <= 50.0
            ),
            "source": str(state.source),
            "age_ms": snapshot_age_ms,
            "flags": int(state.flags),
            "accel_mps2": vector(state.accel_mps2),
            "gyro_rad_s": vector(state.gyro_rad_s),
            "attitude_deg": {
                "roll": math.degrees(roll) if roll is not None else None,
                "pitch": math.degrees(pitch) if pitch is not None else None,
                "yaw": math.degrees(yaw) % 360.0 if yaw is not None else None,
            },
            "temperature_c": finite(state.temperature_c),
            "calibration_state": int(state.calibration_state),
            "sample_count": int(state.sample_count),
            "missed_sample_count": int(state.missed_sample_count),
            "spi_error_count": int(state.spi_error_count),
        }

    def _is_control_runtime_initialized(self) -> bool:
        runtime = sys.modules.get("LunaPre.runtime")
        return bool(runtime is not None and getattr(runtime, "_runtime_initialized", False))

    def _ensure_control_runtime(self):
        """Initialize LunaPre runtime on demand for manual control."""
        if not getattr(self, "_allow_runtime_init", True):
            self._control_runtime_error = "Runtime initialization disabled"
            return None
        try:
            import importlib

            runtime = importlib.import_module("LunaPre.runtime")
            if not getattr(runtime, "_runtime_initialized", False):
                runtime._init_runtime(auto_start_lidar=False)
            self.bits = getattr(runtime, "Bits", None)
            runtime_vision = getattr(runtime, "vision", None)
            if runtime_vision is not None and self.vision is not runtime_vision:
                old_vision = self.vision
                self.vision = runtime_vision
                if old_vision is not None:
                    close = getattr(old_vision, "close", None)
                    if callable(close):
                        close()
                self._register_vision_cameras()
                self.set_vision_status("ready", backend=type(runtime_vision).__name__)
            elif self.vision is None:
                self.vision = runtime_vision
                if self.vision is not None:
                    self._register_vision_cameras()
                    self.set_vision_status("ready", backend=type(self.vision).__name__)
            self._control_runtime_error = None
            return runtime
        except Exception as e:
            self._control_runtime_error = str(e)
            print(f"[ControlPanel] Control runtime unavailable: {e}")
            return None, None

    def _get_chassis_and_peripheral(self, *, initialize: bool = False):
        """Return runtime control objects, optionally initializing hardware."""
        runtime = self._ensure_control_runtime() if initialize else sys.modules.get("LunaPre.runtime")
        if isinstance(runtime, tuple):
            return runtime
        if runtime is None or not getattr(runtime, "_runtime_initialized", False):
            return None, None
        return getattr(runtime, "chassis", None), getattr(runtime, "peripheral", None)

    def _control_stop(self):
        """停止机器人移动"""
        chassis, _ = self._get_chassis_and_peripheral()
        if chassis:
            try:
                chassis.stop()
            except Exception as e:
                print(f"[ControlPanel] Error stopping chassis: {e}")

    def _control_disable_hardware(self):
        """Stop and Disable a real DM backend; preserve legacy stop behavior."""
        stop = getattr(self.bits, "stop", None)
        disable = getattr(self.bits, "disable", None)
        if callable(stop) and callable(disable):
            stop()
            disable()
            return
        self._control_stop()

    def _ensure_control_worker(self):
        """Keep the latest command fresh for the C-board's 100 ms watchdog."""
        thread = self._control_worker_thread
        if thread is not None and thread.is_alive():
            return
        self._control_worker_stop.clear()
        self._control_worker_thread = threading.Thread(
            target=self._control_worker_loop,
            daemon=True,
            name="ControlPanel-DM-Heartbeat",
        )
        self._control_worker_thread.start()

    def _control_worker_loop(self):
        while not self._control_worker_stop.wait(0.03):
            with self._control_lock:
                if not self._control_enabled:
                    continue
                lease_expired = (
                    time.monotonic() - self._control_last_lease
                    > DM_CONTROL_LEASE_TIMEOUT_S
                )
                target = self._control_target
                chassis, _ = self._get_chassis_and_peripheral(initialize=False)

                if lease_expired:
                    self._control_enabled = False
                    self._control_target = (0.0, 0.0, 0.0)
                    self._control_disable_reason = "lease_expired"
                    print("[ControlPanel] Browser lease expired; disabling chassis")
                    try:
                        self._control_disable_hardware()
                    except Exception as e:
                        self._control_runtime_error = str(e)
                        print(f"[ControlPanel] Lease-expiry Disable failed: {e}")
                    continue

                if chassis is not None:
                    try:
                        self._move_chassis_si(chassis, *target)
                    except Exception as e:
                        self._control_runtime_error = str(e)
                        print(f"[ControlPanel] Control heartbeat failed: {e}")

    def control_shutdown(self):
        """Leave the chassis Disabled when the web process exits."""
        with self._control_lock:
            was_enabled = self._control_enabled
            self._control_enabled = False
            self._control_target = (0.0, 0.0, 0.0)
        if was_enabled:
            try:
                self._control_disable_hardware()
            except Exception as e:
                print(f"[ControlPanel] Shutdown Disable failed: {e}")
        self._control_worker_stop.set()

    @staticmethod
    def _move_chassis_si(chassis, vx_mps: float, vy_mps: float, wz_deg_s: float):
        """Use the DM SI interface while preserving older chassis backends."""
        move_twist = getattr(chassis, "RobotMoveTwist", None)
        if callable(move_twist):
            return move_twist(vx_mps, vy_mps, math.radians(wz_deg_s))
        return chassis.RobotMoveVector(vx_mps, vy_mps, wz_deg_s)

    def control_move(self, direction: str) -> dict:
        """控制机器人移动（单方向，保留兼容性）

        direction: 'forward', 'backward', 'left', 'right', 'turn_left', 'turn_right', 'stop'
        """
        with self._control_lock:
            if not self._control_enabled:
                return {"success": False, "error": "Control not enabled"}

            chassis, _ = self._get_chassis_and_peripheral(initialize=True)
            if not chassis:
                return {"success": False, "error": self._control_runtime_error or "Chassis not available"}

            speed = self._control_speed
            turn_speed = self._control_turn_speed

            try:
                if direction == 'forward':
                    self._move_chassis_si(chassis, speed, 0, 0)
                elif direction == 'backward':
                    self._move_chassis_si(chassis, -speed, 0, 0)
                elif direction == 'left':
                    self._move_chassis_si(chassis, 0, speed, 0)
                elif direction == 'right':
                    self._move_chassis_si(chassis, 0, -speed, 0)
                elif direction == 'turn_left':
                    self._move_chassis_si(chassis, 0, 0, turn_speed)
                elif direction == 'turn_right':
                    self._move_chassis_si(chassis, 0, 0, -turn_speed)
                elif direction == 'stop':
                    self._control_target = (0.0, 0.0, 0.0)
                    self._control_last_lease = time.monotonic()
                    chassis.stop()
                else:
                    return {"success": False, "error": f"Unknown direction: {direction}"}

                if direction != 'stop':
                    targets = {
                        'forward': (speed, 0.0, 0.0),
                        'backward': (-speed, 0.0, 0.0),
                        'left': (0.0, speed, 0.0),
                        'right': (0.0, -speed, 0.0),
                        'turn_left': (0.0, 0.0, turn_speed),
                        'turn_right': (0.0, 0.0, -turn_speed),
                    }
                    self._control_target = targets[direction]
                    self._control_last_lease = time.monotonic()

                return {"success": True, "direction": direction}
            except Exception as e:
                return {"success": False, "error": str(e)}

    def control_move_vector(self, vx: float, vy: float, wz: float) -> dict:
        """控制机器人移动（全键无冲，支持组合方向）

        vx: X方向速度，m/s，前进为正
        vy: Y方向速度，m/s，左移为正
        wz: 角速度，deg/s，左转为正
        """
        with self._control_lock:
            if not self._control_enabled:
                return {"success": False, "error": "Control not enabled"}

            chassis, _ = self._get_chassis_and_peripheral(initialize=True)
            if not chassis:
                return {"success": False, "error": self._control_runtime_error or "Chassis not available"}

            try:
                vx = float(vx)
                vy = float(vy)
                wz = float(wz)
                if not all(math.isfinite(value) for value in (vx, vy, wz)):
                    raise ValueError("control vector must be finite")
                # Accept vectors from cached legacy pages that still use
                # millimetres per second, then enforce one radial speed limit.
                if max(abs(vx), abs(vy)) > DM_CHASSIS_MAX_LINEAR_MPS:
                    vx *= 0.001
                    vy *= 0.001
                magnitude = math.hypot(vx, vy)
                if magnitude > DM_CHASSIS_MAX_LINEAR_MPS:
                    scale = DM_CHASSIS_MAX_LINEAR_MPS / magnitude
                    vx *= scale
                    vy *= scale
                self._move_chassis_si(chassis, vx, vy, wz)
                self._control_target = (vx, vy, wz)
                self._control_last_lease = time.monotonic()
                return {"success": True, "vx": vx, "vy": vy, "wz": wz}
            except Exception as e:
                return {"success": False, "error": str(e)}

    def control_kick(self) -> dict:
        """弹射（踢球）"""
        with self._control_lock:
            if not self._control_enabled:
                return {"success": False, "error": "Control not enabled"}

            _, peripheral = self._get_chassis_and_peripheral(initialize=True)
            if not peripheral:
                return {"success": False, "error": self._control_runtime_error or "Peripheral not available"}

            try:
                peripheral.ShootBall()
                return {"success": True, "action": "kick"}
            except Exception as e:
                return {"success": False, "error": str(e)}

    def control_dribble(self, active: bool) -> dict:
        """控制盘球

        active: True 开启盘球 (PWM 0.2), False 关闭盘球
        """
        with self._control_lock:
            if not self._control_enabled:
                return {"success": False, "error": "Control not enabled"}

            _, peripheral = self._get_chassis_and_peripheral(initialize=True)
            if not peripheral:
                return {"success": False, "error": self._control_runtime_error or "Peripheral not available"}

            try:
                if active:
                    peripheral.Dribble(0.8)  # PWM 0.2
                else:
                    peripheral.Dribble(False)
                self._dribble_active = active
                return {"success": True, "dribble_active": active}
            except Exception as e:
                return {"success": False, "error": str(e)}

    def control_dribble_toggle(self) -> dict:
        """切换盘球状态"""
        return self.control_dribble(not self._dribble_active)

    # ==================== 鸟瞰图预览 ====================

    def _load_calibration(self, calib_file: str) -> dict:
        """加载标定文件"""
        path = Path(calib_file)
        if not path.exists():
            return {"mode": "missing", "path": str(path)}

        try:
            data = np.load(path, allow_pickle=True)
            keys = set(data.files)

            if {"K", "dist", "H"}.issubset(keys) or ("version" in keys and int(np.atleast_1d(data["version"])[0]) >= 2):
                H, rvec, tvec = _modern_pose_from_npz(data)
                return {
                    "mode": "modern",
                    "path": str(path),
                    "K": np.asarray(data["K"], dtype=np.float64),
                    "dist": np.asarray(data["dist"], dtype=np.float64),
                    "H": H,
                    "rvec": rvec,
                    "tvec": tvec,
                    "undistort_points": _npz_bool(data, "undistort_points", True),
                    "homography_space": _npz_text(data, "homography_space"),
                    "origin": _npz_text(data, "origin", "bottom_left_inner_corner"),
                    "coordinate_convention": _npz_text(data, "coordinate_convention"),
                    "origin_offset_cm": (
                        np.asarray(data["origin_offset_cm"], dtype=np.float64).tolist()
                        if "origin_offset_cm" in data.files
                        else None
                    ),
                }

            if {"matrix", "p2c"}.issubset(keys):
                return {
                    "mode": "legacy",
                    "path": str(path),
                    "matrix": np.asarray(data["matrix"], dtype=np.float64),
                    "p2c": np.asarray(data["p2c"], dtype=np.float64),
                }

            return {"mode": "unknown", "path": str(path)}
        except Exception as e:
            return {"mode": "error", "error": str(e)}

    def birdeye_start(self, cam_index: int, calib_file: str, width: int, height: int,
                      pixels_per_mm: float, grid_mm: float, preview_scale: float, use_mjpg: bool) -> dict:
        """启动鸟瞰图预览 - 完全依赖 Vision 模块获取画面"""
        with self.birdeye_session.lock:
            if self.birdeye_session.running:
                return {"success": False, "error": "Already running"}

            self.birdeye_session.cam_index = int(cam_index)
            self.birdeye_session.width = int(width)
            self.birdeye_session.height = int(height)
            self.birdeye_session.calib_file = calib_file
            self.birdeye_session.pixels_per_mm = float(pixels_per_mm)
            self.birdeye_session.grid_mm = float(grid_mm)
            self.birdeye_session.preview_scale = float(preview_scale)
            self.birdeye_session.use_mjpg = use_mjpg

            # Load calibration
            self.birdeye_session.calib_data = self._load_calibration(calib_file)

            # 检查 Vision 模块是否可用
            if not self.vision:
                return {"success": False, "error": "Vision module not available. Please start vision first."}

            frames = getattr(self.vision, "Frames", None)
            if frames is None:
                return {"success": False, "error": "Vision module not initialized. Frames not available."}

            # 验证摄像头索引
            cam_ports = getattr(self.vision, 'CamPorts', None)
            if cam_ports is not None:
                # 非模拟器环境：验证索引是否在 CamPorts 中
                if cam_index not in cam_ports:
                    return {"success": False, "error": f"Camera {cam_index} not in Vision CamPorts {cam_ports}"}
                print(f"[BirdEye] Using Vision module for camera {cam_index} (CamPorts: {cam_ports})")
            else:
                # 模拟器环境：验证索引是否在 Frames 范围内
                if cam_index < 0 or cam_index >= len(frames):
                    return {"success": False, "error": f"Camera index {cam_index} out of range (0-{len(frames)-1})"}
                print(f"[BirdEye] Using Vision module for camera slot {cam_index} (simulator mode)")

            # 不打开摄像头，完全依赖 Vision 模块
            self.birdeye_session.cap = None
            self.birdeye_session.running = True
            self.birdeye_session.status = "Running (Vision)"

            return {"success": True, "message": f"Started bird-eye preview via Vision module"}

    def birdeye_stop(self) -> dict:
        """停止鸟瞰图预览"""
        with self.birdeye_session.lock:
            self.birdeye_session.running = False
            if self.birdeye_session.cap:
                self.birdeye_session.cap.release()
                self.birdeye_session.cap = None
            self.birdeye_session.status = "Stopped"

        return {"success": True, "message": "Stopped"}

    def birdeye_get_frames(self) -> tuple:
        """获取原始帧和鸟瞰图帧 - 完全依赖 Vision 模块获取画面"""
        with self.birdeye_session.lock:
            if not self.birdeye_session.running:
                # Return placeholders
                img = np.zeros((self.birdeye_session.height, self.birdeye_session.width, 3), dtype=np.uint8)
                cv2.putText(img, "Not running", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
                ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                return bytes(buf) if ok else b"", bytes(buf) if ok else b""

            frame = self._get_frame_from_vision(self.birdeye_session.cam_index)

            if frame is None:
                img = np.zeros((self.birdeye_session.height, self.birdeye_session.width, 3), dtype=np.uint8)
                cv2.putText(img, "No frame", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
                ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                return bytes(buf) if ok else b"", bytes(buf) if ok else b""

            raw = frame
            bird = self._make_bird_eye(frame)

            # Resize bird for preview
            if self.birdeye_session.preview_scale < 0.999:
                new_w = max(1, int(round(bird.shape[1] * self.birdeye_session.preview_scale)))
                new_h = max(1, int(round(bird.shape[0] * self.birdeye_session.preview_scale)))
                bird = cv2.resize(bird, (new_w, new_h), interpolation=cv2.INTER_AREA)

            # Add info overlay
            info = f"Cam={self.birdeye_session.cam_index} | Mode={self.birdeye_session.calib_data.get('mode', 'unknown')}"
            cv2.putText(raw, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(raw, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1, cv2.LINE_AA)

            ok1, buf1 = cv2.imencode(".jpg", raw, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            ok2, buf2 = cv2.imencode(".jpg", bird, [int(cv2.IMWRITE_JPEG_QUALITY), 80])

            return (bytes(buf1) if ok1 else b""), (bytes(buf2) if ok2 else b"")

    def _make_bird_eye(self, frame_bgr: np.ndarray) -> np.ndarray:
        """生成鸟瞰图"""
        calib = self.birdeye_session.calib_data
        mode = calib.get("mode")

        # 最大输出尺寸限制
        MAX_OUTPUT_SIZE = 800

        if mode == "modern":
            K = calib.get("K")
            dist = calib.get("dist")
            H = calib.get("H")
            undistort_points = bool(calib.get("undistort_points", True))
            ppm = max(0.1, self.birdeye_session.pixels_per_mm)
            margin_px = max(40, int(round(ppm * 10.0)))

            src = frame_bgr
            if undistort_points and K is not None and dist is not None:
                try:
                    src = cv2.undistort(frame_bgr, K, dist)
                except Exception:
                    src = frame_bgr

            try:
                H_inv = np.linalg.inv(H)
            except Exception:
                H_inv = np.eye(3, dtype=np.float64)

            h, w = src.shape[:2]
            image_corners = np.array(
                [[[0.0, 0.0]], [[w - 1.0, 0.0]], [[w - 1.0, h - 1.0]], [[0.0, h - 1.0]]],
                dtype=np.float64,
            )
            world_corners = cv2.perspectiveTransform(image_corners, H_inv).reshape(-1, 2)
            min_x, min_y = np.min(world_corners, axis=0)
            max_x, max_y = np.max(world_corners, axis=0)
            min_x = max(float(min_x), -2500.0)
            max_x = min(float(max_x), 2500.0)
            min_y = max(float(min_y), -2500.0)
            max_y = min(float(max_y), 2500.0)
            min_x = min(min_x, 0.0)
            max_x = max(max_x, 0.0)
            min_y = min(min_y, 0.0)
            max_y = max(max_y, 0.0)
            if min_x >= max_x:
                min_x, max_x = -500.0, 500.0
            if min_y >= max_y:
                min_y, max_y = -500.0, 500.0
            world_w_mm = max(1.0, float(max_x - min_x))
            world_h_mm = max(1.0, float(max_y - min_y))
            out_w = max(1, int(round(world_w_mm * ppm)) + 2 * margin_px)
            out_h = max(1, int(round(world_h_mm * ppm)) + 2 * margin_px)

            # 限制输出尺寸，避免溢出
            if out_w > MAX_OUTPUT_SIZE or out_h > MAX_OUTPUT_SIZE:
                scale_factor = min(MAX_OUTPUT_SIZE / out_w, MAX_OUTPUT_SIZE / out_h)
                out_w = int(out_w * scale_factor)
                out_h = int(out_h * scale_factor)
                ppm = ppm * scale_factor
                margin_px = int(margin_px * scale_factor)

            scale = np.array(
                [
                    [ppm, 0.0, -min_x * ppm + margin_px],
                    [0.0, -ppm, max_y * ppm + margin_px],
                    [0.0, 0.0, 1.0],
                ],
                dtype=np.float64,
            )
            M = scale @ H_inv
            bird = cv2.warpPerspective(src, M, (out_w, out_h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))

            # Draw grid
            grid_px = max(1, int(round(self.birdeye_session.grid_mm * ppm)))
            for x in range(0, out_w, grid_px):
                cv2.line(bird, (x, 0), (x, out_h - 1), (0, 180, 255), 1, cv2.LINE_AA)
            for y in range(0, out_h, grid_px):
                cv2.line(bird, (0, y), (out_w - 1, y), (0, 180, 255), 1, cv2.LINE_AA)

            origin_px = (
                int(round(-min_x * ppm + margin_px)),
                int(round(max_y * ppm + margin_px)),
            )
            if 0 <= origin_px[0] < out_w and 0 <= origin_px[1] < out_h:
                axis_len = max(20, int(round(100.0 * ppm)))
                cv2.circle(bird, origin_px, 6, (255, 255, 255), -1, cv2.LINE_AA)
                cv2.circle(bird, origin_px, 8, (0, 0, 0), 2, cv2.LINE_AA)
                cv2.arrowedLine(
                    bird,
                    origin_px,
                    (min(out_w - 1, origin_px[0] + axis_len), origin_px[1]),
                    (255, 80, 80),
                    2,
                    cv2.LINE_AA,
                    tipLength=0.25,
                )
                cv2.arrowedLine(
                    bird,
                    origin_px,
                    (origin_px[0], max(0, origin_px[1] - axis_len)),
                    (80, 255, 80),
                    2,
                    cv2.LINE_AA,
                    tipLength=0.25,
                )

            return bird

        if mode == "legacy":
            matrix = calib.get("matrix", np.eye(3, dtype=np.float64))
            # 最大输出尺寸限制
            MAX_OUTPUT_SIZE = 800
            try:
                # 计算输出尺寸
                h, w = frame_bgr.shape[:2]
                corners = np.array(
                    [[[0.0, 0.0]], [[w - 1.0, 0.0]], [[w - 1.0, h - 1.0]], [[0.0, h - 1.0]]],
                    dtype=np.float64,
                )
                transformed = cv2.perspectiveTransform(corners, matrix).reshape(-1, 2)
                min_x, min_y = np.min(transformed, axis=0)
                max_x, max_y = np.max(transformed, axis=0)
                out_w = max(1, int(max_x - min_x))
                out_h = max(1, int(max_y - min_y))

                # 限制输出尺寸
                if out_w > MAX_OUTPUT_SIZE or out_h > MAX_OUTPUT_SIZE:
                    scale_factor = min(MAX_OUTPUT_SIZE / out_w, MAX_OUTPUT_SIZE / out_h)
                    out_w = int(out_w * scale_factor)
                    out_h = int(out_h * scale_factor)
                    # 缩放矩阵
                    scale_matrix = np.array([
                        [scale_factor, 0, 0],
                        [0, scale_factor, 0],
                        [0, 0, 1]
                    ], dtype=np.float64)
                    matrix = scale_matrix @ matrix

                bird = cv2.warpPerspective(frame_bgr, matrix, (out_w, out_h))
            except Exception:
                bird = frame_bgr.copy()
            return bird

        # Missing calibration
        out = frame_bgr.copy()
        cv2.putText(out, f"Calibration missing: {calib.get('path', '')}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return out


# ==================== HTTP 请求处理器 ====================

class ControlPanelHandler(BaseHTTPRequestHandler):
    """HTTP 请求处理器"""

    app: ControlPanelApp = None

    def log_message(self, format, *args):
        # Suppress default logging
        pass

    def _send_json(self, data, status=200):
        body = json.dumps(_json_safe(data), ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', len(body))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(body)

    def _send_jpeg(self, data: bytes, processing_time_ms: float = None):
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', len(data))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        if processing_time_ms is not None:
            self.send_header('X-Processing-Time-Ms', str(processing_time_ms))
        self.end_headers()
        self.wfile.write(data)

    def _send_file(self, path: Path, content_type: str = None):
        if not path.exists():
            self.send_error(404, 'File not found')
            return

        if content_type is None:
            suffix = path.suffix.lower()
            content_types = {
                '.html': 'text/html; charset=utf-8',
                '.css': 'text/css; charset=utf-8',
                '.js': 'application/javascript; charset=utf-8',
                '.json': 'application/json; charset=utf-8',
                '.png': 'image/png',
                '.jpg': 'image/jpeg',
                '.jpeg': 'image/jpeg',
                '.gif': 'image/gif',
                '.svg': 'image/svg+xml',
                '.ico': 'image/x-icon',
            }
            content_type = content_types.get(suffix, 'application/octet-stream')

        data = path.read_bytes()
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', len(data))
        if path.suffix.lower() in {'.html', '.css', '.js'}:
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        self.end_headers()
        self.wfile.write(data)

    def _parse_json_body(self) -> dict:
        try:
            length = int(self.headers.get('Content-Length', 0))
            if length == 0:
                return {}
            body = self.rfile.read(length)
            return json.loads(body.decode('utf-8'))
        except Exception:
            return {}

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, DELETE, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def do_GET(self):
        app = self.app
        parsed = urlparse(self.path)
        path = parsed.path

        # Static files
        if path == '/' or path == '/index.html':
            self._send_file(app.web_dir / 'index.html')
            return

        if path == '/favicon.ico':
            self.send_response(204)
            self.end_headers()
            return

        if path.startswith('/static/'):
            file_path = app.web_dir / path[1:]
            self._send_file(file_path)
            return

        # API endpoints
        if path == '/api/state':
            self._send_json(app.get_state_dict())
            return

        if path == '/api/simulator-state':
            self._send_json(app.get_simulator_state())
            return

        if path == '/api/system-info':
            self._send_json(app.get_system_info())
            return

        if path == '/api/lidar/status':
            self._send_json(app.get_lidar_status())
            return

        if path == '/api/lidar/usb':
            self._send_json(app.scan_lidar_usb())
            return

        if path == '/api/config':
            self._send_json(app.get_config())
            return

        if path == '/api/config/schema':
            self._send_json(app.get_config_schema())
            return

        if path == '/api/dynamic-options':
            self._send_json(app.get_dynamic_options())
            return

        if path == '/api/calibration-files':
            self._send_json({"files": app.get_calibration_files()})
            return

        # Recording
        if path == '/api/recording/status':
            self._send_json(app.get_recording_status())
            return

        if path == '/api/recording/sessions':
            self._send_json({"sessions": app.list_sessions()})
            return

        if path == '/api/sessions':
            self._send_json(app.list_sessions())
            return

        if path.startswith('/api/sessions/') and path.endswith('/data'):
            parts = path.split('/')
            if len(parts) >= 4:
                self._send_json(app.get_session_data(parts[3]))
            return

        if path.startswith('/api/sessions/') and path.endswith('/export'):
            parts = path.split('/')
            if len(parts) >= 4:
                session_id = parts[3]
                data = app.export_session(session_id)
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Disposition', f'attachment; filename="{session_id}.json"')
                self.send_header('Content-Length', len(data))
                self.end_headers()
                self.wfile.write(data)
            return

        # Camera frames
        if path.startswith('/api/cam/'):
            parts = path.split('/')
            if len(parts) >= 4:
                try:
                    # parts[3] may be like '0.jpg' or '0', extract camera index
                    cam_str = parts[3].split('.')[0]  # Handle '0.jpg' -> '0'
                    cam_index = int(cam_str)
                    resize_to = None
                    # Check query params for size
                    query_params = parsed.query.split('&')
                    for param in query_params:
                        if param.startswith('size='):
                            resize_to = int(param.split('=')[1])
                    jpeg_data, proc_time = app.get_cam_jpeg(cam_index, resize_to)
                    if jpeg_data:
                        self._send_jpeg(jpeg_data, proc_time)
                    else:
                        self.send_error(404, 'Camera not available')
                except ValueError:
                    self.send_error(400, 'Invalid camera index')
            return

        # Calibration API
        if path == '/api/calib/cameras':
            self._send_json({"cameras": app.calib_list_cameras()})
            return

        if path == '/api/calib/status':
            self._send_json(app.calib_get_status())
            return

        if path == '/api/calib/frame':
            jpeg = app.calib_get_frame()
            if jpeg:
                self._send_jpeg(jpeg)
            else:
                self.send_error(404, 'No frame')
            return

        # BirdEye API
        if path == '/api/birdeye/frame':
            raw_jpeg, bird_jpeg = app.birdeye_get_frames()
            # Return both frames as a JSON with base64
            import base64
            self._send_json({
                "raw": base64.b64encode(raw_jpeg).decode('ascii') if raw_jpeg else "",
                "bird": base64.b64encode(bird_jpeg).decode('ascii') if bird_jpeg else "",
            })
            return

        if path == '/api/control/status':
            self._send_json(app.control_get_status())
            return

        if path == '/api/control/telemetry':
            self._send_json(app.control_get_telemetry())
            return

        self.send_error(404, 'Not found')

    def do_POST(self):
        app = self.app
        parsed = urlparse(self.path)
        path = parsed.path

        # Config
        if path == '/api/config':
            data = self._parse_json_body()
            result = app.save_config(data)
            self._send_json(result)
            return

        # Recording
        if path == '/api/recording/start':
            data = self._parse_json_body()
            result = app.start_recording(
                session_id=data.get('session_id'),
                freq=data.get('freq', 5),
                description=data.get('description', '')
            )
            self._send_json(result)
            return

        if path == '/api/recording/stop':
            result = app.stop_recording()
            self._send_json(result)
            return

        if path.startswith('/api/recording/delete/'):
            session_id = path.split('/')[-1]
            result = app.delete_session(session_id)
            self._send_json(result)
            return

        if path.startswith('/api/recording/export/'):
            session_id = path.split('/')[-1]
            data = app.export_session(session_id)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Disposition', f'attachment; filename="{session_id}.json"')
            self.send_header('Content-Length', len(data))
            self.end_headers()
            self.wfile.write(data)
            return

        if path.startswith('/api/sessions/') and path.endswith('/export'):
            parts = path.split('/')
            if len(parts) >= 4:
                session_id = parts[3]
                data = app.export_session(session_id)
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Disposition', f'attachment; filename="{session_id}.json"')
                self.send_header('Content-Length', len(data))
                self.end_headers()
                self.wfile.write(data)
            return

        if path.startswith('/api/sessions/'):
            session_id = path.split('/')[3]
            result = app.delete_session(session_id)
            self._send_json(result)
            return

        # Calibration API
        if path == '/api/calib/start':
            data = self._parse_json_body()
            result = app.calib_start_stream(
                cam_index=data.get('cam_index', 0),
                width=data.get('width', 640),
                height=data.get('height', 480),
                board_w=data.get('board_w', 9),
                board_h=data.get('board_h', 9),
                square_mm=data.get('square_mm', 15.0),
                origin_x_cm=data.get('origin_x_cm', DEFAULT_CALIB_ORIGIN_X_CM),
                origin_y_cm=data.get('origin_y_cm', DEFAULT_CALIB_ORIGIN_Y_CM),
            )
            self._send_json(result)
            return

        if path == '/api/calib/stop':
            result = app.calib_stop_stream()
            self._send_json(result)
            return

        if path == '/api/calib/capture':
            result = app.calib_capture_sample()
            self._send_json(result)
            return

        if path == '/api/calib/clear':
            result = app.calib_clear_samples()
            self._send_json(result)
            return

        if path == '/api/calib/calibrate':
            result = app.calib_calibrate()
            self._send_json(result)
            return

        if path == '/api/calib/save':
            data = self._parse_json_body()
            result = app.calib_save(data.get('cam_index', 0))
            self._send_json(result)
            return

        # BirdEye API
        if path == '/api/birdeye/start':
            data = self._parse_json_body()
            result = app.birdeye_start(
                cam_index=data.get('cam_index', 0),
                calib_file=data.get('calib_file', ''),
                width=data.get('width', 640),
                height=data.get('height', 480),
                pixels_per_mm=data.get('pixels_per_mm', 4.0),
                grid_mm=data.get('grid_mm', 10.0),
                preview_scale=data.get('preview_scale', 0.4),
                use_mjpg=data.get('use_mjpg', False)
            )
            self._send_json(result)
            return

        if path == '/api/birdeye/stop':
            result = app.birdeye_stop()
            self._send_json(result)
            return

        # Keyboard Control API
        if path == '/api/control/status':
            self._send_json(app.control_get_status())
            return

        if path == '/api/control/enable':
            data = self._parse_json_body()
            result = app.control_enable(data.get('enabled', True))
            self._send_json(result)
            return

        if path == '/api/control/lease':
            self._send_json(app.control_renew_lease())
            return

        if path == '/api/control/speed':
            data = self._parse_json_body()
            result = app.control_set_speed(
                speed=data.get('speed', 100),
                turn_speed=data.get('turn_speed')
            )
            self._send_json(result)
            return

        if path == '/api/control/profile':
            data = self._parse_json_body()
            self._send_json(app.control_set_profile(data.get('profile', 'normal')))
            return

        if path == '/api/control/move':
            data = self._parse_json_body()
            result = app.control_move(data.get('direction', 'stop'))
            self._send_json(result)
            return

        if path == '/api/control/kick':
            result = app.control_kick()
            self._send_json(result)
            return

        if path == '/api/control/dribble':
            data = self._parse_json_body()
            result = app.control_dribble(data.get('active', True))
            self._send_json(result)
            return

        if path == '/api/control/dribble/toggle':
            result = app.control_dribble_toggle()
            self._send_json(result)
            return

        if path == '/api/control/move_vector':
            data = self._parse_json_body()
            result = app.control_move_vector(
                vx=data.get('vx', 0),
                vy=data.get('vy', 0),
                wz=data.get('wz', 0)
            )
            self._send_json(result)
            return

        self.send_error(404, 'Not found')

    def do_DELETE(self):
        app = self.app
        parsed = urlparse(self.path)
        path = parsed.path

        if path.startswith('/api/sessions/'):
            session_id = path.split('/')[3]
            result = app.delete_session(session_id)
            self._send_json(result)
            return

        self.send_error(404, 'Not found')


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
    ap.add_argument(
        "--attach-runtime",
        action="store_true",
        help="Deprecated compatibility flag; runtime is initialized by default unless --no-runtime is set.",
    )
    ap.add_argument(
        "--no-runtime",
        action="store_true",
        help="Keep control panel vision-only and do not initialize LunaPre runtime/lidar.",
    )
    args = ap.parse_args()

    bits = None
    vision = None

    if not args.no_runtime:
        try:
            import importlib

            runtime = importlib.import_module("LunaPre.runtime")
            if not getattr(runtime, "_runtime_initialized", False):
                runtime._init_runtime(auto_start_lidar=True)
            bits = getattr(runtime, "Bits", None)
            vision = getattr(runtime, "vision", None)
            print(f"[ControlPanel] Attached LunaPre runtime Bits: {type(bits).__name__ if bits else None}")
            print(f"[ControlPanel] Attached LunaPre runtime vision: {type(vision).__name__ if vision else None}")
        except Exception as e:
            print(f"[ControlPanel] Could not initialize LunaPre runtime; continuing vision-only: {e}")
    else:
        print("[ControlPanel] Runtime disabled by --no-runtime; lidar/pose will be unavailable")

    # Initialize app with bits
    app = ControlPanelApp(
        web_dir=Path(args.web_dir),
        jpeg_quality=args.jpeg_quality,
        bits=bits,
        vision=vision,
        allow_runtime_init=not args.no_runtime,
    )

    def attach_vision_module():
        errors = []

        with app._vision_attach_lock:
            if app.vision is not None or app._vision_attaching:
                return
            app._vision_attaching = True

        app.set_vision_status("starting")
        try:
            import importlib

            app.set_vision_status("starting", backend="vision-utils")
            module = importlib.import_module("vision_utils.Vision")
            new_vision = module.UnitedVision()
            with app._vision_attach_lock:
                if app.vision is None:
                    app.vision = new_vision
                    new_vision = None
            if new_vision is not None:
                close = getattr(new_vision, "close", None)
                if callable(close):
                    close()
                if app.vision is not None:
                    app.set_vision_status("ready", backend=type(app.vision).__name__)
                return
            app._register_vision_cameras()
            app.set_vision_status("ready", backend="vision-utils")
            print("[ControlPanel] Vision module attached via vision-utils")
            return
        except Exception as e:
            errors.append(f"vision-utils: {e}")
        finally:
            with app._vision_attach_lock:
                app._vision_attaching = False

        if app.vision is None:
            error_msg = " | ".join(errors)
            app.set_vision_status("error", error=error_msg)
            print("[ControlPanel] Vision module not available: " + error_msg)

    # Attach vision in the background so the web UI is available even when
    # camera/model startup is slow or broken.
    if args.no_vision:
        app.set_vision_status("disabled")
    elif app.vision is None:
        threading.Thread(target=attach_vision_module, daemon=True).start()

    # Setup HTTP server
    ControlPanelHandler.app = app
    httpd = ThreadingHTTPServer((args.host, args.port), ControlPanelHandler)

    print(f"\n{'='*60}")
    print(f"  Xelerator Control Panel")
    print(f"  http://{args.host}:{args.port}")
    print(f"{'='*60}\n")

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[ControlPanel] Shutting down...")
    finally:
        app.control_shutdown()
        try:
            app.calib_stop_stream()
        except Exception:
            pass
        try:
            app.birdeye_stop()
        except Exception:
            pass
        # Release vision/cameras so a subsequent launch can reopen devices.
        try:
            vision = getattr(app, "vision", None)
            close = getattr(vision, "close", None)
            if callable(close):
                close(join_timeout=1.0)
            app.vision = None
            try:
                app.camera_manager.unregister_vision_cams()
            except Exception:
                pass
        except Exception as e:
            print(f"[ControlPanel] Vision cleanup failed: {e}")
        try:
            httpd.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
