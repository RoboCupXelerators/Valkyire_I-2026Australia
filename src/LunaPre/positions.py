"""Position, localization, vision, and Pos2Pos facade for LunaPre.

TODO: Split vision subprocess/UDP startup and lidar recovery helpers out of
this facade. Positions should stay focused on semantic position/ball APIs.
"""

from __future__ import annotations

import math
import errno
import pickle
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

import simple_pid

from ReasonData import logger, Settings

from . import runtime
from .strategy_tools import BALL_HELD, BALL_NOT_FOUND, POSE_NOT_FOUND, angle_error_deg, clamp

cfg = runtime.cfg

# Pos2Pos is a 30 Hz field-position controller. The pose error is in cm and
# the command is in m/s, so keep this conservative and use PD only: integral
# action accumulates during delayed pose updates and causes a late overshoot.
POS2POS_KP = 0.009
POS2POS_KI = 0.0
POS2POS_KD = 0.0006
POS2POS_OUTPUT_LIMIT_MPS = 0.55
POS2POS_COMMAND_LIMIT_MPS = 0.65
POS2POS_NEAR_COMMAND_LIMIT_MPS = 0.18
POS2POS_NEAR_DISTANCE_CM = 30.0
# Keep recovery commands above the SIM bridge's 0.6 planar scale and 0.064
# command deadzone (0.12 * 0.6 = 0.072). Normal positioning does not use this
# floor, so fine final approach remains unchanged.
POS2POS_MIN_SPEED_MPS = 0.12
POS2POS_ARRIVAL_SCALE = 0.5
ROBOT_BALL_MAX_DISTANCE_CM = 240.0
ROBOT_BALL_MAX_ABS_X_CM = 260.0
ROBOT_BALL_MAX_ABS_Y_CM = 220.0
SHOOT_WINDOW_DEFAULT_CAMERA_FOV_DEG = 95.0
SHOOT_WINDOW_GOAL_KEYS = ("blue_goal", "yellow_goal")
SHOOT_WINDOW_CAMERA_BASE_BEARING_DEG = {
    0: 180.0,  # back
    1: 90.0,   # left
    2: 0.0,    # front
    3: -90.0,  # right
}
BALL_PREDICT_DEFAULT_HORIZON_S = 0.45
BALL_PREDICT_MAX_HORIZON_S = 1.0
BALL_PREDICT_MAX_AGE_S = 0.35
BALL_PREDICT_MIN_DT_S = 0.02
BALL_PREDICT_MAX_GAP_S = 0.40
BALL_PREDICT_MIN_SPEED_CM_S = 12.0
BALL_PREDICT_MAX_SPEED_CM_S = 500.0


class BallLandingPredictor:
    """Short-horizon field-frame ball predictor for defensive interception."""

    def __init__(
        self,
        *,
        min_dt_s: float = BALL_PREDICT_MIN_DT_S,
        max_gap_s: float = BALL_PREDICT_MAX_GAP_S,
        max_speed_cm_s: float = BALL_PREDICT_MAX_SPEED_CM_S,
        velocity_alpha: float = 0.55,
    ):
        self.min_dt_s = max(0.001, float(min_dt_s))
        self.max_gap_s = max(self.min_dt_s, float(max_gap_s))
        self.max_speed_cm_s = max(1.0, float(max_speed_cm_s))
        self.velocity_alpha = clamp(float(velocity_alpha), 0.05, 1.0)
        self.reset()

    @staticmethod
    def _sentinel_pair(value, sentinel: list[int]) -> bool:
        try:
            return float(value[0]) == float(sentinel[0]) and float(value[1]) == float(sentinel[1])
        except Exception:
            return False

    @classmethod
    def _as_xy(cls, value):
        if value is None or not isinstance(value, (list, tuple)) or len(value) < 2:
            return None
        if cls._sentinel_pair(value, BALL_NOT_FOUND) or cls._sentinel_pair(value, BALL_HELD):
            return None
        try:
            x = float(value[0])
            y = float(value[1])
        except Exception:
            return None
        if not math.isfinite(x) or not math.isfinite(y):
            return None
        return [x, y]

    def reset(self) -> None:
        self.last_pos: list[float] | None = None
        self.last_ts: float | None = None
        self.velocity_cm_s = [0.0, 0.0]
        self.sample_count = 0

    def observe(self, field_pos, timestamp_s: float | None = None) -> bool:
        xy = self._as_xy(field_pos)
        if xy is None:
            return False

        ts = time.monotonic() if timestamp_s is None else float(timestamp_s)
        if not math.isfinite(ts):
            return False

        if self.last_pos is None or self.last_ts is None:
            self.last_pos = xy
            self.last_ts = ts
            self.velocity_cm_s = [0.0, 0.0]
            self.sample_count = 1
            return True

        dt = ts - self.last_ts
        if dt < self.min_dt_s:
            return True
        if dt > self.max_gap_s:
            self.last_pos = xy
            self.last_ts = ts
            self.velocity_cm_s = [0.0, 0.0]
            self.sample_count = 1
            return True

        raw_vx = (xy[0] - self.last_pos[0]) / dt
        raw_vy = (xy[1] - self.last_pos[1]) / dt
        raw_speed = math.hypot(raw_vx, raw_vy)
        if raw_speed > self.max_speed_cm_s:
            self.last_pos = xy
            self.last_ts = ts
            self.velocity_cm_s = [0.0, 0.0]
            self.sample_count = 1
            return True

        if self.sample_count < 2:
            self.velocity_cm_s = [raw_vx, raw_vy]
        else:
            alpha = self.velocity_alpha
            self.velocity_cm_s = [
                self.velocity_cm_s[0] * (1.0 - alpha) + raw_vx * alpha,
                self.velocity_cm_s[1] * (1.0 - alpha) + raw_vy * alpha,
            ]
        self.last_pos = xy
        self.last_ts = ts
        self.sample_count += 1
        return True

    def _empty_prediction(self, reason: str) -> dict:
        current = None if self.last_pos is None else [round(self.last_pos[0], 2), round(self.last_pos[1], 2)]
        return {
            "valid": False,
            "reason": reason,
            "field_pos": list(BALL_NOT_FOUND),
            "current_field_pos": current,
            "velocity_cm_s": [round(self.velocity_cm_s[0], 2), round(self.velocity_cm_s[1], 2)],
            "speed_cm_s": round(math.hypot(self.velocity_cm_s[0], self.velocity_cm_s[1]), 2),
            "horizon_s": 0.0,
            "age_s": None,
            "confidence": 0.0,
            "clamped": False,
            "samples": self.sample_count,
        }

    def predict(
        self,
        *,
        horizon_s: float = BALL_PREDICT_DEFAULT_HORIZON_S,
        now_s: float | None = None,
        max_age_s: float = BALL_PREDICT_MAX_AGE_S,
        min_speed_cm_s: float = BALL_PREDICT_MIN_SPEED_CM_S,
        max_horizon_s: float = BALL_PREDICT_MAX_HORIZON_S,
        field_long_cm: float | None = None,
        field_short_cm: float | None = None,
        field_margin_cm: float = 0.0,
    ) -> dict:
        if self.last_pos is None or self.last_ts is None:
            return self._empty_prediction("no_history")

        now = time.monotonic() if now_s is None else float(now_s)
        if not math.isfinite(now):
            return self._empty_prediction("bad_time")

        max_age = max(0.001, float(max_age_s))
        age = max(0.0, now - self.last_ts)
        if age > max_age:
            result = self._empty_prediction("stale")
            result["age_s"] = round(age, 3)
            return result

        horizon = clamp(float(horizon_s), 0.0, max(0.0, float(max_horizon_s)))
        vx, vy = self.velocity_cm_s
        speed = math.hypot(vx, vy)
        if self.sample_count < 2:
            reason = "warming_up"
            lead_s = 0.0
            base_confidence = 0.35
        elif speed < max(0.0, float(min_speed_cm_s)):
            reason = "stationary"
            lead_s = 0.0
            base_confidence = 0.55
        else:
            reason = "ok"
            lead_s = age + horizon
            base_confidence = 0.90

        pred_x = self.last_pos[0] + vx * lead_s
        pred_y = self.last_pos[1] + vy * lead_s
        clamped = False
        if field_long_cm is not None and field_short_cm is not None:
            half_long = max(0.0, float(field_long_cm) / 2.0 - max(0.0, float(field_margin_cm)))
            half_short = max(0.0, float(field_short_cm) / 2.0 - max(0.0, float(field_margin_cm)))
            next_x = clamp(pred_x, -half_long, half_long)
            next_y = clamp(pred_y, -half_short, half_short)
            clamped = abs(next_x - pred_x) > 1e-6 or abs(next_y - pred_y) > 1e-6
            pred_x, pred_y = next_x, next_y

        age_factor = max(0.0, 1.0 - age / max_age)
        confidence = base_confidence * age_factor
        if clamped:
            confidence *= 0.85

        return {
            "valid": True,
            "reason": reason,
            "field_pos": [round(pred_x, 2), round(pred_y, 2)],
            "current_field_pos": [round(self.last_pos[0], 2), round(self.last_pos[1], 2)],
            "velocity_cm_s": [round(vx, 2), round(vy, 2)],
            "speed_cm_s": round(speed, 2),
            "horizon_s": round(horizon, 3),
            "age_s": round(age, 3),
            "confidence": round(confidence, 3),
            "clamped": clamped,
            "samples": self.sample_count,
        }


class Positions:
    def __init__(self):
        # Values for Setup Position System
        self.LidarPos = list(POSE_NOT_FOUND)
        self.ballPos = list(BALL_NOT_FOUND)
        # Values for log system
        self.WarnedLidarCount = 0
        # Values for Adjust the scale of Position System
        self.LidarScale = 10
        self.BoundsScale = 1
        # Values for timer
        self.WaitTime = 3.5
        # Values for configs
        self.FullLog = cfg.Debug.FullLog
        # Vision data (lazy start)
        self.sock: socket.socket | None = None
        self.vision_data = {
            "ball_pos": [0, 0],
            "chassis_pos": [],
            "blue_goal": None,
            "yellow_goal": None,
            "goal_bboxes": [None, None, None, None],
        }
        self.vision_process = None
        self.vision_data_stop_event = threading.Event()
        self.get_vision_data_process: threading.Thread | None = None
        self._vision_started = False
        # Vision should be opt-in on dev/no-hardware runtime; otherwise strategy loops may spawn heavy processes.
        self._pos2pos_record_enabled = bool(getattr(cfg.Debug, "RecordPos2Pos", False))
        self._pos2pos_record_dir = str(getattr(cfg.Debug, "Pos2PosRecordDir", "./Records/pos2pos"))
        self.Position_X_PID = self._build_pos2pos_pid()
        self.Position_Y_PID = self._build_pos2pos_pid()
        self._pose_revision = None
        self._vision_enabled = runtime.default_vision_enabled()
        self._ball_landing_predictor = BallLandingPredictor()

    def _build_pos2pos_pid(self):
        return simple_pid.PID(
            POS2POS_KP,
            POS2POS_KI,
            POS2POS_KD,
            setpoint=0,
            output_limits=(-POS2POS_OUTPUT_LIMIT_MPS, POS2POS_OUTPUT_LIMIT_MPS),
            sample_time=0,
            auto_mode=True,
        )

    def _reset_pos2pos_pid(self):
        self.Position_X_PID.reset()
        self.Position_Y_PID.reset()

    @staticmethod
    def _pos2pos_command_limit(distance_cm: float) -> float:
        """Limit final planar speed as the robot enters the target zone."""

        distance_cm = max(0.0, float(distance_cm))
        if distance_cm >= POS2POS_NEAR_DISTANCE_CM:
            return POS2POS_COMMAND_LIMIT_MPS
        return max(
            POS2POS_NEAR_COMMAND_LIMIT_MPS,
            POS2POS_COMMAND_LIMIT_MPS * distance_cm / POS2POS_NEAR_DISTANCE_CM,
        )

    def _reset_pos2pos_on_pose_change(self):
        bits = getattr(runtime, "Bits", None)
        getter = getattr(bits, "GetPoseRevision", None)
        if not callable(getter):
            return
        revision = int(getter())
        if self._pose_revision is not None and revision != self._pose_revision:
            self._reset_pos2pos_pid()
        self._pose_revision = revision

    def _resolve_speed_gain(self, speed_gain: int | float | None) -> float:
        if speed_gain is None:
            return 1.0
        return clamp(float(speed_gain), 0.2, 6.0)

    @staticmethod
    def _ensure_motion_runtime() -> None:
        """Ensure command paths have a concrete chassis, not only a lazy facade."""

        if runtime.chassis is None:
            runtime.ensure_runtime(auto_start_lidar=True)

    @staticmethod
    def _stop_chassis() -> None:
        """Stop when a chassis is available; pose queries may run pre-init."""

        chassis = runtime.chassis
        if chassis is not None:
            chassis.stop()

    def StartVision(self) -> bool:
        """Start UDP receiver + Vision subprocess lazily.

        Importing LunaPre should not start external processes automatically.
        Call this when you really need vision data.
        """

        # Simulator fast-path: use lightweight vision adapter (no socket/subprocess)
        v = getattr(runtime, "vision", None)
        if v is not None:
            self._vision_started = True
            return True

        if not self._vision_enabled:
            # keep returning default vision_data without starting threads/subprocess
            return False

        if self._vision_started:
            return True
        self._vision_started = True

        # Socket for vision data
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind(("127.0.0.1", 5005))
        except OSError as e:
            logger.warning(f"Vision UDP bind failed: {e}")
        self.sock.settimeout(0.01)

        self.get_vision_data_process = threading.Thread(target=self._get_vision_data)
        self.get_vision_data_process.daemon = True
        self.get_vision_data_process.start()
        return True
        
    def _get_vision_data(self):
        # Backoff for restarting vision subprocess when it's crashing repeatedly.
        restart_backoff_s = 0.2
        last_restart_t = 0.0
        restart_count = 0
        max_restart = 3
        while not self.vision_data_stop_event.is_set():
            time.sleep(0.005)
            try:
                if self.sock is None:
                    time.sleep(0.05)
                    continue
                data, addr = self.sock.recvfrom(4096)
                self.vision_data = pickle.loads(data)
                if isinstance(self.vision_data, dict):
                    self.vision_data.setdefault("ball_pos", [0, 0])
                    self.vision_data.setdefault("chassis_pos", [])
                    self.vision_data.setdefault("blue_goal", None)
                    self.vision_data.setdefault("yellow_goal", None)
                    self.vision_data.setdefault("goal_bboxes", [None, None, None, None])
                # if we can receive packets, reset restart counter
                restart_count = 0

            except (socket.timeout, TimeoutError):
                if not self._vision_enabled:
                    # vision explicitly disabled
                    self.vision_data = {
                        "ball_pos": [0, 0],
                        "chassis_pos": [],
                        "blue_goal": None,
                        "yellow_goal": None,
                        "goal_bboxes": [None, None, None, None],
                    }
                    time.sleep(0.05)
                    continue

                # normal timeout: no vision UDP packets this cycle.
                # Make sure vision subprocess is running.
                if self.vision_process is None or self.vision_process.poll() is not None:
                    # If it keeps dying, stop respawning (prevents resource_tracker leaked semaphores).
                    if restart_count >= max_restart:
                        self._vision_enabled = False
                        logger.warning(f"Vision disabled: subprocess crashed too many times (max_restart={max_restart}).")
                        continue

                    now = time.time()
                    if (now - last_restart_t) < restart_backoff_s:
                        time.sleep(restart_backoff_s)
                    last_restart_t = now
                    project_root = Path(__file__).resolve().parents[2]
                    vision_script = project_root / "src" / "vision_utils" / "Vision.py"
                    if not vision_script.exists():
                        vision_script = project_root / "src" / "Vision.py"
                    self.vision_process = subprocess.Popen([sys.executable, str(vision_script)], cwd=str(project_root))
                    logger.info("Started vision server process")
                    restart_count += 1

                self.vision_data = {
                    "ball_pos": [0, 0],
                    "chassis_pos": [],
                    "blue_goal": None,
                    "yellow_goal": None,
                    "goal_bboxes": [None, None, None, None],
                }

            except OSError as e:
                # Socket closed while thread is still running (e.g. Stop called).
                # Exit gracefully to avoid noisy "Bad file descriptor".
                if self.vision_data_stop_event.is_set():
                    break

                # Some platforms may raise OSError(ETIMEDOUT) for timeouts. Treat it like a timeout.
                if getattr(e, "errno", None) == errno.ETIMEDOUT or str(e).lower().strip() == "timed out":
                    continue

                if getattr(e, "errno", None) == errno.EBADF:
                    break

                logger.warning(f"Vision socket OSError: {e}")
                time.sleep(0.05)
        
    def GetVisionData(self):
        # Simulator fast-path
        v = getattr(runtime, "vision", None)
        if v is not None:
            return v.get_vision_data()

        # ensure vision thread is running
        self.StartVision()
        return self.vision_data

    def _get_ball_pos(self):
        v = getattr(runtime, "vision", None)
        if v is not None:
            return v.get_ball_pos()

        self.StartVision()
        return self.vision_data["ball_pos"]

    def _robot_ball_out_of_bounds(self, ball_pos) -> bool:
        # Vision can occasionally output physically impossible coordinates while
        # losing the ball. This is only a sanity filter; malformed ball_pos should
        # raise immediately instead of being silently repaired here.
        bx = float(ball_pos[0])
        by = float(ball_pos[1])
        if not math.isfinite(bx) or not math.isfinite(by):
            return True

        return (
            abs(bx) > ROBOT_BALL_MAX_ABS_X_CM
            or abs(by) > ROBOT_BALL_MAX_ABS_Y_CM
            or math.hypot(bx, by) > ROBOT_BALL_MAX_DISTANCE_CM
        )
    
    def _get_chassis_pos(self):
        v = getattr(runtime, "vision", None)
        if v is not None:
            try:
                return v.get_chassis_pos()
            except Exception:
                return []

        self.StartVision()
        return self.vision_data["chassis_pos"]

    def _get_goals_pos(self):
        v = getattr(runtime, "vision", None)
        if v is not None:
            try:
                return v.get_goals_pos()
            except Exception:
                return []

        self.StartVision()
        return {
            "blue_goal": self.vision_data.get("blue_goal", None),
            "yellow_goal": self.vision_data.get("yellow_goal", None),
        }

    def _get_goal_bboxes(self):
        v = getattr(runtime, "vision", None)
        if v is not None:
            for method_name in ("GetGoalBBoxes", "get_goal_bboxes"):
                method = getattr(v, method_name, None)
                if callable(method):
                    try:
                        value = method()
                        return list(value) if value else [None, None, None, None]
                    except Exception:
                        continue

        self.StartVision()
        value = self.vision_data.get("goal_bboxes", None)
        if value is None:
            value = self.vision_data.get("goal_bboxes_640", None)
        return list(value) if value else [None, None, None, None]
    
    def Local2Angle(self, AimPos: list) -> int:
        '''
        `非阻塞函数`
        
        AimPos 一个坐标 示例：[0,0]
        '''
        iAimX = AimPos[0]
        iAimY = AimPos[1]
        iLocX, iLocY, _ = self.FieldRobotPose()
        iDeltaX = iAimX - iLocX
        iDeltaY = iAimY - iLocY
        try:
            iDeltaAngle = math.degrees(math.atan2(iDeltaY,iDeltaX))
        except:
            if iDeltaX > 0:
                iDeltaAngle = -90
            else:
                iDeltaAngle = 90
        return int(iDeltaAngle)

    def Pos2Angle(self, lInputPos: list, lAimPos: list) -> int:
        '''
        `非阻塞函数`

        lInputPos 输入坐标
        lAimPos 目标坐标 示例：[0,0]
        '''
        iAimX = lAimPos[0]
        iAimY = lAimPos[1]
        iLocX = lInputPos[0]
        iLocY = lInputPos[1]
        iDeltaX = iAimX - iLocX
        iDeltaY = iAimY - iLocY
        iDeltaAngle = math.degrees(math.atan2(iDeltaY,iDeltaX))
        return round(iDeltaAngle%360,2)

    def PointInShape(self, point, polygon: list[list,list]):
        try:
            x, y = point
        except:
            x, y, w = point
        if len(polygon) == 2:
            x1, y1 = polygon[0][0], polygon[0][1]
            x2, y2 = polygon[1][0], polygon[1][1]
            left, right = min(x1, x2), max(x1, x2)
            bottom, top = min(y1, y2), max(y1, y2)
            polygon = [
                [left, bottom],
                [right, bottom],
                [right, top],
                [left, top],
            ]
        inside = False

        n = len(polygon)
        for i in range(n):
            try:
                x1, y1 = polygon[i]
                x2, y2 = polygon[(i + 1) % n]
            except:
                x1, y1, w1 = polygon[i]
                x2, y2, w2 = polygon[(i + 1) % n]

            # 判断点是否刚好在边上
            if self.point_on_segment(point, (x1, y1), (x2, y2)):
                return True

            # 判断从 point 向右的射线是否和当前边相交
            intersects = (y1 > y) != (y2 > y)

            if intersects:
                x_intersect = x1 + (y - y1) * (x2 - x1) / (y2 - y1)

                if x < x_intersect:
                    inside = not inside
            try:
                if not min(w1,w2) <= w <= max(w1,w2):
                    inside = False  
                    print('angle')
            except:
                pass

        return inside


    def point_on_segment(self, point, a, b):
        try:
            x, y = point
        except:
            x, y, w = point
        x1, y1 = a
        x2, y2 = b

        # 判断三点是否共线
        cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
        if cross != 0:
            return False

        # 判断 point 是否在线段范围内
        return (
            min(x1, x2) <= x <= max(x1, x2)
            and min(y1, y2) <= y <= max(y1, y2)
        )

    # ********* BASIC FUNCTIONS ********
    # ===== Public API (recommended) =====
    def RobotBallPos(self):
        '''
        `非阻塞函数`

        Get the [Relative] Position of the ball

        获取[球]相对于[机器几何中心]的位置
        #### Args:
        
        #### Returns:
            BallPos: [0,0]
        #### Note:
            `0xfff` stand for `NoBallFounded`
            `0xddd` stand for `CatchBall`
        '''
        # 20251017 already changed X,Y dimension.
        # Ball semantics belong to the active vision provider. Runtime only wires
        # providers together; it should not bypass vision with a second Bits path.
        self.ballPos = self._get_ball_pos()
        authoritative_possession = self._authoritative_ball_possession()
        if authoritative_possession is True:
            self.ballPosOut = BALL_HELD
        elif self.ballPos == BALL_HELD or self.ballPos == BALL_NOT_FOUND:
            self.ballPosOut = self.ballPos
        elif self.ballPos == [0, 0]:
            self.ballPosOut = BALL_NOT_FOUND
        elif self._robot_ball_out_of_bounds(self.ballPos):
            self.ballPosOut = BALL_NOT_FOUND
        elif authoritative_possession is False:
            # Simulator status already distinguishes physical possession from a
            # nearby free ball. Do not widen that state through camera tolerance.
            self.ballPosOut = self.ballPos
        else:
            catch_x, catch_y = cfg.Motion.CatchVal
            # Near-field vision noise / dribbler occlusion is often ~1cm.
            # Keep a usable catch window instead of ErrorRange/20 (=0.5cm).
            x_tol = max(1.5, float(cfg.Motion.ErrorRange) / 8.0)
            y_tol = max(2.0, float(cfg.Motion.ErrorRange) / 5.0)
            # Asymmetric: accept slightly closer-than-catch readings so a ball
            # that is almost seated is not forced back into free-ball chasing.
            dx = float(self.ballPos[0]) - float(catch_x)
            dy = abs(float(self.ballPos[1]) - float(catch_y))
            if dy <= y_tol and (-x_tol <= dx <= x_tol * 0.75):
                self.ballPosOut = BALL_HELD
            else:
                self.ballPosOut = self.ballPos
        if getattr(self, "WarnedLidarCount", 0):
            if Settings.Debug.FullLog:
                logger.debug("[Relative] Ball Position: %s"%self.ballPosOut)
        return self.ballPosOut

    @staticmethod
    def _authoritative_ball_possession():
        provider = getattr(runtime, "vision", None)
        getter = getattr(provider, "has_ball", None)
        if not callable(getter):
            return None
        try:
            return bool(getter())
        except Exception:
            return None

    def BallState(self, ball_pos: list | None = None) -> str:
        """Return the semantic ball state: ``lost``, ``held``, or ``free``.

        This is the canonical wrapper for the legacy sentinel values returned by
        ``RobotBallPos()``:

        - ``[0xFFF, 0xFFF]`` means the ball is not visible.
        - ``[0xDDD, 0xDDD]`` means the robot has possession.
        - any other ``[x, y]`` means the ball is free in robot frame.
        """

        if ball_pos is None:
            ball_pos = self.RobotBallPos()
        if ball_pos == BALL_NOT_FOUND:
            return "lost"
        if ball_pos == BALL_HELD:
            return "held"
        return "free"

    def _ball_predictor(self) -> BallLandingPredictor:
        predictor = getattr(self, "_ball_landing_predictor", None)
        if predictor is None:
            predictor = BallLandingPredictor()
            self._ball_landing_predictor = predictor
        return predictor

    def ResetBallLandingPredictor(self) -> None:
        """Clear short-horizon ball prediction history."""

        self._ball_predictor().reset()

    def _observe_field_ball_for_prediction(self, ball_field, timestamp_s: float | None = None) -> None:
        if BallLandingPredictor._sentinel_pair(ball_field, BALL_HELD):
            self.ResetBallLandingPredictor()
            return
        if BallLandingPredictor._sentinel_pair(ball_field, BALL_NOT_FOUND):
            return
        self._ball_predictor().observe(ball_field, timestamp_s=timestamp_s)

    def DirectDistance(self):
        """Return semantic wall ranges [front, left, back, right] in millimetres."""

        service = getattr(runtime, "sensor", None) or getattr(runtime, "lidar", None)
        getter = getattr(service, "DirectDistance", None)
        if not callable(getter):
            from .sensor import LIDAR_DISABLED_DISTS

            return list(LIDAR_DISABLED_DISTS)
        self.BoundsDistance = list(getter())
        return self.BoundsDistance

    def DirectDistanceSnapshot(self):
        """Return the latest semantic wall ranges without movement side effects."""

        service = getattr(runtime, "sensor", None) or getattr(runtime, "lidar", None)
        getter = getattr(service, "DirectDistanceSnapshot", None)
        if callable(getter):
            self.BoundsDistance = list(getter())
            return self.BoundsDistance
        return self.DirectDistance()

    def NearbyLidarObjects(self):
        """Return native near-field clusters in robot coordinates."""

        service = getattr(runtime, "sensor", None) or getattr(runtime, "lidar", None)
        getter = getattr(service, "GetNearbyObjects", None)
        if not callable(getter):
            return []
        try:
            return getter()
        except Exception:
            return []

    def FieldRobotPose(self):
        """Return the configured runtime pose, or ``POSE_NOT_FOUND`` when unavailable.

        The selected Sensor source owns localization. This query has no retry,
        motion, process, or sensor recovery side effects.
        """

        pose = runtime.get_field_pose()
        if pose is None:
            return list(POSE_NOT_FOUND)
        self.LidarPos = list(pose)
        return self.LidarPos

    # Documented API: Robot_Chassis_Pos (robot frame)
    def RobotChassisPos(self):
        """获取底盘在机体坐标系下的位置。

        Returns:
            list: vision server 原始输出（通常为 [[x, y, w, h, conf], ...]）
        """
        return self._get_chassis_pos()

    def RobotGoalsPos(self):
        """获取球门检测结果（蓝/黄球门）。"""
        return self._get_goals_pos()

    @staticmethod
    def _normalize_signed_angle(angle_deg: float) -> float:
        return ((float(angle_deg) + 180.0) % 360.0) - 180.0

    @staticmethod
    def _bearing_deg(x_cm: float, y_cm: float) -> float:
        return Positions._normalize_signed_angle(math.degrees(math.atan2(float(y_cm), float(x_cm))))

    @staticmethod
    def _normalize_robot_xy(item):
        if item is None:
            return None
        try:
            if len(item) < 2:
                return None
            x = float(item[0])
            y = float(item[1])
            if not math.isfinite(x) or not math.isfinite(y):
                return None
            if x == 0.0 and y == 0.0:
                return None
            conf = float(item[2]) if len(item) > 2 else None
            if conf is not None and not math.isfinite(conf):
                conf = None
            return {"x": x, "y": y, "conf": conf, "distance": math.hypot(x, y)}
        except Exception:
            return None

    @staticmethod
    def _format_angle(angle_deg: float) -> float:
        return round(Positions._normalize_signed_angle(angle_deg), 2)

    @staticmethod
    def _format_interval(interval) -> list[float]:
        return [Positions._format_angle(interval[0]), Positions._format_angle(interval[1])]

    @staticmethod
    def _interval_contains(interval, angle_deg: float, eps: float = 1e-6) -> bool:
        return interval[0] - eps <= float(angle_deg) <= interval[1] + eps

    @staticmethod
    def _subtract_interval(intervals: list[list[float]], cut: list[float]) -> list[list[float]]:
        next_intervals = []
        cut_start, cut_end = cut
        for start, end in intervals:
            if cut_end <= start or cut_start >= end:
                next_intervals.append([start, end])
                continue
            if cut_start > start:
                next_intervals.append([start, min(cut_start, end)])
            if cut_end < end:
                next_intervals.append([max(cut_end, start), end])
        return [item for item in next_intervals if (item[1] - item[0]) > 0.25]

    @staticmethod
    def _nearest_unwrapped_angle(angle_deg: float, reference_deg: float) -> float:
        return float(reference_deg) + angle_error_deg(reference_deg, angle_deg)

    @staticmethod
    def _best_open_angle(open_windows: list[list[float]]) -> float | None:
        if not open_windows:
            return None
        for window in open_windows:
            if Positions._interval_contains(window, 0.0):
                return 0.0

        best = None
        best_score = None
        for start, end in open_windows:
            width = max(0.0, end - start)
            inset = min(2.0, width / 2.0)
            if end < 0.0:
                candidate = end - inset
            elif start > 0.0:
                candidate = start + inset
            else:
                candidate = 0.0
            score = (abs(Positions._normalize_signed_angle(candidate)), -width)
            if best is None or score < best_score:
                best = candidate
                best_score = score
        return None if best is None else Positions._normalize_signed_angle(best)

    @staticmethod
    def _empty_shoot_window(target: str, reason: str) -> dict:
        """创建一个空的射门窗口结果字典。

        当无法计算有效的射门窗口时，返回此默认结果。

        Args:
            target: 目标球门标识，可以是 "opponent"（对方球门）、"own"（己方球门）
                   或具体的球门键名如 "blue_goal"、"yellow_goal"
            reason: 无法计算射门窗口的原因字符串

        Returns:
            dict: 空的射门窗口结果字典，包含以下字段：
                - target: 目标球门标识
                - goal_key: 实际选中的球门键名（None表示未找到）
                - goal_color: 球门颜色（None表示未找到）
                - reason: 结果原因说明
                - window_open: 射门窗口是否开启（False表示关闭）
                - can_shoot_now: 是否可以立即射门（False表示不可）
                - raw_window_deg: 原始射门窗口角度范围（None表示无窗口）
                - open_windows_deg: 可用射门窗口列表（空列表表示无可用窗口）
                - best_bearing_deg: 最佳射门方向角度（None表示无最佳方向）
                - turn_error_deg: 需要转向的角度误差（None表示无法计算）
                - target_yaw_deg: 目标航向角（None表示无法计算）
                - goal_distance_cm: 球门距离（厘米）（None表示未知）
                - blockers: 阻挡物列表（空列表表示无阻挡）
        """
        return {
            "target": target,
            "goal_key": None,
            "goal_color": None,
            "reason": reason,
            "window_open": False,
            "can_shoot_now": False,
            "raw_window_deg": None,
            "open_windows_deg": [],
            "best_bearing_deg": None,
            "turn_error_deg": None,
            "target_yaw_deg": None,
            "goal_distance_cm": None,
            "blockers": [],
        }

    def _normalize_goals(self, goals) -> dict:
        if isinstance(goals, dict):
            return {
                "blue_goal": goals.get("blue_goal"),
                "yellow_goal": goals.get("yellow_goal"),
            }
        if isinstance(goals, (list, tuple)):
            return {
                "blue_goal": goals[0] if len(goals) > 0 else None,
                "yellow_goal": goals[1] if len(goals) > 1 else None,
            }
        return {"blue_goal": None, "yellow_goal": None}

    def _goal_bboxes_by_key(self) -> dict:
        result = {"blue_goal": None, "yellow_goal": None}
        try:
            bboxes = self._get_goal_bboxes()
        except Exception:
            bboxes = [None, None, None, None]
        for bbox in bboxes or []:
            try:
                if bbox is None or len(bbox) < 7:
                    continue
                cls_id = int(bbox[1])
                key = "blue_goal" if cls_id == 1 else "yellow_goal" if cls_id == 2 else None
                if key is None:
                    continue
                conf = float(bbox[6])
                if result[key] is None or conf > float(result[key][6]):
                    result[key] = bbox
            except Exception:
                continue
        return result

    def _goal_bbox_sector(self, bbox, camera_fov_deg: float | None = None):
        """从球门边界框计算射门窗口的角度扇区。

        根据球门在相机图像中的边界框位置，计算该球门在机器人坐标系下
        占据的角度范围（扇区）。

        Args:
            bbox: 球门边界框数据，格式为 [camera_id, class_id, x1, y1, x2, y2, conf]
                - camera_id: 相机编号（0=后, 1=左, 2=前, 3=右）
                - class_id: 类别编号（1=蓝色球门, 2=黄色球门）
                - x1, y1, x2, y2: 边界框坐标（像素，640x分辨率）
                - conf: 检测置信度
            camera_fov_deg: 相机视野角度（度），如果为 None 则使用默认值

        Returns:
            dict | None: 扇区信息字典，包含：
                - start: 扇区起始角度（度）
                - end: 扇区结束角度（度）
                - center: 扇区中心角度（度）
                - span: 扇区宽度（度）
            如果边界框无效则返回 None
        """
        if bbox is None or len(bbox) < 7:
            return None
        try:
            cam = int(bbox[0])
            x1 = max(0.0, min(640.0, float(bbox[2])))
            x2 = max(0.0, min(640.0, float(bbox[4])))
        except Exception:
            return None

        base_bearing = SHOOT_WINDOW_CAMERA_BASE_BEARING_DEG.get(cam)
        if base_bearing is None:
            return None
        fov = self._shoot_camera_fov_deg(camera_fov_deg)
        left_x = min(x1, x2)
        right_x = max(x1, x2)

        def bearing_for_x(pixel_x: float) -> float:
            return float(base_bearing) + ((320.0 - pixel_x) / 640.0) * fov

        bearing_a = bearing_for_x(left_x)
        bearing_b = bearing_for_x(right_x)
        start = min(bearing_a, bearing_b)
        end = max(bearing_a, bearing_b)
        return {
            "start": start,
            "end": end,
            "center": (start + end) / 2.0,
            "span": max(0.0, end - start),
        }

    @staticmethod
    def _shoot_goal_width_cm(goal_width_cm: float | None = None) -> float:
        if goal_width_cm is not None:
            return max(1.0, float(goal_width_cm))
        vision_cfg = getattr(Settings, "Vision", object())
        vision_vals = getattr(Settings, "VisionVals", object())
        value = getattr(vision_cfg, "GoalWidth", None)
        if value is None:
            value = getattr(vision_vals, "BlueGoalWidthCM", 90.0)
        return max(1.0, float(value))

    @staticmethod
    def _shoot_chassis_width_cm() -> float:
        vision_cfg = getattr(Settings, "Vision", object())
        vision_vals = getattr(Settings, "VisionVals", object())
        value = getattr(vision_cfg, "ChassisWidth", None)
        if value is None:
            value = getattr(vision_vals, "ChassisWidthCM", 18.0)
        return max(1.0, float(value))

    @staticmethod
    def _shoot_camera_fov_deg(camera_fov_deg: float | None = None) -> float:
        if camera_fov_deg is not None:
            return max(1.0, float(camera_fov_deg))
        vision_cfg = getattr(Settings, "Vision", object())
        value = getattr(vision_cfg, "CameraFOVDeg", SHOOT_WINDOW_DEFAULT_CAMERA_FOV_DEG)
        return max(1.0, float(value))

    @staticmethod
    def _shoot_fallback_goal_distance_cm(fallback_goal_distance_cm: float | None = None) -> float:
        if fallback_goal_distance_cm is not None:
            return max(1.0, float(fallback_goal_distance_cm))
        vision_cfg = getattr(Settings, "Vision", object())
        value = getattr(vision_cfg, "ShootFallbackGoalDistanceCM", None)
        if value is None:
            value = getattr(getattr(Settings, "Bounds", object()), "Long", 260.0)
        return max(1.0, float(value))

    def _goal_window_from_position(self, point: dict, goal_width_cm: float) -> dict | None:
        """根据球门位置和宽度计算射门窗口的角度范围。

        通过球门中心点位置和球门宽度，计算从机器人视角看球门占据的角度范围。
        使用三角函数计算球门边缘相对于球门中心的角度偏移。

        Args:
            point: 球门位置信息字典，包含：
                - x: 球门x坐标（厘米）
                - y: 球门y坐标（厘米）
                - distance: 球门距离（厘米）
                - conf: 检测置信度（可选）
            goal_width_cm: 球门宽度（厘米）

        Returns:
            dict | None: 射门窗口信息字典，包含：
                - start: 窗口起始角度（度）
                - end: 窗口结束角度（度）
                - center: 窗口中心角度（度），即球门方向
                - span: 窗口宽度（度）
            如果距离过近（<=1厘米）则返回 None
        """
        dist = float(point["distance"])
        if dist <= 1.0:
            return None
        bearing = self._bearing_deg(point["x"], point["y"])
        half_width = math.degrees(math.atan2(goal_width_cm / 2.0, dist))
        return {
            "start": bearing - half_width,
            "end": bearing + half_width,
            "center": bearing,
            "span": half_width * 2.0,
        }

    def _collect_shoot_goal_candidates(
        self,
        *,
        goal_width_cm: float | None = None,
        camera_fov_deg: float | None = None,
        min_goal_conf: float = 0.0,
    ) -> list[dict]:
        """收集所有可见球门的候选信息列表。

        遍历蓝色和黄色球门，检测它们的可见性并收集射门窗口相关信息。
        支持从位置坐标和边界框两种来源计算射门窗口。

        Args:
            goal_width_cm: 球门宽度（厘米）
            camera_fov_deg: 相机视野角度（度）
            min_goal_conf: 球门检测的最小置信度阈值

        Returns:
            list[dict]: 球门候选信息列表，每个元素包含：
                - key: 球门键名（"blue_goal" 或 "yellow_goal"）
                - color: 球门颜色（"blue" 或 "yellow"）
                - point: 球门位置信息字典（包含x, y, conf, distance）
                - bbox: 球门边界框（如果有）
                - source: 数据来源（"bbox" 或 "position"）
                - bearing: 球门中心相对于机器人的方向角度（度）
                - window: 射门窗口信息字典
                - distance: 球门距离（厘米）
                - conf: 检测置信度
        """
        goals = self._normalize_goals(self.RobotGoalsPos())
        bboxes = self._goal_bboxes_by_key()
        width = self._shoot_goal_width_cm(goal_width_cm)
        candidates = []

        for key in SHOOT_WINDOW_GOAL_KEYS:
            point = self._normalize_robot_xy(goals.get(key))
            bbox_sector = self._goal_bbox_sector(bboxes.get(key), camera_fov_deg)
            if point is not None and point["conf"] is not None and point["conf"] < float(min_goal_conf):
                point = None
            if point is None and bbox_sector is None:
                continue

            if bbox_sector is not None:
                window = bbox_sector
                source = "bbox"
            else:
                window = self._goal_window_from_position(point, width)
                source = "position"
            if window is None:
                continue

            bearing = self._normalize_signed_angle(window["center"])
            candidates.append({
                "key": key,
                "color": "blue" if key == "blue_goal" else "yellow",
                "point": point,
                "bbox": bboxes.get(key),
                "source": source,
                "bearing": bearing,
                "window": window,
                "distance": None if point is None else point["distance"],
                "conf": None if point is None else point["conf"],
            })
        return candidates

    def _estimated_shoot_goal_candidate(
        self,
        target: str,
        *,
        goal_width_cm: float | None = None,
        fallback_goal_distance_cm: float | None = None,
    ) -> dict | None:
        if target not in ("opponent", "own"):
            return None

        distance = self._shoot_fallback_goal_distance_cm(fallback_goal_distance_cm)
        x = distance if target == "opponent" else -distance
        point = {"x": x, "y": 0.0, "conf": None, "distance": distance}
        window = self._goal_window_from_position(point, self._shoot_goal_width_cm(goal_width_cm))
        if window is None:
            return None
        bearing = self._normalize_signed_angle(window["center"])
        return {
            "key": f"estimated_{target}_goal",
            "color": "estimated",
            "point": point,
            "bbox": None,
            "source": "estimated",
            "bearing": bearing,
            "window": window,
            "distance": distance,
            "conf": None,
        }

    @staticmethod
    def _is_front_goal(candidate: dict) -> bool:
        return abs(float(candidate["bearing"])) <= 90.0

    def _select_shoot_goal(self, candidates: list[dict], target: str):
        target = str(target or "opponent")
        if target in SHOOT_WINDOW_GOAL_KEYS:
            for candidate in candidates:
                if candidate["key"] == target:
                    return candidate, None
            return None, "no_goal_visible"

        front_goals = [item for item in candidates if self._is_front_goal(item)]
        back_goals = [item for item in candidates if not self._is_front_goal(item)]
        if target == "opponent":
            if front_goals:
                return min(front_goals, key=lambda item: abs(float(item["bearing"]))), None
            if back_goals:
                return None, "only_own_goal_visible"
            return None, "no_goal_visible"
        if target == "own":
            if back_goals:
                return min(back_goals, key=lambda item: abs(abs(float(item["bearing"])) - 180.0)), None
            if front_goals:
                return None, "only_opponent_goal_visible"
            return None, "no_goal_visible"
        return None, "invalid_target"

    def _shoot_blocker_windows(
        self,
        raw_window: list[float],
        center_bearing: float,
        goal_distance_cm: float | None,
        blocker_margin_deg: float,
    ) -> tuple[list[list[float]], list[dict]]:
        """计算射门窗口被阻挡物遮挡后的剩余可用窗口。

        遍历所有检测到的对方机器人（底盘），判断它们是否在射门路径上，
        并计算被遮挡的角度范围。通过区间减法算法计算剩余的可射门窗口。

        Args:
            raw_window: 原始射门窗口角度范围 [起始角度, 结束角度]（度）
            center_bearing: 球门中心相对于机器人的方向角度（度）
            goal_distance_cm: 球门距离（厘米），用于判断阻挡物是否在球门前方
            blocker_margin_deg: 阻挡物的额外边距角度（度），用于安全冗余

        Returns:
            tuple[list[list[float]], list[dict]]:
                - list[list[float]]: 剩余可用射门窗口列表，每个元素为 [起始角度, 结束角度]
                - list[dict]: 阻挡物信息列表，每个元素包含：
                    - position: 阻挡物位置 [x_cm, y_cm]
                    - bearing_deg: 阻挡物相对于机器人的方向角度（度）
                    - distance_cm: 阻挡物距离（厘米）
                    - window_deg: 被该阻挡物遮挡的角度范围 [起始, 结束]
                    - conf: 检测置信度

        Note:
            阻挡物的判断条件：
            1. 阻挡物在射门方向上有正投影（即在球门方向前方）
            2. 阻挡物距离小于球门距离（即在机器人与球门之间）
            3. 阻挡物与射门窗口有角度重叠
        """
        open_windows = [list(raw_window)]
        blockers = []
        if goal_distance_cm is None:
            return open_windows, blockers

        chassis_width = self._shoot_chassis_width_cm()
        center_rad = math.radians(float(center_bearing))
        center_cos = math.cos(center_rad)
        center_sin = math.sin(center_rad)

        try:
            chassis_list = self.RobotChassisPos()
        except Exception:
            chassis_list = []

        for item in chassis_list or []:
            point = self._normalize_robot_xy(item)
            if point is None:
                continue
            projection = point["x"] * center_cos + point["y"] * center_sin
            if projection <= 0.0:
                continue
            if goal_distance_cm is not None and projection >= float(goal_distance_cm):
                continue

            bearing = self._bearing_deg(point["x"], point["y"])
            bearing_unwrapped = self._nearest_unwrapped_angle(bearing, center_bearing)
            half_width = math.degrees(math.atan2(chassis_width / 2.0, max(1.0, point["distance"])))
            half_width += max(0.0, float(blocker_margin_deg))
            blocked = [bearing_unwrapped - half_width, bearing_unwrapped + half_width]
            overlap_start = max(raw_window[0], blocked[0])
            overlap_end = min(raw_window[1], blocked[1])
            if overlap_end <= overlap_start:
                continue

            blockers.append({
                "position": [round(point["x"], 2), round(point["y"], 2)],
                "bearing_deg": self._format_angle(bearing),
                "distance_cm": round(point["distance"], 2),
                "window_deg": self._format_interval([overlap_start, overlap_end]),
                "conf": None if point["conf"] is None else round(point["conf"], 3),
            })
            open_windows = self._subtract_interval(open_windows, [overlap_start, overlap_end])
            if not open_windows:
                break

        return open_windows, blockers

    def ShootWindow(
        self,
        target: str = "opponent",
        *,
        current_yaw_deg: float | None = None,
        goal_width_cm: float | None = None,
        camera_fov_deg: float | None = None,
        fallback_goal_distance_cm: float | None = None,
        use_estimated_goal: bool = True,
        blocker_margin_deg: float = 3.0,
        min_goal_conf: float = 0.0,
    ) -> dict:
        """计算机器人坐标系下的射门窗口。

        该方法用于分析当前机器人朝向目标球门的射门机会，计算射门窗口的角度范围、
        是否被阻挡、以及最佳射门方向等信息。

        角度采用机器人相对坐标系（度）：
        - 0度 表示正前方
        - 正值 表示左侧（如 +90度 表示正左方）
        - 负值 表示右侧（如 -90度 表示正右方）
        - 角度范围: [-180, 180]

        Args:
            target: 目标球门标识，可选值：
                - "opponent": 对方球门（默认，通常在前方）
                - "own": 己方球门（通常在后方）
                - "blue_goal": 指定蓝色球门
                - "yellow_goal": 指定黄色球门
            current_yaw_deg: 当前机器人的航向角（度），用于计算目标航向角。
                如果为 None，则 target_yaw_deg 将相对于当前朝向计算。
            goal_width_cm: 球门宽度（厘米），用于计算射门窗口角度。
                如果为 None，则从配置文件读取默认值。
            camera_fov_deg: 相机视野角度（度），用于从边界框计算射门窗口。
                如果为 None，则使用默认值 95.0 度。
            fallback_goal_distance_cm: 当无法检测到球门时，使用的估计球门距离（厘米）。
                如果为 None，则使用场地长度作为默认值。
            use_estimated_goal: 是否在无法检测到球门时使用估计位置。
                True: 使用估计位置计算射门窗口（默认）
                False: 如果检测不到球门则返回空窗口
            blocker_margin_deg: 阻挡物的额外边距角度（度），用于扩大阻挡区域以确保安全。
                默认值为 3.0 度。
            min_goal_conf: 球门检测的最小置信度阈值，低于此值的检测结果将被忽略。
                默认值为 0.0（接受所有检测结果）。

        Returns:
            dict: 射门窗口信息字典，包含以下字段：
                - target (str): 请求的目标球门标识
                - goal_key (str | None): 实际选中的球门键名，如 "blue_goal"、"yellow_goal"
                    或 "estimated_opponent_goal"、"estimated_own_goal"；None 表示未找到
                - goal_color (str | None): 球门颜色，如 "blue"、"yellow"、"estimated"；
                    None 表示未找到
                - reason (str): 结果原因说明，可能的值：
                    - "ok": 射门窗口开启且可以立即射门
                    - "blocked": 射门窗口被阻挡
                    - "need_turn": 射门窗口开启但需要转向
                    - "no_goal_visible": 未检测到球门
                    - "only_own_goal_visible": 只能看到己方球门（当目标是对方球门时）
                    - "only_opponent_goal_visible": 只能看到对方球门（当目标是己方球门时）
                    - "invalid_target": 无效的目标参数
                    - 前缀 "estimated_" 表示使用了估计位置
                - window_open (bool): 射门窗口是否开启（存在可用的射门角度）
                - can_shoot_now (bool): 是否可以立即射门（当前朝向在射门窗口内）
                - raw_window_deg (list[float] | None): 原始射门窗口角度范围 [起始角度, 结束角度]
                    表示不考虑阻挡物时球门占据的角度范围；None 表示无法计算
                - open_windows_deg (list[list[float]]): 可用射门窗口列表
                    每个元素为 [起始角度, 结束角度]，表示一个连续的可射门区域
                - best_bearing_deg (float | None): 最佳射门方向角度（度）
                    优先选择朝向正前方的窗口；None 表示无可用窗口
                - turn_error_deg (float | None): 需要转向的角度误差（度）
                    正值表示需要左转，负值表示需要右转；None 表示无法计算
                - target_yaw_deg (float | None): 目标航向角（度），范围 [0, 360]
                    如果提供了 current_yaw_deg，则 target_yaw_deg = (current_yaw_deg + turn_error_deg) % 360
                    如果未提供 current_yaw_deg，则 target_yaw_deg = turn_error_deg（归一化到 [-180, 180]）
                - goal_distance_cm (float | None): 球门距离（厘米）；None 表示未知
                - blockers (list[dict]): 阻挡物信息列表，每个元素包含：
                    - position (list[float]): 阻挡物位置 [x_cm, y_cm]
                    - bearing_deg (float): 阻挡物相对于机器人的方向角度
                    - distance_cm (float): 阻挡物距离（厘米）
                    - window_deg (list[float]): 被该阻挡物遮挡的角度范围 [起始, 结束]
                    - conf (float | None): 检测置信度

        Example:
            >>> pos = Positions()
            >>> result = pos.ShootWindow("opponent")
            >>> if result["can_shoot_now"]:
            ...     print("可以立即射门！")
            >>> elif result["window_open"]:
            ...     print(f"需要转向 {result['turn_error_deg']:.1f} 度后射门")
            >>> else:
            ...     print("射门窗口被阻挡")
        """

        target = str(target or "opponent")
        candidates = self._collect_shoot_goal_candidates(
            goal_width_cm=goal_width_cm,
            camera_fov_deg=camera_fov_deg,
            min_goal_conf=min_goal_conf,
        )
        if not candidates:
            goal = (
                self._estimated_shoot_goal_candidate(
                    target,
                    goal_width_cm=goal_width_cm,
                    fallback_goal_distance_cm=fallback_goal_distance_cm,
                )
                if use_estimated_goal
                else None
            )
            if goal is None:
                return self._empty_shoot_window(target, "no_goal_visible")
        else:
            goal, reason = self._select_shoot_goal(candidates, target)
            if goal is None:
                goal = (
                    self._estimated_shoot_goal_candidate(
                        target,
                        goal_width_cm=goal_width_cm,
                        fallback_goal_distance_cm=fallback_goal_distance_cm,
                    )
                    if use_estimated_goal and target in ("opponent", "own")
                    else None
                )
                if goal is None:
                    return self._empty_shoot_window(target, reason or "no_goal_visible")

        estimated = goal.get("source") == "estimated"

        center = float(goal["window"]["center"])
        raw_window = [float(goal["window"]["start"]), float(goal["window"]["end"])]
        if raw_window[1] < raw_window[0]:
            raw_window = [raw_window[1], raw_window[0]]

        goal_distance = goal["distance"]
        open_windows, blockers = self._shoot_blocker_windows(
            raw_window,
            center,
            goal_distance,
            blocker_margin_deg,
        )
        best_bearing = self._best_open_angle(open_windows)
        window_open = best_bearing is not None
        can_shoot_now = bool(window_open and any(self._interval_contains(window, 0.0) for window in open_windows))
        if not window_open:
            reason = "blocked"
        elif can_shoot_now:
            reason = "ok"
        else:
            reason = "need_turn"
        if estimated:
            reason = f"estimated_{reason}"

        if best_bearing is None:
            turn_error = None
            target_yaw = None
        else:
            turn_error = self._normalize_signed_angle(best_bearing)
            if current_yaw_deg is None:
                target_yaw = self._normalize_signed_angle(turn_error)
            else:
                target_yaw = (float(current_yaw_deg) + turn_error) % 360.0

        return {
            "target": target,
            "goal_key": goal["key"],
            "goal_color": goal["color"],
            "reason": reason,
            "window_open": bool(window_open),
            "can_shoot_now": can_shoot_now,
            "raw_window_deg": self._format_interval(raw_window),
            "open_windows_deg": [self._format_interval(window) for window in open_windows],
            "best_bearing_deg": None if best_bearing is None else self._format_angle(best_bearing),
            "turn_error_deg": None if turn_error is None else self._format_angle(turn_error),
            "target_yaw_deg": None if target_yaw is None else round(float(target_yaw), 2),
            "goal_distance_cm": None if goal_distance is None else round(float(goal_distance), 2),
            "blockers": blockers,
        }

    def CanShoot(self, target: str = "opponent", **kwargs) -> bool:
        """Return True when the current heading is inside an open shooting window."""

        return bool(self.ShootWindow(target=target, **kwargs).get("can_shoot_now", False))

    def FieldGoalsPos(self):
        """获取球门在场地坐标系下的位置。"""
        goals = self._get_goals_pos()
        if not goals:
            return {"blue_goal": None, "yellow_goal": None}

        self_pos = self.FieldRobotPose()
        result = {}
        for color in ["blue_goal", "yellow_goal"]:
            goal_pos = goals.get(color)
            if goal_pos is not None:
                angle_to_goal = self.Pos2Angle(self_pos, goal_pos)
                distance_to_goal = math.sqrt((goal_pos[0] - self_pos[0]) ** 2 + (goal_pos[1] - self_pos[1]) ** 2)
                result[color] = {"position": goal_pos, "angle": angle_to_goal, "distance": distance_to_goal}
            else:
                result[color] = None
        return result

    def FieldChassisPos(self):
        '''
        `非阻塞函数`

        获取[对方机器人]相对于[场地几何中心]的位置 [[x_forward,y_left],[x_forward,y_left],[x_forward,y_left]]

        ##### retrun a list of the absloute position of all runtime.chassis
        #### Returns:
            OutputDistanceList: [c1,c2]
                c1: 
                    runtime.chassis X Position
                    runtime.chassis Y Position
                    runtime.chassis height
                    runtime.chassis weidth
        '''
        # TODO 20260503 This method is to be tested.
        ChassisRawList = self._get_chassis_pos()
        SelfX, SelfY, SelfZ = self.FieldRobotPose()
        OutputDistanceList = []
        th = math.radians(float(SelfZ))
        cos_yaw = math.cos(th)
        sin_yaw = math.sin(th)
        for c in ChassisRawList:
            if len(c) < 2:
                continue
            rel_x = float(c[0])
            rel_y = float(c[1])
            dx = rel_x * cos_yaw - rel_y * sin_yaw
            dy = rel_x * sin_yaw + rel_y * cos_yaw
            AbsChassisCache = [
                SelfX + dx,
                SelfY + dy,
                c[2] if len(c) > 2 else 0,
                c[3] if len(c) > 3 else 0
                ]
            OutputDistanceList.append(AbsChassisCache)
        return OutputDistanceList
    
    # ********* UPPER FUNCTIONS ********
    def FieldBallPos(self):
        # TODO 20260503 This method is to be tested.
        '''
        `非阻塞函数`

        获取[球]相对于[场地几何中心]的距离

        Get the [Absolute] Position for the Ball in the Field
        #### Returns:
            BallPos: list | [0,0]

        #### Note:
            `0xfff` stand for `NoBall`
            `0xddd` stand for `CatchBall`
        '''
        ball_pos = self.RobotBallPos()
        ballX, ballY = ball_pos
        ball_state = self.BallState(ball_pos)
        if ball_state == "lost":
            return list(BALL_NOT_FOUND)
        if ball_state == "held":
            self.ResetBallLandingPredictor()
            return list(BALL_HELD)
        else:
            SelfX, SelfY, SelfZ = self.FieldRobotPose()
            # Convert robot-frame relative (x_forward, y_left) into field-frame delta using yaw.
            # yaw=0 => robot forward aligns with +x_forward.
            th = math.radians(float(SelfZ))
            c = math.cos(th)
            s = math.sin(th)
            dx = float(ballX) * c - float(ballY) * s
            dy = float(ballX) * s + float(ballY) * c
            field_ball = [SelfX + dx, SelfY + dy]
            self._observe_field_ball_for_prediction(field_ball)
            return field_ball

    def PredictBallLanding(
        self,
        *,
        field_ball_pos: list | tuple | None = None,
        horizon_s: float = BALL_PREDICT_DEFAULT_HORIZON_S,
        max_age_s: float = BALL_PREDICT_MAX_AGE_S,
        min_speed_cm_s: float = BALL_PREDICT_MIN_SPEED_CM_S,
        max_horizon_s: float = BALL_PREDICT_MAX_HORIZON_S,
        clamp_to_field: bool = True,
        field_margin_cm: float = 0.0,
    ) -> dict:
        """Predict the short-term field-frame ball point for interception.

        The predictor consumes recent ``FieldBallPos()`` observations, estimates
        field-frame ball velocity, and projects a configurable short horizon.
        It keeps recent history through brief lost-ball gaps but resets on held
        ball, because possession changes invalidate the free-ball trajectory.
        """

        try:
            ball_field = self.FieldBallPos() if field_ball_pos is None else field_ball_pos
            self._observe_field_ball_for_prediction(ball_field)
            if BallLandingPredictor._sentinel_pair(ball_field, BALL_HELD):
                result = self._ball_predictor()._empty_prediction("held")
                result["field_pos"] = list(BALL_HELD)
                return result
        except Exception:
            pass

        field_long = None
        field_short = None
        if clamp_to_field:
            field_long = float(getattr(cfg.Bounds, "Long", 260.0))
            field_short = float(getattr(cfg.Bounds, "Short", 200.0))

        return self._ball_predictor().predict(
            horizon_s=horizon_s,
            max_age_s=max_age_s,
            min_speed_cm_s=min_speed_cm_s,
            max_horizon_s=max_horizon_s,
            field_long_cm=field_long,
            field_short_cm=field_short,
            field_margin_cm=field_margin_cm,
        )

    def RobotChassisAngle(self):
        # TODO 20260503 This method is to be tested.
        '''
        `非阻塞函数`

        获取[球]相对于[机器人几何中心]的角度列表
        ##### retrun a list of the relative angle from the robot to the runtime.chassis
        #### Returns:
            [a1,a2,a3,etc]
        '''
        ChassisRawList = self._get_chassis_pos()
        ChassisAngleList = []
        for c in ChassisRawList:
            if c[1] == 0:
                return 0
            try:
                if -int(math.degrees(math.atan2(c[1],c[0])) - 90) < 0:
                    ChassisRltAngle = -int(math.degrees(math.atan2(c[1],c[0])) - 90) + 360
                else:
                    ChassisRltAngle = -int(math.degrees(math.atan2(c[1],c[0])) - 90)
                ChassisAngleList.append(ChassisRltAngle)
            except ZeroDivisionError:
                ChassisAngleList.append(0)
        return ChassisAngleList

    def Pos2Pos(self, AimPos: list, speed_gain: int | float | None = None) -> bool:
        """非阻塞到点控制。

        AimPos 为 [x_cm, y_cm, yaw_deg]。每次调用只发一帧速度：
        距离没到就平移并朝向目标 yaw；位置到了但角度没到就原地转向。

        目标路径穿过禁区时，先走一个有限的中继点。中继点只在当前
        ``Pos2Pos`` 调用内被替换，不会把“到达中继点”误报成到达最终目标；
        下一帧会重新检查从中继点到最终目标的路径。
        """
        self._ensure_motion_runtime()
        loc_x, loc_y, loc_yaw = self.FieldRobotPose()
        if [loc_x, loc_y, loc_yaw] == POSE_NOT_FOUND:
            self._stop_chassis()
            runtime.play_morse("No Position Data!")
            return False

        start = [loc_x, loc_y, loc_yaw]
        AimPos = self._navigate_waypoint(start, AimPos)
        aim_x, aim_y, aim_yaw = AimPos
        aim_yaw = float(aim_yaw) % 360.0
        err_x = float(aim_x) - float(loc_x)
        err_y = float(aim_y) - float(loc_y)
        err_yaw = angle_error_deg(loc_yaw, aim_yaw)
        dist = math.hypot(err_x, err_y)
        arrival_cm = max(1.0, float(Settings.Motion.ErrorRange) * 0.5)

        # A recovery target can intentionally be inside Pos2Pos's normal
        # arrival window.  In that case stopping would leave the robot in the
        # restricted area, so keep translating until the pose itself is safe.
        recovery_escape_pending = (
            self.IsPoseInRestrictedArea(start)
            and not self.IsPoseInRestrictedArea(AimPos)
        )
        if dist <= arrival_cm and not recovery_escape_pending:
            if abs(err_yaw) <= float(Settings.Motion.ErrorRange):
                self._stop_chassis()
                self._reset_pos2pos_pid()
                return True
            self._reset_pos2pos_pid()
            runtime.chassis.FieldTurn(aim_yaw)
            return False

        gain = self._resolve_speed_gain(speed_gain)
        self.Position_X_PID.setpoint = float(aim_x)
        self.Position_Y_PID.setpoint = float(aim_y)
        cmd_x = self.Position_X_PID(float(loc_x)) * gain
        cmd_y = self.Position_Y_PID(float(loc_y)) * gain
        cmd_norm = math.hypot(cmd_x, cmd_y)
        command_limit = self._pos2pos_command_limit(dist)
        if cmd_norm > command_limit:
            scale = command_limit / cmd_norm
            cmd_x *= scale
            cmd_y *= scale
        if recovery_escape_pending:
            # Simulator and some motor controllers have a small planar command
            # deadzone.  A recovery PID command can fall below it while the
            # robot is still technically restricted, leaving the guard active
            # forever.  Keep only this safety escape above the configured
            # minimum; normal position control should retain its fine approach.
            cmd_norm = math.hypot(cmd_x, cmd_y)
            if 0.0 < cmd_norm < POS2POS_MIN_SPEED_MPS:
                scale = POS2POS_MIN_SPEED_MPS / cmd_norm
                cmd_x *= scale
                cmd_y *= scale
        runtime.chassis.FieldMoveVector(cmd_x, cmd_y, aim_yaw)
        return False

    def _path_hits_restricted_area(self, start, target, samples: int = 32) -> bool:
        if not start or not target or len(start) < 3 or len(target) < 3:
            return False
        sample_count = max(2, int(samples))
        # If localization reports that the robot is already outside the legal
        # envelope, allow the initial escape segment. Without this exception
        # every recovery command would be rejected because its first samples
        # necessarily still lie in the current illegal state.
        escaped = not self.IsPoseInRestrictedArea(start)
        for idx in range(1, sample_count + 1):
            t = idx / float(sample_count)
            pose = [
                float(start[0]) + (float(target[0]) - float(start[0])) * t,
                float(start[1]) + (float(target[1]) - float(start[1])) * t,
                float(start[2]) + angle_error_deg(start[2], target[2]) * t,
            ]
            if self.IsPoseInRestrictedArea(pose):
                if escaped:
                    return True
            else:
                escaped = True
        return False

    def _path_has_intermediate_safe_pose(self, start, target, samples: int = 32) -> bool:
        """Return whether an already-illegal recovery path reaches safety early."""

        sample_count = max(2, int(samples))
        for idx in range(1, sample_count):
            t = idx / float(sample_count)
            pose = [
                float(start[0]) + (float(target[0]) - float(start[0])) * t,
                float(start[1]) + (float(target[1]) - float(start[1])) * t,
                float(start[2]) + angle_error_deg(start[2], target[2]) * t,
            ]
            if not self.IsPoseInRestrictedArea(pose):
                return True
        return False

    def _navigate_waypoint(self, start, target):
        safe_target = self.ProjectSafePose(target)
        if safe_target is not None:
            target = safe_target
        path_hits = self._path_hits_restricted_area(start, target)
        start_is_restricted = self.IsPoseInRestrictedArea(start)
        if not path_hits and (
            not start_is_restricted
            or self._path_has_intermediate_safe_pose(start, target)
        ):
            return target

        params = self._restricted_area_params()
        y_room = max(0.0, params["y_limit"] - params["recovery_clearance"])
        y_detour = min(y_room, params["goal_box_y_half"] + params["recovery_clearance"])
        if y_detour <= params["goal_box_y_half"]:
            return target

        arrival_cm = max(1.0, float(Settings.Motion.ErrorRange) * POS2POS_ARRIVAL_SCALE)

        # Follow the near side of the forbidden box one leg at a time. A
        # single point that can see the final target creates a large triangle
        # (and is especially bad for Move2Path). The edge route instead is:
        #   current -> outside-x at current-y
        #   outside-x at current-y -> outside-x at target-y
        #   outside-x at target-y -> target
        # The current/target y values are already clamped when they are close
        # to the field boundary, so even an out-of-range requested point gets
        # a stable route.
        route_sign = 1.0
        route_threshold = max(1.0, params["goal_box_route_x"])
        if abs(float(start[0])) >= route_threshold:
            route_sign = 1.0 if float(start[0]) >= 0.0 else -1.0
        elif abs(float(target[0])) >= route_threshold:
            route_sign = 1.0 if float(target[0]) >= 0.0 else -1.0

        route_x = route_sign * params["goal_box_route_x"]
        boundary_recover_y = max(
            0.0,
            params["y_limit"] - params["boundary_recovery_margin"],
        )
        route_y_limit = max(y_detour, boundary_recover_y)
        route_start_y = clamp(float(start[1]), -route_y_limit, route_y_limit)
        route_target_y = clamp(float(target[1]), -route_y_limit, route_y_limit)

        edge_candidates = [
            [route_x, route_start_y, float(target[2])],
            [route_x, route_target_y, float(target[2])],
        ]
        for waypoint in edge_candidates:
            if math.hypot(float(waypoint[0]) - float(start[0]), float(waypoint[1]) - float(start[1])) <= arrival_cm:
                continue
            if self.ProjectSafePose(waypoint) is None and not self._path_hits_restricted_area(start, waypoint):
                return waypoint

        side = 1.0
        if abs(float(start[1])) > 1.0:
            side = 1.0 if float(start[1]) > 0 else -1.0
        elif abs(float(target[1])) > 1.0:
            side = 1.0 if float(target[1]) > 0 else -1.0

        # Keep the old outer-corridor candidates as a fallback for unusual
        # layouts where the direct edge route is unavailable. The centre
        # corridor is deliberately last because it produces a long triangle.
        candidates = [
            [float(start[0]), side * y_detour, float(target[2])],
            [float(target[0]), side * y_detour, float(target[2])],
            [float(start[0]), -side * y_detour, float(target[2])],
            [float(target[0]), -side * y_detour, float(target[2])],
            [0.0, side * y_detour, float(target[2])],
        ]
        for waypoint in candidates:
            if math.hypot(float(waypoint[0]) - float(start[0]), float(waypoint[1]) - float(start[1])) <= arrival_cm:
                continue
            if self.ProjectSafePose(waypoint) is None and not self._path_hits_restricted_area(start, waypoint):
                return waypoint
        return target

    def Navigate2(self, AimPos: list, speed_gain: int | float | None = None) -> bool:
        """非阻塞简单导航。

        保留这个接口是为了兼容已有策略；``Pos2Pos`` 现在也会执行同样的
        路径检查，避免直接调用 ``Pos2Pos`` 时绕过禁区保护。
        """
        self._ensure_motion_runtime()
        start = self.FieldRobotPose()
        if start == POSE_NOT_FOUND:
            self._stop_chassis()
            runtime.play_morse("No Position Data!")
            return False
        waypoint = self._navigate_waypoint(start, AimPos)
        return self.Pos2Pos(waypoint, speed_gain)

    def Move2Path(
        self,
        Posistions: list,
        iWaitMs: float,
        *,
        timeout_s: float | None = None,
        tick_s: float = 0.02,
    ):
        '''
        `阻塞函数`

        遍历`Posistions`里的每一个点，直到到达最后一个点完成后返回`True`
        
        ### Args:
            Posistions 移动路径列表
            iWaitMs 每次移动后等待时间
        '''
        if not Posistions:
            return True

        point_timeout = float(timeout_s) if timeout_s is not None else 12.0
        tick_s = max(0.005, float(tick_s))

        for idx, target in enumerate(Posistions):
            start_t = time.time()
            while True:
                if self.Pos2Pos(target):
                    time.sleep(max(0.0, float(iWaitMs)))
                    if idx == len(Posistions) - 1:
                        return True
                    break

                if point_timeout > 0 and time.time() - start_t > point_timeout:
                    self._stop_chassis()
                    logger.warning(
                        "Move2Path timeout at point {}/{} target={} current={}",
                        idx + 1,
                        len(Posistions),
                        target,
                        self.FieldRobotPose(),
                    )
                    return False

                time.sleep(tick_s)

    def Cover2Start(self):
        '''
        `阻塞函数`

        直到前方被遮挡后才会返回True

        '''
        while 1:
            if 100 < self.DirectDistance()[0] <= 500:
                return True
            else:
                pass

    def Referee(self):
        '''
        `非阻塞函数`

        获取裁判系统的状态

        #### Returns:
            RefereeStatus: bool | `True` for unlocked, `False` for locked
        '''
        return runtime.get_referee_status()

    def RefereeState(self):
        """Return the structured referee state from LunaPre's input provider."""

        return runtime.get_referee_state()

    def RobotVelocity(self):
        """Return [vx_cm_s, vy_cm_s, wz_deg_s, valid]."""

        return runtime.get_robot_velocity()

    def _restricted_area_params(self) -> dict:
        """Return coordinate-space safety limits in field centimeters."""

        field_long = float(getattr(cfg.Bounds, "Long", 260))
        field_short = float(getattr(cfg.Bounds, "Short", 200))
        safety_cfg = getattr(getattr(cfg, "Motion", None), "RestrictedArea", None)

        x_limit = float(getattr(safety_cfg, "XLimit", min(90.0, field_long / 2.0 - 40.0)))
        y_limit = float(getattr(safety_cfg, "YLimit", min(70.0, field_short / 2.0 - 30.0)))
        goal_box_width = float(getattr(safety_cfg, "GoalBoxWidth", 25.0))
        goal_box_y_half = float(getattr(safety_cfg, "GoalBoxYHalf", 50.0))
        goal_box_corner_radius = float(getattr(safety_cfg, "GoalBoxCornerRadius", 20.0))
        clearance = float(getattr(safety_cfg, "Clearance", max(3.0, cfg.Motion.ErrorRange / 2.0)))
        recovery_clearance = float(getattr(safety_cfg, "RecoveryClearance", 0.0))
        if recovery_clearance <= 0.0:
            arrival_range = float(getattr(cfg.Motion, "ErrorRange", 10.0)) * POS2POS_ARRIVAL_SCALE
            recovery_extra = float(getattr(safety_cfg, "RecoveryExtra", 3.0))
            recovery_clearance = clearance + arrival_range + recovery_extra

        try:
            protection_level = int(getattr(safety_cfg, "ProtectionLevel", 1))
        except (TypeError, ValueError):
            protection_level = 1
        protection_level = 2 if protection_level == 2 else 1

        chassis_cfg = getattr(cfg, "Chassis", None)
        try:
            robot_radius_cm = max(0.0, float(getattr(chassis_cfg, "RobotRadiusM", 0.0)) * 100.0)
        except (TypeError, ValueError):
            robot_radius_cm = 0.0

        clearance = max(0.0, clearance)
        recovery_clearance = max(clearance, recovery_clearance)
        if protection_level == 2:
            # RCJA's out-of-bounds wording judges the *whole* robot. Erode the
            # forbidden shape by the chassis radius so a partial line crossing
            # is possible, while the complete robot cannot fit inside it.
            boundary_margin = clearance - robot_radius_cm
            goal_box_margin = clearance - robot_radius_cm
            boundary_recovery_margin = boundary_margin
            goal_box_escape_margin = clearance
            goal_box_route_x = max(0.0, x_limit - goal_box_width - goal_box_margin)
            goal_box_safe_y = max(0.0, goal_box_y_half + goal_box_margin)
            goal_box_sidestep_y = goal_box_safe_y
        else:
            # Strict mode keeps the robot body outside the marked boundary.
            boundary_margin = clearance + robot_radius_cm
            goal_box_margin = clearance + robot_radius_cm
            boundary_recovery_margin = recovery_clearance
            goal_box_escape_margin = recovery_clearance
            goal_box_route_x = max(0.0, x_limit - goal_box_width - recovery_clearance)
            goal_box_safe_y = max(0.0, goal_box_y_half + goal_box_margin)
            goal_box_sidestep_y = min(
                max(0.0, y_limit - recovery_clearance),
                max(0.0, goal_box_y_half + recovery_clearance),
            )

        goal_box_x_min = max(0.0, x_limit - goal_box_width)

        return {
            "x_limit": max(1.0, x_limit),
            "y_limit": max(1.0, y_limit),
            "protection_level": protection_level,
            "robot_radius_cm": robot_radius_cm,
            "goal_box_x_min": goal_box_x_min,
            "goal_box_y_half": max(0.0, min(goal_box_y_half, y_limit)),
            "goal_box_corner_radius": max(
                0.0,
                min(goal_box_corner_radius, goal_box_width, goal_box_y_half),
            ),
            "clearance": clearance,
            "recovery_clearance": recovery_clearance,
            "boundary_margin": boundary_margin,
            "boundary_recovery_margin": boundary_recovery_margin,
            "goal_box_margin": goal_box_margin,
            "goal_box_escape_margin": goal_box_escape_margin,
            "goal_box_route_x": goal_box_route_x,
            "goal_box_safe_y": goal_box_safe_y,
            "goal_box_sidestep_y": goal_box_sidestep_y,
        }

    @staticmethod
    def _rounded_goal_box_contains(x, y, params: dict, margin: float = 0.0) -> bool:
        """Return whether a point is inside the safety-offset rounded goal box.

        The front two corners of the goal box are quarter circles. A positive
        ``margin`` expands the forbidden shape; a negative margin erodes it.
        The latter is used by the competition mode because the Australian
        rule judges whether the whole robot has entered the area.
        """

        x_abs = abs(float(x))
        y_abs = abs(float(y))
        offset = float(margin)
        x_min = max(0.0, float(params["goal_box_x_min"]) - offset)
        y_half = max(0.0, float(params["goal_box_y_half"]) + offset)
        radius = max(0.0, float(params["goal_box_corner_radius"]) + offset)

        if x_abs <= x_min or y_abs >= y_half:
            return False
        if radius <= 0.0:
            return True

        corner_x = float(params["goal_box_x_min"]) + float(params["goal_box_corner_radius"])
        corner_y = float(params["goal_box_y_half"]) - float(params["goal_box_corner_radius"])
        if x_abs >= corner_x or y_abs <= corner_y:
            return True

        dx = x_abs - corner_x
        dy = y_abs - corner_y
        return dx * dx + dy * dy < radius * radius

    def _rounded_corner_escape(self, x, y, params: dict):
        """Return a short, clearance-aware exit from a rounded goal corner."""

        radius = float(params["goal_box_corner_radius"])
        if radius <= 0.0:
            return None

        x_abs = abs(float(x))
        y_abs = abs(float(y))
        x_min = float(params["goal_box_x_min"])
        y_half = float(params["goal_box_y_half"])
        effective_radius = max(0.0, radius + float(params["goal_box_margin"]))
        sign_x = 1.0 if float(x) >= 0.0 else -1.0
        sign_y = 1.0 if float(y) >= 0.0 else -1.0
        corner_x = sign_x * (x_min + radius)
        corner_y = sign_y * (y_half - radius)
        if x_abs >= abs(corner_x) or y_abs <= abs(corner_y):
            return None

        dx = float(x) - corner_x
        dy = float(y) - corner_y
        distance = math.hypot(dx, dy)
        if distance >= effective_radius:
            return None

        # Use an exit margin so the robot does not stop on the arc and
        # immediately re-enter it on the next PID tick.
        exit_radius = effective_radius + float(params["goal_box_escape_margin"])
        if distance <= 1e-6:
            dx, dy, distance = -sign_x, sign_y, math.sqrt(2.0)
        scale = exit_radius / distance
        target_x = corner_x + dx * scale
        target_y = corner_y + dy * scale

        recover_x = max(
            0.0,
            float(params["x_limit"]) - float(params["boundary_recovery_margin"]),
        )
        recover_y = max(
            0.0,
            float(params["y_limit"]) - float(params["boundary_recovery_margin"]),
        )
        target_x = sign_x * min(abs(target_x), recover_x)
        target_y = sign_y * min(abs(target_y), recover_y)
        return [target_x, target_y]

    def ProjectSafePose(self, pose: list | tuple | None):
        """Project an unsafe field pose to the nearest legal coordinate.

        Returns:
            list | None: a safe `[x, y, yaw]` target when correction is needed,
            otherwise `None`.
        """

        if not pose or len(pose) < 3 or list(pose[:3]) == POSE_NOT_FOUND:
            return None

        x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
        params = self._restricted_area_params()
        x_limit = params["x_limit"]
        y_limit = params["y_limit"]
        goal_box_margin = params["goal_box_margin"]

        guard_x = max(0.0, x_limit - params["boundary_margin"])
        guard_y = max(0.0, y_limit - params["boundary_margin"])
        recover_x = max(0.0, x_limit - params["boundary_recovery_margin"])
        recover_y = max(0.0, y_limit - params["boundary_recovery_margin"])

        target_x = x
        target_y = y
        corrected = False

        if x > guard_x:
            target_x = recover_x
            corrected = True
        elif x < -guard_x:
            target_x = -recover_x
            corrected = True

        if y > guard_y:
            target_y = recover_y
            corrected = True
        elif y < -guard_y:
            target_y = -recover_y
            corrected = True

        in_goal_box = self._rounded_goal_box_contains(
            target_x,
            target_y,
            params,
            margin=goal_box_margin,
        )
        if in_goal_box:
            sign_x = 1.0 if target_x >= 0 else -1.0
            sign_y = 1.0 if target_y >= 0 else -1.0

            retreat_x = sign_x * params["goal_box_route_x"]
            sidestep_limit = recover_y
            sidestep_y = sign_y * min(sidestep_limit, params["goal_box_sidestep_y"])

            retreat_cost = abs(target_x - retreat_x)
            sidestep_cost = abs(target_y - sidestep_y)
            sidestep_has_room = sidestep_limit > params["goal_box_safe_y"]
            candidates = []
            if sidestep_has_room:
                candidates.append([target_x, sidestep_y])
            candidates.append([retreat_x, target_y])

            corner_escape = self._rounded_corner_escape(target_x, target_y, params)
            if corner_escape is not None:
                candidates.append(corner_escape)

            safe_candidates = [
                candidate
                for candidate in candidates
                if not self._rounded_goal_box_contains(
                    candidate[0], candidate[1], params, margin=goal_box_margin
                )
            ]
            if safe_candidates:
                target_x, target_y = min(
                    safe_candidates,
                    key=lambda candidate: math.hypot(
                        candidate[0] - target_x,
                        candidate[1] - target_y,
                    ),
                )
            elif retreat_cost <= sidestep_cost or not sidestep_has_room:
                target_x = retreat_x
            else:
                target_y = sidestep_y
            corrected = True

        if not corrected:
            return None

        return [target_x, target_y, yaw]

    def IsPoseInRestrictedArea(self, pose: list | tuple | None) -> bool:
        """Pure coordinate-space restricted-area predicate."""

        return self.ProjectSafePose(pose) is not None

    def RestrictedArea(self):
        """Coordinate-based boundary/restricted-area guard.

        The legacy method name is kept for strategy compatibility. Internally it
        only consumes `FieldRobotPose()`, so the guard stays stable when the pose
        source changes from runtime.lidar to simulator truth or fused localization.
        """

        robo_pos = self.FieldRobotPose()
        safe_target = self.ProjectSafePose(robo_pos)
        if safe_target is None:
            return True

        self.Pos2Pos(safe_target)
        return False

    def Stop(self):
        '''
        ### Function for stopping the vision data thread
        '''
        try:
            if runtime.chassis is not None:
                runtime.chassis.stop()
        except Exception:
            pass
        try:
            if runtime.peripheral is not None:
                runtime.peripheral.Dribble(False)
        except Exception:
            pass
        try:
            if runtime.lidar is not None:
                runtime.lidar.stop()
        except Exception:
            pass
        try:
            if runtime.chassis is not None:
                runtime.chassis.stop()
        except Exception:
            pass

        self.vision_data_stop_event.set()
        # Close UDP socket after signalling stop. Closing can raise in receiver thread;
        # _get_vision_data handles it.
        try:
            if self.sock is not None:
                try:
                    self.sock.shutdown(socket.SHUT_RDWR)
                except Exception:
                    pass
                self.sock.close()
        except Exception:
            pass

        # Terminate vision subprocess. Prefer graceful terminate first so Vision.py
        # can run its finally-close path and release cameras; escalate to kill if needed.
        if self.vision_process is not None:
            proc = self.vision_process
            try:
                if proc.poll() is None:
                    proc.terminate()
                    try:
                        proc.wait(timeout=1.5)
                    except Exception:
                        proc.kill()
                        try:
                            proc.wait(timeout=1.0)
                        except Exception:
                            pass
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass
            self.vision_process = None

        # best-effort join
        try:
            if self.get_vision_data_process is not None:
                self.get_vision_data_process.join(timeout=0.5)
        except Exception:
            pass

        # Also close in-process simulator/runtime vision adapters if present.
        try:
            vision = getattr(runtime, "vision", None)
            close = getattr(vision, "close", None)
            if callable(close):
                close(join_timeout=0.5)
        except Exception:
            pass
