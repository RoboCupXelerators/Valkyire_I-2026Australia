"""XeleratorSimBits

Unity 模拟器 UDP Bits（XEL v1 协议）。

目标：让上层策略代码通过 LunaPre/chassis/peripheral 的统一接口
控制 Unity 里的机器人，并且能获取：
- yaw（Bits.GetYaw）
- pose（Bits.GetPose）
- ball relative position（Bits.GetBallPos）

同时提供一个 Lidar stub：通过 pose 计算与四面墙的距离。

协议对应：SoccerSimulator/Assets/Scripts/XelProtocol.cs

CommandPacket: <BBHBBH9f  (44 bytes)
StatusPacket : <BBHBBH6f  (32 bytes)

Status 扩展：如果 status 包长度 > 32 bytes，则在尾部额外携带 2 个 float:
    ball_x_forward (cm)
    ball_y_left    (cm)

注意单位：
- Unity 侧 StatusPacket 的 x/y/yaw/vx/vy/wz 由 controller 填充，
  在当前工程中 x,y 是以“场地坐标（米）”输出的；为了兼容 SoccerSoftware 的历史代码，
  这里统一转换为 **cm**（位置/相对球坐标），yaw 为 **deg**。
"""

from __future__ import annotations

import errno
import json
import math
import os
import socket
import struct
import threading
import time
import re
from pathlib import Path
from dataclasses import dataclass
from typing import Callable, Optional


def create_runtime(cfg):
    """Create simulator Bits, lidar, and vision providers from project config."""

    sim_cfg = getattr(cfg, "Sim", None)
    unity_ip = os.environ.get("XEL_UNITY_IP", getattr(sim_cfg, "UnityIP", "127.0.0.1"))

    robot_id = int(
        os.environ.get(
            "XEL_ROBOT_ID",
            str(getattr(sim_cfg, "RobotId", getattr(cfg.RoboInfo, "number", 1))),
        )
    )

    multi_mode = str(getattr(sim_cfg, "MultiRobotMode", "per_port")).lower()
    if multi_mode == "per_port":
        cmd_base = int(getattr(sim_cfg, "CmdPortBase", 20001))
        st_base = int(getattr(sim_cfg, "StatusPortBase", 20002))
        stride = int(getattr(sim_cfg, "PortStride", 10))
        default_cmd_port = cmd_base + (max(robot_id, 1) - 1) * stride
        default_status_port = st_base + (max(robot_id, 1) - 1) * stride
    else:
        default_cmd_port = int(getattr(sim_cfg, "CmdPort", 20001))
        default_status_port = int(getattr(sim_cfg, "StatusPort", 20002))

    cmd_port = int(os.environ.get("XEL_CMD_PORT", str(default_cmd_port)))
    status_port = int(os.environ.get("XEL_STATUS_PORT", str(default_status_port)))

    _apply_sim_env_defaults(cfg, sim_cfg)

    bits = XeleratorSimBits(unity_ip=unity_ip, cmd_port=cmd_port, status_port=status_port, robot_id=robot_id)
    lidar = Lidar(
        bits.GetYaw,
        get_pose_callable=bits.GetPose,
        bounds_long_cm=float(getattr(cfg.Bounds, "Long", 260)),
        bounds_short_cm=float(getattr(cfg.Bounds, "Short", 200)),
        auto_start=True,
    )

    from vision_utils.XeleratorVision import XeleratorVision

    vision = XeleratorVision(bits)
    return bits, lidar, vision


def _apply_sim_env_defaults(cfg, sim_cfg) -> None:
    try:
        sim_twist_from_wheels = bool(
            getattr(sim_cfg, "TwistFromWheels", getattr(cfg.Debug, "SimTwistFromWheels", True))
        )
        sim_wheel_to_v = float(getattr(sim_cfg, "WheelToV", getattr(cfg.Debug, "SimWheelToV", 0.0029321531433504737)))
        sim_wheel_to_wz = float(getattr(sim_cfg, "WheelToWz", getattr(cfg.Debug, "SimWheelToWz", 0.02)))
        sim_yaw_offset_deg = float(getattr(sim_cfg, "YawOffsetDeg", getattr(cfg.Debug, "SimYawOffsetDeg", 0.0)))
        sim_yaw_zero_at_start = bool(
            getattr(sim_cfg, "YawZeroAtStart", getattr(cfg.Debug, "SimYawZeroAtStart", False))
        )
        sim_kick_strength = float(getattr(sim_cfg, "KickStrength", 1.0))
        sim_dribble_scale = float(getattr(sim_cfg, "DribbleStrengthScale", 1.0))
        sim_planar_move_scale = float(getattr(sim_cfg, "PlanarMoveSpeedScale", 1.0))
        sim_vision_noise_level = float(getattr(sim_cfg, "VisionNoiseLevel", 0.0))
        sim_vision_noise_seed = int(getattr(sim_cfg, "VisionNoiseSeed", 0))
    except Exception:
        sim_twist_from_wheels = True
        sim_wheel_to_v = 0.0029321531433504737
        sim_wheel_to_wz = 0.02
        sim_yaw_offset_deg = 0.0
        sim_yaw_zero_at_start = False
        sim_kick_strength = 1.0
        sim_dribble_scale = 1.0
        sim_planar_move_scale = 1.0
        sim_vision_noise_level = 0.0
        sim_vision_noise_seed = 0

    if os.environ.get("XEL_TWIST_FROM_WHEELS") is None:
        os.environ["XEL_TWIST_FROM_WHEELS"] = "1" if sim_twist_from_wheels else "0"
    os.environ.setdefault("XEL_WHEEL_TO_V", str(sim_wheel_to_v))
    os.environ.setdefault("XEL_WHEEL_TO_WZ", str(sim_wheel_to_wz))
    os.environ.setdefault("XEL_YAW_OFFSET_DEG", str(sim_yaw_offset_deg))
    os.environ.setdefault("XEL_YAW_ZERO_AT_START", "1" if sim_yaw_zero_at_start else "0")
    os.environ.setdefault("XEL_SIM_KICK_STRENGTH", str(sim_kick_strength))
    os.environ.setdefault("XEL_SIM_DRIBBLE_SCALE", str(sim_dribble_scale))
    os.environ.setdefault("XEL_SIM_PLANAR_MOVE_SCALE", str(sim_planar_move_scale))
    os.environ.setdefault("XEL_SIM_VISION_NOISE_LEVEL", str(sim_vision_noise_level))
    os.environ.setdefault("XEL_SIM_VISION_NOISE_SEED", str(sim_vision_noise_seed))


@dataclass
class SimStatus:
    ts: float = 0.0
    seq: int = 0
    robot_id: int = 1
    has_ball: bool = False
    # pose in Unity field frame (meters & degrees) before scaling
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_deg: float = 0.0
    vx_mps: float = 0.0
    vy_mps: float = 0.0
    wz_rps: float = 0.0
    # optional relative ball position (meters in Unity, will be converted to cm)
    ball_x_m: float | None = None
    ball_y_m: float | None = None
    # enemy robot positions (relative to this robot, in meters, will be converted to cm)
    # list of (x_forward, y_left) pairs
    enemies: list[tuple[float, float]] | None = None


class XeleratorSimBits:
    """Bits adapter used by `LunaPre` when ROBOCUPX_BITS=SIM."""

    VERSION = 1
    TYPE_COMMAND = 1
    TYPE_STATUS = 2

    CMD_STRUCT = struct.Struct("<BBHBBH9f")  # 44 bytes
    ST_STRUCT = struct.Struct("<BBHBBH6f")  # 32 bytes

    FLAG_USE_WHEELS = 1 << 0
    FLAG_KICK = 1 << 1
    FLAG_DRIBBLE = 1 << 2

    def __init__(
        self,
        *,
        unity_ip: str = "127.0.0.1",
        cmd_port: int = 20001,
        status_port: int = 20002,
        robot_id: int = 1,
        hz: float = 50.0,
    ):
        self.unity_ip = unity_ip
        base_cmd_port = int(cmd_port)
        base_status_port = int(status_port)
        base_robot_id = int(robot_id)

        self._tx = None
        self._rx = None

        bind_error: OSError | None = None
        # 防呆：如果状态端口被占用，自动尝试 robot_id+1、端口+10，最多 4 次。
        # 这样在旧进程残留、多个脚本并行、或 Unity 尚未释放端口时，能更快恢复。
        for attempt in range(4):
            attempt_robot_id = base_robot_id + attempt
            attempt_cmd_port = base_cmd_port + attempt * 10
            attempt_status_port = base_status_port + attempt * 10

            try:
                tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                rx.bind(("0.0.0.0", attempt_status_port))
                rx.settimeout(0.05)

                self._tx = tx
                self._rx = rx
                self.cmd_port = attempt_cmd_port
                self.status_port = attempt_status_port
                self.robot_id = attempt_robot_id & 0xFF

                if attempt > 0:
                    print(
                        f"[XEL_SIM] status_port occupied, auto-fallback to robot_id={self.robot_id} "
                        f"cmd_port={self.cmd_port} status_port={self.status_port} (attempt {attempt + 1}/4)"
                    )
                break
            except OSError as e:
                bind_error = e
                try:
                    tx.close()
                except Exception:
                    pass
                try:
                    rx.close()
                except Exception:
                    pass

                if e.errno != errno.EADDRINUSE:
                    raise
                if attempt == 3:
                    raise

        if self._tx is None or self._rx is None:
            raise bind_error if bind_error is not None else OSError("Failed to initialize UDP sockets")

        self._seq = 0
        self._running = True
        self._status = SimStatus(robot_id=self.robot_id)
        self._status_lock = threading.Lock()

        # current command cache
        self._flags = 0
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._w = (0.0, 0.0, 0.0, 0.0)
        self._kick_strength = float(os.environ.get("XEL_SIM_KICK_STRENGTH", "1.0"))
        self._kick_strength = max(0.0, self._kick_strength)
        self._dribble_scale = float(os.environ.get("XEL_SIM_DRIBBLE_SCALE", "1.0"))
        self._dribble_scale = max(0.0, self._dribble_scale)
        self._dribble_strength = 0.0
        self._send_period = 1.0 / float(hz)
        self._ball_status_fallback = str(os.environ.get("XEL_SIM_BALL_STATUS_FALLBACK", "0")).lower() in {
            "1",
            "true",
            "yes",
        }
        self._ball_observation_max_cm = float(os.environ.get("XEL_SIM_BALL_OBSERVATION_MAX_CM", "350"))
        self._ball_observation_min_cm = float(os.environ.get("XEL_SIM_BALL_OBSERVATION_MIN_CM", "0.25"))
        self._infer_dribbler_possession = str(os.environ.get("XEL_SIM_INFER_DRIBBLER_POSSESSION", "1")).lower() not in {
            "0",
            "false",
            "no",
        }
        self._infer_dribbler_min_strength = float(os.environ.get("XEL_SIM_INFER_DRIBBLER_MIN_STRENGTH", "0.2"))
        self._infer_dribbler_x_min_cm = float(os.environ.get("XEL_SIM_INFER_DRIBBLER_X_MIN_CM", "-2"))
        self._infer_dribbler_x_max_cm = float(os.environ.get("XEL_SIM_INFER_DRIBBLER_X_MAX_CM", "22"))
        self._infer_dribbler_y_abs_cm = float(os.environ.get("XEL_SIM_INFER_DRIBBLER_Y_ABS_CM", "10"))
        self._enemy_scene_filter = str(os.environ.get("XEL_SIM_ENEMY_SCENE_FILTER", "1")).lower() not in {
            "0",
            "false",
            "no",
        }
        self._scene_control = None
        self._scene_status_cache = None
        self._scene_status_cache_ts = 0.0

        # Convert wheel-speed commands (strategy code) into twist for Unity controller.
        self._twist_from_wheels = str(os.environ.get("XEL_TWIST_FROM_WHEELS", "1")).lower() not in {"0", "false", "no"}
        # map strategy wheel units to simulator SI
        self._wheel_to_v = float(os.environ.get("XEL_WHEEL_TO_V", "0.0029321531433504737"))
        self._wheel_to_wz = float(os.environ.get("XEL_WHEEL_TO_WZ", "0.02"))
        # Scale planar translation in simulator to reduce overshoot during chase behaviors
        self._planar_move_scale = float(os.environ.get("XEL_SIM_PLANAR_MOVE_SCALE", "1.0"))
        self._planar_move_scale = max(0.0, self._planar_move_scale)

        self._debug = str(os.environ.get("XEL_DEBUG", "")).lower() in {"1", "true", "yes"}
        self._tx_count = 0
        self._rx_count = 0
        self._last_debug_t = 0.0

        # Lightweight realistic vision disturbance for strategy simulation.
        # Two public knobs are intentionally enough:
        #   XEL_SIM_VISION_NOISE_LEVEL: 0..1, where 0 is deterministic truth.
        #   XEL_SIM_VISION_NOISE_SEED: reproducible pseudo-random seed.
        self._vision_noise_level = self._clamp01_float(os.environ.get("XEL_SIM_VISION_NOISE_LEVEL", "0"))
        self._vision_noise_seed = int(float(os.environ.get("XEL_SIM_VISION_NOISE_SEED", "0")))
        self._vision_rng = None
        if self._vision_noise_level > 0.0:
            import random

            self._vision_rng = random.Random(self._vision_noise_seed)

        # ===== Coordinate calibration (Unity -> SoccerSoftware) =====
        # Unity StatusPacket already uses x_forward=pos.z, y_left=-pos.x in *Unity units*.
        # Unity units are meters for physics, while SoccerSoftware historically uses centimeters.
        # Additionally, in SampleScene the whole field is offset in world space, so we provide
        # an origin shift so that (0,0) is approximately the field center.

        self._unity_to_cm = float(os.environ.get("XEL_UNITY_TO_CM", "100"))

        # origin offset in cm (subtracted from converted pose)
        ox = os.environ.get("XEL_ORIGIN_X_CM", "")
        oy = os.environ.get("XEL_ORIGIN_Y_CM", "")
        if ox.strip() and oy.strip():
            self._origin_x_cm = float(ox)
            self._origin_y_cm = float(oy)
        else:
            # Best-effort: derive origin from Unity SampleScene's `field_fixed` transform.
            # This makes simulator pose align with SoccerSoftware's expectation that field center is (0,0).
            self._origin_x_cm, self._origin_y_cm = self._try_guess_origin_from_unity_scene()

        # optional additional scale (rare): apply after origin shift
        self._pose_scale = float(os.environ.get("XEL_POSE_SCALE", "1"))

        # Yaw calibration:
        # Unity reports yaw as transform.eulerAngles.y (deg, 0..360).
        # Strategy stack assumes a stable field yaw convention. Some prefabs/scenes
        # have a baked rotation or different zero-direction, which can manifest as
        # a constant offset (commonly 45/90 deg).
        self._yaw_offset_deg = float(os.environ.get("XEL_YAW_OFFSET_DEG", "0"))
        self._yaw_zero_at_start = str(os.environ.get("XEL_YAW_ZERO_AT_START", "0")).lower() in {"1", "true", "yes"}
        self._yaw_zero_deg: float | None = None

        # Enemy-side robots are already reset in Unity with a 180 degree heading.
        # Adding another readback offset makes field-frame movement run backward,
        # so keep the default readback physical. The env override is retained for
        # old scenes/calibrations that still need an extra yaw correction.
        side_default_offset = "0"
        self._side_yaw_offset_deg = float(os.environ.get("XEL_SIDE_YAW_OFFSET_DEG", side_default_offset))

        # ===== Match recording support =====
        # Optional callback for recording match data to database.
        # These fields are read by the RX thread, so they must exist before
        # the thread is started.
        self._recorder_callback: Optional[Callable] = None
        self._record_every: int = 1  # Record every N status updates
        self._record_counter: int = 0
        self._recording_enabled: bool = False

        # NOTE: start threads AFTER calibration attributes are ready.
        # Otherwise debug printing in _tx_loop may call GetPose() before these exist.
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self._tx_thread.start()

    # ===== Match recording API =====

    def SetRecorder(self, callback: Optional[Callable] = None):
        """Set a callback function to record status data.
        
        The callback will be called with a dict containing:
        - tick: timestamp
        - robot_id: robot ID
        - x_cm, y_cm, yaw_deg: pose in cm/deg
        - vx, vy, wz: velocities
        - ball_x_cm, ball_y_cm: relative ball position
        - has_ball: ball possession status
        
        Args:
            callback: A callable that accepts a dict, or None to disable recording.
        """
        self._recorder_callback = callback
        self._recording_enabled = callback is not None

    def EnableRecording(self, session_id: str = None, record_every: int = 5):
        """Enable database recording with automatic session management.
        
        Args:
            session_id: Optional session ID. If None, auto-generates from timestamp.
            record_every: Record every N status updates (default 5 to reduce DB load).
        """
        try:
            from ReasonData.data import SetupDatabase, Positions, MatchSession
            
            # Ensure database is initialized
            SetupDatabase()
            
            # Create or get session
            if session_id is None:
                session_id = time.strftime("%Y%m%d_%H%M%S", time.localtime())
            
            self._record_session_id = session_id
            self._record_every = max(1, int(record_every))
            
            # Create session record
            try:
                MatchSession.create_session(session_id, description="Simulator match recording")
            except Exception:
                pass  # Session may already exist
            
            # Create recorder callback
            def _record_to_db(data: dict):
                try:
                    Positions(
                        Tick=data['tick'],
                        RobotId=data['robot_id'],
                        SessionId=self._record_session_id,
                        XPosition=data['x_cm'],
                        YPosition=data['y_cm'],
                        YawDeg=data['yaw_deg'],
                        Vx=data['vx'],
                        Vy=data['vy'],
                        Wz=data['wz'],
                        BallXPosition=data['ball_x_cm'],
                        BallYPosition=data['ball_y_cm'],
                        HasBall=data['has_ball'],
                        CreatedAt=time.time(),
                    ).save()
                except Exception:
                    pass  # Silently ignore DB errors to avoid blocking control loop
            
            self.SetRecorder(_record_to_db)
            
        except ImportError:
            print("[XEL_SIM] Warning: ReasonData not available, recording disabled")
            self._recording_enabled = False

    def StopRecording(self):
        """Stop recording and close the session."""
        if self._recording_enabled and hasattr(self, '_record_session_id'):
            try:
                from ReasonData.data import MatchSession
                session = MatchSession.get_or_none(MatchSession.SessionId == self._record_session_id)
                if session:
                    session.end_session()
            except Exception:
                pass
        
        self.SetRecorder(None)
        self._recording_enabled = False

    def IsRecording(self) -> bool:
        """Check if recording is enabled."""
        return self._recording_enabled

    def GetSessionId(self) -> Optional[str]:
        """Get current recording session ID."""
        return getattr(self, '_record_session_id', None)

    # ===== Bits-like API expected by upper layers =====

    def GetYaw(self) -> float:
        """Return yaw in degrees."""
        with self._status_lock:
            yaw = float(self._status.yaw_deg)

        # lazy capture initial yaw as zero
        if self._yaw_zero_at_start and self._yaw_zero_deg is None:
            self._yaw_zero_deg = yaw

        if self._yaw_zero_deg is not None:
            yaw = yaw - self._yaw_zero_deg

        yaw = yaw + self._yaw_offset_deg
        yaw = yaw + self._side_yaw_offset_deg
        # normalize [0, 360)
        yaw = yaw % 360.0
        return yaw

    def GetKey(self) -> int:
        return 0

    def SetWheelSpeed(self, w1, w2, w3, w4):
        if self._twist_from_wheels:
            w1 = float(w1)
            w2 = float(w2)
            w3 = float(w3)
            w4 = float(w4)

            # Car.SetMotor calls SetWheelSpeed in physical port order:
            #   left-front, left-back, right-front, right-back.
            # RobotMoveVector's logical wheel equations are:
            #   LF=x-y-z, LB=x+y-z, RB=x+y+z, RF=x-y+z.
            # Reconstruct the chassis vector using the incoming physical order.
            sx = (w1 + w2 + w3 + w4) * 0.25
            sy = (-w1 + w2 - w3 + w4) * 0.25
            sz = (-w1 - w2 + w3 + w4) * 0.25

            self.SetTwist(sx * self._wheel_to_v, sy * self._wheel_to_v, sz * self._wheel_to_wz)
            return

        # raw wheel mode: mimic real hardware sign convention
        self._flags |= self.FLAG_USE_WHEELS
        self._w = (float(w1), float(w2), -float(w3), -float(w4))

    def SetTwist(self, vx: float, vy: float, wz: float):
        self._flags &= ~self.FLAG_USE_WHEELS
        self._vx = float(vx) * self._planar_move_scale
        self._vy = float(vy) * self._planar_move_scale
        self._wz = float(wz)

    def VirtualPorts(self, port, status):
        # port=3: kick, port=4: dribble strength
        if int(port) == 3:
            # Kick strength is fixed by config and does not change per call.
            # Any kick command is treated as a one-shot trigger.
            self._flags |= self.FLAG_KICK
            # Restore the configured kick strength every time a kick is triggered.
            # Do NOT depend on the previous command packet value; peripheral.ShootBall()
            # only toggles the port, it does not continuously stream strength.
            self._kick_strength = max(0.0, float(os.environ.get("XEL_SIM_KICK_STRENGTH", "1.0")))
        elif int(port) == 4:
            # Dribble strength = config coefficient * command value [0.0, 1.0]
            try:
                raw = float(status)
            except Exception:
                raw = 0.0
            raw = max(0.0, min(1.0, raw))
            self._dribble_strength = max(0.0, min(1.0, raw * self._dribble_scale))
            if self._dribble_strength > 0:
                self._flags |= self.FLAG_DRIBBLE
            else:
                self._flags &= ~self.FLAG_DRIBBLE

    # ===== extra helpers for simulator =====

    @staticmethod
    def _clamp01_float(value) -> float:
        try:
            v = float(value)
        except Exception:
            return 0.0
        if not math.isfinite(v):
            return 0.0
        return max(0.0, min(1.0, v))

    def _status_planar_speed_mps_locked(self) -> float:
        return math.hypot(float(self._status.vx_mps), float(self._status.vy_mps))

    def _valid_ball_relative_cm(self, bx: float, by: float) -> bool:
        """判断模拟器给出的机器人坐标系球观测是否可信。"""
        return (
            math.isfinite(bx)
            and math.isfinite(by)
            and math.hypot(bx, by) >= self._ball_observation_min_cm
            and abs(bx) <= self._ball_observation_max_cm
            and abs(by) <= self._ball_observation_max_cm
        )

    def _status_ball_relative_cm_locked(self) -> list[float] | None:
        """把状态包里的相对球坐标转成厘米；调用方需持有 ``_status_lock``。"""
        ball_x_m = self._status.ball_x_m
        ball_y_m = self._status.ball_y_m
        if ball_x_m is None or ball_y_m is None:
            return None

        bx = float(ball_x_m) * self._unity_to_cm * self._pose_scale
        by = float(ball_y_m) * self._unity_to_cm * self._pose_scale
        if not self._valid_ball_relative_cm(bx, by):
            return None
        return [bx, by]

    def _infer_has_ball_from_relative(self, ball: list[float] | None) -> bool:
        """SIM 专用兜底：吸球开启且球在捕获窗口内时，补齐 Unity 漏报的持球语义。"""
        if not self._infer_dribbler_possession or ball is None:
            return False
        if self._dribble_strength < self._infer_dribbler_min_strength:
            return False

        bx = float(ball[0])
        by = float(ball[1])
        center_x = 8.5
        x_min = max(float(self._infer_dribbler_x_min_cm), center_x - 0.6)
        x_max = min(float(self._infer_dribbler_x_max_cm), center_x + 4.0)
        y_abs = min(float(self._infer_dribbler_y_abs_cm), 0.6)
        return (
            x_min <= bx <= x_max
            and abs(by) <= y_abs
        )

    def _vision_dropout(self, speed_mps: float, *, multiplier: float = 1.0) -> bool:
        level = self._vision_noise_level
        if level <= 0.0 or self._vision_rng is None:
            return False

        # Dropout rises with robot speed. At ~2 m/s and level=1, ball observations
        # can drop around 30% of frames; enemy observations use a smaller multiplier.
        speed_norm = min(1.0, max(0.0, speed_mps / 2.0))
        p = min(0.85, 0.30 * level * speed_norm * max(0.0, multiplier))
        return self._vision_rng.random() < p

    def _vision_noise_cm(self, *, multiplier: float = 1.0) -> float:
        level = self._vision_noise_level
        if level <= 0.0 or self._vision_rng is None:
            return 0.0
        # Level=1 gives roughly camera-derived position noise, not wild chaos.
        sigma = 5.0 * level * max(0.0, multiplier)
        return self._vision_rng.gauss(0.0, sigma)

    def _apply_noisy_ball_observation(self, value: list[float], speed_mps: float) -> list[float] | list[int]:
        if self._vision_noise_level <= 0.0:
            return value
        if self._vision_dropout(speed_mps):
            return [0xFFF, 0xFFF]

        noisy = [
            float(value[0]) + self._vision_noise_cm(),
            float(value[1]) + self._vision_noise_cm(),
        ]
        return noisy

    def _apply_noisy_enemy_observations(self, values: list[list[float]], speed_mps: float) -> list[list[float]]:
        if self._vision_noise_level <= 0.0 or not values:
            return values
        if self._vision_dropout(speed_mps, multiplier=0.6):
            return []

        noisy = []
        for x_cm, y_cm in values:
            noisy.append([
                float(x_cm) + self._vision_noise_cm(multiplier=1.3),
                float(y_cm) + self._vision_noise_cm(multiplier=1.3),
            ])
        return noisy

    def GetPose(self):
        """Return (x_forward_cm, y_left_cm, yaw_deg)."""
        with self._status_lock:
            x_cm = float(self._status.x_m) * self._unity_to_cm
            y_cm = float(self._status.y_m) * self._unity_to_cm

        # apply origin shift + scale outside the lock
        x_cm = (x_cm - float(self._origin_x_cm)) * self._pose_scale
        y_cm = (y_cm - float(self._origin_y_cm)) * self._pose_scale
        return (x_cm, y_cm, float(self.GetYaw()))

    def HasBall(self) -> bool:
        with self._status_lock:
            if bool(self._status.has_ball):
                return True
            ball = self._status_ball_relative_cm_locked()
            if self._infer_has_ball_from_relative(ball):
                return True

        if self._ball_status_fallback:
            return self._infer_has_ball_from_relative(self._get_scene_ball_pos_fallback())
        return False

    def GetBallPos(self):
        """Return RobotBallPos semantic relative ball position [x, y] in cm.

        - has_ball=True: [0xDDD,0xDDD]
        - no extension provided: [0xFFF,0xFFF]
        - else: [ball_x_cm, ball_y_cm]
        """
        with self._status_lock:
            if self._status.has_ball:
                return [0xDDD, 0xDDD]
            speed_mps = self._status_planar_speed_mps_locked()
            ball = self._status_ball_relative_cm_locked()
            if ball is not None:
                if self._infer_has_ball_from_relative(ball):
                    return [0xDDD, 0xDDD]
                return self._apply_noisy_ball_observation(ball, speed_mps)

        if self._ball_status_fallback:
            fallback = self._get_scene_ball_pos_fallback()
            if fallback is not None:
                if self._infer_has_ball_from_relative(fallback):
                    return [0xDDD, 0xDDD]
                return self._apply_noisy_ball_observation(fallback, 0.0)
        return [0xFFF, 0xFFF]

    def _scene_status_snapshot(self) -> dict | None:
        """返回一份短缓存的 Unity 场景状态，避免高频 HTTP 查询。"""
        try:
            now = time.time()
            if self._scene_status_cache is None or now - self._scene_status_cache_ts > 0.08:
                if self._scene_control is None:
                    from utils.XeleratorSceneControl import XeleratorSceneControl

                    self._scene_control = XeleratorSceneControl(timeout_s=0.08)
                self._scene_status_cache = self._scene_control.status()
                self._scene_status_cache_ts = now

            status = self._scene_status_cache
            if not isinstance(status, dict) or not status.get("ok", False):
                return None
            return status
        except Exception:
            return None

    def _get_scene_ball_pos_fallback(self) -> list[float] | None:
        """Debug-only fallback: synthesize RobotBallPos from scene-control state."""

        try:
            status = self._scene_status_snapshot()
            if status is None:
                return None
            ball = status.get("ball")
            robots = status.get("robots") or []
            if not isinstance(ball, dict):
                return None

            robot = None
            for item in robots:
                if int(item.get("id", -1)) == int(self.robot_id):
                    robot = item
                    break
            if not isinstance(robot, dict) or not robot.get("active", True):
                return None

            dx = float(ball.get("x_cm", 0.0)) - float(robot.get("x_cm", 0.0))
            dy = float(ball.get("y_cm", 0.0)) - float(robot.get("y_cm", 0.0))
            yaw = math.radians(float(robot.get("yaw_deg", 0.0)))
            c = math.cos(yaw)
            s = math.sin(yaw)
            rel_x = dx * c + dy * s
            rel_y = -dx * s + dy * c
            if not self._valid_ball_relative_cm(rel_x, rel_y):
                return None
            return [rel_x, rel_y]
        except Exception:
            return None

    def _scene_enemy_positions(self) -> list[list[float]] | None:
        """从 Unity 场景状态生成真正敌方机器人的机体系相对坐标。"""
        if not self._enemy_scene_filter:
            return None

        try:
            status = self._scene_status_snapshot()
            if status is None:
                return None

            robots = status.get("robots") or []
            self_robot = None
            for item in robots:
                if int(item.get("id", -1)) == int(self.robot_id):
                    self_robot = item
                    break
            if not isinstance(self_robot, dict) or not self_robot.get("active", True):
                return None

            self_id = int(self.robot_id)
            enemy_ids = {3, 4} if self_id in (1, 2) else {1, 2}
            sx = float(self_robot.get("x_cm", 0.0))
            sy = float(self_robot.get("y_cm", 0.0))
            yaw = math.radians(float(self_robot.get("yaw_deg", 0.0)))
            c = math.cos(yaw)
            s = math.sin(yaw)

            enemies: list[list[float]] = []
            for item in robots:
                try:
                    rid = int(item.get("id", -1))
                except Exception:
                    continue
                if rid not in enemy_ids or not item.get("active", True):
                    continue
                dx = float(item.get("x_cm", 0.0)) - sx
                dy = float(item.get("y_cm", 0.0)) - sy
                rel_x = dx * c + dy * s
                rel_y = -dx * s + dy * c
                if math.isfinite(rel_x) and math.isfinite(rel_y):
                    enemies.append([rel_x, rel_y])
            return enemies
        except Exception:
            return None

    def GetEnemyPos(self):
        """Return enemy robot positions relative to this robot.

        Returns:
            list of [x_forward_cm, y_left_cm] pairs for each detected enemy.
            Empty list if no enemies detected or extension not provided.
        
        Positions are relative to this robot's frame (same as ball position).
        """
        scene_enemies = self._scene_enemy_positions()
        with self._status_lock:
            speed_mps = self._status_planar_speed_mps_locked()
            if self._status.enemies is None:
                raw_enemies = []
            else:
                raw_enemies = list(self._status.enemies)

        if scene_enemies is not None:
            return self._apply_noisy_enemy_observations(scene_enemies, speed_mps)
        if not raw_enemies:
            return []

        # Convert to cm (Unity uses meters)
        result = []
        for ex, ey in raw_enemies:
            ex_cm = float(ex) * self._unity_to_cm * self._pose_scale
            ey_cm = float(ey) * self._unity_to_cm * self._pose_scale
            result.append([ex_cm, ey_cm])
        return self._apply_noisy_enemy_observations(result, speed_mps)

    # ===== scene-origin guesser (SIM convenience) =====

    @staticmethod
    def _try_guess_origin_from_unity_scene() -> tuple[float, float]:
        """Try to guess field center offset from Unity SampleScene.unity.

        Returns:
            (origin_x_forward_cm, origin_y_left_cm)

        If parsing fails, returns (0,0).
        """

        try:
            saved = XeleratorSimBits._load_saved_origin_calibration()
            if saved is not None:
                return saved

            # If Unity scene is not available and no saved calibration exists,
            # fall back to zero origin rather than failing startup.
            # This keeps communication and wheel control usable even on machines
            # that only have the Python side or only the Unity binary.
            # The calibration can be filled in later when Unity becomes available.

            # Find repo root robustly instead of assuming a fixed parent depth.
            # This script may live under either:
            #   <repo>/Software/src/utils/XeleratorSimBits.py
            # or a copied tree with a different nesting level.
            # We walk upward and look for a parent that contains the Unity project.
            here = Path(__file__).resolve()
            scene = None
            for parent in here.parents:
                candidates = [
                    parent / "SoccerSimulatorForXels" / "Assets" / "Scenes" / "SampleScene.unity",
                    parent / "SoccerSimulator" / "Assets" / "Scenes" / "SampleScene.unity",
                ]
                scene = next((p for p in candidates if p.exists()), None)
                if scene is not None:
                    break
            if scene is None:
                return (0.0, 0.0)
            txt = scene.read_text(encoding="utf-8", errors="ignore")

            # Find GameObject id for `field_fixed` (scene-level holder)
            m = re.search(r"--- !u!1 &(?P<gid>\d+)\nGameObject:\n(?:(?!\n--- !u!).)*?\n\s*m_Name:\s*field_fixed\s*$", txt, re.S | re.M)
            if not m:
                return (0.0, 0.0)
            gid = m.group("gid")

            # Find its Transform block by scanning all Transform blocks.
            # (The previous regex required m_GameObject to be the last line, which is not true in Unity YAML.)
            block = None
            for b in re.finditer(r"--- !u!4 &(?P<tid>\d+)\nTransform:\n(?:(?!\n--- !u!).)*", txt, re.S):
                bb = b.group(0)
                if f"m_GameObject: {{fileID: {gid}}}" in bb:
                    block = bb
                    break
            if block is None:
                return (0.0, 0.0)

            # Extract localPosition {x:..., y:..., z:...}
            mp = re.search(r"m_LocalPosition:\s*\{x:\s*([\-0-9.eE]+),\s*y:\s*([\-0-9.eE]+),\s*z:\s*([\-0-9.eE]+)\}", block)
            if not mp:
                return (0.0, 0.0)

            x_u = float(mp.group(1))
            z_u = float(mp.group(3))

            # Convert Unity world position into status frame:
            # status.x = XForward = pos.z
            # status.y = YLeft    = -pos.x
            origin_x_forward_cm = z_u * 100.0
            origin_y_left_cm = (-x_u) * 100.0

            # Persist the calibration so future runs don't need Unity present.
            XeleratorSimBits._save_origin_calibration(origin_x_forward_cm, origin_y_left_cm)
            return (origin_x_forward_cm, origin_y_left_cm)
        except Exception:
            return (0.0, 0.0)

    @staticmethod
    def _calibration_file() -> Path:
        """Calibration file path stored inside the Python package tree."""

        base_dir = Path(__file__).resolve().parents[1]  # .../Software/src
        return base_dir / "ReasonData" / "data" / "sim_origin_calibration.json"

    @staticmethod
    def _load_saved_origin_calibration() -> tuple[float, float] | None:
        """Load persisted origin calibration if available."""

        path = XeleratorSimBits._calibration_file()
        try:
            if not path.is_file():
                return None
            data = json.loads(path.read_text(encoding="utf-8"))
            ox = float(data.get("origin_x_forward_cm", 0.0))
            oy = float(data.get("origin_y_left_cm", 0.0))
            return (ox, oy)
        except Exception:
            return None

    @staticmethod
    def _save_origin_calibration(origin_x_forward_cm: float, origin_y_left_cm: float) -> None:
        """Persist origin calibration for Unity-less startup."""

        path = XeleratorSimBits._calibration_file()
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            payload = {
                "origin_x_forward_cm": float(origin_x_forward_cm),
                "origin_y_left_cm": float(origin_y_left_cm),
                "source": "unity_scene",
                "updated_at": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime()),
            }
            path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        except Exception:
            pass

    def Close(self):
        self._running = False
        try:
            self._rx.close()
        except Exception:
            pass
        try:
            self._tx.close()
        except Exception:
            pass

    # ===== internal loops =====

    def _tx_loop(self):
        next_t = time.time()
        while self._running:
            now = time.time()
            if now < next_t:
                time.sleep(min(0.002, next_t - now))
                continue
            next_t = now + self._send_period

            self._seq = (self._seq + 1) & 0xFFFF
            flags = self._flags

            payload = self.CMD_STRUCT.pack(
                self.VERSION,
                self.TYPE_COMMAND,
                self._seq,
                self.robot_id,
                flags & 0xFF,
                self.status_port,
                float(self._vx),
                float(self._vy),
                float(self._wz),
                float(self._w[0]),
                float(self._w[1]),
                float(self._w[2]),
                float(self._w[3]),
                float(self._kick_strength),
                float(self._dribble_strength),
            )
            try:
                self._tx.sendto(payload, (self.unity_ip, self.cmd_port))
                self._tx_count += 1
            except Exception:
                pass

            if self._debug:
                now = time.time()
                if (now - self._last_debug_t) > 1.0:
                    self._last_debug_t = now
                    x, y, yaw = self.GetPose()
                    print(
                        f"[XEL_DEBUG] tx={self._tx_count} rx={self._rx_count} seq={self._seq} "
                        f"to={self.unity_ip}:{self.cmd_port} pose_cm=({x:.1f},{y:.1f},{yaw:.1f}) hasBall={self.HasBall()}"
                    )

            # kick is one-shot
            if (flags & self.FLAG_KICK) != 0:
                self._flags &= ~self.FLAG_KICK
                # keep configured kick strength available for the next trigger
                self._kick_strength = max(0.0, float(os.environ.get("XEL_SIM_KICK_STRENGTH", "1.0")))

    def _rx_loop(self):
        while self._running:
            try:
                data, _ = self._rx.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                break
            except Exception:
                continue

            if not data or len(data) < self.ST_STRUCT.size:
                continue

            try:
                (
                    ver,
                    typ,
                    seq,
                    rid,
                    flags,
                    _reserved,
                    x,
                    y,
                    yaw,
                    vx,
                    vy,
                    wz,
                ) = self.ST_STRUCT.unpack_from(data, 0)
            except Exception:
                continue

            if ver != self.VERSION or typ != self.TYPE_STATUS:
                continue
            if (rid & 0xFF) != self.robot_id:
                continue

            ball_x = None
            ball_y = None
            enemies = None
            
            # Parse ball position extension (8 bytes after base status)
            if len(data) >= self.ST_STRUCT.size + 8:
                try:
                    ball_x, ball_y = struct.unpack_from("<ff", data, self.ST_STRUCT.size)
                except Exception:
                    ball_x, ball_y = None, None
            
            # Parse enemy positions extension (after ball extension)
            # Format: 1 byte enemy_count + N * 8 bytes (enemy positions as floats)
            # Ball extension is 8 bytes, enemy extension starts after that
            ball_ext_size = 8
            enemy_ext_offset = self.ST_STRUCT.size + ball_ext_size
            
            if len(data) >= enemy_ext_offset + 1:
                try:
                    enemy_count = data[enemy_ext_offset]
                    if enemy_count > 0 and len(data) >= enemy_ext_offset + 1 + enemy_count * 8:
                        enemies = []
                        for i in range(enemy_count):
                            offset = enemy_ext_offset + 1 + i * 8
                            ex, ey = struct.unpack_from("<ff", data, offset)
                            enemies.append((float(ex), float(ey)))
                except Exception:
                    enemies = None

            with self._status_lock:
                self._status.ts = time.time()
                self._status.seq = int(seq)
                self._status.robot_id = int(rid)
                self._status.has_ball = (flags & 0x01) != 0
                self._status.x_m = float(x)
                self._status.y_m = float(y)
                self._status.yaw_deg = float(yaw)
                self._status.vx_mps = float(vx)
                self._status.vy_mps = float(vy)
                self._status.wz_rps = float(wz)
                self._status.ball_x_m = ball_x
                self._status.ball_y_m = ball_y
                self._status.enemies = enemies

            self._rx_count += 1

            # Record to database if recording is enabled
            if self._recording_enabled and self._recorder_callback is not None:
                self._record_counter += 1
                if self._record_counter >= self._record_every:
                    self._record_counter = 0
                    try:
                        # Convert to cm for recording
                        x_cm = float(x) * self._unity_to_cm
                        y_cm = float(y) * self._unity_to_cm
                        x_cm = (x_cm - float(self._origin_x_cm)) * self._pose_scale
                        y_cm = (y_cm - float(self._origin_y_cm)) * self._pose_scale
                        
                        ball_x_cm = 0.0
                        ball_y_cm = 0.0
                        if ball_x is not None and ball_y is not None:
                            ball_x_cm = float(ball_x) * self._unity_to_cm * self._pose_scale
                            ball_y_cm = float(ball_y) * self._unity_to_cm * self._pose_scale
                        
                        self._recorder_callback({
                            'tick': time.time(),
                            'robot_id': int(rid),
                            'x_cm': x_cm,
                            'y_cm': y_cm,
                            'yaw_deg': float(yaw),
                            'vx': float(vx),
                            'vy': float(vy),
                            'wz': float(wz),
                            'ball_x_cm': ball_x_cm,
                            'ball_y_cm': ball_y_cm,
                            'has_ball': (flags & 0x01) != 0,
                        })
                    except Exception:
                        pass  # Silently ignore recording errors


class Lidar:
    """Simulator Lidar.

    API subset:
    - GetDists() -> [front, left, back, right]  (mm)
    - stop()

    Distances are computed from pose (x_forward_cm, y_left_cm).
    Field size comes from config (default 260x200 cm).
    """

    def __init__(
        self,
        get_yaw_callable=None,
        *,
        get_pose_callable=None,
        bounds_long_cm: float = 260.0,
        bounds_short_cm: float = 200.0,
        auto_start: bool = True,
    ):
        self._get_pose = get_pose_callable
        self._long = float(bounds_long_cm)
        self._short = float(bounds_short_cm)
        self._running = bool(auto_start)
        self._get_yaw = get_yaw_callable

    def stop(self):
        self._running = False

    def GetDists(self):
        if not self._running or self._get_pose is None:
            return [0xFFF, 0xFFF, 0xFFF, 0xFFF]

        try:
            x_cm, y_cm, yaw_deg = self._get_pose()
        except Exception:
            return [0xFFF, 0xFFF, 0xFFF, 0xFFF]

        # Compute distances in *robot frame* directions [front, left, back, right]
        # by ray-casting against the field rectangle in *field frame*.
        # Field frame:
        #   x_forward in [-long/2, +long/2]
        #   y_left    in [-short/2, +short/2]
        # Yaw convention:
        #   yaw=0 => robot forward aligned with +x_forward.

        x = float(x_cm)
        y = float(y_cm)
        yaw = float(yaw_deg)

        long2 = self._long / 2.0
        short2 = self._short / 2.0

        import math

        def _ray_to_rect_dist_cm(px: float, py: float, dx: float, dy: float) -> float:
            """Distance (cm) from (px,py) to axis-aligned rectangle along direction (dx,dy)."""
            eps = 1e-9
            best_t = None

            # x = ±long2
            if abs(dx) > eps:
                for xb in (-long2, long2):
                    t = (xb - px) / dx
                    if t <= 0:
                        continue
                    yy = py + t * dy
                    if -short2 - 1e-6 <= yy <= short2 + 1e-6:
                        if best_t is None or t < best_t:
                            best_t = t

            # y = ±short2
            if abs(dy) > eps:
                for yb in (-short2, short2):
                    t = (yb - py) / dy
                    if t <= 0:
                        continue
                    xx = px + t * dx
                    if -long2 - 1e-6 <= xx <= long2 + 1e-6:
                        if best_t is None or t < best_t:
                            best_t = t

            if best_t is None:
                # If we're outside the field or direction is degenerate, return 0 (clamped).
                return 0.0
            if best_t < 0:
                return 0.0
            return float(best_t)

        th = math.radians(yaw)
        c = math.cos(th)
        s = math.sin(th)

        # unit vectors in field frame
        f_dx, f_dy = (c, s)            # forward
        l_dx, l_dy = (-s, c)           # left
        b_dx, b_dy = (-c, -s)          # back
        r_dx, r_dy = (s, -c)           # right

        front_cm = _ray_to_rect_dist_cm(x, y, f_dx, f_dy)
        left_cm = _ray_to_rect_dist_cm(x, y, l_dx, l_dy)
        back_cm = _ray_to_rect_dist_cm(x, y, b_dx, b_dy)
        right_cm = _ray_to_rect_dist_cm(x, y, r_dx, r_dy)

        # Return mm (historical lidar): [front, left, back, right]
        return [front_cm * 10.0, left_cm * 10.0, back_cm * 10.0, right_cm * 10.0]
