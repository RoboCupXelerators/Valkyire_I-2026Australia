"""RCJA Australia real-robot strategy; run from Software/src with config.json.

Robot identity comes from RoboInfo.number. Team cooperation is pruned to the
single-robot path automatically when the peer link or heartbeat is unavailable.
"""
from __future__ import annotations

import heapq
import hashlib
import json
import math
import os
import secrets
import signal
import struct
import subprocess
import time
from pathlib import Path
from types import SimpleNamespace as _Namespace

SRC_DIR = Path(__file__).resolve().parent
from ReasonData import Settings

from LunaPre import (
    Communication,
    UnitedPosition,
    chassis,
    cfg,
    ensure_runtime,
    logger,
    peripheral,
    shutdown_runtime,
)
from LunaPre.strategy_tools import (
    BALL_HELD,
    BALL_NOT_FOUND,
    angle_error_deg,
    clamp,
    safe_stop,
    valid_pose,
)

# Australia Open tuning inlined from the former AustraliaStrategy config
# section, so runtime config.json stays clean. Unit suffixes in names are
# centimetres, seconds, m/s or degrees. Field size, goal geometry and ball
# diameter are shared with the global Field/Vision config.

AUSTRALIA = _Namespace(
    Real=_Namespace(
        RCJA_WHITE_LINE_OFFSET_CM=25.0,
        RCJA_WHITE_LINE_WIDTH_CM=5.0,
        REAL_WALL_CLEARANCE_CM=0.05,
        REAL_DETECTION_TOLERANCE_CM=5.0,
        SIDE_MIN_X_CM=20.0,
        SIDE_CONFIRM_S=0.3,
        SIDE_POSITION_TOLERANCE_CM=5.0,
        CONTROL_OVERRUN_LOG_INTERVAL_S=2.0,
        CONTROL_PERF_LOG_INTERVAL_S=5.0,
        CONTROL_PERF_SAMPLE_WINDOW=300,
        REAL_MAINBOARD_RX_TIMEOUT_S=0.3,
        REAL_MAINBOARD_TX_TIMEOUT_S=0.3,
        REAL_POSE_RX_TIMEOUT_S=0.3,
        REAL_REFEREE_RX_TIMEOUT_S=0.3,
        REAL_RUNTIME_TELEMETRY_LOG_INTERVAL_S=5.0,
        REAL_ABSOLUTE_MAX_MPS=2.0,
        REAL_SAFE_FAST_MPS=1.5,
        REAL_ALIGNMENT_MPS=0.55,
        CHASE_MAX_MPS=3.0,
        REAL_DIRECT_COMMAND_HORIZON_S=0.06,
        REAL_POSE_VELOCITY_MIN_SAMPLE_S=0.05,
        REAL_POSE_VELOCITY_MAX_SAMPLE_S=0.4,
        REAL_POSE_VELOCITY_MAX_CM_S=300.0,
        REAL_POSE_VELOCITY_MAX_DEG_S=720.0,
        REAL_BOUNDARY_BRAKE_MPS2=1.8,
        REAL_HOLD_CONFIRM_S=0.25,
        REAL_HOLD_LOSS_GRACE_S=0.15,
        REAL_CARRY_MPS=1.5,
        REAL_BACK_CARRY_MPS=1.5,
        REAL_REPOSITION_MPS=0.75,
        REAL_RELEASE_BACK_MPS=0.35,
        REAL_RELEASE_TIMEOUT_S=0.45,
        REAL_REACQUIRE_TIMEOUT_S=3.5,
        REAL_LOST_WAIT_S=0.3,
        REAL_REACQUIRE_RADIUS_CM=28.0,
        REAL_CARRY_RECOVERY_MAX_ATTEMPTS=2,
        REAL_CARRY_RECOVERY_ARRIVAL_CM=7.0,
        REAL_HELD_BALL_STALL_TIMEOUT_S=0.75,
        REAL_VISION_TIMEOUT_S=0.18,
        REAL_BALL_DIAMETER_CM=float(Settings.Vision.BallDiameter),
    ),
    Team=_Namespace(
        SHOT_ALIGN_DEG=5.0,
        SHOT_COOLDOWN_S=1.2,
        SHOT_GOAL_MARGIN_CM=6.0,
        TEAMMATE_VISION_MATCH_CM=30.0,
        RobotIds=[1, 2],
        RobotRoles=['PRIMARY', 'SUPPORT'],
        ContestStates=['contest_ball', 'contest_carrier_challenge', 'contest_block_helper', 'contest_push_enemy', 'front_defence_challenge'],
    ),
    Tactics=_Namespace(
        SIM_PENALTY_RULE_ENABLED=False,
        SIM_GOAL_WIDTH_CM=45.0,
        SIM_GOAL_DEPTH_CM=7.4,
        SIM_PENALTY_FRONT_CM=70.0,
        SIM_PENALTY_HALF_WIDTH_CM=45.0,
        SIM_PENALTY_RAW_HALF_WIDTH_CM=45.0,
        ATTACK_DIR=None,
        LAST_BALL_MEMORY_S=0.8,
        CHASE_SENSOR_PREDICT_S=0.12,
        SIM_BODY_RADIUS_CM=11.0,
        MOTION_CLEARANCE_CM=5.0,
        BOUNDARY_CONTACT_EDGE_GUARD_CM=30.0,
        BOUNDARY_CONTACT_APPROACH_CM=38.0,
        TEAM_STATE_SEND_PERIOD_S=0.05,
        TEAM_STATE_MAX_AGE_S=0.3,
        TEAMMATE_LOSS_CONFIRM_S=0.75,
        TEAMMATE_STARTUP_TAKEOVER_S=1.0,
        TEAMMATE_HELD_BALL_MATCH_CM=20.0,
        MOTION_MIN_MPS=0.08,
        SIM_MAX_SPEED_MPS=3.0,
        MOTION_COMMAND_BRAKE_MPS2=4.0,
        MOTION_LATENCY_S=0.06,
        MOTION_TELEMETRY_DELAY_S=0.18,
        SIM_RULE_EDGE_EPSILON_CM=0.05,
        HIGH_SPEED_ENEMY_CM_S=35.0,
        HIGH_SPEED_ENEMY_MIN_PROGRESS_CM=-55.0,
        CHASE_ALIGN_DEG=10.0,
        CHASE_TURN_MAX_DEG_S=360.0,
        CHASE_TURN_GAIN=5.0,
        CHASE_LEAD_MAX_S=0.25,
        CHASE_CAPTURE_KP=8.0,
        SIM_DRIBBLER_CAPTURE_RELATIVE_LIMIT_MPS=1.6,
        CHASE_CAPTURE_SLOWDOWN_DISTANCE_M=0.98,
        DIRECT_CARRY_ALIGN_DEG=20.0,
        SCREEN_THREAT_RADIUS_CM=85.0,
        SCREEN_IMMEDIATE_RADIUS_CM=45.0,
        SCREEN_BALL_CLEARANCE_CM=26.0,
        SCREEN_LANE_CLEARANCE_CM=28.0,
        SCREEN_LATERAL_OFFSET_CM=18.0,
        CONTEST_ASSIST_CONTACT_CM=46.0,
        CONTEST_ASSIST_CHASER_CM=48.0,
        CONTEST_SUPPORT_BACK_CM=24.0,
        CONTEST_SUPPORT_SIDE_CM=34.0,
        CONTEST_HELPER_MIN_CM=48.0,
        CONTEST_HELPER_MAX_CM=95.0,
        CONTEST_HELPER_BLOCK_CM=38.0,
        TEAMMATE_HOLD_GRACE_S=0.2,
        HELD_TURN_MAX_DEG_S=120.0,
        TURN_MIN_DEG_S=16.0,
        SHOT_RELEASE_TIMEOUT_S=0.1,
        SHOT_OPENING_ALIGN_MAX_DEG=12.0,
        DIRECT_KICK_POWER_LEVEL=3,
        WALL_KICK_POWER_LEVEL=4,
        MACAU_ENTRY_MIN_ABS_Y_CM=58.0,
        MACAU_LANE_ENEMY_CLEARANCE_CM=58.0,
        MACAU_RETRY_S=1.2,
        MACAU_POCKET_MIN_PROGRESS_CM=98.0,
        MACAU_POCKET_MAX_PROGRESS_CM=108.0,
        MACAU_POCKET_TARGET_PROGRESS_CM=104.0,
        MACAU_POCKET_ARRIVAL_CM=6.0,
        MACAU_POCKET_MIN_ABS_Y_CM=40.0,
        MACAU_POCKET_MAX_ABS_Y_CM=80.0,
        MACAU_POCKET_TARGET_ABS_Y_CM=60.0,
        MACAU_SHIELD_SAMPLE_DEG=15.0,
        MACAU_SHIELD_REPLAN_GAIN=12.0,
        MACAU_SHIELD_MAX_STEP_DEG=36.0,
        MACAU_SHIELD_BODY_MARGIN_CM=2.0,
        MACAU_SHIELD_LANE_STEP_CM=6.0,
        MACAU_SHIELD_REAR_PROGRESS_CM=10.0,
        MACAU_DEFENCE_LANE_MIN_CM=52.0,
        MACAU_DEFENCE_LEAD_CM=32.0,
        MACAU_DEFENCE_TRACK_S=0.45,
        MACAU_DEFENCE_MIN_ADVANCE_CM_S=15.0,
        MACAU_DEFENCE_MAX_LATERAL_SHIFT_CM=16.0,
        MACAU_DEFENCE_HOLD_S=0.35,
        HIDDEN_MACAU_COLLAPSE_S=0.5,
        SPLIT_SHIFT_CM=24.0,
        SPLIT_TIMEOUT_S=4.0,
        WALL_AIM_TIMEOUT_S=1.5,
        FRONT_PRESSURE_DISTANCE_CM=45.0,
        DUEL_BREAKAWAY_CONFIRM_S=0.18,
        DUEL_BREAKAWAY_CONTACT_CM=26.0,
        DUEL_BREAKAWAY_RETREAT_CM=30.0,
        DUEL_BREAKAWAY_MAX_S=0.55,
        DUEL_BREAKAWAY_ARRIVAL_CM=6.0,
        DUEL_EJECT_FALLBACK_S=1.0,
        ENEMY_SAMPLE_MIN_S=0.05,
        ENEMY_SAMPLE_MAX_S=0.4,
        ENEMY_MAX_SPEED_CM_S=300.0,
        SUPPORT_SIDE=-1.0,
        SUPPORT_Y_CM=38.0,
        SUPPORT_BALL_MIN_X_CM=-10.0,
        SUPPORT_CAPTURE_RADIUS_CM=60.0,
        ALLY_CARRIER_CLEARANCE_CM=36.0,
        ALLY_SHOT_LANE_CLEARANCE_CM=34.0,
        SUPPORT_COVER_FORWARD_CM=30.0,
        SUPPORT_COVER_LATERAL_CM=55.0,
        COOP_MIN_PROGRESS_CM=-35.0,
        COOP_SETUP_TIMEOUT_S=2.4,
        COOP_RETRY_S=2.5,
        PASS_READY_S=0.08,
        PASS_READY_DISTANCE_CM=7.0,
        PASS_RECEIVE_ALIGN_DEG=8.0,
        PASS_RECEIVE_SPEED_MPS=0.65,
        PASS_MAX_LAUNCH_MPS=4.0,
        MAX_PASS_CUSHIONS=1,
        PINNED_PASS_CONTACT_CM=26.0,
        PINNED_PASS_MAX_SPEED_MPS=0.03,
        SHOT_MAX_LAUNCH_MPS=4.0,
        PASS_KICK_IMPULSE_MPS=4.0,
        PASS_WALL_RESTITUTION=0.5,
        PASS_RESISTANCE_MODEL='rolling',
        PASS_CALIBRATED_DECEL_MPS2=0.3,
        PASS_CALIBRATED_DRAG_MPS2=0.0,
        PASS_CALIBRATED_BACKSPIN_MPS=0.0,
        PASS_MAX_AGE_S=0.35,
        PASS_FLIGHT_TIMEOUT_S=3.0,
        SHOT_RECEIVE_SPEED_MPS=2.0,
        PASS_SETUP_ESCAPE_CM=18.0,
        PASS_SETUP_SHIELD_RADIUS_CM=55.0,
        PASS_SETUP_TIMEOUT_S=1.25,
        SHIELD_PASS_TIMEOUT_S=0.7,
        SLIP_SIDE_TIMEOUT_S=1.6,
        SLIP_SIDE_ESCAPE_CM=30.0,
        SLIP_SIDE_LATERAL_CM=24.0,
        MAX_CUSHIONS=1,
        FRONT_DEFENCE_X=0.0,
        SOLO_GOAL_THREAT_RADIUS_CM=68.0,
        HELD_ENEMY_OVERLAP_CM=14.0,
        DEFENCE_BALL_ENTER_CM=10.0,
        DEFENCE_RELEASE_DELAY_S=0.6,
        INTERCEPT_HORIZON_S=1.2,
        INTERCEPT_MIN_SPEED_CM_S=15.0,
        INTERCEPT_MAX_SPEED_CM_S=3000.0,
        INTERCEPT_CAPTURE_RADIUS_CM=24.0,
        INTERCEPT_CAPTURE_MAX_SPEED_CM_S=70.0,
        RADICAL_MEMORY_TIME=0.45,
        RADICAL_TRACK_CM=45.0,
        RADICAL_STANDOFF_CM=28.0,
        RADICAL_PREP_FORWARD_CM=14.0,
        RADICAL_PREP_LATERAL_CM=30.0,
        RADICAL_REAR_ENTRY_CM=12.0,
        RADICAL_PREP_ARRIVAL_CM=2.0,
        RADICAL_CONTACT_RUSH_CM=44.0,
        CARRIER_GOAL_LINE_BLOCK_STANDOFF_CM=52.0,
        CARRIER_GOAL_LINE_BLOCK_ARRIVAL_CM=2.0,
        DEADBALL_CONTACT_CM=25.0,
        DEADBALL_SLOT_OCCUPIED_CM=28.0,
        DEADBALL_SLOT_INSET_CM=45.0,
        RADICAL_APPROACH_SPEED_CM_S=10.0,
        RADICAL_NEAR_MIDFIELD_PROGRESS_CM=-25.0,
        PRESS_MAX_MPS=3.0,
        GOAL_BACK_WALL_AIM_MARGIN_CM=4.5,
        GOAL_POST_RETREAT_TRIGGER_CM=34.0,
        GOAL_POST_RETREAT_ARRIVAL_CM=6.0,
        GOAL_POST_PATH_GRID_CM=10.0,
        GOAL_POST_PATH_MAX_DISTANCE_CM=80.0,
        GOAL_POST_PATH_REPLAN_S=1.2,
    ),
    Runtime=_Namespace(
        ControlPeriodSeconds=0.016666666666666666,
        TeamLinkPollIntervalSeconds=0.016666666666666666,
        TeamLinkEnabled=True,
    ),
)

# Dedicated RoboCup Junior Australia Open 2026 physical strategy. Rule 2.1's
# field diagram gives the 2430×1820mm wall envelope. Its 50mm white line is
# 250mm from each wall, so the active white-line interior is 1830×1220mm.
# Bits coordinates use the wall-centred physical envelope; free-ball tactics
# separately reject a ball that has left the white-line playing area.
RCJA_PLAYING_HALF_X_CM = Settings.Field.Long / 2.0 - AUSTRALIA.Real.RCJA_WHITE_LINE_OFFSET_CM - AUSTRALIA.Real.RCJA_WHITE_LINE_WIDTH_CM
RCJA_PLAYING_HALF_Y_CM = Settings.Field.Short / 2.0 - AUSTRALIA.Real.RCJA_WHITE_LINE_OFFSET_CM - AUSTRALIA.Real.RCJA_WHITE_LINE_WIDTH_CM
# Keep the full chassis inside the physical wall envelope, with a small
# numerical margin for field-pose noise.  This is separate from the ball's
# white-line playing-area rule: a robot may legally traverse the out area,
# but it must never be projected beyond the boards.
# The immutable Australia tactical reference has one-cushion wall rebound
# enabled. Keep the REAL profile aligned with that reference; the physical
# executor may still reject a specific route when the observed geometry is
# unsafe.

# IceLoong's real command thread is paced at 60 Hz. Keep tactical refresh at
# the same cadence so each latest vision sample can replace a command without
# accumulating a fixed-sleep drift in the Jetson process.
# REAL motion is projected from SIM requests into the physical envelope. Keep
# the tactical request ceiling equal to the immutable SIM source; the executor
# below is the only layer that applies the physical 1.5/2.0 m/s limits.
# A real camera can lose the orange ball briefly behind a robot or a wall.
# Keep the last fused, field-frame sighting for an aggressive re-acquisition
# run. This is not a synthetic ball observation: it cannot authorize a kick,
# a held-ball branch, or a defence possession decision.
# Established strategy baseline: one immediate chassis command covers this
# command-path horizon. The real API is still refreshed at 60 Hz.
# Established strategy braking baseline. It only determines whether an
# already-outward measured velocity needs an immediate zero command, never a
# general speed derate.
# Hardware constraint: with a held ball the dribbler can retain it only while
# driving along its own forward axis. The requested competition command cap is
# 1.5 m/s in either longitudinal direction. This sits just below the current
# 500 RPM / 29 mm wheel theoretical straight-line command limit (~1.52 m/s);
# it is a deployable established baseline, not a physical-test gate.
# A floor observation may refine it later without changing the strategy's
# interface contract.
# Stable possession uses longitudinal motion only. Heading changes are
# executed by release/reorient/reacquire, never by a carried-ball orbit.
# A frontal carrier contest must persist briefly before it changes an attack.
# The distance envelope is derived from the configured chassis/catcher/ball
# geometry at runtime instead of preserving a scene-specific contact radius.
# Established strategy state-transport baseline. Four independent live camera
# timestamps and a current fusion output are still mandatory every tick.
REAL_FORBIDDEN_MOTION_OVERRIDE_ENV = (
    "ROBOCUPX_SKIP_REFEREE",
    "ROBOCUPX_DEBUG",
)

# Physical coordinate contract used by every real-only adapter below:
# field +x/+y are the wall-centred centimetre axes returned by MainBoard
# localisation; robot +x is forward, +y is left, and +yaw is CCW; strategy
# linear commands are m/s, yaw commands are deg/s, and MainBoard room sizes
# are millimetres.  The team wire format quantises field x/y/yaw to tenths of
# a centimetre/degree and is deliberately independent of the simulator frame.
REAL_TEAM_MAGIC = b"RT"
REAL_TEAM_PROTOCOL_VERSION = 1
REAL_TEAM_FLAG_DRIBBLE_READY = 0x01
REAL_TEAM_FRAME = struct.Struct("<2sBBHBBBBhhh")
REAL_TEAM_MAX_SESSION_ID = 0xFFFF
REAL_TEAM_SEQ_MODULUS = 0x100
REAL_TEAM_SEQ_HALF_RANGE = 0x80

class RealTeamLink:
    """Versioned physical LINK adapter with replay-safe heartbeat freshness.

    MainBoard ``LINK_GET`` is a latest-value read and does not consume data.
    This adapter therefore treats a payload as new only when its bytes change,
    then validates a session/sequence pair.  The first value observed after
    startup or a pair-generation change is a cache baseline, not proof that a
    teammate is alive; the following changed heartbeat is required.
    """

    FRAME = REAL_TEAM_FRAME

    def __init__(self, bits, robot_id, peer_id, *, session_id=None):
        self.bits = bits
        self.robot_id = int(robot_id)
        self.peer_id = int(peer_id)
        if session_id is None:
            session_id = secrets.randbelow(REAL_TEAM_MAX_SESSION_ID) or 1
        self.session_id = int(session_id) & REAL_TEAM_MAX_SESSION_ID
        if self.session_id == 0:
            self.session_id = 1
        self.send_seq = 0
        self.peer_session = None
        self.peer_seq = None
        self._last_raw = None
        self._baseline_required = True
        self._pair_id = None
        self.generation = 0

    @staticmethod
    def _seq_is_newer(current, previous):
        delta = (int(current) - int(previous)) & (REAL_TEAM_SEQ_MODULUS - 1)
        return 0 < delta < REAL_TEAM_SEQ_HALF_RANGE

    @staticmethod
    def encode(*, robot_id, session_id, seq, role, status, flags, x_cm, y_cm, yaw_deg):
        values = (float(x_cm), float(y_cm), float(yaw_deg))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("team heartbeat coordinates must be finite")
        if not 1 <= int(robot_id) <= 2:
            raise ValueError("team heartbeat robot id must be 1 or 2")
        if not 1 <= int(session_id) <= REAL_TEAM_MAX_SESSION_ID:
            raise ValueError("team heartbeat session id must be non-zero")
        quantized = tuple(int(round(value * 10.0)) for value in values)
        if any(not -32768 <= value <= 32767 for value in quantized):
            raise ValueError("team heartbeat coordinate is outside int16 range")
        return RealTeamLink.FRAME.pack(
            REAL_TEAM_MAGIC,
            REAL_TEAM_PROTOCOL_VERSION,
            int(robot_id),
            int(session_id),
            int(seq) & 0xFF,
            int(role) & 0xFF,
            int(status) & 0xFF,
            int(flags) & 0xFF,
            quantized[0], quantized[1], quantized[2],
        )

    @staticmethod
    def decode(raw):
        if isinstance(raw, bytearray):
            raw = bytes(raw)
        if not isinstance(raw, bytes) or len(raw) != REAL_TEAM_FRAME.size:
            return None
        try:
            (magic, version, robot_id, session_id, seq, role, status,
             flags, x_tenth, y_tenth, yaw_tenth) = REAL_TEAM_FRAME.unpack(raw)
        except struct.error:
            return None
        if magic != REAL_TEAM_MAGIC or version != REAL_TEAM_PROTOCOL_VERSION:
            return None
        if robot_id not in (1, 2) or session_id == 0:
            return None
        return {
            "id": int(robot_id),
            "version": int(version),
            "session": int(session_id),
            "seq": int(seq),
            "role": int(role),
            "status": int(status),
            "flags": int(flags),
            "reserve": int(flags),
            "signed_xy": [x_tenth / 10.0, y_tenth / 10.0],
            "yaw": (yaw_tenth / 10.0) % 360.0,
        }

    def _get_pair_id(self):
        for name in ("GetLinkPairId", "GetPeerPairId", "GetPairId"):
            getter = getattr(self.bits, name, None)
            if callable(getter):
                try:
                    value = int(getter())
                except (TypeError, ValueError, OSError, RuntimeError):
                    return None
                return value if 0 <= value <= REAL_TEAM_MAX_SESSION_ID else None
        return None

    def _reset_peer_generation(self, pair_id=None):
        self._pair_id = pair_id
        self.peer_session = None
        self.peer_seq = None
        self._last_raw = None
        self._baseline_required = True
        self.generation += 1

    @property
    def pair_id(self):
        return self._pair_id

    @property
    def paired(self):
        return self._pair_id is None or self._pair_id != 0

    def _nonblocking_reader_ready(self):
        alive = getattr(self.bits, "IsThreadAlive", None)
        if callable(alive):
            try:
                return bool(alive())
            except Exception:
                return False
        return True

    def send_status(self, *, role, status, flags, position, yaw):
        if not self._nonblocking_reader_ready():
            return False
        payload = self.encode(
            robot_id=self.robot_id,
            session_id=self.session_id,
            seq=self.send_seq,
            role=role,
            status=status,
            flags=flags,
            x_cm=position[0],
            y_cm=position[1],
            yaw_deg=yaw,
        )
        self.send_seq = (self.send_seq + 1) & 0xFF
        try:
            return bool(self.bits.SendLinkData(payload, include_pose=False))
        except TypeError:
            # Older local adapters accept only the payload positional arg.
            try:
                return bool(self.bits.SendLinkData(payload))
            except (AttributeError, OSError, RuntimeError, TypeError, ValueError):
                return False
        except (AttributeError, OSError, RuntimeError, ValueError):
            return False

    def poll(self):
        if not self._nonblocking_reader_ready():
            return None
        pair_id = self._get_pair_id()
        if pair_id is not None:
            if pair_id == 0:
                if self._pair_id != 0:
                    self._reset_peer_generation(0)
                return None
            if self._pair_id != pair_id:
                self._reset_peer_generation(pair_id)
        try:
            raw = self.bits.GetLinkData()
        except (AttributeError, OSError, RuntimeError, TypeError, ValueError):
            return None
        if raw is None:
            return None
        try:
            raw = bytes(raw)
        except (TypeError, ValueError):
            return None
        if raw == self._last_raw:
            return None
        # Store every changed raw value, including malformed data, so an
        # invalid cache cannot be reinterpreted as a new heartbeat forever.
        self._last_raw = raw
        frame = self.decode(raw)
        if frame is None or frame["id"] != self.peer_id:
            return None
        if self._baseline_required:
            self._baseline_required = False
            self.peer_session = frame["session"]
            self.peer_seq = frame["seq"]
            return None
        if frame["session"] != self.peer_session:
            self.peer_session = frame["session"]
            self.peer_seq = frame["seq"]
            return frame
        if self.peer_seq is not None and not self._seq_is_newer(frame["seq"], self.peer_seq):
            return None
        self.peer_seq = frame["seq"]
        return frame

    def close(self):
        return None

def _configure_real_safety():
    """Apply the RCJA full-body white-line safety limit to the loaded config.

    RCJA does not make penalty-area entry an automatic damage call. Position
    safety therefore protects the full-body white-line out-area limit only;
    retaining the legacy goal-box guard would reject legal Australian shots
    and defensive challenges. The body radius stays hardware-configured so an
    18cm robot and a larger legal chassis get their own correct limit.
    """

    safety = Settings.Motion.RestrictedArea
    safety.ProtectionLevel = 2
    # At protection level 2 Positions computes its coordinate guard as
    # ``XLimit - (Clearance - robot_radius)``.  XLimit is the physical wall
    # envelope in this wall-centred coordinate system, therefore the configured
    # clearance must include the real chassis radius.  Zero here would expand
    # the permitted centre position one radius *beyond* the wall.  The
    # white-line ball-out policy remains in ``_playing_ball_xy`` and does not
    # constrain legal robot traversal.
    try:
        robot_radius_cm = float(Settings.Chassis.RobotRadiusM) * 100.0
    except (AttributeError, TypeError, ValueError) as exc:
        raise RuntimeError("physical chassis must provide RobotRadiusM") from exc
    if not math.isfinite(robot_radius_cm) or robot_radius_cm <= 0.0:
        raise RuntimeError("physical chassis RobotRadiusM must be positive")
    safety.XLimit = float(Settings.Field.Long) / 2.0 - 0.05
    safety.YLimit = float(Settings.Field.Short) / 2.0 - 0.05
    safety.GoalBoxWidth = 0.0
    safety.GoalBoxYHalf = 0.0
    safety.GoalBoxCornerRadius = 0.0
    safety.Clearance = robot_radius_cm + AUSTRALIA.Real.REAL_WALL_CLEARANCE_CM
    safety.RecoveryClearance = safety.Clearance

def _configure_mainboard_field(bits):
    """Publish the official wall envelope to the real lidar localiser."""

    configure = getattr(bits, "SetRoomSize", None)
    if not callable(configure):
        raise RuntimeError("physical MainBoard does not expose SET_ROOM_SIZE")
    result = configure(Settings.Field.Long * 10.0, Settings.Field.Short * 10.0)
    # IceLoongBits returns the decoded response list.  An empty list means the
    # serial command was not acknowledged; ``None`` remains accepted for thin
    # wrappers whose setter is intentionally fire-and-forget.
    if result is False or (
            isinstance(result, (list, tuple)) and not result
    ):
        raise RuntimeError("MainBoard SET_ROOM_SIZE was rejected")

# MainBoardFirmware maps the physical kicker's four calibrated pulse levels
# (8/10/11/50 ms).  Tactics select only these real controller levels.
# The held ball remains inside the white line, which is 30 cm before the wall
# goal line. Start the escape slightly earlier so a legal carried ball can
# still clear a post-side blockage before its catcher reaches that line.
# Leave one catcher-radius of white-line room for a shield/orbit transition.
MACAU_LANE_Y_CM = min(50.0, RCJA_PLAYING_HALF_Y_CM - 10.0)
# A teammate is declared unavailable only from its own, fresh radio heartbeat.
# This is usable on physical robots (Wi-Fi/BLE) and deliberately contains no
# external field state, referee truth for the other robot, or guessed vision.
# MainBoard LINK_GET must outrun the 20 Hz LINK_SEND heartbeat.  The physical
# command worker performs this poll asynchronously; the strategy thread only
# reads its latest cache and never performs a serial transaction itself.
# Bits pose in a heartbeat can be 16.5 cm old at the permitted top speed;
# leave room for vision localisation error without ever discarding more than
# the one observed chassis that best matches the identified teammate.
TEAM_DRIBBLE_READY_MASK = 0x80
TEAM_STATUS_CONTESTING = 0x02
TEAM_YAW_MASK = TEAM_DRIBBLE_READY_MASK - 1
TEAM_VALID_ROLES = frozenset({
    Communication.ROLE_OFFENCE_YELLOW,
    Communication.ROLE_DEFENCE_YELLOW,
    Communication.ROLE_DEFENCE_BLUE,
    Communication.ROLE_OFFENCE_BLUE,
    Communication.ROLE_BAN,
})
TEAM_VALID_STATUSES = frozenset({
    Communication.STATUS_CATCH_BALL,
    Communication.STATUS_CHASING,
    TEAM_STATUS_CONTESTING,
    Communication.STATUS_AVOIDING,
    Communication.STATUS_NO_BALL,
})
# The IceLoong kicker exposes four discrete levels. These distance bands are
# deliberately provisional: use them only as a bounded initial mapping and
# replace them with measured floor distances before a match.

_MACAU_SINCE = 0.0
_SIDE_CANDIDATE = None
_SIDE_CANDIDATE_SINCE = None
_REAL_HOLD_SINCE = None
_REAL_HOLD_CONFIRMED = False
_REAL_HOLD_LOST_SINCE = None
_REAL_WAS_HELD = False
_REAL_REACQUIRE = None
_REAL_EXECUTOR_GENERATION = 0
_REAL_CARRY_RECOVERY = None
_REAL_LAST_ARC = -math.inf
_MACAU_CARRY_YAW = None
_SPLIT_CARRY_YAW = None
_REAL_POSE_SAMPLE = None
_REAL_ESTIMATED_VELOCITY = None
_REAL_ESTIMATED_VELOCITY_TIME = -math.inf
_BACK_CARRY_YAW = None
_BACK_CARRY_SINCE = None
_BACK_CARRY_PHASE = None
_REAL_DUEL_ENEMY = None
_REAL_DUEL_SINCE = None
TEAM_LINK_DISABLED = False
SOLO_TAKEOVER = False
_TEAM_LINK_GENERATION = 0
# Tactical state is the semantic decision selected by the Australia strategy.
# Executor state describes only how the physical robot is currently carrying
# out that decision; it must never be used to select a different tactic.
_TACTICAL_STATE = None
_TACTICAL_INTENT = None
_TACTICAL_TARGET = None
_POSSESSION_EXECUTOR_STATE = "IDLE"
_TACTICAL_TRACE = []
_HELD_BALL_STALL = None
_MOTION_LAST_REQUESTED_SPEED_MPS = 0.0
_MOTION_LAST_EXECUTED_SPEED_MPS = 0.0
_MOTION_LAST_REASON = "IDLE"
_MOTION_LAST_COMMAND_MONO = None
_MOTION_METRICS = {
    "commands": 0,
    "moving_commands": 0,
    "stop_commands": 0,
    "speed_clips": 0,
    "boundary_recoveries": 0,
    "unnecessary_stops": 0,
    "moving_time_s": 0.0,
    "stopped_time_s": 0.0,
}
_MOTION_REQUESTED_SAMPLES = []
_MOTION_EXECUTED_SAMPLES = []
_REAL_VISION_STATE = None
_LAST_RUNTIME_FAULT_LOG_TIME = -math.inf
_REAL_MAINBOARD_STATE = None
_REAL_MAINBOARD_STATE_REASON = None
_REAL_DEPLOYMENT_ACTIVE = False
_LAST_REAL_RUNTIME_TELEMETRY_LOG_TIME = -math.inf
_MAINBOARD_FRESHNESS_SAMPLES = {
    "tx_age_s": [],
    "rx_age_s": [],
    "pose_age_s": [],
    "referee_age_s": [],
}

def _set_possession_executor_state(state):
    global _POSSESSION_EXECUTOR_STATE
    _POSSESSION_EXECUTOR_STATE = str(state)

def _record_motion_event(requested_speed, executed_speed, reason):
    """Record the latest physical command without influencing tactics."""

    global _MOTION_LAST_REQUESTED_SPEED_MPS, _MOTION_LAST_EXECUTED_SPEED_MPS
    global _MOTION_LAST_REASON, _MOTION_LAST_COMMAND_MONO

    now = time.monotonic()
    requested = max(0.0, float(requested_speed))
    executed = max(0.0, float(executed_speed))
    if _MOTION_LAST_COMMAND_MONO is not None:
        elapsed = max(0.0, now - _MOTION_LAST_COMMAND_MONO)
        if _MOTION_LAST_EXECUTED_SPEED_MPS > 1e-6:
            _MOTION_METRICS["moving_time_s"] += elapsed
        else:
            _MOTION_METRICS["stopped_time_s"] += elapsed
    _MOTION_LAST_COMMAND_MONO = now
    _MOTION_LAST_REQUESTED_SPEED_MPS = requested
    _MOTION_LAST_EXECUTED_SPEED_MPS = executed
    _MOTION_LAST_REASON = str(reason)
    _MOTION_METRICS["commands"] += 1
    if executed > 1e-6:
        _MOTION_METRICS["moving_commands"] += 1
    else:
        _MOTION_METRICS["stop_commands"] += 1
    if executed + 1e-6 < requested:
        _MOTION_METRICS["speed_clips"] += 1
    if str(reason) == "BOUNDARY_RECOVERY":
        _MOTION_METRICS["boundary_recoveries"] += 1
    if str(reason) == "UNNECESSARY_STOP":
        _MOTION_METRICS["unnecessary_stops"] += 1
    _MOTION_REQUESTED_SAMPLES.append(requested)
    _MOTION_EXECUTED_SAMPLES.append(executed)
    del _MOTION_REQUESTED_SAMPLES[:-600]
    del _MOTION_EXECUTED_SAMPLES[:-600]

def _motion_stop(reason):
    safe_stop(chassis, peripheral)
    _record_motion_event(0.0, 0.0, reason)
    return False

def _motion_hold_stop(reason):
    """Stop wheel motion while preserving an already enabled dribbler."""

    try:
        chassis.stop()
    except Exception:
        pass
    _record_motion_event(0.0, 0.0, reason)
    return False

def _reset_motion_metrics():
    global _MOTION_LAST_REQUESTED_SPEED_MPS, _MOTION_LAST_EXECUTED_SPEED_MPS
    global _MOTION_LAST_REASON, _MOTION_LAST_COMMAND_MONO

    _MOTION_LAST_REQUESTED_SPEED_MPS = 0.0
    _MOTION_LAST_EXECUTED_SPEED_MPS = 0.0
    _MOTION_LAST_REASON = "IDLE"
    _MOTION_LAST_COMMAND_MONO = None
    for key in _MOTION_METRICS:
        _MOTION_METRICS[key] = 0
    _MOTION_REQUESTED_SAMPLES.clear()
    _MOTION_EXECUTED_SAMPLES.clear()
    for values in _MAINBOARD_FRESHNESS_SAMPLES.values():
        values.clear()

def _reset_real_ball_control():
    global _REAL_HOLD_SINCE, _REAL_HOLD_CONFIRMED, _REAL_HOLD_LOST_SINCE
    global _REAL_WAS_HELD, _REAL_REACQUIRE, _REAL_EXECUTOR_GENERATION
    global _REAL_CARRY_RECOVERY, _HELD_BALL_STALL
    global _REAL_LAST_ARC

    _REAL_HOLD_SINCE = None
    _REAL_HOLD_CONFIRMED = False
    _REAL_HOLD_LOST_SINCE = None
    _REAL_WAS_HELD = False
    _REAL_REACQUIRE = None
    _REAL_EXECUTOR_GENERATION = 0
    _REAL_CARRY_RECOVERY = None
    _HELD_BALL_STALL = None
    _REAL_LAST_ARC = -math.inf
    _set_possession_executor_state("IDLE")

def _reset_last_seen_ball():
    """Drop cached vision when play or possession is reset."""

    global _LAST_SEEN_BALL_FIELD, _LAST_SEEN_BALL_TIME

    _LAST_SEEN_BALL_FIELD = None
    _LAST_SEEN_BALL_TIME = -math.inf

ATTACK_DIR = None

def _reset_attack_direction():
    global ATTACK_DIR, _SIDE_CANDIDATE, _SIDE_CANDIDATE_SINCE

    ATTACK_DIR = None
    _SIDE_CANDIDATE = None
    _SIDE_CANDIDATE_SINCE = None

def _update_attack_direction(pose):
    global ATTACK_DIR, _SIDE_CANDIDATE, _SIDE_CANDIDATE_SINCE

    if ATTACK_DIR in (-1.0, 1.0):
        return True
    if not _pose_valid(pose):
        _SIDE_CANDIDATE = None
        _SIDE_CANDIDATE_SINCE = None
        return False
    try:
        x = float(pose[0])
        y = float(pose[1])
    except (TypeError, ValueError, IndexError):
        _SIDE_CANDIDATE = None
        _SIDE_CANDIDATE_SINCE = None
        return False

    # Keep the recovery path available after an accidental boundary crossing.
    # The old gate rejected every pose outside the white-line envelope before
    # ProjectSafePose() could issue an inward command.  The sign of the Bits
    # field X coordinate still identifies the starting half, so it is safe to
    # lock the direction immediately while outside the X boundary.  A pose
    # beyond only the Y line may likewise use its valid X sign.
    if abs(x) >= Settings.Field.Long / 2.0 and math.isfinite(x):
        ATTACK_DIR = -1.0 if x > 0.0 else 1.0
        _SIDE_CANDIDATE = None
        _SIDE_CANDIDATE_SINCE = None
        logger.warning(
            "Attack direction recovered from out-of-bounds Bits pose: {}X",
            "+" if ATTACK_DIR > 0 else "-",
        )
        return True
    if abs(x) < AUSTRALIA.Real.SIDE_MIN_X_CM or not math.isfinite(y):
        _SIDE_CANDIDATE = None
        _SIDE_CANDIDATE_SINCE = None
        return False
    now = time.monotonic()
    if (_SIDE_CANDIDATE is None
            or math.dist((x, y), _SIDE_CANDIDATE) > AUSTRALIA.Real.SIDE_POSITION_TOLERANCE_CM):
        _SIDE_CANDIDATE = [x, y]
        _SIDE_CANDIDATE_SINCE = now
        return False
    if now - _SIDE_CANDIDATE_SINCE < AUSTRALIA.Real.SIDE_CONFIRM_S:
        return False
    ATTACK_DIR = -1.0 if pose[0] > 0.0 else 1.0
    logger.info("Attack direction locked: {}X, source=Bits pose", "+" if ATTACK_DIR > 0 else "-")
    return True

def _catch_point():
    point = tuple(map(float, Settings.Motion.CatchVal[:2]))
    if len(point) != 2 or not all(math.isfinite(v) for v in point) or point[0] <= 0.0:
        raise ValueError("Motion.CatchVal must be a finite forward dribbler position in cm")
    return point

def _dribble_on():
    """Enable the dribbler only when the physical controller acknowledges it."""

    try:
        if peripheral.Dribble(True) is False:
            safe_stop(chassis, peripheral)
            return False
    except (AttributeError, OSError, RuntimeError, TypeError, ValueError):
        safe_stop(chassis, peripheral)
        return False
    return True

def _dribble_off():
    """Disable the dribbler before a no-ball or recovery motion."""

    try:
        if peripheral.Dribble(False) is False:
            safe_stop(chassis, peripheral)
            return False
    except (AttributeError, OSError, RuntimeError, TypeError, ValueError):
        safe_stop(chassis, peripheral)
        return False
    return True

def _dribbler_controls_ball(ball):
    """Confirm a camera ball at the configured physical catcher position."""

    if not bool(getattr(peripheral, "DribbleStatus", False)):
        return False
    try:
        ball_x, ball_y = float(ball[0]), float(ball[1])
        catch_x, catch_y = _catch_point()
    except (TypeError, ValueError, IndexError):
        return False
    if not all(math.isfinite(value) for value in (ball_x, ball_y, catch_x, catch_y)):
        return False
    return abs(ball_x - catch_x) <= 2.0 and abs(ball_y - catch_y) <= 1.5

def _real_ball_state(ball=None):
    """Return the stable local possession state used by every tactic.

    A ball is held only when the local camera reports the catcher state, or
    when it is visibly in the configured catcher window while the dribbler is
    enabled. It must persist for 0.25 seconds before shots/carries are allowed.
    Once confirmed, a single missing camera frame is kept for 0.15 seconds so
    dribbler occlusion cannot flip the state every control tick.
    """

    global _REAL_HOLD_SINCE, _REAL_HOLD_CONFIRMED, _REAL_HOLD_LOST_SINCE

    if ball is None:
        ball = UnitedPosition.RobotBallPos()
    raw_state = UnitedPosition.BallStatus(ball)
    observed_hold = raw_state == "held" or (
        raw_state == "free" and _dribbler_controls_ball(ball)
    )
    now = time.monotonic()
    if observed_hold:
        if _REAL_HOLD_SINCE is None:
            _REAL_HOLD_SINCE = now
        _REAL_HOLD_LOST_SINCE = None
        if now - _REAL_HOLD_SINCE >= AUSTRALIA.Real.REAL_HOLD_CONFIRM_S:
            _REAL_HOLD_CONFIRMED = True
        return "held"

    _REAL_HOLD_SINCE = None
    if _REAL_HOLD_CONFIRMED and bool(getattr(peripheral, "DribbleStatus", False)):
        if _REAL_HOLD_LOST_SINCE is None:
            _REAL_HOLD_LOST_SINCE = now
        if now - _REAL_HOLD_LOST_SINCE < AUSTRALIA.Real.REAL_HOLD_LOSS_GRACE_S:
            return "held"
    _REAL_HOLD_CONFIRMED = False
    _REAL_HOLD_LOST_SINCE = None
    return raw_state

def _real_robot_velocity(pose):
    """Read real field velocity, or estimate it from timestamped real poses."""

    global _REAL_POSE_SAMPLE, _REAL_ESTIMATED_VELOCITY, _REAL_ESTIMATED_VELOCITY_TIME

    now = time.monotonic()
    try:
        measured = UnitedPosition.RobotVelocity()
        vx, vy, wz, valid = measured[:4]
        vx, vy, wz = map(float, (vx, vy, wz))
        if (bool(valid) and all(math.isfinite(value) for value in (vx, vy, wz))
                and math.hypot(vx, vy) <= AUSTRALIA.Real.REAL_POSE_VELOCITY_MAX_CM_S
                and abs(wz) <= AUSTRALIA.Real.REAL_POSE_VELOCITY_MAX_DEG_S):
            return [vx, vy, wz]
    except (AttributeError, OSError, RuntimeError, TypeError, ValueError, IndexError):
        pass

    if not _pose_valid(pose):
        return None
    sample = _REAL_POSE_SAMPLE
    current = [float(pose[0]), float(pose[1]), float(pose[2]), now]
    _REAL_POSE_SAMPLE = current
    if sample is not None:
        elapsed = now - sample[3]
        if AUSTRALIA.Real.REAL_POSE_VELOCITY_MIN_SAMPLE_S <= elapsed <= AUSTRALIA.Real.REAL_POSE_VELOCITY_MAX_SAMPLE_S:
            vx = (current[0] - sample[0]) / elapsed
            vy = (current[1] - sample[1]) / elapsed
            wz = angle_error_deg(sample[2], current[2]) / elapsed
            if (all(math.isfinite(value) for value in (vx, vy, wz))
                    and math.hypot(vx, vy) <= AUSTRALIA.Real.REAL_POSE_VELOCITY_MAX_CM_S
                    and abs(wz) <= AUSTRALIA.Real.REAL_POSE_VELOCITY_MAX_DEG_S):
                _REAL_ESTIMATED_VELOCITY = [vx, vy, wz]
                _REAL_ESTIMATED_VELOCITY_TIME = now
    if (isinstance(_REAL_ESTIMATED_VELOCITY, list)
            and 0.0 <= now - _REAL_ESTIMATED_VELOCITY_TIME <= AUSTRALIA.Real.REAL_POSE_VELOCITY_MAX_SAMPLE_S):
        return list(_REAL_ESTIMATED_VELOCITY)
    return None

def _braking_path_safe(pose, velocity):
    """Reject only an outward measured coast that cannot stop inside the field."""

    if velocity is None:
        return True
    try:
        vx, vy = map(float, velocity[:2])
        x, y, yaw = map(float, pose[:3])
    except (TypeError, ValueError, IndexError):
        return False
    speed_cm_s = math.hypot(vx, vy)
    if speed_cm_s <= 1e-6:
        return True
    brake = AUSTRALIA.Real.REAL_BOUNDARY_BRAKE_MPS2
    if not math.isfinite(brake) or brake <= 0.0:
        return False
    speed_mps = speed_cm_s / 100.0
    travel_cm = 100.0 * (
        speed_mps * AUSTRALIA.Real.REAL_DIRECT_COMMAND_HORIZON_S + speed_mps * speed_mps / (2.0 * brake)
    )
    samples = max(2, math.ceil(travel_cm / 2.0))
    for index in range(1, samples + 1):
        fraction = index / samples
        target = [
            x + vx * (travel_cm / speed_cm_s) * fraction,
            y + vy * (travel_cm / speed_cm_s) * fraction,
            yaw,
        ]
        try:
            if UnitedPosition.ProjectSafePose(target) is not None:
                return False
        except (AttributeError, OSError, RuntimeError, TypeError, ValueError):
            return False
    return True

def _direct_motion_path_safe(pose, forward, lateral, turn, horizon_s, *, recovery=False):
    """Check the true short body-twist path against the real field envelope.

    ``RobotMoveVector`` is open-loop for one tick.  It cannot ask
    ``Pos2Pos`` to route around an edge, so integrate its body-frame twist
    through the already-configured physical ``ProjectSafePose`` geometry.
    This is a rejection-only guard: it never scales an otherwise legal
    attacking command.
    """

    try:
        x, y, yaw = map(float, pose[:3])
        forward, lateral, turn, horizon_s = map(float, (forward, lateral, turn, horizon_s))
    except (TypeError, ValueError, IndexError):
        return False
    if (not all(math.isfinite(value) for value in (x, y, yaw, forward, lateral, turn, horizon_s))
            or horizon_s < 0.0):
        return False
    if recovery:
        # When the robot is already outside the legal envelope, use the
        # monotonic inward-recovery rule instead of rejecting every command.
        return _motion_path_safe(
            [x, y, yaw], forward, lateral, turn, horizon_s, recovery=True
        )
    speed = math.hypot(forward, lateral)
    travel_cm = speed * horizon_s * 100.0
    if travel_cm > 300.0:
        return False
    if not _braking_path_safe([x, y, yaw], _real_robot_velocity([x, y, yaw])):
        return False
    steps = max(2, math.ceil(travel_cm / 2.0))
    heading = math.radians(yaw)
    omega = math.radians(turn)
    for index in range(1, steps + 1):
        elapsed = horizon_s * index / steps
        if abs(omega) < 1e-6:
            local_x, local_y = forward * elapsed, lateral * elapsed
        else:
            sine = math.sin(omega * elapsed) / omega
            cosine = (1.0 - math.cos(omega * elapsed)) / omega
            local_x = sine * forward - cosine * lateral
            local_y = cosine * forward + sine * lateral
        target = [
            x + 100.0 * (local_x * math.cos(heading) - local_y * math.sin(heading)),
            y + 100.0 * (local_x * math.sin(heading) + local_y * math.cos(heading)),
            yaw + turn * elapsed,
        ]
        try:
            if UnitedPosition.ProjectSafePose(target) is not None:
                return False
        except (AttributeError, OSError, RuntimeError, TypeError, ValueError):
            return False
    return True

def _bounded_robot_move(
        pose, forward, lateral, turn, *, holding=False, recovery=False,
        motion_reason="COMMAND", requested_speed=None):
    """Send one robot-frame command only when its near-term pose stays legal.

    ``RobotMoveVector`` is intentionally used for close ball control because a
    position controller cannot reliably seat a ball in the dribbler.  Unlike
    ``Pos2Pos`` it does not route around the restricted area, so every direct
    translation needs a short-horizon field-frame guard of its own.
    """

    try:
        forward, lateral, turn = map(float, (forward, lateral, turn))
    except (TypeError, ValueError):
        _motion_stop("VISION_STOP")
        return False
    if (not _pose_valid(pose)
            or not all(math.isfinite(value) for value in (forward, lateral, turn))
            or not UnitedPosition.Referee()
            or not chassis.IsFieldIMUReady()):
        _motion_stop("REFEREE_STOP")
        return False
    if holding:
        # The real catcher cannot retain a ball during a side-slip or a
        # centre-spin. Keep the command longitudinal and unrotated even if a
        # higher tactic accidentally asks for a holonomic carry.
        if abs(lateral) > 1e-6 or abs(turn) > 1e-6:
            _motion_stop("POSSESSION_REALIGN")
            return False
        forward = clamp(forward, -AUSTRALIA.Real.REAL_BACK_CARRY_MPS, AUSTRALIA.Real.REAL_CARRY_MPS)
    else:
        speed = math.hypot(forward, lateral)
        if speed > AUSTRALIA.Real.REAL_ABSOLUTE_MAX_MPS:
            scale = AUSTRALIA.Real.REAL_ABSOLUTE_MAX_MPS / speed
            forward *= scale
            lateral *= scale
    # The public chassis API is an immediate velocity step. Check its
    # permission contract before issuing the command so a denied command
    # cannot be reported as movement.
    can_move = getattr(chassis, "CanMove", None)
    if callable(can_move):
        try:
            if not bool(can_move()):
                _motion_stop("MAINBOARD_STOP")
                return False
        except Exception:
            _motion_stop("MAINBOARD_STOP")
            return False

    if abs(forward) > 1e-6 or abs(lateral) > 1e-6 or abs(turn) > 1e-6:
        if not _direct_motion_path_safe(
                pose, forward, lateral, turn, AUSTRALIA.Real.REAL_DIRECT_COMMAND_HORIZON_S,
                recovery=recovery):
            _motion_stop("BOUNDARY_STOP")
            return False

    try:
        result = chassis.RobotMoveVector(forward, lateral, turn)
    except (AttributeError, OSError, RuntimeError, TypeError, ValueError):
        _motion_stop("MAINBOARD_STOP")
        return False
    if result is False:
        _motion_stop("MAINBOARD_STOP")
        return False
    executed_speed = math.hypot(forward, lateral)
    _record_motion_event(
        executed_speed if requested_speed is None else requested_speed,
        executed_speed,
        motion_reason,
    )
    return True

def _realign_possession_to_yaw(yaw, tolerance=AUSTRALIA.Team.SHOT_ALIGN_DEG, aim_point=None):
    """Realign possession without rotating or translating while holding.

    The physical catcher can retain a ball only during longitudinal motion.
    A SIM-requested heading change therefore starts the possession executor's
    release/reorient/reacquire sequence.  The tactical shot, pass, or carry
    state remains owned by ``strategy_duel_australia`` while this executor is
    active.
    """

    if not UnitedPosition.Referee() or not chassis.IsFieldIMUReady():
        _motion_stop("REFEREE_STOP")
        return False
    if _real_ball_state() != "held" or not _REAL_HOLD_CONFIRMED:
        _motion_stop("POSSESSION_REALIGN")
        return False
    try:
        pose = UnitedPosition.FieldRobotPose()
        current_yaw = float(pose[2])
    except (TypeError, ValueError, RuntimeError, AttributeError):
        _motion_stop("VISION_STOP")
        return False
    if (not _pose_valid(pose)
            or not all(math.isfinite(v) for v in (current_yaw, yaw, tolerance))
            or tolerance <= 0):
        _motion_stop("VISION_STOP")
        return False
    error = angle_error_deg(current_yaw, yaw)
    if abs(error) <= tolerance:
        _motion_hold_stop("POSSESSION_STABILIZE")
        return True
    if _REAL_REACQUIRE is None:
        _set_possession_executor_state("RETREAT")
        _begin_reacquire(
            yaw,
            phase="release",
            aim_point=aim_point,
            tolerance=tolerance,
        )
    else:
        _motion_stop("POSSESSION_REALIGN")
    return False

# All SIM shot, pass, carry, and Macau call sites retain this name, but the
# physical implementation releases before any heading change.
def _turn_held(yaw, tolerance=AUSTRALIA.Team.SHOT_ALIGN_DEG, aim_point=None):
    return _realign_possession_to_yaw(yaw, tolerance=tolerance, aim_point=aim_point)

def _kick_referee_gate(bits=None):
    """Read the referee permission immediately before a physical KICK.

    ``UnitedPosition.Referee`` is the semantic runtime facade.  The IL Bits
    object exposes the same GPIO through ``GetRefereeUnlockPin``; when present,
    checking both closes the gap between a cached facade value and the command
    that is about to be emitted.  Thin CPU-only test adapters may omit the raw
    getter and rely on the facade check.
    """

    try:
        if not bool(UnitedPosition.Referee()):
            return False
        getter = getattr(bits, "GetRefereeUnlockPin", None) if bits is not None else None
        if callable(getter) and not bool(getter()):
            return False
        return True
    except Exception:
        return False

def _fire_real_kicker(power_level):
    """Issue one immediate MainBoard calibrated kick level (1..4).

    The final referee check deliberately lives in this lowest strategy-level
    wrapper as well as in ``_fire_shot``.  A PLAY->STOP edge can occur while
    the dribbler is being disabled; no direct ``runtime.Bits.Kick`` call is
    allowed to bypass that second gate.  The physical Peripherals adapter is
    a legacy electromagnet/IO facade on this image, so the real level-aware
    command is ``IceLoongBits.Kick(level)``.  A custom adapter may still expose
    a level-aware ``ShootBall`` together with ``KickFunc``; that path remains
    compatible but is not assumed for the deployed IL wrapper.
    """

    if not _kick_referee_gate() or not _real_vision_ready():
        safe_stop(chassis, peripheral)
        return False
    can_move = getattr(chassis, "CanMove", None)
    if callable(can_move):
        try:
            if not bool(can_move()):
                safe_stop(chassis, peripheral)
                return False
        except Exception:
            safe_stop(chassis, peripheral)
            return False
    try:
        from LunaPre import runtime
        bits = runtime.Bits
        if bits is None or not callable(getattr(bits, "Kick", None)):
            return False
        # The deployed IL Peripherals object has an IO-only ShootBall method;
        # use it only when a custom adapter explicitly exposes a level-aware
        # KickFunc.  Otherwise issue the documented MainBoard KICK command.
        shoot = getattr(peripheral, "ShootBall", None)
        kick_callback = getattr(peripheral, "KickFunc", None)
        if callable(shoot) and callable(kick_callback):
            if not _kick_referee_gate(bits):
                safe_stop(chassis, peripheral)
                return False
            result = shoot(power_level)
        else:
            if callable(getattr(bits, "SetKickPowerLevel", None)):
                bits.SetKickPowerLevel(power_level)
            # Re-check both the strategy facade and the raw MainBoard gate
            # immediately before emitting the lower-board command.
            if not _kick_referee_gate(bits):
                safe_stop(chassis, peripheral)
                return False
            result = bits.Kick(power_level)
    except (AttributeError, OSError, RuntimeError, TypeError, ValueError):
        safe_stop(chassis, peripheral)
        return False
    # IceLoongBits.Kick returns the decoded response list.  A serial timeout
    # therefore appears as ``[]`` rather than an exception; do not mark the
    # shot as fired when the lower board never acknowledged the command.
    if result is False or (
            isinstance(result, (list, tuple)) and not result
    ):
        return False
    return True

def _fire_shot(route=None):
    global _LAST_SHOT_TIME, _AWAIT_BALL_RELEASE

    if _AWAIT_BALL_RELEASE or time.monotonic() - _LAST_SHOT_TIME < AUSTRALIA.Team.SHOT_COOLDOWN_S:
        chassis.stop()
        return False
    if _real_ball_state() != "held" or not _REAL_HOLD_CONFIRMED:
        safe_stop(chassis, peripheral)
        return False
    if not UnitedPosition.Referee() or not chassis.IsFieldIMUReady():
        safe_stop(chassis, peripheral)
        return False
    if _REAL_REACQUIRE is not None or not _REAL_HOLD_CONFIRMED:
        chassis.stop()
        return False
    bank_shot = isinstance(route, dict) and len(route.get("points", ())) > 2
    if isinstance(route, dict) and not bank_shot:
        points = route.get("points") or []
        try:
            end_x, end_y = map(float, points[-1][:2])
        except (TypeError, ValueError, IndexError):
            chassis.stop()
            return False
        if (abs(end_x - _opponent_goal_back_wall_x()) > 0.25
                or not _is_inside_goal_scoring_band(end_y)):
            chassis.stop()
            return False
    if isinstance(route, int) and not isinstance(route, bool):
        power_level = route
    else:
        power_level = AUSTRALIA.Tactics.WALL_KICK_POWER_LEVEL if bank_shot else AUSTRALIA.Tactics.DIRECT_KICK_POWER_LEVEL
    chassis.stop()
    # Open League permits a dribbler but requires it to release the ball to
    # score. IceLoong snapshots the dribbler setting into its immediate kick
    # frame, so turn it off before issuing the physical kick command.
    if not _dribble_off():
        safe_stop(chassis, peripheral)
        return False
    if not _fire_real_kicker(power_level):
        safe_stop(chassis, peripheral)
        return False
    _LAST_SHOT_TIME = time.monotonic()
    _AWAIT_BALL_RELEASE = True
    return True

def _held_ball_in_playing_area(pose):
    """Keep a carried ball inside the official white-line playing area."""

    try:
        return _playing_ball_xy(_held_ball_field_pos(pose)) is not None
    except (TypeError, ValueError, IndexError):
        return False

def _field_xy(value):
    """Return a finite physical wall-envelope field point, excluding sentinels."""

    if value is None or value in (BALL_NOT_FOUND, BALL_HELD):
        return None
    try:
        x, y = float(value[0]), float(value[1])
    except (TypeError, ValueError, IndexError):
        return None
    if not all(math.isfinite(item) for item in (x, y)):
        return None
    if abs(x) > Settings.Field.Long / 2.0 or abs(y) > Settings.Field.Short / 2.0:
        return None
    return [x, y]

def _playing_ball_xy(value):
    """Return a current free ball only while it remains inside the white lines."""

    point = _field_xy(value)
    if point is None:
        return None
    if abs(point[0]) > RCJA_PLAYING_HALF_X_CM or abs(point[1]) > RCJA_PLAYING_HALF_Y_CM:
        return None
    return point

def _field_ball_from_local_observation(local_ball, pose=None):
    """Transform the exact local ball packet for this decision into field XY."""

    try:
        ball_x, ball_y = float(local_ball[0]), float(local_ball[1])
        pose = UnitedPosition.FieldRobotPose() if pose is None else pose
        robot_x, robot_y, robot_yaw = map(float, pose[:3])
    except (TypeError, ValueError, IndexError):
        return None
    if not all(math.isfinite(value) for value in (ball_x, ball_y, robot_x, robot_y, robot_yaw)):
        return None
    theta = math.radians(robot_yaw)
    return _playing_ball_xy([
        robot_x + ball_x * math.cos(theta) - ball_y * math.sin(theta),
        robot_y + ball_x * math.sin(theta) + ball_y * math.cos(theta),
    ])

def _physical_xy(value):
    """Return a finite physical point through the white line and out area.

    Camera chassis detections and a Bits heartbeat can legitimately place a
    robot in the official outer area while it is recovering. Keep those
    points for obstacle/teammate identity, but reject values beyond the wall
    envelope so a corrupt detector cannot create an unbounded obstacle.
    """

    if value is None:
        return None
    try:
        x, y = float(value[0]), float(value[1])
    except (TypeError, ValueError, IndexError):
        return None
    if not all(math.isfinite(item) for item in (x, y)):
        return None
    if (x, y) in {
        (float(BALL_NOT_FOUND[0]), float(BALL_NOT_FOUND[1])),
        (float(BALL_HELD[0]), float(BALL_HELD[1])),
    }:
        return None
    if (
        abs(x) > Settings.Field.Long / 2.0 + AUSTRALIA.Real.REAL_DETECTION_TOLERANCE_CM
        or abs(y) > Settings.Field.Short / 2.0 + AUSTRALIA.Real.REAL_DETECTION_TOLERANCE_CM
    ):
        return None
    return [x, y]

def _current_enemies():
    """Return locally observed opponents, excluding a fresh identified peer.

    Camera chassis detections do not carry a team identifier.  The only
    physical identity source available to the strategy is the peer's own
    short-lived radio heartbeat.  If it is fresh, remove at most the nearest
    matching visual chassis; with no such evidence, keep every detection as a
    conservative obstacle/opponent rather than guessing from unavailable data.
    """

    try:
        enemies = UnitedPosition.FieldChassisPos()
    except (TypeError, ValueError, AttributeError):
        return []
    candidates = [point for enemy in enemies or [] if (point := _physical_xy(enemy)) is not None]
    teammate = _fresh_teammate_field_position()
    if teammate is None or not candidates:
        return candidates

    teammate_index, teammate_distance = min(
        enumerate(math.dist(point, teammate) for point in candidates),
        key=lambda item: item[1],
    )
    if teammate_distance <= AUSTRALIA.Team.TEAMMATE_VISION_MATCH_CM:
        del candidates[teammate_index]
    return candidates

def _safe_target(x, y, yaw):
    target = [float(x), float(y), float(yaw) % 360.0]
    projected = UnitedPosition.ProjectSafePose(target)
    return projected if projected is not None else target

def _translation_safe(pose, target, *, recovery=False):
    if not _pose_valid(pose) or not _pose_valid(target):
        return False
    if recovery:
        projected = UnitedPosition.ProjectSafePose(pose)
        if projected is not None:
            current_distance = math.dist(pose[:2], projected[:2])
            target_distance = math.dist(target[:2], projected[:2])
            return target_distance < current_distance - 0.01
    distance = math.dist(pose[:2], target[:2])
    for index in range(1, max(2, math.ceil(distance / 5.0)) + 1):
        fraction = index / max(2, math.ceil(distance / 5.0))
        point = [pose[0] + (target[0] - pose[0]) * fraction,
                 pose[1] + (target[1] - pose[1]) * fraction, target[2]]
        if UnitedPosition.ProjectSafePose(point) is not None:
            return False
    return _lane_clear(pose, target, _current_enemies(), float(Settings.Vision.ChassisWidth) + 4.0)

def _real_move(pose, target, yaw, speed, holding=False, recovery=False):
    if holding:
        _set_possession_executor_state("APPROACH_TARGET")
    if holding and not _held_ball_in_playing_area([target[0], target[1], yaw]):
        _set_possession_executor_state("STABILIZE")
        _motion_stop("BOUNDARY_STOP")
        return False
    if not _translation_safe(pose, target, recovery=recovery):
        _motion_stop("BOUNDARY_STOP")
        return False
    error = angle_error_deg(pose[2], yaw)
    if holding and abs(error) > AUSTRALIA.Team.SHOT_ALIGN_DEG:
        _set_possession_executor_state("REALIGN")
        _turn_held(yaw)
        return False
    if not holding and abs(error) > 5.0:
        return _safe_robot_move(
            0.0, 0.0, clamp(error * 2.0, -75.0, 75.0),
            speed_limit=AUSTRALIA.Real.REAL_ALIGNMENT_MPS, turn_priority=True,
        )
    dx, dy = target[0] - pose[0], target[1] - pose[1]
    distance = math.hypot(dx, dy)
    if distance <= 1.0:
        if holding:
            _set_possession_executor_state("STABILIZE")
        _motion_hold_stop("POSSESSION_STABILIZE")
        return True
    theta = math.radians(pose[2])
    gain = min(speed, distance * 0.015) / distance
    vx = (dx * math.cos(theta) + dy * math.sin(theta)) * gain
    vy = (-dx * math.sin(theta) + dy * math.cos(theta)) * gain
    if holding:
        if _real_ball_state() != "held":
            _set_possession_executor_state("ACQUIRE")
            _real_ball_control(pose, "free")
            _motion_stop("REACQUIRE")
            return False
        # A target that is not directly in front of or behind the dribbler is
        # reached by reorienting without possession, never by lateral carry.
        if abs(vy) > 0.01:
            _set_possession_executor_state("REALIGN")
            _turn_held(UnitedPosition.Pos2Angle(pose, target), AUSTRALIA.Team.SHOT_ALIGN_DEG, target)
            return False
        vx = clamp(vx, -AUSTRALIA.Real.REAL_BACK_CARRY_MPS, AUSTRALIA.Real.REAL_CARRY_MPS)
        _set_possession_executor_state("RETREAT" if vx < 0.0 else "FORWARD_REACQUIRE")
    return _bounded_robot_move(
        pose,
        vx,
        0.0 if holding else vy,
        0.0 if holding else clamp(error * 2.0, -20.0, 20.0),
        holding=holding, recovery=recovery,
    )

def _possession_motion_adapter(pose, target, yaw, speed=AUSTRALIA.Real.REAL_CARRY_MPS, *, intent=None):
    """Execute a SIM-selected held-ball target using legal real motion.

    The adapter may retreat, reorient, reacquire, and then advance, but it
    never changes the tactical action or target selected by the strategy.
    """

    global _TACTICAL_INTENT
    if intent is not None:
        _TACTICAL_INTENT = str(intent)
    if not _REAL_HOLD_CONFIRMED:
        _set_possession_executor_state("HOLD_SETTLE")
        _motion_stop("POSSESSION_REALIGN")
        return False
    _set_possession_executor_state("APPROACH_TARGET")
    return _real_move(pose, target, yaw, speed, holding=True)

def _carry_recovery_target_matches(state, target):
    if not isinstance(state, dict) or not isinstance(state.get("target"), (list, tuple)):
        return False
    try:
        return math.dist(state["target"][:2], target[:2]) <= 2.0
    except (TypeError, ValueError, IndexError):
        return False

def _carry_recovery_waypoint(pose, target):
    """Choose a short physical detour without changing the tactical target."""

    try:
        dx, dy = float(target[0]) - float(pose[0]), float(target[1]) - float(pose[1])
        distance = math.hypot(dx, dy)
    except (TypeError, ValueError, IndexError):
        return None
    if distance <= AUSTRALIA.Real.REAL_CARRY_RECOVERY_ARRIVAL_CM:
        return None
    direction = (dx / distance, dy / distance)
    normal = (-direction[1], direction[0])
    enemies = _current_enemies()
    midpoint = [(float(pose[0]) + float(target[0])) * 0.5,
                (float(pose[1]) + float(target[1])) * 0.5]
    blocker = min(enemies, key=lambda item: math.dist(item, midpoint), default=None)
    if blocker is None:
        preferred_side = 1.0 if float(pose[1]) >= float(target[1]) else -1.0
    else:
        cross = dx * (float(blocker[1]) - float(pose[1])) - dy * (float(blocker[0]) - float(pose[0]))
        preferred_side = -1.0 if cross >= 0.0 else 1.0
    try:
        clearance = float(Settings.Vision.ChassisWidth) + 4.0
    except (AttributeError, TypeError, ValueError):
        clearance = 26.0
    offset = max(AUSTRALIA.Real.REAL_REACQUIRE_RADIUS_CM + 8.0, clearance + 8.0)
    retreat = min(18.0, distance * 0.25)
    for side in (preferred_side, -preferred_side):
        waypoint = [
            float(pose[0]) - direction[0] * retreat + normal[0] * side * offset,
            float(pose[1]) - direction[1] * retreat + normal[1] * side * offset,
            float(pose[2]),
        ]
        waypoint = _safe_target(*waypoint)
        if not _pose_valid(waypoint) or not _held_ball_in_playing_area(waypoint):
            continue
        if not _lane_clear(pose, waypoint, enemies, clearance):
            continue
        if any(math.dist(waypoint[:2], enemy) < clearance for enemy in enemies):
            continue
        # The second leg must also be clear. Otherwise the next held-ball
        # command immediately enters the same blocked lane again.
        if not _lane_clear(waypoint, target, enemies, clearance):
            continue
        return waypoint, side
    return None

def _start_carry_recovery(pose, target, travel_yaw):
    """Start one bounded physical detour while retaining the SIM target."""

    global _REAL_CARRY_RECOVERY, _TACTICAL_TARGET

    now = time.monotonic()
    previous = _REAL_CARRY_RECOVERY
    if _carry_recovery_target_matches(previous, target):
        if previous.get("phase") == "aborted":
            return False
        attempts = int(previous.get("attempts", 0)) + 1
    else:
        attempts = 1
    _TACTICAL_TARGET = list(target)
    if attempts > AUSTRALIA.Real.REAL_CARRY_RECOVERY_MAX_ATTEMPTS:
        _REAL_CARRY_RECOVERY = {
            "target": list(target), "attempts": attempts,
            "phase": "aborted", "reason": "max_attempts",
        }
        _set_possession_executor_state("BLOCKED_ABORT")
        _motion_stop("POSSESSION_BLOCKED")
        return False
    selected = _carry_recovery_waypoint(pose, target)
    if selected is None:
        _REAL_CARRY_RECOVERY = {
            "target": list(target), "attempts": attempts,
            "phase": "aborted", "reason": "no_clear_waypoint",
        }
        _set_possession_executor_state("BLOCKED_ABORT")
        _motion_stop("POSSESSION_BLOCKED")
        return False
    waypoint, side = selected
    _REAL_CARRY_RECOVERY = {
        "target": list(target), "waypoint": list(waypoint), "side": side,
        "attempts": attempts, "started": now, "phase": "reacquire",
        "progress_cm": 0.0,
    }
    waypoint_yaw = UnitedPosition.Pos2Angle(pose, waypoint)
    _set_possession_executor_state("RETREAT")
    _begin_reacquire(
        waypoint_yaw,
        phase="release",
        aim_point=waypoint[:2],
        tolerance=AUSTRALIA.Team.SHOT_ALIGN_DEG,
        owner_intent=_TACTICAL_INTENT or "CARRY",
    )
    return True

def _held_ball_stall_watch(state, ball_state, pose, now=None):
    """Report repeated zero-progress hold/aim cycles without recovery."""

    global _HELD_BALL_STALL

    if ball_state != "held" or state != "attack_hold_aim" or _REAL_REACQUIRE is not None:
        _HELD_BALL_STALL = None
        return None
    now = time.monotonic() if now is None else float(now)
    key = (str(state), str(_TACTICAL_INTENT), tuple(_TACTICAL_TARGET or ()))
    moving_count = int(_MOTION_METRICS.get("moving_commands", 0))
    if not isinstance(_HELD_BALL_STALL, dict) or _HELD_BALL_STALL.get("key") != key:
        _HELD_BALL_STALL = {
            "key": key, "since": now, "moving_count": moving_count,
            "reports": 0,
        }
        return None
    if moving_count > int(_HELD_BALL_STALL.get("moving_count", 0)):
        _HELD_BALL_STALL["since"] = now
        _HELD_BALL_STALL["moving_count"] = moving_count
        return None
    if now - float(_HELD_BALL_STALL.get("since", now)) < AUSTRALIA.Real.REAL_HELD_BALL_STALL_TIMEOUT_S:
        return None
    reports = int(_HELD_BALL_STALL.get("reports", 0)) + 1
    _HELD_BALL_STALL["reports"] = reports
    _HELD_BALL_STALL["since"] = now
    _set_possession_executor_state("STALL_DIAGNOSTIC")
    _journal_event(
        "REAL_HELD_BALL_EXECUTOR_STALL",
        state=state,
        intent=_TACTICAL_INTENT,
        pose=pose,
        reports=reports,
    )
    return "held_ball_stall"

def _begin_reacquire(yaw, phase="observe", aim_point=None, tolerance=AUSTRALIA.Team.SHOT_ALIGN_DEG,
                     owner_intent=None):
    global _REAL_REACQUIRE, _REAL_EXECUTOR_GENERATION, _CHASE_ALIGNED

    _REAL_EXECUTOR_GENERATION += 1
    _REAL_REACQUIRE = {"yaw": float(yaw) % 360.0, "phase": phase,
                       "since": time.monotonic(), "lost_since": None,
                       "aim_point": aim_point, "tolerance": min(3.0, tolerance * 0.6),
                       "ball_sample": None,
                       "owner_intent": owner_intent or _TACTICAL_INTENT,
                       "generation": _REAL_EXECUTOR_GENERATION}
    _CHASE_ALIGNED = False
    _motion_stop("POSSESSION_REALIGN")

def _abort_reacquire(reason):
    global _REAL_REACQUIRE

    _REAL_REACQUIRE = None
    _reset_attack()
    _motion_stop("REACQUIRE")
    return reason

def _real_reacquire_step(pose, state):
    global _REAL_REACQUIRE

    plan = _REAL_REACQUIRE
    now = time.monotonic()
    elapsed = now - plan["since"]
    if elapsed >= AUSTRALIA.Real.REAL_REACQUIRE_TIMEOUT_S:
        return _abort_reacquire("reacquire_timeout")
    if plan["phase"] == "release":
        if not _dribble_off():
            return _abort_reacquire("dribble_release_fault")
        if state == "held":
            if elapsed >= AUSTRALIA.Real.REAL_RELEASE_TIMEOUT_S:
                return _abort_reacquire("release_timeout")
            theta = math.radians(pose[2])
            target = [pose[0] - 8.0 * math.cos(theta), pose[1] - 8.0 * math.sin(theta), pose[2]]
            if not _translation_safe(pose, target, recovery=True):
                return _abort_reacquire("release_blocked")
            if not _bounded_robot_move(
                    pose, -AUSTRALIA.Real.REAL_RELEASE_BACK_MPS, 0.0, 0.0,
                    recovery=True, motion_reason="BOUNDARY_RECOVERY",
                    requested_speed=AUSTRALIA.Real.REAL_RELEASE_BACK_MPS):
                return _abort_reacquire("release_blocked")
            return "release_backoff"
        plan["phase"] = "observe"
        _motion_stop("REACQUIRE")
        return "release_observe"
    if state == "held":
        _motion_hold_stop("REACQUIRE")
        if not _dribble_on():
            return _abort_reacquire("dribble_fault")
        if _REAL_HOLD_CONFIRMED:
            recovery = _REAL_CARRY_RECOVERY
            if isinstance(recovery, dict) and recovery.get("phase") == "reacquire":
                recovery["phase"] = "carry_waypoint"
                recovery["progress_cm"] = 0.0
            _REAL_REACQUIRE = None
            return "reacquired"
        return "reacquire_settle"
    ball = _playing_ball_xy(UnitedPosition.FieldBallPos()) if state == "free" else None
    if ball is None:
        _motion_stop("REACQUIRE")
        if plan["lost_since"] is None:
            plan["lost_since"] = now
        if now - plan["lost_since"] >= AUSTRALIA.Real.REAL_LOST_WAIT_S:
            return _abort_reacquire("reacquire_lost")
        return "reacquire_wait_vision"
    plan["lost_since"] = None
    if math.dist(pose[:2], ball) > AUSTRALIA.Real.REAL_REACQUIRE_RADIUS_CM + 70.0:
        return _abort_reacquire("reacquire_out_of_range")
    if _nearest_enemy(_current_enemies(), ball)[1] < AUSTRALIA.Tactics.HELD_ENEMY_OVERLAP_CM:
        return _abort_reacquire("reacquire_contested")
    if plan["aim_point"] is not None:
        plan["yaw"] = UnitedPosition.Pos2Angle(ball, plan["aim_point"])
    catch_x, catch_y = _catch_point()
    catch_bearing = math.degrees(math.atan2(catch_y, catch_x))
    desired_bearing = (plan["yaw"] + catch_bearing + 180.0) % 360.0
    radial_bearing = UnitedPosition.Pos2Angle(ball, pose)
    bearing_error = angle_error_deg(radial_bearing, desired_bearing)
    distance = math.dist(pose[:2], ball)
    facing = (UnitedPosition.Pos2Angle(pose, ball) - catch_bearing) % 360.0
    aligned = abs(bearing_error) <= plan["tolerance"]
    if aligned and distance <= AUSTRALIA.Real.REAL_REACQUIRE_RADIUS_CM + 5.0:
        plan["phase"] = "approach"
        target = [ball[0], ball[1], facing]
        if not _translation_safe(pose, target):
            return _abort_reacquire("reacquire_blocked")
        chase_ball()
        return "reacquire_approach"
    plan["phase"] = "circle"
    if not _dribble_off():
        return _abort_reacquire("dribble_release_fault")
    next_bearing = radial_bearing
    if distance >= AUSTRALIA.Real.REAL_REACQUIRE_RADIUS_CM - 4.0:
        next_bearing += clamp(bearing_error, -30.0, 30.0)
    theta = math.radians(next_bearing)
    target = [ball[0] + AUSTRALIA.Real.REAL_REACQUIRE_RADIUS_CM * math.cos(theta),
              ball[1] + AUSTRALIA.Real.REAL_REACQUIRE_RADIUS_CM * math.sin(theta), plan["yaw"]]
    if not _real_move(
            pose, target, plan["yaw"], AUSTRALIA.Real.REAL_REPOSITION_MPS, recovery=True):
        return _abort_reacquire("reacquire_blocked")
    return "reacquire_circle"

def _real_ball_control(pose, state):
    global _REAL_WAS_HELD

    was_held = _REAL_WAS_HELD
    _REAL_WAS_HELD = state == "held" and _REAL_HOLD_CONFIRMED
    if was_held and state != "held" and not _AWAIT_BALL_RELEASE and _REAL_REACQUIRE is None:
        _reset_attack()
        goal = [ATTACK_DIR * Settings.Field.Long / 2.0, 0.0]
        _begin_reacquire(UnitedPosition.Pos2Angle(pose, goal), aim_point=goal)
        return "ball_slip_stop"
    if _REAL_REACQUIRE is not None:
        return _real_reacquire_step(pose, state)
    if state == "held" and not _REAL_HOLD_CONFIRMED:
        _motion_stop("POSSESSION_STABILIZE")
        if not _dribble_on():
            return "dribble_fault"
        return "hold_settle"
    return None

def _carry_to(pose, target, yaw, *, goal_post_escape=False):
    global _TACTICAL_TARGET, _REAL_CARRY_RECOVERY

    target = _carry_safe_target(
        target[0], target[1], yaw, goal_post_escape=goal_post_escape,
    )
    if target is None:
        safe_stop(chassis, peripheral)
        return False
    if goal_post_escape and ATTACK_DIR * (target[0] - pose[0]) > 0.25:
        safe_stop(chassis, peripheral)
        return False
    if not _held_ball_in_playing_area(target):
        safe_stop(chassis, peripheral)
        return False
    _TACTICAL_TARGET = list(target)
    recovery = _REAL_CARRY_RECOVERY
    if recovery is not None and not _carry_recovery_target_matches(recovery, target):
        _REAL_CARRY_RECOVERY = None
        recovery = None
    if _carry_recovery_target_matches(recovery, target) and recovery.get("phase") == "aborted":
        # The bounded executor attempt has ended. Do not restart it every
        # tick while the tactical brain is still presenting the same target.
        _set_possession_executor_state("BLOCKED_ABORT")
        return False
    physical_target = target
    if _carry_recovery_target_matches(recovery, target):
        if _REAL_REACQUIRE is not None:
            _set_possession_executor_state("REACQUIRE")
            _motion_stop("REACQUIRE")
            return False
        phase = recovery.get("phase")
        waypoint = recovery.get("waypoint")
        if phase == "reacquire":
            _start_carry_recovery(pose, target, UnitedPosition.Pos2Angle(pose, target))
            return False
        if phase == "carry_waypoint":
            if not _pose_valid(waypoint):
                _start_carry_recovery(pose, target, UnitedPosition.Pos2Angle(pose, target))
                return False
            if math.dist(pose[:2], waypoint[:2]) <= AUSTRALIA.Real.REAL_CARRY_RECOVERY_ARRIVAL_CM:
                recovery["phase"] = "return_target"
            else:
                physical_target = waypoint
        if recovery.get("phase") == "return_target":
            physical_target = target
    distance = math.hypot(physical_target[0] - pose[0], physical_target[1] - pose[1])
    if distance <= 6.0:
        if (_carry_recovery_target_matches(recovery, target)
                and recovery.get("phase") == "carry_waypoint"):
            recovery["phase"] = "return_target"
            physical_target = target
            distance = math.hypot(target[0] - pose[0], target[1] - pose[1])
        else:
            _REAL_CARRY_RECOVERY = None
            return _turn_held(yaw, tolerance=AUSTRALIA.Team.SHOT_ALIGN_DEG)
    if distance <= 6.0:
        _REAL_CARRY_RECOVERY = None
        return _turn_held(yaw, tolerance=AUSTRALIA.Team.SHOT_ALIGN_DEG)
    travel_yaw = UnitedPosition.Pos2Angle(pose, physical_target)
    if not _lane_clear(
            pose, physical_target, _current_enemies(), float(Settings.Vision.ChassisWidth) + 4.0):
        # A blocked target is an executor condition, not a new tactical
        # decision. Keep the SIM-selected target and create one bounded
        # physical detour before returning control to the next strategy tick.
        _set_possession_executor_state("BLOCKED")
        if _REAL_REACQUIRE is None:
            _start_carry_recovery(pose, target, travel_yaw)
        else:
            _motion_stop("REACQUIRE")
        return False
    if not _dribble_on():
        return False
    _possession_motion_adapter(
        pose, physical_target, travel_yaw, AUSTRALIA.Real.REAL_CARRY_MPS,
        intent=_TACTICAL_INTENT or "CARRY",
    )
    return False

def _peer_robot_id():
    robot_id = _robot_id()
    return next((ident for ident in AUSTRALIA.Team.RobotIds if ident != robot_id), None)

def _fresh_teammate_field_position():
    """Return an identified peer pose only while its own heartbeat is fresh."""

    if _TEAM_LINK is None:
        return None
    frame, timestamp = _TEAM_SAMPLE
    age = time.monotonic() - timestamp
    if not isinstance(frame, dict) or not 0.0 <= age <= AUSTRALIA.Tactics.TEAM_STATE_MAX_AGE_S:
        return None
    if frame.get("id") != _peer_robot_id() or not _valid_team_frame(frame):
        return None
    try:
        role = int(frame.get("role", Communication.ROLE_BAN))
    except (TypeError, ValueError):
        return None
    if role == Communication.ROLE_BAN:
        return None
    return _physical_xy(frame.get("signed_xy"))

def _team_frame_yaw(frame):
    """Read yaw from the versioned real payload or the legacy reserve byte."""

    if not isinstance(frame, dict):
        return None
    try:
        if int(frame.get("version", -1)) == REAL_TEAM_PROTOCOL_VERSION:
            yaw = float(frame["yaw"])
        else:
            reserve = int(frame.get("reserve", 0))
            yaw = (reserve & TEAM_YAW_MASK) * 360.0 / TEAM_YAW_STEPS
    except (KeyError, TypeError, ValueError):
        return None
    return yaw % 360.0 if math.isfinite(yaw) else None

def _team_transport_configured():
    """Return whether the live MainBoard/ESP link contract is available."""

    try:
        from LunaPre import runtime
        bits = runtime.Bits
    except (AttributeError, ImportError):
        return False
    return bits is not None and all(callable(getattr(bits, name, None)) for name in (
        "SendLinkData", "GetLinkData",
    ))

def _configure_team_link_polling(bits):
    """Set asynchronous LINK_GET cadence across old and new IL wrappers."""

    interval = float(AUSTRALIA.Runtime.TeamLinkPollIntervalSeconds)
    setter = getattr(bits, "SetLinkPollInterval", None)
    if callable(setter):
        setter(interval)
        return interval
    if not hasattr(bits, "link_poll_interval_s"):
        raise RuntimeError("physical Bits exposes no asynchronous LINK_GET interval")
    lock = getattr(bits, "_command_lock", None)
    if lock is None:
        bits.link_poll_interval_s = interval
    else:
        with lock:
            bits.link_poll_interval_s = interval
    return interval

def _start_team_communication():
    """Open the configured physical peer channel, without making it mandatory."""

    global _TEAM_LINK, _TEAM_SAMPLE, _TEAM_LINK_GENERATION
    global _TEAM_LAST_SEND_TIME, _TEAM_LINK_STARTED_AT
    global _TEAMMATE_EVER_SEEN, _TEAMMATE_MODE, _TEAMMATE_MODE_REASON, _TEAM_ACTION_STATE
    global SOLO_TAKEOVER, TEAM_LINK_DISABLED

    _stop_team_communication()
    if TEAM_LINK_DISABLED:
        SOLO_TAKEOVER = True
        logger.info("TEAM LINK DISABLED: configured single-robot mode or unavailable transport")
        return False
    if not _team_transport_configured():
        TEAM_LINK_DISABLED = True
        SOLO_TAKEOVER = True
        logger.warning(
            "TEAM LINK DISABLED: MainBoard/ESP transport unavailable; "
            "DEGRADED SINGLE-ROBOT TACTIC ACTIVE; using single-robot PRIMARY mode"
        )
        return False
    try:
        from LunaPre import runtime
        _configure_team_link_polling(runtime.Bits)
        peer_id = _peer_robot_id()
        if peer_id is None:
            raise RuntimeError("no peer robot id is configured")
        _TEAM_LINK = RealTeamLink(runtime.Bits, _robot_id(), peer_id)
    except (AttributeError, ImportError, OSError, RuntimeError, TypeError, ValueError) as error:
        _TEAM_LINK = None
        TEAM_LINK_DISABLED = True
        SOLO_TAKEOVER = True
        logger.warning(
            "TEAM LINK DISABLED: real radio unavailable; "
            "DEGRADED SINGLE-ROBOT TACTIC ACTIVE; using single-robot PRIMARY mode: {}",
            error,
        )
        return False

    _TEAM_SAMPLE = (None, -math.inf)
    _TEAM_LINK_GENERATION = _TEAM_LINK.generation
    _TEAM_LAST_SEND_TIME = -math.inf
    _TEAM_LINK_STARTED_AT = -math.inf
    _TEAMMATE_EVER_SEEN = False
    _TEAMMATE_MODE = "pairing"
    _TEAMMATE_MODE_REASON = None
    _TEAM_ACTION_STATE = None
    return True

def _stop_team_communication():
    """Drop only the optional team-radio resource; it never owns motion."""

    global _TEAM_LINK, _TEAM_SAMPLE, _TEAM_LINK_GENERATION
    global _TEAM_LAST_SEND_TIME, _TEAM_LINK_STARTED_AT
    global _TEAMMATE_EVER_SEEN, _TEAMMATE_MODE, _TEAMMATE_MODE_REASON, _TEAM_ACTION_STATE

    link = _TEAM_LINK
    _TEAM_LINK = None
    _TEAM_SAMPLE = (None, -math.inf)
    _TEAM_LINK_GENERATION = 0
    _TEAM_LAST_SEND_TIME = -math.inf
    _TEAM_LINK_STARTED_AT = -math.inf
    _TEAMMATE_EVER_SEEN = False
    _TEAMMATE_MODE = "static"
    _TEAMMATE_MODE_REASON = None
    _TEAM_ACTION_STATE = None
    if link is not None:
        try:
            close = getattr(link, "close", None) or getattr(link, "Close", None)
            if callable(close):
                close()
        except (AttributeError, OSError, RuntimeError):
            pass

def _team_status_code(ball_state):
    # A one-frame catcher observation is deliberately not a team possession
    # claim.  The peer must not yield a free ball or launch a pass until the
    # same 0.25s physical hold confirmation that gates our own carries/kicks.
    if ball_state == "held" and _REAL_HOLD_CONFIRMED:
        return Communication.STATUS_CATCH_BALL
    if ball_state == "held":
        return Communication.STATUS_CHASING
    if _TEAM_ACTION_STATE in AUSTRALIA.Team.ContestStates:
        return TEAM_STATUS_CONTESTING
    if ball_state == "free":
        return Communication.STATUS_CHASING
    return Communication.STATUS_NO_BALL

def _team_role_code():
    color = str(getattr(Settings.RoboInfo, "Color", getattr(Settings.RoboInfo, "color", "Y"))).upper()
    defence = _strategy_role() == "SUPPORT" or _is_defence_role(_strategy_role())
    if color.startswith("B"):
        return Communication.ROLE_DEFENCE_BLUE if defence else Communication.ROLE_OFFENCE_BLUE
    return Communication.ROLE_DEFENCE_YELLOW if defence else Communication.ROLE_OFFENCE_YELLOW

def _valid_team_frame(frame):
    """Accept only a versioned, bounded real-team heartbeat."""

    if not isinstance(frame, dict):
        return False
    try:
        version = int(frame.get("version"))
        identity = int(frame.get("id"))
        session = int(frame.get("session"))
        sequence = int(frame.get("seq"))
        role = int(frame.get("role"))
        status = int(frame.get("status"))
        flags = int(frame.get("flags", frame.get("reserve", 0)))
    except (TypeError, ValueError):
        return False
    point = _physical_xy(frame.get("signed_xy"))
    yaw = frame.get("yaw")
    try:
        yaw = float(yaw)
    except (TypeError, ValueError):
        return False
    return (
        version == REAL_TEAM_PROTOCOL_VERSION
        and identity in (1, 2)
        and session != 0
        and 0 <= sequence < REAL_TEAM_SEQ_MODULUS
        and role in TEAM_VALID_ROLES
        and status in TEAM_VALID_STATUSES
        and 0 <= flags <= 0xFF
        and point is not None
        and math.isfinite(yaw)
    )

def _valid_team_frame_legacy(frame):
    """Validate the pre-existing in-process facade used only by old tests."""

    if not isinstance(frame, dict):
        return False
    try:
        role = int(frame.get("role"))
        status = int(frame.get("status"))
    except (TypeError, ValueError):
        return False
    return role in TEAM_VALID_ROLES and status in TEAM_VALID_STATUSES

def _sync_real_team_generation(link):
    """Drop cached peer state immediately when the radio generation changes."""

    global _TEAM_SAMPLE, _TEAM_LINK_GENERATION, _TEAMMATE_EVER_SEEN

    if not isinstance(link, RealTeamLink):
        return False
    if link.generation == _TEAM_LINK_GENERATION:
        return False
    _TEAM_LINK_GENERATION = link.generation
    _TEAM_SAMPLE = (None, -math.inf)
    _TEAMMATE_EVER_SEEN = False
    return True

def _refresh_team_communication(pose, ball_state):
    """Send one heartbeat and accept only a genuinely new peer payload."""

    global _TEAM_SAMPLE, _TEAM_LINK_GENERATION
    global _TEAM_LAST_SEND_TIME, _TEAM_LINK_STARTED_AT, _TEAMMATE_EVER_SEEN

    link = _TEAM_LINK
    point = _physical_xy(pose)
    if link is None or point is None or not _pose_valid(pose):
        return
    now = time.monotonic()
    if not math.isfinite(_TEAM_LINK_STARTED_AT):
        _TEAM_LINK_STARTED_AT = now

    if not isinstance(link, RealTeamLink):
        # Backward-compatible test/legacy facade path.  Real deployment always
        # constructs RealTeamLink above, so replay protection is never lost on
        # the physical path.
        if now - _TEAM_LAST_SEND_TIME >= AUSTRALIA.Tactics.TEAM_STATE_SEND_PERIOD_S:
            status = _team_status_code(ball_state)
            try:
                link.SendStatus(
                    role=_team_role_code(),
                    status=status,
                    position=point,
                    reserve=_encode_team_yaw(pose[2])
                    | (TEAM_DRIBBLE_READY_MASK if (
                        ball_state != "held"
                        and status in {Communication.STATUS_CHASING, Communication.STATUS_NO_BALL}
                        and bool(getattr(peripheral, "DribbleStatus", False))
                    ) else 0),
                )
            except (AttributeError, OSError, RuntimeError, TypeError, ValueError):
                pass
            _TEAM_LAST_SEND_TIME = now
        try:
            frame = link.Receive()
        except (AttributeError, OSError, RuntimeError, TypeError, ValueError):
            return
        if isinstance(frame, dict) and frame.get("id") == _peer_robot_id():
            if _physical_xy(frame.get("signed_xy")) is not None and _valid_team_frame_legacy(frame):
                _TEAM_SAMPLE = (dict(frame), now)
                _TEAMMATE_EVER_SEEN = True
        return

    # Pair-id changes are exposed by the latest lower-board bridge when the
    # active Bits wrapper provides a getter.  Clear the old sample immediately
    # so a previous pairing cannot remain a teammate obstacle/carrier.
    _sync_real_team_generation(link)

    if now - _TEAM_LAST_SEND_TIME >= AUSTRALIA.Tactics.TEAM_STATE_SEND_PERIOD_S:
        status = _team_status_code(ball_state)
        # Commanded dribbler state alone cannot mean a receiver is ready. A
        # tentative catcher observation must be settled locally before this
        # robot advertises either carrier or receiver availability.
        dribble_ready = (
            ball_state != "held"
            and status in {Communication.STATUS_CHASING, Communication.STATUS_NO_BALL}
            and bool(getattr(peripheral, "DribbleStatus", False))
        )
        link.send_status(
            role=_team_role_code(),
            status=status,
            position=point,
            yaw=pose[2],
            flags=(TEAM_DRIBBLE_READY_MASK if dribble_ready else 0),
        )
        _TEAM_LAST_SEND_TIME = now

    frame = link.poll()
    # ``poll()`` can discover a pair-id change while returning no frame.  Do
    # this second check in the same control tick so the old cached teammate
    # cannot be used for obstacle removal or role selection for one more tick.
    if _sync_real_team_generation(link):
        return
    if frame is None:
        return
    if _physical_xy(frame.get("signed_xy")) is None or not _valid_team_frame(frame):
        return
    _TEAM_SAMPLE = (dict(frame), now)
    _TEAMMATE_EVER_SEEN = True

def _teammate_mode():
    """Classify static pairing versus a confirmed physical teammate loss."""

    if SOLO_TAKEOVER:
        return "solo", "team_link_disabled"
    if _TEAM_LINK is None:
        return "static", "radio_unconfigured"
    if isinstance(_TEAM_LINK, RealTeamLink) and not _TEAM_LINK.paired:
        peer, timestamp = _TEAM_SAMPLE
        age = time.monotonic() - timestamp
        if _TEAMMATE_EVER_SEEN and age > AUSTRALIA.Tactics.TEAMMATE_LOSS_CONFIRM_S:
            return "solo", "pair_generation_lost"
        if (not _TEAMMATE_EVER_SEEN and math.isfinite(_TEAM_LINK_STARTED_AT)
                and time.monotonic() - _TEAM_LINK_STARTED_AT > AUSTRALIA.Tactics.TEAMMATE_STARTUP_TAKEOVER_S):
            return "solo", "startup_pair_timeout"
        return "pairing", "not_paired"
    peer, timestamp = _TEAM_SAMPLE
    age = time.monotonic() - timestamp
    if isinstance(peer, dict) and 0.0 <= age <= AUSTRALIA.Tactics.TEAM_STATE_MAX_AGE_S:
        if not _valid_team_frame(peer):
            return "pairing", "invalid_heartbeat"
        try:
            peer_role = int(peer.get("role", Communication.ROLE_BAN))
        except (TypeError, ValueError):
            peer_role = Communication.ROLE_BAN
        if peer_role != Communication.ROLE_BAN:
            return "paired", None
        return "solo", "teammate_declared_unavailable"
    if (not _TEAMMATE_EVER_SEEN and math.isfinite(_TEAM_LINK_STARTED_AT)
            and time.monotonic() - _TEAM_LINK_STARTED_AT > AUSTRALIA.Tactics.TEAMMATE_STARTUP_TAKEOVER_S):
        return "solo", "startup_pair_timeout"
    if _TEAMMATE_EVER_SEEN and age > AUSTRALIA.Tactics.TEAMMATE_LOSS_CONFIRM_S:
        return "solo", "heartbeat_lost"
    return "pairing", None

def _robot_id():
    configured = getattr(Settings.RoboInfo, "number", 1)
    return int(configured)

# BEGIN GENERATED AUSTRALIA TACTICAL PORT
# Source SHA-256: f7d90ecabf50d30caa9e0baea54eb166ee30c810e48ab8c3bc8db08d09a4ad55
TACTICAL_SOURCE_SHA256 = "f7d90ecabf50d30caa9e0baea54eb166ee30c810e48ab8c3bc8db08d09a4ad55"

MATCH_LOG_DIR = SRC_DIR.parent / "match_logs"

_MATCH_JOURNAL = None

_MATCH_JOURNAL_PATH = None

restricted = getattr(cfg.Motion, "RestrictedArea", None)

penalty_x_limit = float(getattr(restricted, "XLimit", Settings.Field.Long / 2 - 30.0))

clearance = float(getattr(restricted, "Clearance", 5.0))

GOAL_LINE_X = max(20.0, penalty_x_limit - clearance - 25.0)

BOUNDARY_CONTACT_LATERAL_CM = 2.0 * AUSTRALIA.Tactics.SIM_BODY_RADIUS_CM + 4.0

TEAM_YAW_STEPS = 128

CHASE_CONTACT_RELATIVE_TARGET_MPS = AUSTRALIA.Tactics.SIM_DRIBBLER_CAPTURE_RELATIVE_LIMIT_MPS - 0.02

COOP_FIELD_MARGIN_CM = AUSTRALIA.Tactics.SIM_BODY_RADIUS_CM + AUSTRALIA.Tactics.MOTION_CLEARANCE_CM + 2.0

CARRIER_GOAL_LINE_TEAM_CLEARANCE_CM = 2.0 * AUSTRALIA.Tactics.SIM_BODY_RADIUS_CM + 6.0

_RADICAL_LAST_ENEMY = None

_RADICAL_LAST_OVERLAP_TIME = 0.0

_RADICAL_PREVIOUS_ENEMIES = []

_RADICAL_PREVIOUS_ENEMY_TIME = 0.0

_MACAU_DEFENCE_LAST_CARRIER = None

_MACAU_DEFENCE_LAST_TIME = -math.inf

_MACAU_DEFENCE_UNTIL = -math.inf

_HIDDEN_MACAU_RUNNER = None

_HIDDEN_MACAU_UNTIL = -math.inf

_CHASE_ALIGNED = False

_ACTIVE_SHOT = None

_ACTIVE_SHOT_SINCE = 0.0

_ACTIVE_WALL_PLAN = None

_WALL_READY_PLAN = None

_WALL_READY_BALL = None

_WALL_READY_TIME = -math.inf

_WALL_READY_VALID = False

_WALL_RETREAT_TARGET = None

_WALL_RETREAT_SINCE = -math.inf

_GOAL_POST_RETREAT_TARGET = None

_GOAL_POST_RETREAT_SINCE = -math.inf

_GOAL_POST_RETREAT_PHASE = None

_GOAL_POST_RETREAT_STALLS = 0

_BALL_DUEL_SINCE = None

_DUEL_EJECT_UNTIL = -math.inf

_DIRECT_READY_ROUTE = None

_DIRECT_READY_POSE = None

_DIRECT_READY_TIME = -math.inf

_DIRECT_READY_VALID = False

_MACAU_SIDE = None

_MACAU_PHASE = None

_MACAU_SHIELD_YAW = None

_MACAU_LANE_TARGET_Y = None

_SPLIT_TARGET = None

_SPLIT_SIDE = None

_SPLIT_SINCE = 0.0

_SLIP_TARGET = None

_SLIP_SIDE = None

_SLIP_SINCE = 0.0

_DIRECT_BLOCKED = False

_DIRECT_BLOCKED_SINCE = None

_LAST_SHOT_TIME = -math.inf

_LAST_BALL_HELD = False

_AWAIT_BALL_RELEASE = False

_LAST_SEEN_BALL_FIELD = None

_LAST_SEEN_BALL_TIME = -math.inf

_DEFENCE_UNTIL = -math.inf

_DEFENCE_BALL_SAMPLE = None

_DEFENCE_BALL_VELOCITY = None

_TEAM_LINK = None

_TEAM_SELF_STATE = None

_TEAM_SAMPLE = (None, -math.inf)

_TEAM_LAST_SEND_TIME = -math.inf

_TEAM_ACTION_STATE = None

_TEAM_LINK_STARTED_AT = -math.inf

_TEAMMATE_EVER_SEEN = False

_TEAMMATE_MODE = "pairing"

_TEAMMATE_MODE_REASON = None

_TEAMMATE_HOLDER_MEMORY = None

_TEAMMATE_HOLDER_UNTIL = -math.inf

_COOP_PLAN = None

_COOP_SINCE = None

_COOP_READY_SINCE = None

_COOP_RETRY_AFTER = -math.inf

_PASS_FLIGHT = None

_PASS_PHYSICS = None

_DROP_BALL_PREP = None

_WALL_RETRY_AFTER = -math.inf

_MACAU_RETRY_AFTER = -math.inf

_BOUNDARY_POSE_SAMPLE = None

_SEMANTIC_POSE_SAMPLE = None

_SEMANTIC_LAST_VELOCITY = None

_BOUNDARY_BRAKE_UNTIL = -math.inf

_BOUNDARY_BRAKE_VECTOR = None

def _reset_cooperation():
    global _COOP_PLAN, _COOP_SINCE, _COOP_READY_SINCE, _PASS_FLIGHT

    _COOP_PLAN = None
    _COOP_SINCE = None
    _COOP_READY_SINCE = None
    _PASS_FLIGHT = None

def _reset_attack():
    global _ACTIVE_SHOT, _ACTIVE_SHOT_SINCE, _ACTIVE_WALL_PLAN
    global _WALL_RETREAT_TARGET, _WALL_RETREAT_SINCE, _BALL_DUEL_SINCE
    global _GOAL_POST_RETREAT_TARGET, _GOAL_POST_RETREAT_SINCE
    global _GOAL_POST_RETREAT_PHASE, _GOAL_POST_RETREAT_STALLS
    global _MACAU_SIDE, _MACAU_PHASE, _MACAU_SHIELD_YAW, _MACAU_LANE_TARGET_Y
    global _SPLIT_TARGET, _SPLIT_SIDE, _SPLIT_SINCE
    global _SLIP_TARGET, _SLIP_SIDE, _SLIP_SINCE
    global _DIRECT_BLOCKED, _DIRECT_BLOCKED_SINCE

    _ACTIVE_SHOT = None
    _ACTIVE_SHOT_SINCE = 0.0
    _ACTIVE_WALL_PLAN = None
    _WALL_RETREAT_TARGET = None
    _WALL_RETREAT_SINCE = -math.inf
    _GOAL_POST_RETREAT_TARGET = None
    _GOAL_POST_RETREAT_SINCE = -math.inf
    _GOAL_POST_RETREAT_PHASE = None
    _GOAL_POST_RETREAT_STALLS = 0
    _BALL_DUEL_SINCE = None
    _MACAU_SIDE = None
    _MACAU_PHASE = None
    _MACAU_SHIELD_YAW = None
    _MACAU_LANE_TARGET_Y = None
    _SPLIT_TARGET = None
    _SPLIT_SIDE = None
    _SPLIT_SINCE = 0.0
    _SLIP_TARGET = None
    _SLIP_SIDE = None
    _SLIP_SINCE = 0.0
    _DIRECT_BLOCKED = False
    _DIRECT_BLOCKED_SINCE = None

def reset_strategy():
    global _RADICAL_LAST_ENEMY, _RADICAL_LAST_OVERLAP_TIME
    global _RADICAL_PREVIOUS_ENEMIES, _RADICAL_PREVIOUS_ENEMY_TIME
    global _MACAU_DEFENCE_LAST_CARRIER, _MACAU_DEFENCE_LAST_TIME, _MACAU_DEFENCE_UNTIL
    global _HIDDEN_MACAU_RUNNER, _HIDDEN_MACAU_UNTIL
    global _CHASE_ALIGNED, _LAST_BALL_HELD, _LAST_SHOT_TIME, _AWAIT_BALL_RELEASE
    global _LAST_SEEN_BALL_FIELD, _LAST_SEEN_BALL_TIME
    global _COOP_RETRY_AFTER
    global _DROP_BALL_PREP
    global _WALL_RETRY_AFTER
    global _MACAU_RETRY_AFTER
    global _TEAM_SELF_STATE, _TEAM_SAMPLE, _TEAM_LAST_SEND_TIME, _TEAM_ACTION_STATE
    global _TEAMMATE_HOLDER_MEMORY, _TEAMMATE_HOLDER_UNTIL
    global _BOUNDARY_POSE_SAMPLE, _SEMANTIC_POSE_SAMPLE, _SEMANTIC_LAST_VELOCITY
    global _BOUNDARY_BRAKE_UNTIL, _BOUNDARY_BRAKE_VECTOR
    global _DIRECT_READY_ROUTE, _DIRECT_READY_POSE, _DIRECT_READY_TIME, _DIRECT_READY_VALID
    global _WALL_READY_PLAN, _WALL_READY_BALL, _WALL_READY_TIME, _WALL_READY_VALID
    global _WALL_RETREAT_TARGET, _WALL_RETREAT_SINCE, _BALL_DUEL_SINCE, _DUEL_EJECT_UNTIL
    global _MACAU_PHASE, _MACAU_SHIELD_YAW, _MACAU_LANE_TARGET_Y

    _reset_attack()
    _reset_defence()
    _reset_cooperation()
    _COOP_RETRY_AFTER = -math.inf
    _DROP_BALL_PREP = None
    _WALL_RETRY_AFTER = -math.inf
    _MACAU_RETRY_AFTER = -math.inf
    _RADICAL_LAST_ENEMY = None
    _RADICAL_LAST_OVERLAP_TIME = 0.0
    _RADICAL_PREVIOUS_ENEMIES = []
    _RADICAL_PREVIOUS_ENEMY_TIME = 0.0
    _MACAU_DEFENCE_LAST_CARRIER = None
    _MACAU_DEFENCE_LAST_TIME = -math.inf
    _MACAU_DEFENCE_UNTIL = -math.inf
    _HIDDEN_MACAU_RUNNER = None
    _HIDDEN_MACAU_UNTIL = -math.inf
    _CHASE_ALIGNED = False
    _LAST_BALL_HELD = False
    _LAST_SHOT_TIME = -math.inf
    _AWAIT_BALL_RELEASE = False
    _LAST_SEEN_BALL_FIELD = None
    _LAST_SEEN_BALL_TIME = -math.inf
    _TEAM_SELF_STATE = None
    _TEAM_SAMPLE = (None, -math.inf)
    _TEAM_LAST_SEND_TIME = -math.inf
    _TEAM_ACTION_STATE = None
    _TEAMMATE_HOLDER_MEMORY = None
    _TEAMMATE_HOLDER_UNTIL = -math.inf
    _BOUNDARY_POSE_SAMPLE = None
    _SEMANTIC_POSE_SAMPLE = None
    _SEMANTIC_LAST_VELOCITY = None
    _BOUNDARY_BRAKE_UNTIL = -math.inf
    _BOUNDARY_BRAKE_VECTOR = None
    _DIRECT_READY_ROUTE = None
    _DIRECT_READY_POSE = None
    _DIRECT_READY_TIME = -math.inf
    _DIRECT_READY_VALID = False
    _WALL_READY_PLAN = None
    _WALL_READY_BALL = None
    _WALL_READY_TIME = -math.inf
    _WALL_READY_VALID = False
    _WALL_RETREAT_TARGET = None
    _WALL_RETREAT_SINCE = -math.inf
    _MACAU_PHASE = None
    _MACAU_SHIELD_YAW = None
    _MACAU_LANE_TARGET_Y = None
    _BALL_DUEL_SINCE = None
    _DUEL_EJECT_UNTIL = -math.inf
    _reset_real_ball_control()
    _reset_motion_metrics()
    _reset_last_seen_ball()
    global _TACTICAL_STATE, _TACTICAL_INTENT, _TACTICAL_TARGET, _TACTICAL_TRACE
    global _REAL_TACTICAL_PEER_SAMPLE, _REAL_TACTICAL_PEER_STATE
    _TACTICAL_STATE = None
    _TACTICAL_INTENT = None
    _TACTICAL_TARGET = None
    _TACTICAL_TRACE = []
    _REAL_TACTICAL_PEER_SAMPLE = None
    _REAL_TACTICAL_PEER_STATE = None
    try:
        UnitedPosition.ResetBallLandingPredictor()
    except (AttributeError, TypeError, ValueError):
        pass
    if hasattr(safe_defence, "target_y"):
        del safe_defence.target_y

def _encode_team_yaw(yaw_deg):
    return int(round((float(yaw_deg) % 360.0) * TEAM_YAW_STEPS / 360.0)) % TEAM_YAW_STEPS

def _decode_team_yaw(encoded):
    return float(int(encoded) % TEAM_YAW_STEPS) * 360.0 / TEAM_YAW_STEPS

def _encode_team_reserve(yaw_deg, dribble_ready):
    return _encode_team_yaw(yaw_deg) | (TEAM_DRIBBLE_READY_MASK if dribble_ready else 0)

def _semantic_robot_velocity(pose):
    global _SEMANTIC_POSE_SAMPLE, _SEMANTIC_LAST_VELOCITY

    if not _pose_valid(pose):
        _SEMANTIC_POSE_SAMPLE = None
        _SEMANTIC_LAST_VELOCITY = None
        return None
    now = time.monotonic()
    current = (float(pose[0]), float(pose[1]), float(pose[2]), now)
    previous = _SEMANTIC_POSE_SAMPLE
    if previous is None:
        _SEMANTIC_POSE_SAMPLE = current
        _SEMANTIC_LAST_VELOCITY = None
        return None
    dt = current[3] - previous[3]
    if dt < 0.0:
        _SEMANTIC_POSE_SAMPLE = current
        _SEMANTIC_LAST_VELOCITY = None
        return None
    if dt < 0.008:
        # 同一控制周期内通信、追球和日志都会读取这份观测。保留上一份计算结果，
        # 既不把相同位姿重置为零速度，也不让第二个调用破坏下一帧的差分样本。
        return _SEMANTIC_LAST_VELOCITY
    _SEMANTIC_POSE_SAMPLE = current
    if dt > 0.25:
        _SEMANTIC_LAST_VELOCITY = None
        return None
    field_vx = (current[0] - previous[0]) / 100.0 / dt
    field_vy = (current[1] - previous[1]) / 100.0 / dt
    yaw_rate = math.radians(angle_error_deg(previous[2], current[2]) / dt)
    if (
        not all(math.isfinite(value) for value in (field_vx, field_vy, yaw_rate))
        or math.hypot(field_vx, field_vy) > AUSTRALIA.Tactics.SIM_MAX_SPEED_MPS * 1.5
        or abs(yaw_rate) > math.radians(AUSTRALIA.Tactics.CHASE_TURN_MAX_DEG_S * 1.5)
    ):
        # A referee placement or stale sample is a localisation discontinuity,
        # not physical velocity.  Discard it rather than leaking its cause.
        _SEMANTIC_LAST_VELOCITY = None
        return None
    _SEMANTIC_LAST_VELOCITY = field_vx, field_vy, yaw_rate
    return _SEMANTIC_LAST_VELOCITY

def _observed_field_velocity(pose):
    """用连续场地位姿计算制动速度，避免依赖模拟器会变化的本地速度轴。"""

    global _BOUNDARY_POSE_SAMPLE

    now = time.monotonic()
    current = (float(pose[0]), float(pose[1]), now)
    previous = _BOUNDARY_POSE_SAMPLE
    _BOUNDARY_POSE_SAMPLE = current
    if previous is None:
        return None
    dx = current[0] - previous[0]
    dy = current[1] - previous[1]
    dt = current[2] - previous[2]
    if dt < 0.008 or dt > 0.25:
        return None
    field_vx = dx / 100.0 / dt
    field_vy = dy / 100.0 / dt
    if not all(math.isfinite(value) for value in (field_vx, field_vy)):
        return None
    # 裁判重摆和延迟状态包可能使位置瞬间跳变，不能把它当作真实速度。
    if math.hypot(field_vx, field_vy) > 4.5:
        return None
    return field_vx, field_vy

def _decode_team_frame(frame, now):
    """Convert one LunaPre communication frame into the teammate shape used by tactics."""

    if not isinstance(frame, dict):
        return None
    robot_id = frame.get("id")
    if robot_id not in AUSTRALIA.Team.RobotIds or robot_id == _robot_id():
        return None
    point = _field_xy(frame.get("signed_xy"))
    if point is None:
        return None
    try:
        role = int(frame.get("role"))
        status = int(frame.get("status"))
        reserve = int(frame.get("reserve", 0))
        yaw = _decode_team_yaw(reserve)
    except (TypeError, ValueError):
        return None

    local_vx = local_vy = spin = 0.0
    previous, previous_time = _TEAM_SAMPLE
    dt = now - previous_time
    if (isinstance(previous, dict) and previous.get("id") == robot_id
            and AUSTRALIA.Tactics.ENEMY_SAMPLE_MIN_S <= dt <= AUSTRALIA.Tactics.ENEMY_SAMPLE_MAX_S):
        field_vx = (point[0] - previous["x_cm"]) / dt
        field_vy = (point[1] - previous["y_cm"]) / dt
        theta = math.radians(yaw)
        local_vx = (field_vx * math.cos(theta) + field_vy * math.sin(theta)) / 100.0
        local_vy = (-field_vx * math.sin(theta) + field_vy * math.cos(theta)) / 100.0
        spin = math.radians(angle_error_deg(previous["yaw_deg"], yaw) / dt)
        if math.hypot(local_vx, local_vy) > AUSTRALIA.Tactics.SIM_MAX_SPEED_MPS * 1.5:
            local_vx = local_vy = 0.0
        if abs(spin) > math.radians(AUSTRALIA.Tactics.CHASE_TURN_MAX_DEG_S * 1.5):
            spin = 0.0

    enabled = role != Communication.ROLE_BAN
    return {
        "id": robot_id,
        "x_cm": point[0],
        "y_cm": point[1],
        "yaw_deg": yaw,
        "vx_forward_mps": local_vx,
        "vy_left_mps": local_vy,
        "wz_rad_s": spin,
        "active": enabled,
        # This is a teammate's self-reported radio availability, never a
        # simulator referee or penalty field.
        "available": enabled,
        "has_ball": status == Communication.STATUS_CATCH_BALL,
        "contesting": status == TEAM_STATUS_CONTESTING,
        "dribble_command": bool(reserve & TEAM_DRIBBLE_READY_MASK),
        "dribble_available": enabled,
    }

def _available_teammates(packet=None):
    robots = _team_robots(packet)
    if robots is None:
        return []
    return [item for ident, item in robots.items()
            if ident != _robot_id() and item.get("available") is True]

def _teammate_id():
    """Return the fixed partner ID for this two-robot strategy."""

    return next((ident for ident in AUSTRALIA.Team.RobotIds if ident != _robot_id()), None)

def _solo_takeover_active():
    return _TEAMMATE_MODE == "solo"

def _sync_teammate_mode():
    """Persist pairing transitions, log them, and cancel stale two-car plans."""

    global _TEAMMATE_MODE, _TEAMMATE_MODE_REASON

    mode, reason = _teammate_mode()
    if (mode, reason) == (_TEAMMATE_MODE, _TEAMMATE_MODE_REASON):
        return mode
    previous = _TEAMMATE_MODE
    _TEAMMATE_MODE, _TEAMMATE_MODE_REASON = mode, reason
    if mode == "solo":
        _reset_cooperation()
        logger.warning("Robot {} entering solo takeover: {}", _robot_id(), reason)
    elif previous == "solo":
        logger.info("Robot {} teammate link restored; resuming paired tactics", _robot_id())
    _journal_event("teammate_mode", mode=mode, reason=reason, previous=previous)
    return mode

def _sync_cooperation_availability():
    global _COOP_RETRY_AFTER

    plan = _PASS_FLIGHT or _COOP_PLAN
    if plan is None:
        return
    robots = _team_robots()
    ids = (plan["sender"], plan["receiver"])
    invalid = robots is None or any(ident not in robots or robots[ident].get("available") is not True
                                   for ident in ids)
    if invalid:
        _reset_cooperation()
        if _ACTIVE_SHOT == "pass":
            _reset_attack()
        _COOP_RETRY_AFTER = time.monotonic() + AUSTRALIA.Tactics.COOP_RETRY_S

def _position_step(target, speed_gain=0.7, arrival_cm=3.0,
                   speed_limit=AUSTRALIA.Tactics.SIM_MAX_SPEED_MPS, near_speed_limit=AUSTRALIA.Tactics.SIM_MAX_SPEED_MPS):
    pose = UnitedPosition.FieldRobotPose()
    if not _pose_valid(pose) or not _pose_valid(target):
        safe_stop(chassis, peripheral)
        return False
    target = _safe_target(*target[:3])
    waypoint = UnitedPosition._navigate_waypoint(pose, target)
    distance = math.dist(pose[:2], waypoint[:2])
    recovering = UnitedPosition.ProjectSafePose(pose) is not None
    error = angle_error_deg(pose[2], target[2])
    if distance <= arrival_cm and not recovering:
        if abs(error) > AUSTRALIA.Tactics.PASS_RECEIVE_ALIGN_DEG:
            _safe_robot_move(0.0, 0.0, _turn_command(error))
            return False
        chassis.stop()
        return math.dist(pose[:2], target[:2]) <= arrival_cm
    if distance <= 0.01:
        chassis.stop()
        return False
    # ``speed_gain`` used to be ignored, so a receive/cover move requested at
    # 0.5 and a full-speed press requested at 2.0 both sent the same 3m/s
    # command.  That made turns overshoot and produced the jerky movement seen
    # in the recent logs.  Use a distance-proportional profile: long runs still
    # reach the simulator's 3m/s ceiling, while the last few centimetres settle
    # without changing the aggressive chase/press limits.
    try:
        gain = clamp(float(speed_gain), 0.25, 2.0)
        cruise_limit = clamp(float(speed_limit), AUSTRALIA.Tactics.MOTION_MIN_MPS, AUSTRALIA.Tactics.SIM_MAX_SPEED_MPS)
        near_limit = clamp(float(near_speed_limit), AUSTRALIA.Tactics.MOTION_MIN_MPS, AUSTRALIA.Tactics.SIM_MAX_SPEED_MPS)
    except (TypeError, ValueError):
        gain, cruise_limit, near_limit = 1.0, AUSTRALIA.Tactics.SIM_MAX_SPEED_MPS, AUSTRALIA.Tactics.SIM_MAX_SPEED_MPS
    speed = clamp(distance * 0.02 * gain, AUSTRALIA.Tactics.MOTION_MIN_MPS, cruise_limit)
    if distance < 18.0 or recovering:
        speed = min(speed, near_limit)
    theta = math.radians(pose[2])
    vx, vy = [(waypoint[i] - pose[i]) * speed / distance for i in (0, 1)]
    turn = _turn_command(error) if abs(error) > AUSTRALIA.Tactics.PASS_RECEIVE_ALIGN_DEG else 0.0
    _safe_robot_move(vx * math.cos(theta) + vy * math.sin(theta),
                     -vx * math.sin(theta) + vy * math.cos(theta), turn,
                     recovery=recovering, speed_limit=speed_limit)
    return False

def _motion_path_safe(pose, vx, vy, turn, horizon, recovery=False):
    speed = math.hypot(vx, vy)
    travel_cm = speed * horizon * 100.0
    if not math.isfinite(travel_cm) or travel_cm > 300 or horizon < 0:
        return False
    if speed < 1e-6:
        return True
    steps = max(2, math.ceil(travel_cm))
    yaw, omega = math.radians(pose[2]), math.radians(turn)
    rule_origin = UnitedPosition.ProjectSafePose(pose)
    rule_recovering = rule_origin is not None
    if rule_recovering and not recovery:
        return False
    rule_distance = math.dist(pose[:2], rule_origin[:2]) if rule_recovering else 0.0
    for index in range(1, steps + 1):
        dt = horizon * index / steps
        if abs(omega) < 1e-6:
            dx, dy = vx * dt, vy * dt
        else:
            a, b = math.sin(omega * dt) / omega, (1.0 - math.cos(omega * dt)) / omega
            dx, dy = a * vx - b * vy, b * vx + a * vy
        point = [pose[0] + 100.0 * (dx * math.cos(yaw) - dy * math.sin(yaw)),
                 pose[1] + 100.0 * (dx * math.sin(yaw) + dy * math.cos(yaw)), pose[2] + turn * dt]
        rule_unsafe = UnitedPosition.ProjectSafePose(point) is not None
        if rule_unsafe:
            # 机器人已经被裁判放在限制区内时，只允许持续缩短到实际规则线的距离；
            # 这不是停机/回退状态，下一帧仍照常运行当前策略。
            if not rule_recovering:
                return False
            distance_to_rule = math.dist(point[:2], rule_origin[:2])
            if distance_to_rule >= rule_distance - 0.01:
                return False
            rule_distance = distance_to_rule
        elif rule_recovering:
            rule_distance = 0.0
    return True

def _rule_field_approach_clearance_cm(pose, field_vx, field_vy):
    speed = math.hypot(field_vx, field_vy)
    if speed < AUSTRALIA.Tactics.MOTION_MIN_MPS:
        return None
    clearances = []
    x_sign = math.copysign(1.0, pose[0]) if abs(pose[0]) > 0.01 else math.copysign(1.0, field_vx)
    y_sign = math.copysign(1.0, pose[1]) if abs(pose[1]) > 0.01 else math.copysign(1.0, field_vy)
    x_progress = field_vx * x_sign / speed
    y_progress = field_vy * y_sign / speed
    if x_progress > 1e-6:
        clearances.append((SIM_WALL_CONTACT_X_CM - 0.1 - abs(pose[0])) / x_progress)
    if y_progress > 1e-6:
        clearances.append((SIM_WALL_CONTACT_Y_CM - 0.1 - abs(pose[1])) / y_progress)
    # 禁区的两个场地侧角是圆角，不能将其放大为矩形。只在这条速度射线真的
    # 会进入 Unity 的处罚区时才计算制动距离。
    for distance_cm in range(0, int(2.0 * max(Settings.Field.Long, Settings.Field.Short)) + 1):
        point = [pose[0] + field_vx / speed * distance_cm,
                 pose[1] + field_vy / speed * distance_cm]
        if UnitedPosition.ProjectSafePose([point[0], point[1], pose[2]]) is not None:
            clearances.append(float(distance_cm))
            break
    clearances = [value for value in clearances if value >= 0.0]
    return min(clearances) if clearances else None

def _rule_approach_clearance_cm(pose, vx, vy):
    theta = math.radians(pose[2])
    field_vx = vx * math.cos(theta) - vy * math.sin(theta)
    field_vy = vx * math.sin(theta) + vy * math.cos(theta)
    return _rule_field_approach_clearance_cm(pose, field_vx, field_vy)

def _prevent_boundary_contact_push(pose, vx, vy):
    """去掉会把边线附近对手继续顶向墙的速度分量，保留抢球转向和切向移动。"""

    if _effective_ball_state() == "held":
        return vx, vy
    x_clearance = SIM_WALL_CONTACT_X_CM - abs(float(pose[0]))
    y_clearance = SIM_WALL_CONTACT_Y_CM - abs(float(pose[1]))
    if min(x_clearance, y_clearance) > AUSTRALIA.Tactics.BOUNDARY_CONTACT_EDGE_GUARD_CM:
        return vx, vy

    theta = math.radians(pose[2])
    field_velocity = [
        vx * math.cos(theta) - vy * math.sin(theta),
        vx * math.sin(theta) + vy * math.cos(theta),
    ]
    if math.hypot(*field_velocity) < AUSTRALIA.Tactics.MOTION_MIN_MPS:
        return vx, vy

    for enemy in _current_enemies():
        if math.dist(pose[:2], enemy) > AUSTRALIA.Tactics.BOUNDARY_CONTACT_APPROACH_CM:
            continue
        for axis, limit in ((0, SIM_WALL_CONTACT_X_CM), (1, SIM_WALL_CONTACT_Y_CM)):
            for sign in (-1.0, 1.0):
                if limit - sign * enemy[axis] > AUSTRALIA.Tactics.BOUNDARY_CONTACT_EDGE_GUARD_CM:
                    continue
                outward_gap = sign * (enemy[axis] - pose[axis])
                lateral_gap = abs(enemy[1 - axis] - pose[1 - axis])
                outward_speed = sign * field_velocity[axis]
                if (0.0 <= outward_gap <= AUSTRALIA.Tactics.BOUNDARY_CONTACT_APPROACH_CM
                        and lateral_gap <= BOUNDARY_CONTACT_LATERAL_CM
                        and outward_speed > 0.0):
                    field_velocity[axis] -= sign * outward_speed

    return (
        field_velocity[0] * math.cos(theta) + field_velocity[1] * math.sin(theta),
        -field_velocity[0] * math.sin(theta) + field_velocity[1] * math.cos(theta),
    )

def _move_back_to_rule_area(pose):
    """只有位置库判定违规时才接管并回到安全点。"""

    if UnitedPosition.RestrictedArea():
        return False
    _journal_event("motion_recover_rule", pose=pose,
                   target=UnitedPosition.ProjectSafePose(pose))
    return True

def _rule_motion_path_safe(pose, vx, vy, turn, horizon):
    speed = math.hypot(vx, vy)
    travel_cm = speed * horizon * 100.0
    if not math.isfinite(travel_cm) or travel_cm > 300.0 or horizon < 0.0:
        return False
    steps = max(2, math.ceil(travel_cm))
    yaw, omega = math.radians(pose[2]), math.radians(turn)
    for index in range(1, steps + 1):
        dt = horizon * index / steps
        if abs(omega) < 1e-6:
            dx, dy = vx * dt, vy * dt
        else:
            a = math.sin(omega * dt) / omega
            b = (1.0 - math.cos(omega * dt)) / omega
            dx, dy = a * vx - b * vy, b * vx + a * vy
        point = [
            pose[0] + 100.0 * (dx * math.cos(yaw) - dy * math.sin(yaw)),
            pose[1] + 100.0 * (dx * math.sin(yaw) + dy * math.cos(yaw)),
            pose[2] + turn * dt,
        ]
        legal = _rule_legal_target(*point)
        if math.dist(point[:2], legal[:2]) > 0.01:
            return False
    return True

def _cooperation_context():
    robots = _team_robots()
    if robots is None:
        return None
    holders = [item for item in robots.values() if item["has_ball"]]
    if len(holders) != 1:
        return None
    holder = holders[0]
    allies = [item for item in robots.values() if item["id"] != holder["id"]]
    if len(allies) != 1:
        return None
    receiver = allies[0]
    for item in (holder, receiver):
        if item.get("available") is not True:
            return None
        try:
            if not all(math.isfinite(float(item[key])) for key in
                       ("yaw_deg", "vx_forward_mps", "vy_left_mps", "wz_rad_s")):
                return None
        except (KeyError, TypeError, ValueError):
            return None
    if holder.get("dribble_available") is not True:
        return None
    enemies = _current_enemies()
    start = _held_ball_field_pos([holder["x_cm"], holder["y_cm"], holder["yaw_deg"]])
    return holder, receiver, enemies, start

def _holder_pinned(context):
    if context is None:
        return False
    holder, _, _, _ = context
    try:
        holder_speed = math.hypot(float(holder["vx_forward_mps"]), float(holder["vy_left_mps"]))
    except (KeyError, TypeError, ValueError):
        return False
    if not math.isfinite(holder_speed) or holder_speed > AUSTRALIA.Tactics.PINNED_PASS_MAX_SPEED_MPS:
        return False
    holder_xy = [holder["x_cm"], holder["y_cm"]]
    for enemy_xy in _current_enemies():
        if math.dist(holder_xy, enemy_xy) <= AUSTRALIA.Tactics.PINNED_PASS_CONTACT_CM:
            return True
    return False

def _pass_route(start, receiver_xy, wall_side=0, cushions=0):
    try:
        cushions = int(cushions)
    except (TypeError, ValueError):
        return None
    if cushions < 0 or cushions > AUSTRALIA.Tactics.MAX_PASS_CUSHIONS or (cushions == 0) != (wall_side == 0):
        return None
    _, restitution, _ = _PASS_PHYSICS or (AUSTRALIA.Tactics.PASS_RESISTANCE_MODEL, AUSTRALIA.Tactics.PASS_WALL_RESTITUTION, AUSTRALIA.Tactics.PASS_KICK_IMPULSE_MPS)
    if cushions and restitution <= 0:
        return None
    end = list(receiver_xy)
    for _ in range(4):
        route = (_bank_route(start, end, cushions, wall_side) if cushions
                 else dict(points=[list(start), end], lengths=[math.dist(start, end)],
                           speed_ratios=[], speed_ratio=1.0,
                           aim_yaw=UnitedPosition.Pos2Angle(start, end)))
        if route is None:
            return None
        previous = route["points"][-2]
        distance = math.dist(receiver_xy, previous)
        if distance <= 25.0:
            return None
        end = [receiver_xy[i] + SIM_CATCH_POINT_CM[0] * (previous[i] - receiver_xy[i]) / distance for i in (0, 1)]
    route = (_bank_route(start, end, cushions, wall_side) if cushions
             else dict(points=[list(start), end], lengths=[math.dist(start, end)],
                       speed_ratios=[], speed_ratio=1.0,
                       aim_yaw=UnitedPosition.Pos2Angle(start, end)))
    if route is None or not 30.0 <= sum(route["lengths"]) <= (300.0 if cushions else 160.0):
        return None
    incoming_yaw = UnitedPosition.Pos2Angle(end, route["points"][-2])
    route.update(target=[*receiver_xy, incoming_yaw], side=wall_side, cushions=cushions)
    return route

def _pass_launch_speed(plan, arrival_speed=AUSTRALIA.Tactics.PASS_RECEIVE_SPEED_MPS):
    model, _, impulse = _PASS_PHYSICS or (AUSTRALIA.Tactics.PASS_RESISTANCE_MODEL, AUSTRALIA.Tactics.PASS_WALL_RESTITUTION, AUSTRALIA.Tactics.PASS_KICK_IMPULSE_MPS)
    resistance = AUSTRALIA.Tactics.PASS_CALIBRATED_DECEL_MPS2
    drag = AUSTRALIA.Tactics.PASS_CALIBRATED_DRAG_MPS2
    if not all(math.isfinite(v) and v >= 0.0 for v in (resistance, drag, impulse)):
        return None
    maximum = AUSTRALIA.Tactics.PASS_MAX_LAUNCH_MPS if arrival_speed <= AUSTRALIA.Tactics.PASS_RECEIVE_SPEED_MPS else AUSTRALIA.Tactics.SHOT_MAX_LAUNCH_MPS
    backspin = AUSTRALIA.Tactics.PASS_CALIBRATED_BACKSPIN_MPS
    speed = arrival_speed
    ratios = plan.get("speed_ratios", [plan["speed_ratio"]])
    for index in range(len(plan["lengths"]) - 1, -1, -1):
        distance = plan["lengths"][index] / 100.0
        if model == "damping":
            speed += (resistance + drag) * distance
        elif index == 0 and backspin > 0:
            low, high = speed, maximum
            if low >= high:
                return None
            friction_accel = 0.30492 * 9.81
            required = speed + drag * distance
            for _ in range(18):
                launch = (low + high) / 2.0
                slip_time = (launch + backspin) / (3.5 * friction_accel)
                rolling_speed = (5.0 * launch - 2.0 * backspin) / 7.0
                slip_distance = launch * slip_time - 0.5 * friction_accel * slip_time ** 2
                if rolling_speed <= 0:
                    arrival = 0.0
                elif slip_distance >= distance:
                    arrival = math.sqrt(max(0.0, launch ** 2 - 2.0 * friction_accel * distance))
                else:
                    arrival = math.sqrt(max(0.0, rolling_speed ** 2 - 2.0 * resistance * (distance - slip_distance)))
                if arrival < required:
                    low = launch
                else:
                    high = launch
            speed = high
            if high >= maximum - 0.001:
                return None
        else:
            speed = math.sqrt(speed * speed + 2.0 * resistance * distance) + drag * distance
        if index > 0:
            speed /= ratios[index - 1]
    if not math.isfinite(speed) or speed > min(maximum, impulse):
        return None
    return speed

def _pass_kick_level(required_speed):
    """Map the calibrated required speed to one local four-level kicker command."""

    try:
        speed = float(required_speed)
        maximum = float(AUSTRALIA.Tactics.PASS_KICK_IMPULSE_MPS)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(speed) or not math.isfinite(maximum) or maximum <= 0.0:
        return None
    for level, fraction in enumerate((0.5, 2.0 / 3.0, 5.0 / 6.0, 1.0), start=1):
        if speed <= maximum * fraction:
            return level
    return None

def _cooperative_plan(context):
    holder, receiver, enemies, start = context
    progress = ATTACK_DIR * holder["x_cm"]
    if progress < AUSTRALIA.Tactics.COOP_MIN_PROGRESS_CM:
        return None
    receiver_xy = [receiver["x_cm"], receiver["y_cm"]]
    pass_blockers = list(enemies)
    goal = [ATTACK_DIR * Settings.Field.Long / 2.0, 0.0]
    x = ATTACK_DIR * clamp(progress + 35.0, 20.0, 65.0)
    candidates = []
    for y in (-55.0, -32.0, 32.0, 55.0):
        target = _cooperation_target(x, y, 0.0)
        if target is not None:
            candidates.append(target[:2])
    if ATTACK_DIR * receiver_xy[0] >= max(10.0, progress - 8.0):
        receiver_target = _cooperation_target(receiver_xy[0], receiver_xy[1], 0.0)
        if receiver_target is not None:
            candidates.insert(0, receiver_target[:2])
    pinned = _holder_pinned(context)
    route_options = ((1, -1), (1, 1), (0, 0)) if pinned else (
        (0, 0), (1, -1), (1, 1)
    )
    best, best_score = None, math.inf
    for target in candidates:
        if ATTACK_DIR * target[0] < progress - 8.0 or math.dist(target, start) < 32.0:
            continue
        if not _rule_target_is_legal([*target, 0.0]):
            continue
        if not _lane_clear(receiver_xy, target, enemies + [[holder["x_cm"], holder["y_cm"]]],
                           float(Settings.Vision.ChassisWidth) + 4.0):
            continue
        # 传球本来就是直射窗口被封时的替代方案。接应点只要能安全接球，
        # 不能因为它暂时还没有直射窗口就被整个规划器排除。
        receiver_shot_open = _lane_clear(target, goal, enemies)
        if not _lane_clear(start, goal, [target], AUSTRALIA.Tactics.ALLY_SHOT_LANE_CLEARANCE_CM):
            continue
        if _nearest_enemy(enemies, target)[1] < 30.0:
            continue
        direct_open = _lane_clear(start, target, pass_blockers, 20.0)
        for cushions, side in route_options:
            plan = _pass_route(start, target, side, cushions)
            if plan is None or not all(_lane_clear(a, b, pass_blockers, 20.0)
                                       for a, b in zip(plan["points"], plan["points"][1:])):
                continue
            plan["sender"] = holder["id"]
            speed = _pass_launch_speed(plan)
            if speed is None:
                continue
            bank_cost = (0.0 if pinned and cushions == 1
                         else cushions * (28.0 if direct_open else 7.0))
            score = (math.dist(receiver_xy, target) * 0.55 + sum(plan["lengths"]) * 0.15
                     + math.dist(target, goal) * 0.2 + bank_cost
                     + (0.0 if receiver_shot_open else 18.0))
            if score < best_score:
                best_score = score
                best = dict(plan, sender=holder["id"], receiver=receiver["id"], speed=speed,
                            kick_level=_pass_kick_level(speed), stamp=time.monotonic())
    return best

def _remember_cooperative_plan(plan):
    global _COOP_PLAN, _COOP_READY_SINCE

    if (_COOP_PLAN is None or plan["side"] != _COOP_PLAN["side"]
            or plan.get("cushions") != _COOP_PLAN.get("cushions")
            or plan["sender"] != _COOP_PLAN["sender"]
            or plan["receiver"] != _COOP_PLAN["receiver"]
            or math.dist(plan["target"][:2], _COOP_PLAN["target"][:2]) > AUSTRALIA.Tactics.PASS_READY_DISTANCE_CM):
        _COOP_READY_SINCE = None
    _COOP_PLAN = plan

def _macau_release_support_target(holder):
    try:
        holder_x, holder_y = float(holder["x_cm"]), float(holder["y_cm"])
    except (KeyError, TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in (holder_x, holder_y)):
        return None
    if abs(holder_y) < AUSTRALIA.Tactics.MACAU_ENTRY_MIN_ABS_Y_CM:
        return None
    progress = ATTACK_DIR * holder_x
    target_progress = clamp(progress + 28.0, 15.0, AUSTRALIA.Tactics.MACAU_POCKET_MAX_PROGRESS_CM)
    side = 1.0 if holder_y >= 0.0 else -1.0
    # 一库回弹后球会重新向球门方向进入场内，接应机盘球器面向回弹来球而非继续朝球门。
    receive_yaw = 0.0 if ATTACK_DIR < 0.0 else 180.0
    return _cooperation_target(
        ATTACK_DIR * target_progress,
        side * 30.0,
        receive_yaw,
    )

def _cooperative_support(context):
    macau_target = _macau_release_support_target(context[0])
    if macau_target is not None:
        peripheral.Dribble(True)
        _position_step(
            macau_target,
            speed_gain=1.8,
            arrival_cm=2.0,
            speed_limit=AUSTRALIA.Tactics.SIM_MAX_SPEED_MPS,
            near_speed_limit=AUSTRALIA.Tactics.SIM_MAX_SPEED_MPS,
        )
        return "macau_release_support"
    if time.monotonic() < _COOP_RETRY_AFTER:
        holder = context[0]
        return "teammate_cover" if _support_cover_step([holder["x_cm"], holder["y_cm"]]) else "defence_idle"
    plan = _cooperative_plan(context)
    if plan is None:
        _reset_cooperation()
        holder = context[0]
        return "teammate_cover" if _support_cover_step([holder["x_cm"], holder["y_cm"]]) else "defence_idle"
    if _PASS_FLIGHT is not None:
        _reset_cooperation()
    _remember_cooperative_plan(plan)
    pose = UnitedPosition.FieldRobotPose()
    target = plan["target"]
    target = _cooperation_target(*target)
    if target is None:
        _reset_cooperation()
        holder = context[0]
        return "teammate_cover" if _support_cover_step([holder["x_cm"], holder["y_cm"]]) else "defence_idle"
    peripheral.Dribble(True)
    error = angle_error_deg(pose[2], target[2])
    if math.dist(pose[:2], target[:2]) > AUSTRALIA.Tactics.PASS_READY_DISTANCE_CM:
        _position_step(target, speed_gain=1.7, arrival_cm=AUSTRALIA.Tactics.PASS_READY_DISTANCE_CM,
                       speed_limit=AUSTRALIA.Tactics.SIM_MAX_SPEED_MPS, near_speed_limit=AUSTRALIA.Tactics.SIM_MAX_SPEED_MPS)
        return "receive_run"
    if abs(error) > AUSTRALIA.Tactics.PASS_RECEIVE_ALIGN_DEG:
        _safe_robot_move(0.0, 0.0, _turn_command(error))
        return "receive_align"
    chassis.stop()
    return "receive_ready_bank" if plan["side"] else "receive_ready_direct"

def _pass_participants_ready(plan, context):
    if context is None:
        return False
    holder, receiver, enemies, start = context
    now = time.monotonic()
    if (plan["sender"] != holder["id"] or plan["receiver"] != receiver["id"]
            or holder["id"] != _robot_id() or plan.get("kick_level") not in {1, 2, 3, 4}
            or not 0 <= now - plan["stamp"] <= AUSTRALIA.Tactics.PASS_MAX_AGE_S
            or not 0 <= now - _TEAM_SAMPLE[1] <= AUSTRALIA.Tactics.TEAM_STATE_MAX_AGE_S
            or not _rule_target_is_legal(plan["target"])
            or math.dist(start, plan["points"][0]) > 3.0):
        return False
    tolerance = min(3.0, math.degrees(math.atan2(3.0, sum(plan["lengths"]))))
    return (receiver.get("dribble_command") is True and receiver.get("dribble_available") is True
            and math.dist([receiver["x_cm"], receiver["y_cm"]], plan["target"][:2]) <= AUSTRALIA.Tactics.PASS_READY_DISTANCE_CM
            and abs(angle_error_deg(float(receiver["yaw_deg"]), plan["target"][2])) <= AUSTRALIA.Tactics.PASS_RECEIVE_ALIGN_DEG
            and abs(angle_error_deg(float(holder["yaw_deg"]), plan["aim_yaw"])) <= tolerance
            and all(math.hypot(float(item["vx_forward_mps"]), float(item["vy_left_mps"])) <= 0.30
                    and abs(float(item["wz_rad_s"])) <= 0.80 for item in (holder, receiver))
            and all(_lane_clear(a, b, enemies, 20.0) for a, b in zip(plan["points"], plan["points"][1:])))

def _cooperative_attack():
    global _COOP_SINCE, _COOP_READY_SINCE, _COOP_RETRY_AFTER

    now = time.monotonic()
    context = _cooperation_context()
    if context is None or context[0]["id"] != _robot_id() or now < _COOP_RETRY_AFTER:
        _reset_cooperation()
        return None
    plan = _cooperative_plan(context)
    if plan is None:
        _reset_cooperation()
        return None
    if _COOP_SINCE is None:
        _COOP_SINCE = now
    if now - _COOP_SINCE > min(AUSTRALIA.Tactics.COOP_SETUP_TIMEOUT_S, AUSTRALIA.Tactics.PASS_SETUP_TIMEOUT_S):
        _reset_cooperation()
        _COOP_RETRY_AFTER = now + AUSTRALIA.Tactics.COOP_RETRY_S
        return None
    _remember_cooperative_plan(plan)
    receiver, holder = context[1], context[0]
    age = now - _TEAM_SAMPLE[1]
    ready = (0 <= age <= AUSTRALIA.Tactics.PASS_MAX_AGE_S and receiver.get("dribble_command") is True
             and receiver.get("dribble_available") is True
             and math.dist([receiver["x_cm"], receiver["y_cm"]], plan["target"][:2]) <= AUSTRALIA.Tactics.PASS_READY_DISTANCE_CM
             and abs(angle_error_deg(float(receiver["yaw_deg"]), plan["target"][2])) <= AUSTRALIA.Tactics.PASS_RECEIVE_ALIGN_DEG
             and math.hypot(float(receiver["vx_forward_mps"]), float(receiver["vy_left_mps"])) <= 0.30
             and abs(float(receiver["wz_rad_s"])) <= 0.80)
    _COOP_READY_SINCE = (_COOP_READY_SINCE if _COOP_READY_SINCE is not None else now) if ready else None
    if not ready:
        # 队友跑位期间，持球者保持球在对手远侧并持续换位；不能原地自转等接应。
        return _pass_setup_step(plan, holder)
    tolerance = min(3.0, math.degrees(math.atan2(3.0, sum(plan["lengths"]))))
    if not _turn_held(plan["aim_yaw"], tolerance=tolerance):
        return "pass_aim_bank" if plan["side"] else "pass_aim_direct"
    chassis.stop()
    settled = (math.hypot(float(holder["vx_forward_mps"]), float(holder["vy_left_mps"])) <= 0.30
               and abs(float(holder["wz_rad_s"])) <= 0.80
               and abs(angle_error_deg(float(holder["yaw_deg"]), plan["aim_yaw"])) <= tolerance)
    if not settled or _COOP_READY_SINCE is None or now - _COOP_READY_SINCE < AUSTRALIA.Tactics.PASS_READY_S:
        return "pass_settle"
    if _fire_pass(plan):
        return "bank_pass_sent" if plan["side"] else "direct_pass_sent"
    _COOP_RETRY_AFTER = now + AUSTRALIA.Tactics.COOP_RETRY_S
    return None

def _pass_setup_step(plan, holder):
    pose = _motion_guard()
    if pose is None:
        return "pass_setup_guard"
    enemy, distance = _nearest_enemy(_current_enemies(), pose)
    if enemy is not None and distance <= AUSTRALIA.Tactics.PASS_SETUP_SHIELD_RADIUS_CM:
        side = _escape_wall_side(pose, enemy)
        facing = UnitedPosition.Pos2Angle(enemy, pose)
        target = _carry_safe_target(
            pose[0] + ATTACK_DIR * AUSTRALIA.Tactics.PASS_SETUP_ESCAPE_CM,
            pose[1] + side * AUSTRALIA.Tactics.PASS_SETUP_ESCAPE_CM * 0.55,
            facing,
        )
        state = "pass_setup_shield"
    else:
        side = 1.0 if plan["target"][1] >= pose[1] else -1.0
        target = _carry_safe_target(
            pose[0] + ATTACK_DIR * AUSTRALIA.Tactics.PASS_SETUP_ESCAPE_CM,
            pose[1] + side * AUSTRALIA.Tactics.PASS_SETUP_ESCAPE_CM * 0.35,
            plan["aim_yaw"],
        )
        state = "pass_setup_move"
    if target is None:
        _reset_cooperation()
        return "pass_setup_reset"
    peripheral.Dribble(True)
    _position_step(target, speed_gain=1.5, arrival_cm=1.0,
                   speed_limit=AUSTRALIA.Tactics.SIM_MAX_SPEED_MPS, near_speed_limit=AUSTRALIA.Tactics.SIM_MAX_SPEED_MPS)
    return state

def _cooperative_free_ball(robots, ball, velocity):
    global _PASS_FLIGHT

    now = time.monotonic()
    if _PASS_FLIGHT is None and _COOP_PLAN is not None and now - _COOP_PLAN["stamp"] <= 0.5:
        start, end = _COOP_PLAN["points"][:2]
        if velocity is not None:
            speed = math.hypot(*velocity)
            distance = math.dist(start, end)
            alignment = sum(velocity[i] * (end[i] - start[i]) for i in (0, 1)) / max(1.0, speed * distance)
            if speed >= 15.0 and alignment >= 0.94 and math.dist(ball, start) < 65.0:
                _PASS_FLIGHT = dict(_COOP_PLAN, since=now)
    plan = _PASS_FLIGHT
    if plan is None:
        return None
    if (now - plan["since"] > AUSTRALIA.Tactics.PASS_FLIGHT_TIMEOUT_S
            or plan["sender"] not in robots or plan["receiver"] not in robots
            or any(robots[i].get("available") is not True for i in (plan["sender"], plan["receiver"]))
            or robots[plan["receiver"]].get("dribble_available") is not True):
        _reset_cooperation()
        return None
    enemies = _current_enemies()
    if _nearest_enemy(enemies, ball)[1] < 18.0:
        _reset_cooperation()
        return None
    deviation = math.inf
    for a, b in zip(plan["points"], plan["points"][1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        t = clamp(((ball[0] - a[0]) * dx + (ball[1] - a[1]) * dy) / max(0.01, dx * dx + dy * dy), 0.0, 1.0)
        deviation = min(deviation, math.hypot(ball[0] - a[0] - t * dx, ball[1] - a[1] - t * dy))
    if deviation > 25.0:
        _reset_cooperation()
        return None
    pose = UnitedPosition.FieldRobotPose()
    if _robot_id() == plan["sender"]:
        target = _safe_target(ATTACK_DIR * -25.0, clamp(ball[1] * 0.4, -35.0, 35.0),
                              UnitedPosition.Pos2Angle(pose, ball))
        if _lane_clear(pose, target, enemies + [[robots[plan["receiver"]]["x_cm"], robots[plan["receiver"]]["y_cm"]]], 28.0):
            peripheral.Dribble(False)
            _position_step(target, speed_gain=0.65)
        else:
            safe_stop(chassis, peripheral)
        return "pass_cover"
    target = list(plan["target"])
    if velocity is not None and math.hypot(*velocity) >= 15.0:
        mouth = plan["points"][-1]
        dx, dy = mouth[0] - ball[0], mouth[1] - ball[1]
        dot = dx * velocity[0] + dy * velocity[1]
        speed_sq = sum(v * v for v in velocity)
        t = dot / speed_sq
        if dot > 0 and t < 1.0:
            predicted = [ball[i] + velocity[i] * t for i in (0, 1)]
            if math.dist(predicted, mouth) <= 18.0:
                target[2] = math.degrees(math.atan2(-velocity[1], -velocity[0])) % 360.0
                theta = math.radians(target[2])
                target[:2] = [predicted[0] - SIM_CATCH_POINT_CM[0] * math.cos(theta),
                              predicted[1] - SIM_CATCH_POINT_CM[0] * math.sin(theta)]
    if not _rule_target_is_legal(target):
        _reset_cooperation()
        return None
    if math.dist(pose[:2], ball) < 25.0 and (velocity is None or math.hypot(*velocity) < 70.0):
        chase_ball(ball_field=ball)
        return "pass_capture"
    if velocity is not None and math.hypot(*velocity) < 15.0 and now - plan["since"] > 0.5:
        _reset_cooperation()
        return None
    peripheral.Dribble(True)
    error = angle_error_deg(pose[2], target[2])
    if abs(error) > AUSTRALIA.Tactics.PASS_RECEIVE_ALIGN_DEG:
        _safe_robot_move(0.0, 0.0, _turn_command(error))
    elif math.dist(pose[:2], target[:2]) > 3.0 and _lane_clear(pose, target, enemies, 28.0):
        _position_step(target, speed_gain=0.5)
    else:
        chassis.stop()
    return "pass_receive"

def _reset_defence():
    global _DEFENCE_UNTIL, _DEFENCE_BALL_SAMPLE, _DEFENCE_BALL_VELOCITY
    global _RADICAL_LAST_ENEMY, _RADICAL_LAST_OVERLAP_TIME
    global _MACAU_DEFENCE_LAST_CARRIER, _MACAU_DEFENCE_LAST_TIME, _MACAU_DEFENCE_UNTIL
    global _HIDDEN_MACAU_RUNNER, _HIDDEN_MACAU_UNTIL

    _DEFENCE_UNTIL = -math.inf
    _DEFENCE_BALL_SAMPLE = None
    _DEFENCE_BALL_VELOCITY = None
    _RADICAL_LAST_ENEMY = None
    _RADICAL_LAST_OVERLAP_TIME = 0.0
    _MACAU_DEFENCE_LAST_CARRIER = None
    _MACAU_DEFENCE_LAST_TIME = -math.inf
    _MACAU_DEFENCE_UNTIL = -math.inf
    _HIDDEN_MACAU_RUNNER = None
    _HIDDEN_MACAU_UNTIL = -math.inf

def _pose_valid(pose):
    try:
        return valid_pose(pose) and all(math.isfinite(float(v)) for v in pose[:3])
    except (TypeError, ValueError, IndexError):
        return False

def _motion_guard(look_at=None, allow_special_capture=False):
    if not UnitedPosition.Referee() or not chassis.IsFieldIMUReady():
        safe_stop(chassis, peripheral)
        return None
    pose = UnitedPosition.FieldRobotPose()
    if not _pose_valid(pose):
        safe_stop(chassis, peripheral)
        return None
    if not _update_attack_direction(pose):
        safe_stop(chassis, peripheral)
        return None
    return pose

def _is_drop_ball_front_corner(ball):
    x, y = float(ball[0]), float(ball[1])
    tolerance = max(6.0, AUSTRALIA.Real.REAL_BALL_DIAMETER_CM)
    return any(
        math.hypot(x - slot[0], y - slot[1]) <= tolerance
        for slot in _drop_ball_slots()[1:]
    )

def _drop_ball_capture_target(ball):
    if not _is_drop_ball_front_corner(ball):
        return None
    reach = SIM_CATCH_POINT_CM[0] + 1.0
    side = 1.0 if float(ball[0]) >= 0.0 else -1.0
    x = side * max(0.0, abs(float(ball[0])) - reach)
    if AUSTRALIA.Tactics.SIM_PENALTY_RULE_ENABLED:
        x = side * min(abs(x), AUSTRALIA.Tactics.SIM_PENALTY_FRONT_CM - AUSTRALIA.Tactics.SIM_RULE_EDGE_EPSILON_CM)
    return [x, float(ball[1]), 0.0 if side > 0.0 else 180.0]

def _drop_ball_capture_pose_legal(pose, ball):
    target = _drop_ball_capture_target(ball)
    if target is None or not _pose_valid(pose):
        return False
    side = 1.0 if float(ball[0]) >= 0.0 else -1.0
    if AUSTRALIA.Tactics.SIM_PENALTY_RULE_ENABLED and side * float(pose[0]) > AUSTRALIA.Tactics.SIM_PENALTY_FRONT_CM - AUSTRALIA.Tactics.SIM_RULE_EDGE_EPSILON_CM:
        return False
    return abs(float(pose[0])) <= SIM_WALL_CONTACT_X_CM and abs(float(pose[1])) <= SIM_WALL_CONTACT_Y_CM

def _is_sideline_capture_ball(ball):
    point = _field_xy(ball)
    if point is None:
        return False
    params = UnitedPosition._restricted_area_params()
    return abs(point[1]) >= float(params["y_limit"]) - float(params["boundary_margin"])

def _sideline_capture_target(ball):
    if not _is_sideline_capture_ball(ball):
        return None
    x, y = float(ball[0]), float(ball[1])
    max_center = Settings.Field.Short / 2.0 - AUSTRALIA.Tactics.SIM_BODY_RADIUS_CM - 0.5
    if abs(x) > Settings.Field.Long / 2.0 - AUSTRALIA.Tactics.SIM_BODY_RADIUS_CM - 0.5:
        return None
    reach = SIM_CATCH_POINT_CM[0] + 1.0
    side = 1.0 if y >= 0.0 else -1.0
    target_y = side * min(abs(y) - reach, max_center - 1.5)
    return [x, target_y, 90.0 if side > 0.0 else 270.0]

def _sideline_capture_pose_legal(pose, ball):
    target = _sideline_capture_target(ball)
    if target is None or not _pose_valid(pose):
        return False
    return (
        abs(float(pose[0])) <= Settings.Field.Long / 2.0 - AUSTRALIA.Tactics.SIM_BODY_RADIUS_CM - 0.5
        and abs(float(pose[1])) <= Settings.Field.Short / 2.0 - AUSTRALIA.Tactics.SIM_BODY_RADIUS_CM - 0.5
    )

def _edge_capture_normal(ball, pose):
    x, y = float(ball[0]), float(ball[1])
    if _is_drop_ball_front_corner(ball):
        return [1.0 if float(ball[0]) >= 0.0 else -1.0, 0.0]

    corrected_ball = _rule_legal_target(x, y, 0.0)
    if math.dist(corrected_ball[:2], [x, y]) > 0.01:
        normal = [x - corrected_ball[0], y - corrected_ball[1]]
        if math.hypot(*normal) > 0.01:
            return normal
    return [x - pose[0], y - pose[1]]

def _ball_approach_pose(ball, pose):
    if not _pose_valid(pose):
        return None
    try:
        ball = [float(ball[0]), float(ball[1])]
    except (TypeError, ValueError, IndexError):
        return None
    if not all(math.isfinite(value) for value in ball):
        return None

    drop_target = _drop_ball_capture_target(ball)
    if drop_target is not None:
        return drop_target
    sideline_target = _sideline_capture_target(ball)
    if sideline_target is not None:
        return sideline_target

    reach = SIM_CATCH_POINT_CM[0] + 1.0
    outward_x, outward_y = _edge_capture_normal(ball, pose)
    outward_length = math.hypot(outward_x, outward_y)
    if outward_length <= 0.01:
        return None
    base_yaw = math.degrees(math.atan2(outward_y, outward_x)) % 360.0

    candidates = []
    # 边线法线优先，轻微偏角仅用于避开角落或禁区边缘；不再枚举反向朝向。
    for offset in (0.0, -10.0, 10.0, -20.0, 20.0):
        yaw = (base_yaw + offset) % 360.0
        theta = math.radians(yaw)
        raw_target = [ball[0] - reach * math.cos(theta), ball[1] - reach * math.sin(theta), yaw]
        target = raw_target
        if not _rule_target_is_legal(target):
            continue
        # 投影后的机身位置可能发生横移，最后必须以实际接球点重新朝向球。
        target[2] = UnitedPosition.Pos2Angle(target, ball) % 360.0
        heading_error = abs(angle_error_deg(base_yaw, target[2]))
        if heading_error > 45.0:
            continue
        waypoint = UnitedPosition._navigate_waypoint(pose, target)
        travel = math.dist(pose[:2], waypoint[:2]) + math.dist(waypoint[:2], target[:2])
        turn = abs(angle_error_deg(pose[2], target[2]))
        candidates.append(((heading_error, travel, turn), target))
    return min(candidates, key=lambda item: item[0])[1] if candidates else None

def _needs_edge_capture(ball, pose):
    if not _pose_valid(pose):
        return True
    try:
        ball = [float(ball[0]), float(ball[1])]
    except (TypeError, ValueError, IndexError):
        return True
    if not all(math.isfinite(value) for value in ball):
        return True
    yaw = UnitedPosition.Pos2Angle(pose, ball)
    theta = math.radians(yaw)
    center = [
        ball[0] - (SIM_CATCH_POINT_CM[0] + 1.0) * math.cos(theta),
        ball[1] - (SIM_CATCH_POINT_CM[0] + 1.0) * math.sin(theta),
        yaw,
    ]
    ball_target = _rule_legal_target(ball[0], ball[1], yaw)
    center_target = _rule_legal_target(*center)
    return (
        math.dist(ball_target[:2], ball) > 0.01
        or math.dist(center_target[:2], center[:2]) > 0.01
    )

def _edge_capture_step(pose, approach, ball=None):
    drop_ball_capture = ball is not None and _is_drop_ball_front_corner(ball)
    sideline_capture = ball is not None and _is_sideline_capture_ball(ball)
    special_capture = drop_ball_capture or sideline_capture
    if drop_ball_capture and not (
        _drop_ball_capture_pose_legal(pose, ball)
        and _drop_ball_capture_pose_legal(approach, ball)
    ):
        _journal_event("capture_blocked", kind="drop_ball", pose=pose, ball=ball)
        return False
    if sideline_capture and not (
        _sideline_capture_pose_legal(pose, ball)
        and _sideline_capture_pose_legal(approach, ball)
    ):
        _journal_event("capture_blocked", kind="sideline", pose=pose, ball=ball)
        return False
    distance = math.dist(pose[:2], approach[:2])
    arrival_cm = 0.8 if special_capture else 0.25
    if distance <= arrival_cm:
        error = angle_error_deg(pose[2], approach[2])
        if abs(error) > AUSTRALIA.Tactics.CHASE_ALIGN_DEG:
            turn = _turn_command(error, AUSTRALIA.Tactics.CHASE_TURN_MAX_DEG_S)
            if special_capture:
                _rule_safe_robot_move(0.0, 0.0, turn, speed_limit=AUSTRALIA.Real.CHASE_MAX_MPS)
            else:
                _safe_robot_move(0.0, 0.0, turn, speed_limit=AUSTRALIA.Real.CHASE_MAX_MPS)
        elif special_capture and ball is not None:
            # 重置球不能在预留接球点直接停车。根据盘球器的真实相对误差缓慢压入，
            # 一旦被 Unity 判成持球，下一控制周期会自动退出追球。
            theta = math.radians(pose[2])
            dx, dy = float(ball[0]) - pose[0], float(ball[1]) - pose[1]
            local_x = dx * math.cos(theta) + dy * math.sin(theta)
            local_y = -dx * math.sin(theta) + dy * math.cos(theta)
            forward_error = local_x - SIM_CATCH_POINT_CM[0]
            lateral_error = local_y - SIM_CATCH_POINT_CM[1]
            forward = clamp(forward_error * 0.05, 0.0, 0.18)
            lateral = clamp(lateral_error * 0.05, -0.18, 0.18)
            _rule_safe_robot_move(forward, lateral, 0.0, speed_limit=AUSTRALIA.Real.CHASE_MAX_MPS)
        else:
            chassis.stop()
        return False
    if special_capture:
        # 两类重置球都按固定接球线全速接近，沿受限法线进入制动距离后才逐步减速。
        if drop_ball_capture:
            params = UnitedPosition._restricted_area_params()
            limit = float(params["goal_box_x_min"])
            side = 1.0 if float(ball[0]) >= 0.0 else -1.0
            progress_to_limit = max(0.0, side * (approach[0] - pose[0]) / distance)
            remaining_cm = limit - 0.5 - side * pose[0] - 4.0
        else:
            limit = Settings.Field.Short / 2.0 - AUSTRALIA.Tactics.SIM_BODY_RADIUS_CM - 0.5
            side = 1.0 if float(ball[1]) >= 0.0 else -1.0
            progress_to_limit = max(0.0, side * (approach[1] - pose[1]) / distance)
            remaining_cm = limit - side * pose[1] - 1.5
        # 模拟器实测制动加速度是 1.8m/s²，这里按 1.4m/s² 留出方向修正和状态延迟余量。
        remaining_m = max(0.0, remaining_cm / 100.0)
        brake, latency = 1.4, AUSTRALIA.Tactics.MOTION_LATENCY_S
        safe_limit_speed = -brake * latency + math.sqrt(
            (brake * latency) ** 2 + 2.0 * brake * remaining_m
        )
        speed_limit = AUSTRALIA.Real.CHASE_MAX_MPS
        if progress_to_limit > 0.01:
            speed_limit = min(speed_limit, safe_limit_speed / progress_to_limit)
        speed = clamp(distance * 0.04, 0.16, max(0.16, speed_limit))
    else:
        speed = AUSTRALIA.Real.CHASE_MAX_MPS
    field_vx = (approach[0] - pose[0]) * speed / distance
    field_vy = (approach[1] - pose[1]) * speed / distance
    theta = math.radians(pose[2])
    local_vx = field_vx * math.cos(theta) + field_vy * math.sin(theta)
    local_vy = -field_vx * math.sin(theta) + field_vy * math.cos(theta)
    error = angle_error_deg(pose[2], approach[2])
    turn = _turn_command(error, AUSTRALIA.Tactics.CHASE_TURN_MAX_DEG_S) if abs(error) > 3.0 else 0.0
    if special_capture:
        # 重置球也必须经过实际边线和禁区线预测，不能绕开场地规则直接下发底盘命令。
        _rule_safe_robot_move(local_vx, local_vy, turn, speed_limit=AUSTRALIA.Real.CHASE_MAX_MPS)
        return False
    if _safe_robot_move(local_vx, local_vy, turn, speed_limit=AUSTRALIA.Real.CHASE_MAX_MPS):
        return False
    # 转弯弧线可能会暂时压向墙外；此时保留正确朝向，下一帧再继续向接球点移动。
    _safe_robot_move(0.0, 0.0, turn, speed_limit=AUSTRALIA.Real.CHASE_MAX_MPS)
    return False

def _free_ball_chaser(ball, robots=None):
    robots = _team_robots() if robots is None else robots
    if robots is None:
        return _robot_id()
    allies = [item for ident, item in robots.items()
              if ident in AUSTRALIA.Team.RobotIds and item.get("available") is True]
    if not allies:
        return None
    point = _field_xy(ball)
    if point is None:
        return None
    distances = {}
    for item in allies:
        distances[item["id"]] = math.dist([item["x_cm"], item["y_cm"]], point)
    if not distances:
        return None
    return min(distances, key=lambda ident: (distances[ident], ident))

def _communicated_contest_chaser(ball, robots):
    point = _field_xy(ball)
    if point is None or not isinstance(robots, dict):
        return None
    contenders = []
    for ident, item in robots.items():
        if (ident not in AUSTRALIA.Team.RobotIds or item.get("available") is not True
                or item.get("contesting") is not True):
            continue
        distance = math.dist([item["x_cm"], item["y_cm"]], point)
        if distance <= AUSTRALIA.Tactics.CONTEST_ASSIST_CHASER_CM:
            contenders.append((distance, ident))
    return min(contenders)[1] if contenders else None

def _carrier_defence_chaser(ball, robots):
    # 单车接管时不存在可分配的前压队友；无论幸存者是 3 号还是 4 号，都必须
    # 自己贴住盘球器，而不是继续等待固定的 3 号角色。
    if _solo_takeover_active():
        return _robot_id()
    active = [] if not isinstance(robots, dict) else [
        item for ident, item in robots.items()
        if ident in AUSTRALIA.Team.RobotIds and item.get("available") is True
    ]
    if len(active) >= 2:
        return _communicated_contest_chaser(ball, robots) or _free_ball_chaser(ball, robots)
    primary = robots.get(AUSTRALIA.Team.RobotIds[0]) if isinstance(robots, dict) else None
    if isinstance(primary, dict) and primary.get("available") is False:
        return _robot_id()
    return AUSTRALIA.Team.RobotIds[0]

def _screen_chaser_step(ball, chaser_id, robots):
    if chaser_id == _robot_id() or not isinstance(robots, dict):
        return False
    point = _field_xy(ball)
    chaser = robots.get(chaser_id)
    if point is None or chaser is None:
        return False

    threat, threat_score = None, math.inf
    for enemy in _current_enemies():
        distance = math.dist(enemy, point)
        if distance > AUSTRALIA.Tactics.SCREEN_THREAT_RADIUS_CM or distance <= 0.01:
            continue
        if distance > AUSTRALIA.Tactics.SCREEN_IMMEDIATE_RADIUS_CM:
            continue
        score = distance
        if score < threat_score:
            threat, threat_score = enemy, score
    if threat is None:
        return False

    enemy = threat
    pose = _motion_guard(look_at=enemy)
    if pose is None:
        return True
    enemy_distance = math.dist(enemy, point)
    ux, uy = (point[0] - enemy[0]) / enemy_distance, (point[1] - enemy[1]) / enemy_distance
    px, py = -uy, ux
    stand_off = clamp(enemy_distance * 0.5, AUSTRALIA.Tactics.SCREEN_BALL_CLEARANCE_CM, 46.0)
    base = [point[0] - ux * stand_off, point[1] - uy * stand_off]
    chaser_xy = [chaser["x_cm"], chaser["y_cm"]]
    candidates = [base, [base[0] + px * AUSTRALIA.Tactics.SCREEN_LATERAL_OFFSET_CM, base[1] + py * AUSTRALIA.Tactics.SCREEN_LATERAL_OFFSET_CM],
                  [base[0] - px * AUSTRALIA.Tactics.SCREEN_LATERAL_OFFSET_CM, base[1] - py * AUSTRALIA.Tactics.SCREEN_LATERAL_OFFSET_CM]]
    scored = []
    for candidate in candidates:
        target = _safe_target(candidate[0], candidate[1], UnitedPosition.Pos2Angle(pose, enemy))
        if not _rule_target_is_legal(target):
            continue
        dx, dy = point[0] - chaser_xy[0], point[1] - chaser_xy[1]
        length_sq = dx * dx + dy * dy
        if length_sq > 0.01:
            progress = clamp(((target[0] - chaser_xy[0]) * dx + (target[1] - chaser_xy[1]) * dy) / length_sq,
                             0.0, 1.0)
            lane_offset = math.hypot(target[0] - chaser_xy[0] - progress * dx,
                                     target[1] - chaser_xy[1] - progress * dy)
        else:
            lane_offset = math.dist(target[:2], chaser_xy)
        lane_penalty = 180.0 if lane_offset < AUSTRALIA.Tactics.SCREEN_LANE_CLEARANCE_CM else 0.0
        scored.append((lane_penalty + math.dist(pose[:2], target[:2]), target))
    if not scored:
        return False
    peripheral.Dribble(True)
    _position_step(min(scored, key=lambda item: item[0])[1], speed_gain=1.8, arrival_cm=2.0,
                   speed_limit=AUSTRALIA.Tactics.PRESS_MAX_MPS, near_speed_limit=AUSTRALIA.Tactics.PRESS_MAX_MPS)
    return True

def _contest_assist_step(ball, chaser_id, robots, observation=None):
    if chaser_id == _robot_id() or not isinstance(robots, dict):
        return None
    point = _field_xy(ball)
    chaser = robots.get(chaser_id)
    if point is None or chaser is None:
        return None
    chaser_xy = [chaser["x_cm"], chaser["y_cm"]]
    if math.dist(chaser_xy, point) > AUSTRALIA.Tactics.CONTEST_ASSIST_CHASER_CM:
        return None
    pose = _motion_guard()
    if pose is None:
        return "contest_assist_wait"
    enemies = _current_enemies()
    ranked_enemies = sorted(
        ((_field_xy(enemy), math.dist(_field_xy(enemy), point)) for enemy in enemies if _field_xy(enemy) is not None),
        key=lambda item: item[1],
    )
    enemy, enemy_distance = (ranked_enemies[0] if ranked_enemies else (None, math.inf))
    peripheral.Dribble(True)
    # 一台对手已贴球、另一台从外侧沿直线进入争夺圈时，先占住其进场路线。
    # 这只依据本机 YOLO 的相对位置推断，不读取场景中的对手真值或战术状态。
    if (enemy is not None and enemy_distance <= AUSTRALIA.Tactics.CONTEST_ASSIST_CONTACT_CM
            and len(ranked_enemies) >= 2):
        helper, helper_distance = ranked_enemies[1]
        if AUSTRALIA.Tactics.CONTEST_HELPER_MIN_CM <= helper_distance <= AUSTRALIA.Tactics.CONTEST_HELPER_MAX_CM:
            dx, dy = helper[0] - point[0], helper[1] - point[1]
            length = math.hypot(dx, dy)
            if length > 1.0:
                target = _safe_target(
                    point[0] + dx / length * AUSTRALIA.Tactics.CONTEST_HELPER_BLOCK_CM,
                    point[1] + dy / length * AUSTRALIA.Tactics.CONTEST_HELPER_BLOCK_CM,
                    UnitedPosition.Pos2Angle(pose, helper),
                )
                _position_step(target, speed_gain=2.0, arrival_cm=2.0,
                               speed_limit=AUSTRALIA.Tactics.PRESS_MAX_MPS, near_speed_limit=AUSTRALIA.Tactics.PRESS_MAX_MPS)
                return "contest_block_helper"
    if enemy is not None and enemy_distance <= AUSTRALIA.Tactics.CONTEST_ASSIST_CONTACT_CM:
        # 目标放在对手背离球的一侧。辅助车朝此点推进时会将对手推出争夺点，
        # 但不会占住队友通向球的直线。
        dx, dy = enemy[0] - point[0], enemy[1] - point[1]
        length = math.hypot(dx, dy)
        if length > 1.0:
            target = _safe_target(
                enemy[0] + dx / length * AUSTRALIA.Tactics.SIM_BODY_RADIUS_CM,
                enemy[1] + dy / length * AUSTRALIA.Tactics.SIM_BODY_RADIUS_CM,
                UnitedPosition.Pos2Angle(pose, enemy),
            )
            _position_step(target, speed_gain=2.0, arrival_cm=1.0,
                           speed_limit=AUSTRALIA.Tactics.PRESS_MAX_MPS, near_speed_limit=AUSTRALIA.Tactics.PRESS_MAX_MPS)
            return "contest_push_enemy"
    # 对手未进入接触圈即退出推人，固定到球后方的侧接应点，防止两车来回互撞。
    side = 1.0 if pose[1] >= point[1] else -1.0
    target = _safe_target(
        point[0] - ATTACK_DIR * AUSTRALIA.Tactics.CONTEST_SUPPORT_BACK_CM,
        point[1] + side * AUSTRALIA.Tactics.CONTEST_SUPPORT_SIDE_CM,
        UnitedPosition.Pos2Angle(pose, point),
    )
    _position_step(target, speed_gain=1.5, arrival_cm=3.0,
                   speed_limit=AUSTRALIA.Tactics.SIM_MAX_SPEED_MPS, near_speed_limit=AUSTRALIA.Tactics.SIM_MAX_SPEED_MPS)
    return "contest_support"

def _capture_terminal_chassis_speed(turn_deg_s):
    yaw_point_speed = abs(math.radians(float(turn_deg_s))) * (SIM_CATCH_POINT_CM[0] / 100.0)
    usable_squared = max(
        AUSTRALIA.Tactics.MOTION_MIN_MPS ** 2,
        CHASE_CONTACT_RELATIVE_TARGET_MPS ** 2 - yaw_point_speed ** 2,
    )
    return math.sqrt(usable_squared)

def _capture_closing_speed(distance_cm, turn_deg_s):
    terminal = _capture_terminal_chassis_speed(turn_deg_s)
    remaining = max(0.0, (float(distance_cm) - SIM_CATCH_POINT_CM[0]) / 100.0)
    blend = clamp(remaining / AUSTRALIA.Tactics.CHASE_CAPTURE_SLOWDOWN_DISTANCE_M, 0.0, 1.0)
    return terminal + (AUSTRALIA.Real.CHASE_MAX_MPS - terminal) * blend

def _chase_motion(pose, ball, contest=False, local_ball=None):
    distance = math.dist(pose[:2], ball)
    velocity = (0.0, 0.0)
    target = list(ball)
    try:
        local_ball = [float(local_ball[0]), float(local_ball[1])]
    except (TypeError, ValueError, IndexError):
        local_ball = None
    close_capture = (
        local_ball is not None
        and all(math.isfinite(value) for value in local_ball)
        and distance <= 35.0
    )
    if not close_capture:
        try:
            prediction = UnitedPosition.PredictBallLanding(
                field_ball_pos=ball,
                horizon_s=AUSTRALIA.Tactics.CHASE_SENSOR_PREDICT_S,
                max_age_s=AUSTRALIA.Tactics.LAST_BALL_MEMORY_S,
                min_speed_cm_s=1.0,
                field_margin_cm=AUSTRALIA.Tactics.SIM_BODY_RADIUS_CM,
            )
        except (AttributeError, TypeError, ValueError):
            prediction = None
        if isinstance(prediction, dict):
            measured = prediction.get("velocity_cm_s")
            predicted = _field_xy(prediction.get("field_pos"))
            try:
                velocity = (float(measured[0]), float(measured[1]))
            except (TypeError, ValueError, IndexError):
                velocity = (0.0, 0.0)
            if (not prediction.get("valid") or predicted is None
                    or not all(math.isfinite(value) for value in velocity)
                    or math.hypot(*velocity) > AUSTRALIA.Tactics.INTERCEPT_MAX_SPEED_CM_S):
                velocity = (0.0, 0.0)
            else:
                target = predicted

    lead = min(AUSTRALIA.Tactics.CHASE_LEAD_MAX_S, max(0.0, distance - 25.0) / (AUSTRALIA.Real.CHASE_MAX_MPS * 100.0))
    lead = min(lead, 30.0 / max(1.0, math.hypot(*velocity)))
    if target == list(ball):
        target = [ball[index] + velocity[index] * lead for index in (0, 1)]
    if (_field_xy(target) is None
            or not _rule_target_is_legal([target[0], target[1], 0.0])):
        target = list(ball)

    yaw = UnitedPosition.Pos2Angle(pose, target) % 360.0
    error = angle_error_deg(pose[2], yaw)
    spin = 0.0
    measured = _semantic_robot_velocity(pose)
    if measured is not None:
        spin = math.degrees(measured[2])
    # 用统一速度观测中的角速度提前制动，防止盘球器转过球以后来回摆动。
    turn = clamp(error * AUSTRALIA.Tactics.CHASE_TURN_GAIN - spin * 0.65,
                 -AUSTRALIA.Tactics.CHASE_TURN_MAX_DEG_S, AUSTRALIA.Tactics.CHASE_TURN_MAX_DEG_S)
    if abs(error) <= 2.0 and abs(spin) <= AUSTRALIA.Tactics.TURN_MIN_DEG_S:
        turn = 0.0
    elif 0.0 < abs(turn) < AUSTRALIA.Tactics.TURN_MIN_DEG_S:
        turn = math.copysign(AUSTRALIA.Tactics.TURN_MIN_DEG_S, turn)

    if close_capture:
        # 最后收球也沿“车心 -> 球”的直线推进，不走横移弧线，避免在禁区边或门前
        # 绕球时惯性切入禁区。盘球器同时以最高允许转速转向球；朝向未完成时仅降低
        # 直线冲入速度，避免车体侧面先撞球。
        dx, dy = ball[0] - pose[0], ball[1] - pose[1]
        distance = math.hypot(dx, dy)
        if distance < 0.01:
            chassis.stop()
            return
        capture_alignment = clamp((math.cos(math.radians(error)) - 0.10) / 0.90, 0.0, 1.0)
        remaining = (distance - SIM_CATCH_POINT_CM[0]) / 100.0
        alignment_floor = 0.42 if contest else 0.35
        speed_scale = alignment_floor + (1.0 - alignment_floor) * capture_alignment
        if remaining >= 0.0:
            speed = _capture_closing_speed(distance, turn) * speed_scale
        else:
            speed = -clamp(-remaining * 2.5, AUSTRALIA.Tactics.MOTION_MIN_MPS, 0.22) * speed_scale
        field_vx, field_vy = speed * dx / distance, speed * dy / distance
        theta = math.radians(pose[2])
        local_vx = field_vx * math.cos(theta) + field_vy * math.sin(theta)
        local_vy = -field_vx * math.sin(theta) + field_vy * math.cos(theta)
        _safe_robot_move(local_vx, local_vy, turn, speed_limit=AUSTRALIA.Real.CHASE_MAX_MPS, turn_priority=True)
        return

    # 目标是球位于盘球器前方 8cm。最后 0.98m 沿直线线性减速到
    # Unity realistic dribbler 的 1.60m/s 相对速度上限附近。
    mouth_distance = max(1.0, SIM_CATCH_POINT_CM[0] - 0.5)
    remaining = (math.dist(pose[:2], target) - mouth_distance) / 100.0
    closing = _capture_closing_speed(math.dist(pose[:2], target), turn)
    if remaining < 0.0:
        # 球进入盘球器过深但还未捕获时，短暂后退回到捕获中心，不能从侧面顶开球。
        closing = -max(AUSTRALIA.Tactics.MOTION_MIN_MPS, min(0.25, -remaining * AUSTRALIA.Tactics.CHASE_CAPTURE_KP))
    # 转向和前进可以并行；但进入 35cm 接球区后，必须先让盘球器明显朝向球。
    # 否则机器人会保留过高侧向速度，从车体侧面越过球而无法收球。
    near_weight = clamp((35.0 - distance) / 25.0, 0.0, 1.0)
    alignment = max(0.40 if contest else 0.25, math.cos(math.radians(error)))
    closing *= 1.0 - near_weight * (1.0 - alignment)

    heading = math.radians(yaw)
    field_vx = closing * math.cos(heading) + velocity[0] / 100.0
    field_vy = closing * math.sin(heading) + velocity[1] / 100.0
    scale = min(1.0, AUSTRALIA.Real.CHASE_MAX_MPS / max(1e-9, math.hypot(field_vx, field_vy)))
    theta = math.radians(pose[2])
    local_vx = (field_vx * scale) * math.cos(theta) + (field_vy * scale) * math.sin(theta)
    local_vy = -(field_vx * scale) * math.sin(theta) + (field_vy * scale) * math.cos(theta)
    _safe_robot_move(local_vx, local_vy, turn, speed_limit=AUSTRALIA.Real.CHASE_MAX_MPS, turn_priority=True)

def chase_ball(ball_field=None, contest=False):
    global _CHASE_ALIGNED

    focus = _field_xy(ball_field)
    if focus is None:
        _, focus = _sensor_ball_snapshot()
    drop_ball_capture = focus is not None and _is_drop_ball_front_corner(focus)
    sideline_capture = focus is not None and _is_sideline_capture_ball(focus)
    pose = _motion_guard(look_at=focus, allow_special_capture=drop_ball_capture or sideline_capture)
    if pose is None:
        _CHASE_ALIGNED = False
        return False
    ball = _local_robot_ball_pos()
    state = _effective_ball_state(ball)
    if state == "held" and not contest:
        _CHASE_ALIGNED = False
        peripheral.Dribble(True)
        chassis.stop()
        return True
    if ball_field is not None:
        point = _field_xy(ball_field)
        if point is None:
            _CHASE_ALIGNED = False
            safe_stop(chassis, peripheral)
            return False
        theta = math.radians(pose[2])
        dx, dy = point[0] - pose[0], point[1] - pose[1]
        ball = [dx * math.cos(theta) + dy * math.sin(theta),
                -dx * math.sin(theta) + dy * math.cos(theta)]
        state = "free"
    if state == "lost":
        _CHASE_ALIGNED = False
        safe_stop(chassis, peripheral)
        return False
    try:
        ball_x, ball_y = map(float, ball[:2])
    except (TypeError, ValueError, IndexError):
        _CHASE_ALIGNED = False
        safe_stop(chassis, peripheral)
        return False
    if not all(math.isfinite(v) for v in (ball_x, ball_y)):
        _CHASE_ALIGNED = False
        safe_stop(chassis, peripheral)
        return False
    if math.hypot(ball_x, ball_y) <= 0.01:
        _CHASE_ALIGNED = False
        safe_stop(chassis, peripheral)
        return False

    theta = math.radians(pose[2])
    field_point = [pose[0] + ball_x * math.cos(theta) - ball_y * math.sin(theta),
                   pose[1] + ball_x * math.sin(theta) + ball_y * math.cos(theta)]
    # 由本机位姿和本机相对球观测换算出的场地方位用于求最短误差，避免重复映射时翻转朝向。
    target_yaw = UnitedPosition.Pos2Angle(pose, field_point) % 360.0
    _CHASE_ALIGNED = abs(angle_error_deg(pose[2], target_yaw)) <= AUSTRALIA.Tactics.CHASE_ALIGN_DEG
    edge_ball = _needs_edge_capture(field_point, pose)
    peripheral.Dribble(True)
    if not edge_ball:
        _chase_motion(pose, field_point, contest=contest, local_ball=[ball_x, ball_y])
        return False

    # 只有边线或禁区边的球需要从合法位置接近，避免接触球的同时触发越界处罚。
    approach = _ball_approach_pose(field_point, pose)
    if approach is None:
        safe_stop(chassis, peripheral)
        return False
    return _edge_capture_step(pose, approach, field_point)

def _turn_command(error, maximum=AUSTRALIA.Tactics.HELD_TURN_MAX_DEG_S):
    return math.copysign(clamp(abs(error) * 2.5, AUSTRALIA.Tactics.TURN_MIN_DEG_S, maximum), error)

def ball_score():
    #根据球到对方球门中心的场地距离计算一个简单分数

    if not _update_attack_direction(UnitedPosition.FieldRobotPose()):
        return None
    ball_state, ball_position = _sensor_ball_snapshot()
    if ball_state != "free" or ball_position is None:
        return None

    try:
        ball_x, ball_y = float(ball_position[0]), float(ball_position[1])
    except (TypeError, ValueError, IndexError):
        return None
    if not all(math.isfinite(value) for value in (ball_x, ball_y)):
        return None

    goal_center_x = _opponent_goal_x()
    distance_to_goal = math.hypot(goal_center_x - ball_x, -ball_y)
    return max(0.0, 100.0 - distance_to_goal)

def _shot_ready():
    global _AWAIT_BALL_RELEASE

    pose = _motion_guard()
    if pose is None:
        return None
    ball_state = _effective_ball_state()
    if _AWAIT_BALL_RELEASE and time.monotonic() - _LAST_SHOT_TIME > AUSTRALIA.Tactics.SHOT_RELEASE_TIMEOUT_S:
        _AWAIT_BALL_RELEASE = False
        _reset_attack()
        _reset_cooperation()
    if ball_state != "held":
        _AWAIT_BALL_RELEASE = False
        if _ACTIVE_SHOT in {"wall", "emergency_wall"}:
            logger.info("Robot {} wall_cancel_lost_ball", _robot_id())
        _reset_attack()
    if _AWAIT_BALL_RELEASE:
        chassis.stop()
        return None
    if ball_state != "held":
        chase_ball()
        return None
    peripheral.Dribble(True)
    return pose

def _opponent_goal_x():
    return ATTACK_DIR * SIM_GOAL_LINE_X_CM

GOAL_RIGHT_POST_Y_CM = -AUSTRALIA.Tactics.SIM_GOAL_WIDTH_CM / 2.0

GOAL_LEFT_POST_Y_CM = AUSTRALIA.Tactics.SIM_GOAL_WIDTH_CM / 2.0

def _goal_half_opening_cm():
    return GOAL_LEFT_POST_Y_CM - AUSTRALIA.Real.REAL_BALL_DIAMETER_CM / 2.0 - 0.5

def _opponent_goal_back_wall_x():
    return ATTACK_DIR * (SIM_GOAL_LINE_X_CM + AUSTRALIA.Tactics.SIM_GOAL_DEPTH_CM)

def _goal_scoring_half_opening_cm():
    return max(8.0, _goal_half_opening_cm() - AUSTRALIA.Tactics.GOAL_BACK_WALL_AIM_MARGIN_CM)

def _is_inside_goal_scoring_band(y):
    """Return whether a back-wall aim point stays strictly between the posts."""

    try:
        y = float(y)
    except (TypeError, ValueError):
        return False
    return (
        math.isfinite(y)
        and GOAL_RIGHT_POST_Y_CM < y < GOAL_LEFT_POST_Y_CM
        and abs(y) <= _goal_scoring_half_opening_cm()
    )

def _opponent_goal_back_wall_target(goal_y=0.0):
    return [
        _opponent_goal_back_wall_x(),
        clamp(float(goal_y), -_goal_scoring_half_opening_cm(), _goal_scoring_half_opening_cm()),
    ]

GOAL_POST_PATH_ENEMY_CLEARANCE_CM = 2.0 * AUSTRALIA.Tactics.SIM_BODY_RADIUS_CM + 4.0

def _goal_post_path_target(point, origin):
    """Return a legal non-forward navigation node that faces the goal safely."""

    goal_target = _opponent_goal_back_wall_target(0.0)
    yaw = UnitedPosition.Pos2Angle(point, goal_target)
    target = _carry_safe_target(point[0], point[1], yaw, goal_post_escape=True)
    if target is None:
        return None
    # The goal-post exception only bypasses the side-lane *attack* cap so a car
    # can leave the corner.  It may never use it to progress further goalward.
    if ATTACK_DIR * (target[0] - origin[0]) > 0.25:
        return None
    if math.dist(target[:2], point[:2]) > 0.25:
        return None
    return target

def _goal_post_path_segment_clear(start, end, enemies):
    """Keep a path away from visible opponents, including an existing close duel."""

    radius = GOAL_POST_PATH_ENEMY_CLEARANCE_CM
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared < 0.01:
        return False
    for enemy in enemies:
        start_dx, start_dy = start[0] - enemy[0], start[1] - enemy[1]
        start_distance = math.hypot(start_dx, start_dy)
        if start_distance < radius:
            # If a defender is already touching the path start, a strict tube
            # test would reject every escape.  Permit only a first segment that
            # immediately increases separation; it still cannot cut across it.
            end_distance = math.dist(end[:2], enemy)
            if end_distance <= start_distance + 0.25 or dx * start_dx + dy * start_dy < -0.01:
                return False
            continue
        ratio = clamp(((-start_dx) * dx + (-start_dy) * dy) / length_squared, 0.0, 1.0)
        if math.hypot(start_dx + ratio * dx, start_dy + ratio * dy) < radius:
            return False
    return True

def _goal_post_escape_plan(pose):
    """Plan the shortest visible-opponent-safe path to a direct-shot position.

    Search is intentionally local and bounded: all nodes are legal carry poses,
    every segment clears currently visible opponents, and X either retreats from
    goal or remains unchanged.  The first waypoint of the cheapest route is
    returned, then the route is recomputed from fresh perception on arrival.
    """

    origin = [float(pose[0]), float(pose[1])]
    enemies = [list(enemy[:2]) for enemy in _current_enemies()]
    grid = AUSTRALIA.Tactics.GOAL_POST_PATH_GRID_CM
    max_steps = max(1, math.ceil(AUSTRALIA.Tactics.GOAL_POST_PATH_MAX_DISTANCE_CM / grid))
    toward_center = -1.0 if origin[1] >= 0.0 else 1.0
    inward_x = -float(ATTACK_DIR)
    directions = (
        (inward_x, 0.0),
        (inward_x, toward_center),
        (0.0, toward_center),
        (inward_x, -toward_center),
        (0.0, -toward_center),
    )

    sequence = 0
    queue = [(0.0, sequence, origin, None, 0)]
    visited = {(round(origin[0] / grid), round(origin[1] / grid))}
    fallback = None
    fallback_score = -math.inf
    expanded = 0

    while queue:
        cost, _order, node, first_waypoint, steps = heapq.heappop(queue)
        if steps >= max_steps:
            continue
        for dx, dy in directions:
            raw = [node[0] + dx * grid, node[1] + dy * grid]
            target = _goal_post_path_target(raw, origin)
            if target is None:
                continue
            key = (round(target[0] / grid), round(target[1] / grid))
            if key in visited:
                continue
            parent = [node[0], node[1], target[2]]
            if not _carry_path_legal(parent, target, goal_post_escape=True):
                continue
            if not _goal_post_path_segment_clear(parent, target, enemies):
                continue
            visited.add(key)
            expanded += 1
            segment_cost = math.dist(node, target[:2])
            next_cost = cost + segment_cost
            next_first = target if first_waypoint is None else first_waypoint
            virtual_pose = target
            route = _direct_shot_route(virtual_pose, obstacles=enemies)
            ready = not _goal_post_retreat_required(virtual_pose) and route is not None
            if ready:
                return {
                    "target": next_first,
                    "shoot_from": virtual_pose,
                    "shot_target": route["points"][-1],
                    "path_cost_cm": next_cost,
                    "expanded": expanded,
                    "visible_enemies": enemies,
                    "reason": "shortest_clear_direct_route",
                }

            # If no direct-shot node appears within this planning slice, make
            # measurable progress away from the post while maximizing clearance
            # from the visible opponent set.  A fresh plan follows on arrival.
            ball = _held_ball_field_pos(virtual_pose)
            distance_from_goal = SIM_GOAL_LINE_X_CM - ATTACK_DIR * ball[0]
            nearest_enemy = min((math.dist(target[:2], enemy) for enemy in enemies), default=90.0)
            safe_band_bonus = 35.0 if not _goal_post_retreat_required(virtual_pose) else 0.0
            score = safe_band_bonus + distance_from_goal + 0.30 * nearest_enemy - 0.08 * next_cost
            if score > fallback_score:
                fallback_score = score
                fallback = {
                    "target": next_first,
                    "shoot_from": virtual_pose,
                    "shot_target": None,
                    "path_cost_cm": next_cost,
                    "expanded": expanded,
                    "visible_enemies": enemies,
                    "reason": "safe_replan_waypoint",
                }
            sequence += 1
            heapq.heappush(queue, (next_cost, sequence, target[:2], next_first, steps + 1))
    return fallback

def _goal_post_retreat_required(pose):
    ball = _held_ball_field_pos(pose)
    distance_to_goal_line = SIM_GOAL_LINE_X_CM - ATTACK_DIR * ball[0]
    return (
        -AUSTRALIA.Tactics.SIM_GOAL_DEPTH_CM <= distance_to_goal_line <= AUSTRALIA.Tactics.GOAL_POST_RETREAT_TRIGGER_CM
        and not _is_inside_goal_scoring_band(ball[1])
    )

def _start_goal_post_retreat(pose):
    global _ACTIVE_SHOT, _ACTIVE_SHOT_SINCE
    global _GOAL_POST_RETREAT_TARGET, _GOAL_POST_RETREAT_SINCE
    global _GOAL_POST_RETREAT_PHASE, _GOAL_POST_RETREAT_STALLS

    plan = _goal_post_escape_plan(pose)
    if plan is None or math.dist(plan["target"][:2], pose[:2]) < AUSTRALIA.Tactics.GOAL_POST_RETREAT_ARRIVAL_CM:
        _journal_event("goal_post_retreat_rejected", pose=pose, ball=_held_ball_field_pos(pose),
                       reason="no_visible_safe_path_to_direct_shot")
        return False
    _reset_attack()
    _ACTIVE_SHOT = "goal_post_retreat"
    _ACTIVE_SHOT_SINCE = time.monotonic()
    _GOAL_POST_RETREAT_TARGET = plan["target"]
    _GOAL_POST_RETREAT_SINCE = _ACTIVE_SHOT_SINCE
    _GOAL_POST_RETREAT_PHASE = plan["reason"]
    _GOAL_POST_RETREAT_STALLS = 0
    _journal_event("goal_post_retreat_start", pose=pose, ball=_held_ball_field_pos(pose),
                   target=plan["target"], shoot_from=plan["shoot_from"],
                   shot_target=plan["shot_target"], path_cost_cm=round(plan["path_cost_cm"], 2),
                   expanded=plan["expanded"], visible_enemies=plan["visible_enemies"],
                   phase=plan["reason"],
                   posts_y_cm=[GOAL_RIGHT_POST_Y_CM, GOAL_LEFT_POST_Y_CM],
                   scoring_band_half_cm=_goal_scoring_half_opening_cm())
    return True

def _execute_goal_post_retreat(pose):
    global _GOAL_POST_RETREAT_TARGET, _GOAL_POST_RETREAT_SINCE
    global _GOAL_POST_RETREAT_PHASE, _GOAL_POST_RETREAT_STALLS

    # 路径中只要本机当前位置出现两柱之间的直射窗，当帧直接出脚；不再等固定退距。
    route = _ready_direct_shot_route(pose)
    if not _goal_post_retreat_required(pose) and route is not None:
        _journal_event("goal_post_retreat_clear", pose=pose, ball=_held_ball_field_pos(pose),
                       target=_GOAL_POST_RETREAT_TARGET, phase=_GOAL_POST_RETREAT_PHASE,
                       shot_target=route["points"][-1])
        if direct_shoot(route=route):
            _reset_attack()
            return "goal_post_retreat_direct_shoot"
        return "goal_post_retreat_direct_aim"

    target = _GOAL_POST_RETREAT_TARGET
    elapsed = time.monotonic() - _GOAL_POST_RETREAT_SINCE
    reached = target is not None and math.dist(pose[:2], target[:2]) <= AUSTRALIA.Tactics.GOAL_POST_RETREAT_ARRIVAL_CM
    if target is None or reached or elapsed >= AUSTRALIA.Tactics.GOAL_POST_PATH_REPLAN_S:
        plan = _goal_post_escape_plan(pose)
        if plan is None:
            _GOAL_POST_RETREAT_STALLS += 1
            _GOAL_POST_RETREAT_SINCE = time.monotonic()
            _journal_event("goal_post_retreat_blocked", pose=pose, ball=_held_ball_field_pos(pose),
                           target=target, phase=_GOAL_POST_RETREAT_PHASE,
                           stalls=_GOAL_POST_RETREAT_STALLS,
                           reason="no_visible_safe_path_to_direct_shot")
            safe_stop(chassis, peripheral)
            return "goal_post_retreat_blocked"
        previous = target
        _GOAL_POST_RETREAT_TARGET = plan["target"]
        _GOAL_POST_RETREAT_SINCE = time.monotonic()
        _GOAL_POST_RETREAT_PHASE = plan["reason"]
        if elapsed >= AUSTRALIA.Tactics.GOAL_POST_PATH_REPLAN_S and not reached:
            _GOAL_POST_RETREAT_STALLS += 1
        _journal_event("goal_post_path_plan", pose=pose, ball=_held_ball_field_pos(pose),
                       previous_target=previous, target=plan["target"],
                       shoot_from=plan["shoot_from"], shot_target=plan["shot_target"],
                       path_cost_cm=round(plan["path_cost_cm"], 2), expanded=plan["expanded"],
                       visible_enemies=plan["visible_enemies"], phase=plan["reason"],
                       stalls=_GOAL_POST_RETREAT_STALLS)

    target = _GOAL_POST_RETREAT_TARGET
    _carry_to(pose, target, target[2], goal_post_escape=True)
    return "goal_post_retreat_path"

def _direct_shot_route(pose, goal_y=0.0, obstacles=None):
    start = _held_ball_field_pos(pose)
    goal_x = _opponent_goal_back_wall_x()
    if (goal_x - start[0]) * ATTACK_DIR <= 0.0:
        return None
    obstacles = _current_enemies() if obstacles is None else obstacles
    half_goal = _goal_scoring_half_opening_cm()
    targets = [clamp(float(goal_y), -half_goal, half_goal)]
    if goal_y == 0.0:
        targets += [-half_goal * 0.8, half_goal * 0.8, -half_goal * 0.45, half_goal * 0.45]
        theta = math.radians(pose[2])
        if math.cos(theta) * ATTACK_DIR > 0.01:
            hit_y = start[1] + (goal_x - start[0]) * math.tan(theta)
            if abs(hit_y) <= half_goal:
                targets.insert(0, hit_y)
    targets.sort(key=lambda y: abs(angle_error_deg(
        pose[2], UnitedPosition.Pos2Angle(start, [goal_x, y])
    )))
    for y in targets:
        end = _opponent_goal_back_wall_target(y)
        if not _lane_clear(start, end, obstacles):
            continue
        route = dict(points=[start, end], lengths=[math.dist(start, end)], speed_ratio=1.0,
                     aim_yaw=UnitedPosition.Pos2Angle(start, end))
        return route
    return None

def _cache_direct_shot_route(pose):
    global _DIRECT_READY_ROUTE, _DIRECT_READY_POSE, _DIRECT_READY_TIME, _DIRECT_READY_VALID

    _DIRECT_READY_ROUTE = _direct_shot_route(pose)
    _DIRECT_READY_POSE = list(pose[:3])
    _DIRECT_READY_TIME = time.monotonic()
    _DIRECT_READY_VALID = True
    return _DIRECT_READY_ROUTE

def _ready_direct_shot_route(pose, goal_y=0.0):
    """复用本控制周期已算出的直射结果，包括被挡的结果。"""

    if goal_y != 0.0:
        return _direct_shot_route(pose, goal_y)
    if (_DIRECT_READY_VALID and _DIRECT_READY_POSE is not None
            and time.monotonic() - _DIRECT_READY_TIME <= 0.04
            and math.dist(pose[:2], _DIRECT_READY_POSE[:2]) <= 2.0
            and abs(angle_error_deg(pose[2], _DIRECT_READY_POSE[2])) <= 2.0):
        return _DIRECT_READY_ROUTE
    return _cache_direct_shot_route(pose)

def _direct_shot_tolerance(route):
    start = route["points"][0]
    goal_x = _opponent_goal_back_wall_x()
    edge_yaws = [UnitedPosition.Pos2Angle(start, [goal_x, side * _goal_scoring_half_opening_cm()])
                 for side in (-1.0, 1.0)]
    opening = min(abs(angle_error_deg(route["aim_yaw"], yaw)) for yaw in edge_yaws)
    return clamp(opening * 0.95, 0.5, AUSTRALIA.Tactics.SHOT_OPENING_ALIGN_MAX_DEG)

def _carry_aim_step(pose, yaw=None):
    if yaw is None:
        yaw = UnitedPosition.Pos2Angle(pose, [_opponent_goal_x(), 0.0])
    peripheral.Dribble(True)
    return _turn_held(yaw, tolerance=AUSTRALIA.Tactics.DIRECT_CARRY_ALIGN_DEG)

def direct_shoot(goal_y=0.0, route=None):
    global _DIRECT_BLOCKED

    _DIRECT_BLOCKED = False
    pose = _shot_ready()
    if pose is None:
        return False
    try:
        goal_y = float(goal_y)
    except (TypeError, ValueError):
        chassis.stop()
        return False
    if not math.isfinite(goal_y) or abs(goal_y) > _goal_scoring_half_opening_cm():
        chassis.stop()
        return False
    if route is None:
        route = _ready_direct_shot_route(pose, goal_y)
    if route is None:
        _DIRECT_BLOCKED = True
        _journal_event("direct_shot_unavailable", pose=pose, ball=_held_ball_field_pos(pose),
                       reason="no_clear_safe_goal_route")
        chassis.stop()
        return False
    points = route.get("points") if isinstance(route, dict) else None
    try:
        shot_target = points[-1]
        target_x, target_y = float(shot_target[0]), float(shot_target[1])
    except (TypeError, ValueError, IndexError):
        _DIRECT_BLOCKED = True
        _journal_event("direct_shot_rejected", pose=pose, route=route,
                       reason="invalid_goal_route")
        chassis.stop()
        return False
    # 防御性复核：任何调用方都不能把直射的终点设为门框外侧、门线或端墙夹角。
    # 合法直射一定瞄准两柱之间的内后壁安全带。
    if (abs(target_x - _opponent_goal_back_wall_x()) > 0.25
            or not _is_inside_goal_scoring_band(target_y)):
        _DIRECT_BLOCKED = True
        _journal_event("direct_shot_rejected", pose=pose, ball=_held_ball_field_pos(pose),
                       target=[target_x, target_y],
                       posts_y_cm=[GOAL_RIGHT_POST_Y_CM, GOAL_LEFT_POST_Y_CM],
                       scoring_band_half_cm=_goal_scoring_half_opening_cm(),
                       reason="target_outside_between_posts_back_wall_band")
        chassis.stop()
        return False
    tolerance = _direct_shot_tolerance(route)
    if abs(angle_error_deg(pose[2], route["aim_yaw"])) > AUSTRALIA.Tactics.DIRECT_CARRY_ALIGN_DEG:
        _carry_aim_step(pose, route["aim_yaw"])
        return False
    if not _turn_held(route["aim_yaw"], tolerance):
        return False
    return _fire_shot(route)

def _held_ball_field_pos(pose):
    robot_x, robot_y, robot_yaw = map(float, pose[:3])
    catch_x, catch_y = SIM_CATCH_POINT_CM
    theta = math.radians(robot_yaw)
    return [
        robot_x + catch_x * math.cos(theta) - catch_y * math.sin(theta),
        robot_y + catch_x * math.sin(theta) + catch_y * math.cos(theta),
    ]

def _normalize_cushions(cushions):
    if isinstance(cushions, bool):
        raise ValueError("cushions must be exactly 1")
    try:
        numeric_cushions = float(cushions)
    except (TypeError, ValueError) as exc:
        raise ValueError("cushions must be exactly 1") from exc

    if (
        not math.isfinite(numeric_cushions)
        or not numeric_cushions.is_integer()
        or numeric_cushions != 1
    ):
        raise ValueError("cushions must be exactly 1")
    return int(numeric_cushions)

def _bank_route(start, end, cushions, wall_side):
    cushions = _normalize_cushions(cushions)
    restitution = (_PASS_PHYSICS or (AUSTRALIA.Tactics.PASS_RESISTANCE_MODEL, AUSTRALIA.Tactics.PASS_WALL_RESTITUTION, AUSTRALIA.Tactics.PASS_KICK_IMPULSE_MPS))[1]
    half_width = SIM_SIDE_WALL_BALL_CENTER_LIMIT_CM
    if restitution <= 0 or abs(start[1]) >= half_width or abs(end[1]) >= half_width or abs(end[0] - start[0]) < 1.0:
        return None
    side = 1.0 if wall_side >= 0 else -1.0
    ys = [start[1]] + [side * (-1) ** i * half_width for i in range(cushions)] + [end[1]]
    weights = [abs(b - a) / restitution ** i for i, (a, b) in enumerate(zip(ys, ys[1:]))]
    total = sum(weights)
    points, fraction = [list(start)], 0.0
    for i, weight in enumerate(weights):
        fraction += weight / total
        points.append([start[0] + (end[0] - start[0]) * fraction, ys[i + 1]])
    lengths = [math.dist(a, b) for a, b in zip(points, points[1:])]
    ratios = [math.hypot(b[0] - a[0], restitution * (b[1] - a[1])) / lengths[i]
              for i, (a, b) in enumerate(zip(points[:-2], points[1:-1]))]
    return dict(points=points, lengths=lengths, speed_ratios=ratios, speed_ratio=ratios[0],
                aim_yaw=UnitedPosition.Pos2Angle(start, points[1]))

def _bank_shot_tolerance(ball_field, cushions, side):
    half_goal = _goal_half_opening_cm()
    routes = [_bank_route(ball_field, [_opponent_goal_x(), y], cushions, side)
              for y in (0.0, -half_goal, half_goal)]
    if any(route is None for route in routes):
        return 0.0
    yaw = routes[0]["aim_yaw"]
    opening = min(abs(angle_error_deg(yaw, route["aim_yaw"])) for route in routes[1:])
    return clamp(opening * 0.95, 0.5, AUSTRALIA.Tactics.SHOT_OPENING_ALIGN_MAX_DEG)

def _bank_shot_failure(ball_field, cushions, side, require_clear=True):
    route = _bank_route(ball_field, [_opponent_goal_x(), 0.0], cushions, side)
    if route is None:
        return "invalid_geometry"
    if _bank_shot_tolerance(ball_field, cushions, side) < 0.25:
        return "aim_window_too_small"
    # The side-wall contact is valid across the full 240cm length.  Reject
    # only a bounce whose ball centre would already be outside the end wall;
    # the old international-style ``FIELD_LONG/2 - 15`` check rejected valid
    # aggressive routes on the RCJA 240x180cm profile.
    if any(abs(point[0]) > SIM_END_WALL_BALL_CENTER_LIMIT_CM for point in route["points"][1:-1]):
        return "bounce_out_of_bounds"
    if require_clear:
        obstacles = _current_enemies()
        if not all(_lane_clear(a, b, obstacles, 16.0) for a, b in zip(route["points"], route["points"][1:])):
            return "lane_blocked"
    if _pass_launch_speed(route, AUSTRALIA.Tactics.SHOT_RECEIVE_SPEED_MPS) is None:
        return "insufficient_power"
    return None

def _bank_shot_feasible(ball_field, cushions, side, require_clear=True):
    return _bank_shot_failure(ball_field, cushions, side, require_clear=require_clear) is None

def _wall_shot_yaw(ball_field, cushions=1, wall_side=None):
    side = (1.0 if ball_field[1] >= 0 else -1.0) if wall_side is None else float(wall_side)
    route = _bank_route(ball_field, [_opponent_goal_x(), 0.0], cushions, side)
    return route["aim_yaw"] if route is not None else math.nan

def _wall_shot_plan(ball_field, cushions=None, wall_side=None, require_clear=True):
    _, ball_y = map(float, ball_field[:2])
    if wall_side is None:
        selected_side = 1.0 if ball_y >= 0.0 else -1.0
    else:
        if not math.isfinite(float(wall_side)):
            raise ValueError("wall_side must be finite")
        selected_side = 1.0 if float(wall_side) >= 0.0 else -1.0
    if cushions is not None:
        return _normalize_cushions(cushions), selected_side

    candidates = [(1, selected_side), (1, -selected_side)]
    for count, side in dict.fromkeys(candidates):
        if _bank_shot_feasible(ball_field, count, side, require_clear=require_clear):
            return count, side
    return None

def _realtime_wall_shot_plan(ball_field, wall_side=None):
    return (_wall_shot_plan(ball_field, None, wall_side, require_clear=True)
            or _wall_shot_plan(ball_field, None, wall_side, require_clear=False))

def _refresh_wall_shot_cache(pose, ball_state, ball_field):
    """仅在直射被挡且已持球时预算一库，避免无球期间重复枚举墙面。"""

    global _WALL_READY_PLAN, _WALL_READY_BALL, _WALL_READY_TIME, _WALL_READY_VALID
    global _DIRECT_READY_ROUTE, _DIRECT_READY_POSE, _DIRECT_READY_TIME, _DIRECT_READY_VALID

    if ball_state != "held":
        _DIRECT_READY_ROUTE = None
        _DIRECT_READY_POSE = None
        _DIRECT_READY_TIME = -math.inf
        _DIRECT_READY_VALID = False
        _WALL_READY_PLAN = None
        _WALL_READY_BALL = None
        _WALL_READY_TIME = -math.inf
        _WALL_READY_VALID = False
        return
    _cache_direct_shot_route(pose)
    if _DIRECT_READY_ROUTE is not None:
        _WALL_READY_PLAN = None
        _WALL_READY_BALL = None
        _WALL_READY_TIME = -math.inf
        _WALL_READY_VALID = False
        return
    candidate = _held_ball_field_pos(pose)
    try:
        candidate = [float(candidate[0]), float(candidate[1])]
    except (TypeError, ValueError, IndexError):
        _WALL_READY_PLAN = None
        _WALL_READY_BALL = None
        _WALL_READY_TIME = -math.inf
        _WALL_READY_VALID = False
        return
    if not all(math.isfinite(value) for value in candidate):
        _WALL_READY_PLAN = None
        _WALL_READY_BALL = None
        _WALL_READY_TIME = -math.inf
        _WALL_READY_VALID = False
        return
    _WALL_READY_PLAN = _wall_shot_plan(candidate, None, None, require_clear=False)
    _WALL_READY_BALL = candidate
    _WALL_READY_TIME = time.monotonic()
    _WALL_READY_VALID = True

def _ready_wall_shot_plan(ball_field, wall_side=None):
    """优先采用本周期已算出的墙侧；缓存不匹配时才退回实时搜索。"""

    if (wall_side is None and _WALL_READY_VALID and _WALL_READY_BALL is not None
            and time.monotonic() - _WALL_READY_TIME <= 0.04
            and math.dist(ball_field[:2], _WALL_READY_BALL) <= 2.0):
        return _WALL_READY_PLAN
    return _realtime_wall_shot_plan(ball_field, wall_side)

def wall__shoot(cushions=1, wall_side=None):
    pose = _shot_ready()
    if pose is None:
        return False

    ball_field = _held_ball_field_pos(pose)
    selected = _wall_shot_plan(ball_field, cushions, wall_side, require_clear=False)
    if selected is None:
        chassis.stop()
        return False
    cushions, wall_side = selected
    route = _bank_route(ball_field, [_opponent_goal_x(), 0.0], cushions, wall_side)
    if route is None:
        chassis.stop()
        return False
    bank_yaw = route["aim_yaw"]
    goal_dx = _opponent_goal_x() - ball_field[0]
    if goal_dx * ATTACK_DIR <= 0.0:
        chassis.stop()
        return False
    aim_tolerance = _bank_shot_tolerance(ball_field, cushions, wall_side)
    if not _bank_shot_feasible(ball_field, cushions, wall_side, require_clear=False):
        chassis.stop()
        return False
    if not _turn_held(bank_yaw, aim_tolerance):
        return False
    return _fire_shot(route)

def safe_defence(look_at=None):
    pose = _motion_guard()
    if pose is None:
        return False
    if _effective_ball_state() == "held":
        chassis.stop()
        return False

    guard_yaw = 0.0 if ATTACK_DIR > 0 else 180.0
    if look_at is not None:
        guard_yaw = UnitedPosition.Pos2Angle(pose, look_at)
    guard_x = _guard_line_x()

    max_y = min(Settings.Field.Short / 2.0 - 32.0, float(Settings.Vision.GoalWidth) / 2.0)

    if not hasattr(safe_defence, "target_y"):
        # 两台无球巡逻不能从同一门线端出发，否则会在门前互相顶住。
        safe_defence.target_y = max_y if _robot_id() % 2 else -max_y

    target = _safe_target(guard_x, safe_defence.target_y, guard_yaw)
    if math.hypot(pose[0] - target[0], pose[1] - target[1]) < 6.0:
        safe_defence.target_y = (
            max_y if safe_defence.target_y < 0 else -max_y
        )
        target = _safe_target(guard_x, safe_defence.target_y, guard_yaw)
    _position_step(target, speed_gain=1.0)

    peripheral.Dribble(False)
    return True

def _sensor_ball_snapshot():
    global _LAST_SEEN_BALL_FIELD, _LAST_SEEN_BALL_TIME

    local_ball = _local_robot_ball_pos()
    state = _effective_ball_state(local_ball)
    field_ball = None
    if state == "free":
        # 同一循环的语义状态和场地坐标必须由同一份相对球包推导。公共场地球
        # 包装会再次采样，高速收球时可能把相邻两帧拼到同一个决策里。
        field_ball = _field_ball_from_local_observation(local_ball)
        if field_ball is not None:
            _LAST_SEEN_BALL_FIELD = list(field_ball)
            _LAST_SEEN_BALL_TIME = time.monotonic()
    elif state == "held":
        _LAST_SEEN_BALL_FIELD = None
        _LAST_SEEN_BALL_TIME = -math.inf
    return state, field_ball

def _remembered_ball_field():
    if _LAST_SEEN_BALL_FIELD is None:
        return None
    if not 0.0 <= time.monotonic() - _LAST_SEEN_BALL_TIME <= AUSTRALIA.Tactics.LAST_BALL_MEMORY_S:
        return None
    return list(_LAST_SEEN_BALL_FIELD)

def _nearest_enemy(enemies, reference):
    """Find the closest current YOLO enemy to a field-frame reference point."""

    best = None
    best_distance = float("inf")
    if reference is None:
        return None, best_distance
    for enemy in enemies or []:
        point = _field_xy(enemy)
        if point is None:
            continue
        distance = math.hypot(point[0] - reference[0], point[1] - reference[1])
        if distance < best_distance:
            best = point
            best_distance = distance
    return best, best_distance

def _approach_speed(enemy, previous_enemies, dt):
    if not AUSTRALIA.Tactics.ENEMY_SAMPLE_MIN_S <= dt <= AUSTRALIA.Tactics.ENEMY_SAMPLE_MAX_S:
        return 0.0
    previous, previous_distance = _nearest_enemy(previous_enemies, enemy)
    if previous is None or previous_distance > AUSTRALIA.Tactics.RADICAL_TRACK_CM:
        return 0.0
    speed = -ATTACK_DIR * (enemy[0] - previous[0]) / dt
    return speed if abs(speed) <= AUSTRALIA.Tactics.ENEMY_MAX_SPEED_CM_S else 0.0

def _choose_approaching_enemy(enemies, previous_enemies, dt):
    best = None
    best_score = -float("inf")
    for enemy in enemies or []:
        point = _field_xy(enemy)
        if point is None:
            continue

        # 数值越大，说明对手沿 X 轴越接近我方球门。
        progress = -ATTACK_DIR * point[0]
        speed = _approach_speed(point, previous_enemies, dt)
        in_our_half = progress >= 0.0
        moving_toward_us = (
            progress >= AUSTRALIA.Tactics.RADICAL_NEAR_MIDFIELD_PROGRESS_CM
            and speed >= AUSTRALIA.Tactics.RADICAL_APPROACH_SPEED_CM_S
        )
        if not (in_our_half or moving_toward_us):
            continue

        score = progress + max(0.0, speed) * 0.5
        if score > best_score:
            best = point
            best_score = score
    return best

def _choose_high_speed_enemy(enemies, previous_enemies, dt, reference=None):
    if not AUSTRALIA.Tactics.ENEMY_SAMPLE_MIN_S <= dt <= AUSTRALIA.Tactics.ENEMY_SAMPLE_MAX_S:
        return None
    best = None
    best_score = -float("inf")
    for enemy in enemies or []:
        point = _field_xy(enemy)
        if point is None:
            continue
        progress = -ATTACK_DIR * point[0]
        goal_speed = _approach_speed(point, previous_enemies, dt)
        radial_speed = 0.0
        if reference is not None and dt > 0.0:
            previous, previous_distance = _nearest_enemy(previous_enemies, point)
            if previous is not None and previous_distance <= AUSTRALIA.Tactics.RADICAL_TRACK_CM:
                previous_to_reference = math.hypot(
                    previous[0] - float(reference[0]),
                    previous[1] - float(reference[1]),
                )
                current_to_reference = math.hypot(
                    point[0] - float(reference[0]),
                    point[1] - float(reference[1]),
                )
                radial_speed = (previous_to_reference - current_to_reference) / dt
        speed = radial_speed if reference is not None else goal_speed
        if reference is not None and math.hypot(point[0] - reference[0], point[1] - reference[1]) > 90.0:
            continue
        if speed > AUSTRALIA.Tactics.ENEMY_MAX_SPEED_CM_S:
            continue
        if (
            speed < AUSTRALIA.Tactics.HIGH_SPEED_ENEMY_CM_S
            or (
                progress < AUSTRALIA.Tactics.HIGH_SPEED_ENEMY_MIN_PROGRESS_CM
                and radial_speed < AUSTRALIA.Tactics.HIGH_SPEED_ENEMY_CM_S
            )
        ):
            continue
        score = speed + max(0.0, progress) * 0.2
        if score > best_score:
            best = point
            best_score = score
    return best

def _detect_high_speed_enemy(reference=None):
    global _RADICAL_PREVIOUS_ENEMIES, _RADICAL_PREVIOUS_ENEMY_TIME

    now = time.monotonic()
    current_enemies = _current_enemies()
    dt = now - _RADICAL_PREVIOUS_ENEMY_TIME
    if 0.0 < dt < AUSTRALIA.Tactics.ENEMY_SAMPLE_MIN_S:
        return None
    previous_enemies = _RADICAL_PREVIOUS_ENEMIES
    high_speed_enemy = _choose_high_speed_enemy(
        current_enemies,
        previous_enemies,
        dt,
        reference,
    )
    _RADICAL_PREVIOUS_ENEMIES = current_enemies
    _RADICAL_PREVIOUS_ENEMY_TIME = now
    return high_speed_enemy

def _lane_clear(start, end, enemies, clearance_cm=None):
    radius = clearance_cm
    if radius is None:
        radius = (float(Settings.Vision.ChassisWidth) + float(Settings.Vision.BallDiameter)) / 2.0 + 3.0
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared < 0.01:
        return True
    for enemy in enemies:
        t = clamp(((enemy[0] - start[0]) * dx + (enemy[1] - start[1]) * dy) / length_squared, 0.0, 1.0)
        if math.hypot(enemy[0] - start[0] - t * dx, enemy[1] - start[1] - t * dy) < radius:
            return False
    return True

def _rule_boundary_limits_cm():
    """Return Australia 2026's last legal robot-centre coordinates."""

    # The RCJA referee removes a robot only after its whole 22cm body has
    # crossed the white line.  Leave a tiny numeric margin at that exact legal
    # limit; this is not an enlarged out-area buffer.
    return (
        Settings.Field.Long / 2.0 + AUSTRALIA.Tactics.SIM_BODY_RADIUS_CM - AUSTRALIA.Tactics.SIM_RULE_EDGE_EPSILON_CM,
        Settings.Field.Short / 2.0 + AUSTRALIA.Tactics.SIM_BODY_RADIUS_CM - AUSTRALIA.Tactics.SIM_RULE_EDGE_EPSILON_CM,
    )

SIM_WALL_CONTACT_X_CM, SIM_WALL_CONTACT_Y_CM = _rule_boundary_limits_cm()

def _rule_legal_target(x, y, yaw):
    # Unity 的边墙按 11cm 接触半径阻挡；禁区则交给比赛模式的 ProjectSafePose。
    target = [
        clamp(float(x), -SIM_WALL_CONTACT_X_CM + 0.1, SIM_WALL_CONTACT_X_CM - 0.1),
        clamp(float(y), -SIM_WALL_CONTACT_Y_CM + 0.1, SIM_WALL_CONTACT_Y_CM - 0.1),
        float(yaw) % 360.0,
    ]
    projected = UnitedPosition.ProjectSafePose(target)
    return projected if projected is not None else target

def _rule_target_is_legal(target):
    if not _pose_valid(target):
        return False
    if abs(float(target[0])) > SIM_WALL_CONTACT_X_CM or abs(float(target[1])) > SIM_WALL_CONTACT_Y_CM:
        return False
    return UnitedPosition.ProjectSafePose(target) is None

def _carry_safe_target(x, y, yaw, *, goal_post_escape=False):
    x, y, yaw = float(x), float(y), float(yaw)
    if not all(math.isfinite(value) for value in (x, y, yaw)):
        return None
    params = UnitedPosition._restricted_area_params()
    x_limit = min(
        SIM_WALL_CONTACT_X_CM - 0.1,
        max(0.0, float(params["x_limit"]) - 0.1),
    )
    y_limit = min(
        AUSTRALIA.Tactics.MACAU_POCKET_MAX_ABS_Y_CM,
        max(0.0, float(params["y_limit"]) - 0.1),
    )
    x = clamp(x, -x_limit, x_limit)
    y = clamp(y, -y_limit, y_limit)
    if abs(y) >= AUSTRALIA.Tactics.MACAU_ENTRY_MIN_ABS_Y_CM and not goal_post_escape:
        progress = ATTACK_DIR * x
        if progress > AUSTRALIA.Tactics.MACAU_POCKET_MAX_PROGRESS_CM:
            x = ATTACK_DIR * AUSTRALIA.Tactics.MACAU_POCKET_MAX_PROGRESS_CM

    if AUSTRALIA.Tactics.SIM_PENALTY_RULE_ENABLED:
        # International uses the actual 83/32 robot-centre boundary, rather
        # than Position's underlying 72/43 construction geometry.
        goal_front = AUSTRALIA.Tactics.SIM_PENALTY_FRONT_CM - AUSTRALIA.Tactics.SIM_RULE_EDGE_EPSILON_CM
        goal_half = min(y_limit, AUSTRALIA.Tactics.SIM_PENALTY_HALF_WIDTH_CM + AUSTRALIA.Tactics.SIM_RULE_EDGE_EPSILON_CM)
        if abs(x) >= goal_front and abs(y) <= goal_half:
            x = math.copysign(goal_front, x)

    target = _safe_target(x, y, yaw)
    return target if _rule_target_is_legal(target) else None

def _carry_path_legal(start, target, *, goal_post_escape=False):
    try:
        start = [float(start[0]), float(start[1])]
        target = [float(target[0]), float(target[1]), float(target[2])]
    except (TypeError, ValueError, IndexError):
        return False
    distance = math.dist(start, target[:2])
    for index in range(1, max(1, math.ceil(distance / 4.0)) + 1):
        ratio = index / max(1, math.ceil(distance / 4.0))
        x = start[0] + (target[0] - start[0]) * ratio
        y = start[1] + (target[1] - start[1]) * ratio
        legal = _carry_safe_target(x, y, target[2], goal_post_escape=goal_post_escape)
        if legal is None or math.dist(legal[:2], [x, y]) > 0.25:
            return False
    return True

def _cooperation_target(x, y, yaw):
    try:
        x, y, yaw = float(x), float(y), float(yaw) % 360.0
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in (x, y, yaw)):
        return None
    params = UnitedPosition._restricted_area_params()
    x_limit = min(
        Settings.Field.Long / 2.0 - COOP_FIELD_MARGIN_CM,
        float(params["x_limit"]) - float(params["boundary_margin"]) - 1.0,
    )
    y_limit = min(
        Settings.Field.Short / 2.0 - COOP_FIELD_MARGIN_CM,
        float(params["y_limit"]) - float(params["boundary_margin"]) - 1.0,
    )
    if abs(x) > x_limit or abs(y) > y_limit:
        return None
    target = [x, y, yaw]
    return target if _rule_target_is_legal(target) else None

def slip_side_shoot():
    global _SLIP_TARGET, _SLIP_SIDE, _SLIP_SINCE

    pose = _shot_ready()
    if pose is None:
        return False
    now = time.monotonic()
    if _SLIP_SINCE <= 0.0:
        enemy, _ = _nearest_enemy(_current_enemies(), pose)
        _SLIP_SIDE = _escape_wall_side(pose, enemy) if enemy is not None else (
            -1.0 if pose[1] >= 0.0 else 1.0
        )
        shield_yaw = UnitedPosition.Pos2Angle(enemy, pose) if enemy is not None else (
            0.0 if ATTACK_DIR > 0 else 180.0
        )
        _SLIP_TARGET = _carry_safe_target(
            pose[0] + ATTACK_DIR * AUSTRALIA.Tactics.SLIP_SIDE_ESCAPE_CM,
            pose[1] + _SLIP_SIDE * AUSTRALIA.Tactics.SLIP_SIDE_LATERAL_CM,
            shield_yaw,
        )
        if _SLIP_TARGET is None:
            _reset_attack()
            return False
        _SLIP_SINCE = now

    if now - _SLIP_SINCE > AUSTRALIA.Tactics.SLIP_SIDE_TIMEOUT_S:
        _reset_attack()
        return False

    if math.dist(pose[:2], _SLIP_TARGET[:2]) > 7.0 and now - _SLIP_SINCE < 0.8:
        _carry_to(pose, _SLIP_TARGET, _SLIP_TARGET[2])
        return False

    route = _direct_shot_route(pose)
    if route is not None:
        tolerance = _direct_shot_tolerance(route)
        if _turn_held(route["aim_yaw"], tolerance) and _fire_shot(route):
            _reset_attack()
            return True
        return False

    ball = _held_ball_field_pos(pose)
    plan = _realtime_wall_shot_plan(ball, _SLIP_SIDE)
    if plan is not None and _start_shot("wall", pose, wall_plan=plan):
        return False
    _reset_attack()
    return False

def _macau_shield_yaw(pose, enemy):
    """Return the single-opponent fallback shield heading."""

    if enemy is not None:
        away_x = float(pose[0]) - float(enemy[0])
        away_y = float(pose[1]) - float(enemy[1])
        if math.hypot(away_x, away_y) > 1.0:
            return UnitedPosition.Pos2Angle(
                pose, [pose[0] + away_x, pose[1] + away_y]
            )
    # With no visible opponent, keep the catcher pointed toward our own half;
    # this preserves the shield posture instead of turning toward the goal
    # before the final shooting turn.
    return UnitedPosition.Pos2Angle(
        pose, [pose[0] - ATTACK_DIR, pose[1]]
    )

def _macau_ball_for_heading(pose, yaw):
    """Predict the locally held ball point for one candidate robot heading."""

    theta = math.radians(float(yaw))
    catch_x, catch_y = SIM_CATCH_POINT_CM
    return [
        float(pose[0]) + catch_x * math.cos(theta) - catch_y * math.sin(theta),
        float(pose[1]) + catch_x * math.sin(theta) + catch_y * math.cos(theta),
    ]

def _macau_occlusion_score(pose, yaw, enemy, rear_priority):
    """Score whether the robot body blocks one visible opponent's ball view.

    This uses only the opponent point supplied by this robot's vision.  The
    segment test models the opponent's line of sight to the locally predicted
    catcher point; it never asks Unity whether the ball is hidden or owned.
    """

    try:
        enemy_x, enemy_y = float(enemy[0]), float(enemy[1])
    except (TypeError, ValueError, IndexError):
        return -float("inf")
    ball = _macau_ball_for_heading(pose, yaw)
    line_x, line_y = ball[0] - enemy_x, ball[1] - enemy_y
    length_squared = line_x * line_x + line_y * line_y
    if length_squared < 0.01:
        return -float("inf")
    robot_x, robot_y = float(pose[0]), float(pose[1])
    projection = ((robot_x - enemy_x) * line_x + (robot_y - enemy_y) * line_y) / length_squared
    nearest_x = enemy_x + clamp(projection, 0.0, 1.0) * line_x
    nearest_y = enemy_y + clamp(projection, 0.0, 1.0) * line_y
    distance = math.hypot(robot_x - nearest_x, robot_y - nearest_y)
    blocking_radius = AUSTRALIA.Tactics.SIM_BODY_RADIUS_CM + AUSTRALIA.Real.REAL_BALL_DIAMETER_CM / 2.0 + AUSTRALIA.Tactics.MACAU_SHIELD_BODY_MARGIN_CM
    margin = blocking_radius - distance
    weight = 3.0 if rear_priority else 1.0
    if 0.0 <= projection <= 1.0 and margin >= 0.0:
        # A positive margin means the chassis occupies the sight segment.  A
        # rear opponent gets a larger weight when one opponent is ahead and
        # one is behind, as requested for the advanced Macau route.
        return weight * (80.0 + min(margin, 10.0))
    # Keep a weak gradient when not fully occluded, so the heading still turns
    # toward the best available shield instead of making random 15-degree hops.
    return weight * (-20.0 - max(0.0, distance - blocking_radius))

def _macau_replan_shield_yaw(pose, enemies, current_yaw=None):
    """Choose a stable heading that maximizes visual occlusion of opponents."""

    valid_enemies = []
    for enemy in enemies or []:
        point = _field_xy(enemy)
        if point is not None:
            valid_enemies.append(point)
    if not valid_enemies:
        return current_yaw if current_yaw is not None else _macau_shield_yaw(pose, None)

    # A rear opponent is one whose field progress is behind the carrier.  When
    # there are opponents on both sides, rear threats receive the priority
    # weight in _macau_occlusion_score; otherwise the nearest visible machine
    # supplies the natural fallback direction.
    rear_exists = any(
        ATTACK_DIR * (enemy[0] - pose[0]) <= -AUSTRALIA.Tactics.MACAU_SHIELD_REAR_PROGRESS_CM
        for enemy in valid_enemies
    )
    candidates = [float(index) for index in range(0, 360, int(AUSTRALIA.Tactics.MACAU_SHIELD_SAMPLE_DEG))]
    candidates.extend(_macau_shield_yaw(pose, enemy) for enemy in valid_enemies)
    if current_yaw is not None:
        candidates.append(float(current_yaw) % 360.0)
    best_yaw, best_score = None, -float("inf")
    for candidate in candidates:
        score = 0.0
        for enemy in valid_enemies:
            rear = rear_exists and ATTACK_DIR * (enemy[0] - pose[0]) <= -AUSTRALIA.Tactics.MACAU_SHIELD_REAR_PROGRESS_CM
            score += _macau_occlusion_score(pose, candidate, enemy, rear)
        # Avoid unnecessary chassis rotation when two headings shield equally
        # well.  This is a small hysteresis term, not a lock on old geometry.
        if current_yaw is not None:
            score -= 0.08 * abs(angle_error_deg(current_yaw, candidate))
        if score > best_score:
            best_yaw, best_score = candidate, score
    if best_yaw is None:
        return current_yaw if current_yaw is not None else _macau_shield_yaw(pose, valid_enemies[0])
    if current_yaw is None:
        return best_yaw % 360.0
    current_score = sum(
        _macau_occlusion_score(
            pose,
            current_yaw,
            enemy,
            rear_exists and ATTACK_DIR * (enemy[0] - pose[0]) <= -AUSTRALIA.Tactics.MACAU_SHIELD_REAR_PROGRESS_CM,
        )
        for enemy in valid_enemies
    )
    if best_score - current_score < AUSTRALIA.Tactics.MACAU_SHIELD_REPLAN_GAIN:
        return float(current_yaw) % 360.0
    correction = angle_error_deg(current_yaw, best_yaw)
    if abs(correction) > AUSTRALIA.Tactics.MACAU_SHIELD_MAX_STEP_DEG:
        correction = math.copysign(AUSTRALIA.Tactics.MACAU_SHIELD_MAX_STEP_DEG, correction)
    return (float(current_yaw) + correction) % 360.0

def _macau_replan_lane_y(pose, enemies):
    """Nudge the edge lane toward a rear threat without leaving the channel."""

    global _MACAU_LANE_TARGET_Y

    side = _MACAU_SIDE if _MACAU_SIDE in (-1.0, 1.0) else (1.0 if pose[1] >= 0 else -1.0)
    lower, upper = sorted((side * AUSTRALIA.Tactics.MACAU_ENTRY_MIN_ABS_Y_CM, side * AUSTRALIA.Tactics.MACAU_POCKET_MAX_ABS_Y_CM))
    desired = side * AUSTRALIA.Tactics.MACAU_POCKET_TARGET_ABS_Y_CM
    rear_candidates = [
        enemy for enemy in (enemies or [])
        if ATTACK_DIR * (enemy[0] - pose[0]) <= -AUSTRALIA.Tactics.MACAU_SHIELD_REAR_PROGRESS_CM
        and side * enemy[1] >= AUSTRALIA.Tactics.MACAU_ENTRY_MIN_ABS_Y_CM - 12.0
    ]
    if rear_candidates:
        rear = min(rear_candidates, key=lambda item: math.dist(item, pose[:2]))
        desired = clamp(float(rear[1]), lower, upper)
    if _MACAU_LANE_TARGET_Y is None:
        _MACAU_LANE_TARGET_Y = side * AUSTRALIA.Tactics.MACAU_POCKET_TARGET_ABS_Y_CM
    delta = clamp(desired - _MACAU_LANE_TARGET_Y,
                  -AUSTRALIA.Tactics.MACAU_SHIELD_LANE_STEP_CM, AUSTRALIA.Tactics.MACAU_SHIELD_LANE_STEP_CM)
    _MACAU_LANE_TARGET_Y = clamp(_MACAU_LANE_TARGET_Y + delta, lower, upper)
    return _MACAU_LANE_TARGET_Y

def macau_shoot():
    global _MACAU_SIDE, _MACAU_PHASE, _MACAU_SHIELD_YAW, _MACAU_RETRY_AFTER

    pose = _shot_ready()
    if pose is None:
        return False
    enemies = _current_enemies()
    enemy, _ = _nearest_enemy(enemies, pose)
    if _MACAU_SIDE is None:
        if abs(pose[1]) < AUSTRALIA.Tactics.MACAU_ENTRY_MIN_ABS_Y_CM:
            _reset_attack()
            _MACAU_RETRY_AFTER = time.monotonic() + AUSTRALIA.Tactics.MACAU_RETRY_S
            return False
        _MACAU_SIDE = 1.0 if pose[1] >= 0.0 else -1.0
        _MACAU_LANE_TARGET_Y = _MACAU_SIDE * AUSTRALIA.Tactics.MACAU_POCKET_TARGET_ABS_Y_CM
        _MACAU_SHIELD_YAW = _macau_replan_shield_yaw(pose, enemies)
        # 澳门射不在中途切入中路。保持同一条边线通道和背身朝向，
        # 直到门前约 104cm 的射门口袋才停止平移。
        _MACAU_PHASE = "edge_carry"

    goal_x = _opponent_goal_x()
    pressure = any(
        ATTACK_DIR * (item[0] - pose[0]) >= -AUSTRALIA.Tactics.SIM_BODY_RADIUS_CM
        and math.dist(item, pose[:2]) <= AUSTRALIA.Tactics.FRONT_PRESSURE_DISTANCE_CM
        for item in enemies
    )
    progress = ATTACK_DIR * pose[0]
    lateral = abs(pose[1])
    _MACAU_PHASE = "edge_carry"
    # 每帧用本机视觉重新评估遮挡方向。朝向只限速修正，避免对手检测抖动
    # 把机器变成原地左右摆头；边线侧向目标也只小步修正。
    _MACAU_SHIELD_YAW = _macau_replan_shield_yaw(
        pose, enemies, current_yaw=_MACAU_SHIELD_YAW
    )
    lane_target_y = _macau_replan_lane_y(pose, enemies)
    # 固定在边线通道，沿边线一直推进到 104cm；绝不把目标改到中路。
    target = _carry_safe_target(
        ATTACK_DIR * AUSTRALIA.Tactics.MACAU_POCKET_TARGET_PROGRESS_CM,
        lane_target_y,
        _MACAU_SHIELD_YAW,
    )
    in_pocket = (
        target is not None
        and progress >= AUSTRALIA.Tactics.MACAU_POCKET_MIN_PROGRESS_CM
        and lateral >= AUSTRALIA.Tactics.MACAU_POCKET_MIN_ABS_Y_CM
        and math.dist(pose[:2], target[:2]) <= AUSTRALIA.Tactics.MACAU_POCKET_ARRIVAL_CM
    )

    peripheral.Dribble(True)
    if pressure:
        # 对手已经贴住盘球器时不能在边线等窗口。把球按远离贴防的一侧一库强力打出，
        # 即使不进球也会立即形成自由球，队友已按边线持球者的位置向内前插接应。
        ball = _held_ball_field_pos(pose)
        side = _escape_wall_side(pose, enemy) if enemy is not None else -_MACAU_SIDE
        plan = _wall_shot_plan(ball, None, side, require_clear=False)
        if plan is not None and _start_shot("emergency_wall", pose, wall_plan=plan):
            return False
    if in_pocket:
        # 到达 104cm 射门口袋后只原地转向，不再带球切入。优先使用实际
        # 两柱之间的后壁瞄准点；若当前视觉暂时挡住直射线，则保留朝门姿态，
        # 并在本帧按新场地的一库路线出球，避免进攻停死。
        route = _ready_direct_shot_route(pose)
        shoot_yaw = (
            route["aim_yaw"] if route is not None
            else UnitedPosition.Pos2Angle(pose, [goal_x, 0.0])
        )
        # 进入口袋后必须是原地转向；不能让上一帧的边线平移惯性带着球
        # 继续冲过门线。
        chassis.stop()
        if not _turn_held(shoot_yaw, tolerance=12.0):
            return False
        if route is not None and direct_shoot(route=route):
            _reset_attack()
            return True
        ball = _held_ball_field_pos(pose)
        wall_plan = _realtime_wall_shot_plan(ball, _MACAU_SIDE)
        if wall_plan is not None and _start_shot("wall", pose, wall_plan=wall_plan):
            return False
        return False

    # 目标和背身朝向只由启动澳门射时所在边线决定，不能随敌方观测的轻微
    # 跳变来回切换。到达门前口袋后才取消盘球并转向射门。
    if target is None:
        _reset_attack()
        _MACAU_RETRY_AFTER = time.monotonic() + AUSTRALIA.Tactics.MACAU_RETRY_S
        return False
    _carry_to(pose, target, _MACAU_SHIELD_YAW)
    return False

def split_shoot():
    global _SPLIT_TARGET, _SPLIT_SIDE, _SPLIT_SINCE

    pose = _shot_ready()
    if pose is None:
        return False
    now = time.monotonic()
    if _SPLIT_TARGET is None:
        enemy, _ = _nearest_enemy(_current_enemies(), pose)
        preferred_side = _escape_wall_side(pose, enemy) if enemy is not None else (-1.0 if pose[1] > 0.0 else 1.0)
        for side in (preferred_side, -preferred_side):
            target = _carry_safe_target(pose[0], pose[1] + side * AUSTRALIA.Tactics.SPLIT_SHIFT_CM, pose[2])
            if (target is not None and abs(target[1] - pose[1]) >= 12.0
                    and _lane_clear(pose, target, _current_enemies(), float(Settings.Vision.ChassisWidth) + 4.0)):
                _SPLIT_SIDE, _SPLIT_TARGET = side, target
                _SPLIT_SINCE = now
                break
        if _SPLIT_TARGET is None:
            _reset_attack()
            chassis.stop()
            return False
    if now - _SPLIT_SINCE > AUSTRALIA.Tactics.SPLIT_TIMEOUT_S:
        _reset_attack()
        chassis.stop()
        return False

    goal_y = -_SPLIT_SIDE * _goal_half_opening_cm() * 0.55
    yaw = UnitedPosition.Pos2Angle(pose, [_opponent_goal_x(), goal_y])
    if math.hypot(pose[0] - _SPLIT_TARGET[0], pose[1] - _SPLIT_TARGET[1]) > 6.0:
        _carry_to(pose, _SPLIT_TARGET, yaw)
        return False
    if direct_shoot(goal_y):
        _reset_attack()
        return True
    # 已经横移到位却仍然没有直射窗口时，不能留在原地等对方让出通道。
    # 立刻撤销错位动作，下一控制周期改试弹墙、传球或护球换位。
    _reset_attack()
    return False

def _escape_wall_side(pose, enemy):
    if float(enemy[1]) > float(pose[1]):
        return -1.0
    if float(enemy[1]) < float(pose[1]):
        return 1.0
    return -1.0 if float(pose[1]) >= 0.0 else 1.0

def _drop_ball_slots():
    # 对齐 Unity GetDropBallSpotSlotsCm(): goalLine(109.5)-45, y=±50。
    x = Settings.Field.Long / 2.0 - AUSTRALIA.Tactics.SIM_GOAL_DEPTH_CM - AUSTRALIA.Tactics.DEADBALL_SLOT_INSET_CM
    y = AUSTRALIA.Tactics.SIM_PENALTY_RAW_HALF_WIDTH_CM
    return ([0.0, 0.0], [x, y], [x, -y], [-x, y], [-x, -y])

def _predicted_pushing_drop_spot(ball, robots):
    point = _field_xy(ball)
    if point is None or not isinstance(robots, dict):
        return None
    best, best_distance = None, -math.inf
    for slot in _drop_ball_slots():
        occupied = any(
            math.dist([item["x_cm"], item["y_cm"]], slot) < AUSTRALIA.Tactics.DEADBALL_SLOT_OCCUPIED_CM
            for item in robots.values()
        )
        if occupied:
            continue
        distance = math.dist(point, slot)
        if distance > best_distance:
            best, best_distance = list(slot), distance
    return best

def _precomputed_drop_wall_route(slot):
    point = _field_xy(slot)
    if point is None:
        return None
    preferred_side = 1.0 if point[1] >= 0.0 else -1.0
    candidates = [(1, preferred_side), (1, -preferred_side)]
    for count, side in candidates:
        route = _bank_route(point, [_opponent_goal_x(), 0.0], count, side)
        if route is None or _bank_shot_tolerance(point, count, side) < 0.25:
            continue
        if _pass_launch_speed(route, AUSTRALIA.Tactics.SHOT_RECEIVE_SPEED_MPS) is not None:
            return count, side, route["aim_yaw"]
    return None

def _deadball_plan(observation):
    return None

def _deadball_support_step(plan):
    global _DROP_BALL_PREP

    if not isinstance(plan, dict) or plan.get("supporter") != _robot_id():
        return False
    pose = _motion_guard()
    if pose is None:
        return False
    _DROP_BALL_PREP = dict(plan)
    peripheral.Dribble(True)
    if plan["ready"]:
        chassis.stop()
        return True
    _position_step(plan["stage"], speed_gain=1.15, arrival_cm=1.0)
    return True

def _prepared_drop_wall_step(pose, immediate_only=False):
    global _DROP_BALL_PREP

    plan = _DROP_BALL_PREP
    if plan is None:
        return None
    if time.monotonic() - plan.get("stamp", -math.inf) > 3.0:
        _DROP_BALL_PREP = None
        return None
    ball = _held_ball_field_pos(pose)
    if math.dist(ball, plan["slot"]) > 18.0:
        _DROP_BALL_PREP = None
        return None
    route = _bank_route(ball, [_opponent_goal_x(), 0.0], plan["cushions"], plan["side"])
    if route is None or _bank_shot_failure(ball, plan["cushions"], plan["side"]) is not None:
        _DROP_BALL_PREP = None
        return None
    tolerance = _bank_shot_tolerance(ball, plan["cushions"], plan["side"])
    if abs(angle_error_deg(pose[2], route["aim_yaw"])) > tolerance:
        if immediate_only:
            return None
        _turn_held(route["aim_yaw"], tolerance)
        return f"drop_wall_{plan['cushions']}_aim"
    if _fire_shot(route):
        count = plan["cushions"]
        _DROP_BALL_PREP = None
        return f"drop_wall_{count}_shoot"
    return f"drop_wall_{plan['cushions']}_settle"

def _challenge_enemy(enemy, enemy_pose=None, stand_off_cm=AUSTRALIA.Tactics.RADICAL_STANDOFF_CM):
    pose = UnitedPosition.FieldRobotPose()
    if not _pose_valid(enemy_pose):
        enemy_pose = [*enemy, UnitedPosition.Pos2Angle(enemy, [-ATTACK_DIR * Settings.Field.Long / 2.0, 0.0])]
    theta = math.radians(enemy_pose[2])
    forward = (math.cos(theta), math.sin(theta))
    left = (-forward[1], forward[0])
    target_yaw = (enemy_pose[2] + 180.0) % 360.0
    relative = (pose[0] - enemy[0], pose[1] - enemy[1])
    forward_progress = relative[0] * forward[0] + relative[1] * forward[1]
    lateral_progress = relative[0] * left[0] + relative[1] * left[1]
    side = math.copysign(1.0, lateral_progress) if abs(lateral_progress) > 1.0 else (
        1.0 if pose[1] >= enemy[1] else -1.0
    )
    front_prep_distance = stand_off_cm + AUSTRALIA.Tactics.RADICAL_PREP_FORWARD_CM
    rear_side = [
        enemy[i] - AUSTRALIA.Tactics.RADICAL_REAR_ENTRY_CM * forward[i] + side * AUSTRALIA.Tactics.RADICAL_PREP_LATERAL_CM * left[i]
        for i in (0, 1)
    ]
    front_side = [
        enemy[i] + front_prep_distance * forward[i] + side * AUSTRALIA.Tactics.RADICAL_PREP_LATERAL_CM * left[i]
        for i in (0, 1)
    ]
    front_prep = [enemy[i] + front_prep_distance * forward[i] for i in (0, 1)]
    contact = [enemy[i] + stand_off_cm * forward[i] for i in (0, 1)]

    # 已近身时先接触破坏对方盘球和起脚，侧后方顶住也有效；距离较远才走前方预备通道。
    if math.dist(pose[:2], enemy[:2]) <= AUSTRALIA.Tactics.RADICAL_CONTACT_RUSH_CM:
        target_xy = contact
    elif forward_progress < -1.0 and abs(lateral_progress) < AUSTRALIA.Tactics.RADICAL_PREP_LATERAL_CM - 4.0:
        target_xy = rear_side
    elif (abs(lateral_progress) >= AUSTRALIA.Tactics.RADICAL_PREP_LATERAL_CM - 4.0
          and forward_progress < front_prep_distance - 1.0):
        target_xy = front_side
    elif abs(lateral_progress) > 8.0:
        target_xy = front_prep
    else:
        target_xy = contact

    target = _safe_target(*target_xy, target_yaw)
    peripheral.Dribble(True)
    _position_step(
        target,
        speed_gain=2.0,
        arrival_cm=AUSTRALIA.Tactics.RADICAL_PREP_ARRIVAL_CM,
        speed_limit=AUSTRALIA.Tactics.PRESS_MAX_MPS,
        near_speed_limit=AUSTRALIA.Tactics.PRESS_MAX_MPS,
    )
    return True

def _defence_observation(ball_state):
    global _RADICAL_LAST_ENEMY, _RADICAL_LAST_OVERLAP_TIME
    global _RADICAL_PREVIOUS_ENEMIES, _RADICAL_PREVIOUS_ENEMY_TIME
    global _DEFENCE_BALL_SAMPLE, _DEFENCE_BALL_VELOCITY

    now = time.monotonic()
    enemies = _current_enemies()
    dt = now - _RADICAL_PREVIOUS_ENEMY_TIME
    approaching = _choose_approaching_enemy(enemies, _RADICAL_PREVIOUS_ENEMIES, dt)
    if dt >= AUSTRALIA.Tactics.ENEMY_SAMPLE_MIN_S or dt <= 0.0:
        _RADICAL_PREVIOUS_ENEMIES = enemies
        _RADICAL_PREVIOUS_ENEMY_TIME = now

    sensed_state, ball = _sensor_ball_snapshot()
    if ball_state != "held":
        ball_state = sensed_state
    carrier = None
    enemy_pose = None
    if ball is not None:
        nearest, distance = _nearest_enemy(enemies, ball)
        if nearest is not None and distance <= AUSTRALIA.Tactics.HELD_ENEMY_OVERLAP_CM:
            carrier = nearest
            # 相机没有直接提供对手 yaw；球贴在其盘球器前时，中心到球的方向就是
            # 可用于弹墙防守预测的盘球器朝向。
            enemy_pose = [
                nearest[0], nearest[1],
                UnitedPosition.Pos2Angle(nearest, ball) % 360.0,
            ]
            _RADICAL_LAST_ENEMY = nearest
            _RADICAL_LAST_OVERLAP_TIME = now
        else:
            _RADICAL_LAST_ENEMY = None
        if _DEFENCE_BALL_SAMPLE is not None:
            previous_ball, sample_time = _DEFENCE_BALL_SAMPLE
            sample_dt = now - sample_time
            if AUSTRALIA.Tactics.ENEMY_SAMPLE_MIN_S <= sample_dt <= AUSTRALIA.Tactics.ENEMY_SAMPLE_MAX_S:
                vx = (ball[0] - previous_ball[0]) / sample_dt
                vy = (ball[1] - previous_ball[1]) / sample_dt
                _DEFENCE_BALL_VELOCITY = (
                    (vx, vy) if math.hypot(vx, vy) <= AUSTRALIA.Tactics.INTERCEPT_MAX_SPEED_CM_S else None
                )
            elif sample_dt > AUSTRALIA.Tactics.ENEMY_SAMPLE_MAX_S or sample_dt < 0.0:
                _DEFENCE_BALL_VELOCITY = None
            if sample_dt >= AUSTRALIA.Tactics.ENEMY_SAMPLE_MIN_S or sample_dt < 0.0:
                _DEFENCE_BALL_SAMPLE = (ball, now)
        else:
            _DEFENCE_BALL_SAMPLE = (ball, now)
            _DEFENCE_BALL_VELOCITY = None
    else:
        _DEFENCE_BALL_SAMPLE = None
        _DEFENCE_BALL_VELOCITY = None
        if ball_state == "lost" and _RADICAL_LAST_ENEMY is not None:
            if now - _RADICAL_LAST_OVERLAP_TIME <= AUSTRALIA.Tactics.RADICAL_MEMORY_TIME:
                tracked, distance = _nearest_enemy(enemies, _RADICAL_LAST_ENEMY)
                if tracked is not None and distance <= AUSTRALIA.Tactics.RADICAL_TRACK_CM:
                    carrier = tracked
                    _RADICAL_LAST_ENEMY = tracked
            else:
                _RADICAL_LAST_ENEMY = None
        else:
            _RADICAL_LAST_ENEMY = None
    return {
        "ball_state": ball_state,
        "ball": ball,
        "velocity": _DEFENCE_BALL_VELOCITY,
        "enemy": carrier if carrier is not None else approaching,
        "carrier": carrier,
        "enemy_pose": enemy_pose,
    }

def _guard_line_x():
    yaw = 0.0 if ATTACK_DIR > 0 else 180.0
    return _safe_target(-ATTACK_DIR * GOAL_LINE_X, 0.0, yaw)[0]

def _predicted_intercept_y(ball, velocity, guard_x):
    if ball is None or velocity is None:
        return None
    vx, vy = velocity
    if not all(math.isfinite(v) for v in (*ball, vx, vy, guard_x)):
        return None
    if -ATTACK_DIR * vx < AUSTRALIA.Tactics.INTERCEPT_MIN_SPEED_CM_S:
        return None
    arrival = (guard_x - ball[0]) / vx
    if not 0.0 <= arrival <= AUSTRALIA.Tactics.INTERCEPT_HORIZON_S:
        return None
    half_width = (Settings.Field.Short - AUSTRALIA.Real.REAL_BALL_DIAMETER_CM) / 2.0
    if half_width <= 0.0:
        return None
    y, remaining = clamp(ball[1], -half_width, half_width), arrival
    restitution = (_PASS_PHYSICS or (AUSTRALIA.Tactics.PASS_RESISTANCE_MODEL, AUSTRALIA.Tactics.PASS_WALL_RESTITUTION, AUSTRALIA.Tactics.PASS_KICK_IMPULSE_MPS))[1]
    for _ in range(AUSTRALIA.Tactics.MAX_CUSHIONS + 1):
        if abs(vy) < 0.01:
            return y
        wall = math.copysign(half_width, vy)
        hit_time = (wall - y) / vy
        if hit_time >= remaining:
            return y + vy * remaining
        remaining -= max(0.0, hit_time)
        y, vy = wall, -restitution * vy
    return clamp(y, -half_width, half_width)

def _incoming_goal_target(observation):
    """只拦截真正会高速进入我方球门的来球，普通自由球仍由最近者冲抢。"""
    ball, velocity = observation.get("ball"), observation.get("velocity")
    if observation.get("carrier") is not None or ball is None or velocity is None:
        return None
    if math.hypot(*velocity) <= AUSTRALIA.Tactics.INTERCEPT_CAPTURE_MAX_SPEED_CM_S:
        return None
    goal_y = _predicted_intercept_y(ball, velocity, -ATTACK_DIR * Settings.Field.Long / 2.0)
    if goal_y is None or abs(goal_y) > (float(Settings.Vision.GoalWidth) + AUSTRALIA.Real.REAL_BALL_DIAMETER_CM) / 2.0:
        return None
    guard_x = _guard_line_x()
    guard_y = _predicted_intercept_y(ball, velocity, guard_x)
    return None if guard_y is None else [guard_x, guard_y]

def _needs_defence(observation):
    global _DEFENCE_UNTIL

    now = time.monotonic()
    ball = observation["ball"]
    in_our_half = ball is not None and -ATTACK_DIR * ball[0] >= AUSTRALIA.Tactics.DEFENCE_BALL_ENTER_CM
    incoming = _predicted_intercept_y(ball, observation["velocity"], _guard_line_x())
    if in_our_half or incoming is not None or observation["enemy"] is not None:
        _DEFENCE_UNTIL = now + AUSTRALIA.Tactics.DEFENCE_RELEASE_DELAY_S
    return now < _DEFENCE_UNTIL

def intercept_defence(observation=None):
    pose = _motion_guard()
    if pose is None:
        return False
    state = _effective_ball_state()
    if state == "held":
        chassis.stop()
        return False
    if observation is None:
        observation = _defence_observation(state)
    ball = observation["ball"]
    source = ball if ball is not None else observation["enemy"]
    if source is None:
        return safe_defence()

    guard_x = _guard_line_x()
    goal = [-ATTACK_DIR * Settings.Field.Long / 2.0, 0.0]
    goal_dx = goal[0] - source[0]
    fraction = clamp((guard_x - source[0]) / goal_dx, 0.0, 1.0) if abs(goal_dx) > 0.01 else 0.0
    target_y = source[1] * (1.0 - fraction)
    predicted_y = _predicted_intercept_y(ball, observation["velocity"], guard_x)
    if predicted_y is not None:
        target_y = predicted_y
    yaw = UnitedPosition.Pos2Angle(pose, source) % 360.0

    velocity = observation["velocity"]
    capture_speed_ok = velocity is None or math.hypot(*velocity) <= AUSTRALIA.Tactics.INTERCEPT_CAPTURE_MAX_SPEED_CM_S
    if ball is not None and capture_speed_ok and math.dist(pose[:2], ball) <= AUSTRALIA.Tactics.INTERCEPT_CAPTURE_RADIUS_CM:
        if _rule_target_is_legal([ball[0], ball[1], yaw]):
            chase_ball(ball_field=ball)
            return True

    max_y = (Settings.Field.Short / 2.0 - AUSTRALIA.Tactics.SIM_BODY_RADIUS_CM - AUSTRALIA.Tactics.MOTION_CLEARANCE_CM
             if predicted_y is not None
             else min(Settings.Field.Short / 2.0 - 32.0, float(Settings.Vision.GoalWidth) / 2.0))
    target = _safe_target(guard_x, clamp(target_y, -max_y, max_y), yaw)
    peripheral.Dribble(True)
    _position_step(target, speed_gain=1.0, arrival_cm=1.0)
    return True

def radical_defence(observation=None):
    pose = _motion_guard()
    if pose is None:
        return False
    state = _effective_ball_state()
    if observation is None:
        observation = _defence_observation(state)
    enemy = observation["enemy"]
    if enemy is None:
        if state == "held":
            _reset_defence()
        chassis.stop()
        return False
    # 对方已明确持球时先用盘球器正面对顶，封死其踢球口；是否形成死球由裁判规则决定，
    # 不能为了等队友接应而停在 27cm 外。
    return _challenge_enemy(enemy, observation.get("enemy_pose"), AUSTRALIA.Tactics.DEADBALL_CONTACT_CM)

def _macau_side_run_detected(enemy, confirmed=False):
    global _MACAU_DEFENCE_LAST_CARRIER, _MACAU_DEFENCE_LAST_TIME, _MACAU_DEFENCE_UNTIL

    now = time.monotonic()
    previous = _MACAU_DEFENCE_LAST_CARRIER
    dt = now - _MACAU_DEFENCE_LAST_TIME
    detected = confirmed
    if (not detected and previous is not None and AUSTRALIA.Tactics.ENEMY_SAMPLE_MIN_S <= dt <= AUSTRALIA.Tactics.MACAU_DEFENCE_TRACK_S
            and math.dist(previous, enemy) <= AUSTRALIA.Tactics.RADICAL_TRACK_CM):
        advance_speed = -ATTACK_DIR * (enemy[0] - previous[0]) / dt
        lateral_shift = abs(enemy[1] - previous[1])
        detected = (advance_speed >= AUSTRALIA.Tactics.MACAU_DEFENCE_MIN_ADVANCE_CM_S
                    and lateral_shift <= AUSTRALIA.Tactics.MACAU_DEFENCE_MAX_LATERAL_SHIFT_CM)
    if confirmed or previous is None or dt >= AUSTRALIA.Tactics.ENEMY_SAMPLE_MIN_S or dt < 0.0:
        _MACAU_DEFENCE_LAST_CARRIER = list(enemy)
        _MACAU_DEFENCE_LAST_TIME = now
    if detected:
        _MACAU_DEFENCE_UNTIL = now + AUSTRALIA.Tactics.MACAU_DEFENCE_HOLD_S
    return detected or now <= _MACAU_DEFENCE_UNTIL

def _hidden_macau_runner(observation):
    if not isinstance(observation, dict) or observation.get("ball") is not None:
        return None
    if observation.get("ball_state") != "lost":
        return None
    candidates = [
        enemy for enemy in _current_enemies()
        if abs(enemy[1]) >= AUSTRALIA.Tactics.MACAU_DEFENCE_LANE_MIN_CM
    ]
    if not candidates:
        return None
    runner = min(candidates, key=lambda item: (
        math.dist(item, [_guard_line_x(), 0.0]), abs(item[1])
    ))
    return runner if _macau_side_run_detected(runner) else None

def _hidden_macau_flank_step(runner):
    pose = _motion_guard(look_at=runner)
    if pose is None:
        return False
    side = math.copysign(1.0, runner[1])
    clearance = 2.0 * AUSTRALIA.Tactics.SIM_BODY_RADIUS_CM + AUSTRALIA.Tactics.MOTION_CLEARANCE_CM + 2.0
    target_y = runner[1] - side * clearance
    target = _safe_target(
        runner[0],
        target_y,
        UnitedPosition.Pos2Angle(pose, runner),
    )
    peripheral.Dribble(True)
    _position_step(
        target,
        speed_gain=2.0,
        arrival_cm=2.0,
        speed_limit=AUSTRALIA.Tactics.PRESS_MAX_MPS,
        near_speed_limit=AUSTRALIA.Tactics.PRESS_MAX_MPS,
    )
    return True

def _hidden_macau_collapse_step(observation):
    global _HIDDEN_MACAU_RUNNER, _HIDDEN_MACAU_UNTIL

    now = time.monotonic()
    ball = _field_xy(observation.get("ball")) if isinstance(observation, dict) else None
    runner = _field_xy(_HIDDEN_MACAU_RUNNER)
    if ball is None or runner is None or now > _HIDDEN_MACAU_UNTIL:
        if now > _HIDDEN_MACAU_UNTIL:
            _HIDDEN_MACAU_RUNNER = None
        return None
    enclosure = dict(observation, carrier=runner, enemy=runner)
    if _defence_unit("DP", enclosure) == "front":
        if _challenge_enemy(runner, None, AUSTRALIA.Tactics.DEADBALL_CONTACT_CM):
            return "hidden_macau_keep_block"
        return None
    chase_ball(ball_field=ball, contest=True)
    return "hidden_macau_capture"

def macau_defence(observation=None, hidden_ball=False):
    if not isinstance(observation, dict) or observation.get("carrier") is None:
        return False
    enemy = _field_xy(observation.get("carrier"))
    if enemy is None or abs(enemy[1]) < AUSTRALIA.Tactics.MACAU_DEFENCE_LANE_MIN_CM:
        return False
    if not _macau_side_run_detected(enemy, confirmed=not hidden_ball):
        return False
    pose = _motion_guard(look_at=enemy)
    if pose is None:
        return False
    if math.dist(pose[:2], enemy) <= AUSTRALIA.Tactics.RADICAL_CONTACT_RUSH_CM:
        return _challenge_enemy(enemy, observation.get("enemy_pose"), AUSTRALIA.Tactics.DEADBALL_CONTACT_CM)
    progress = -ATTACK_DIR * enemy[0]
    target_progress = clamp(progress + AUSTRALIA.Tactics.MACAU_DEFENCE_LEAD_CM, 24.0, Settings.Field.Long / 2.0 - 34.0)
    target_x = -ATTACK_DIR * target_progress
    target_y = math.copysign(
        clamp(abs(enemy[1]), AUSTRALIA.Tactics.MACAU_DEFENCE_LANE_MIN_CM, Settings.Field.Short / 2.0 - 18.0),
        enemy[1],
    )
    target = _safe_target(target_x, target_y, UnitedPosition.Pos2Angle([target_x, target_y], enemy))
    peripheral.Dribble(True)
    _position_step(
        target,
        speed_gain=2.0,
        arrival_cm=AUSTRALIA.Tactics.RADICAL_PREP_ARRIVAL_CM,
        speed_limit=AUSTRALIA.Tactics.PRESS_MAX_MPS,
        near_speed_limit=AUSTRALIA.Tactics.PRESS_MAX_MPS,
    )
    return True

def _home_target():
    check_points = getattr(Settings, "CheckPoints", None)
    home = list(getattr(check_points, "Home", [-70.0, 0.0, 0.0]))
    while len(home) < 3:
        home.append(0.0)
    return [-ATTACK_DIR * abs(float(home[0])), float(home[1]), 0.0 if ATTACK_DIR > 0 else 180.0]

def _strategy_role():
    try:
        ids = AUSTRALIA.Team.RobotIds
        return AUSTRALIA.Team.RobotRoles[ids.index(_robot_id())]
    except (AttributeError, TypeError, ValueError):
        return "INVALID"

def _defence_unit(role, observation=None):
    if role in {"DP_FRONT", "DEFENCE_FRONT", "DEFENDER_FRONT"}:
        return "front"
    if role in {"DP_GOAL", "DEFENCE_GOAL", "DEFENDER_GOAL"}:
        return "goal"
    # 只剩一台时不能继续把 4 号锁在门线上；它需要接管自由球和带球人的第一道
    # 压迫。真正的单车攻防权衡由 _solo_defence_step 统一处理。
    if _solo_takeover_active():
        return "front"
    robots = _team_robots()
    if robots is None:
        return "goal"
    allies = [item for item in robots.values() if item.get("available") is True]
    # 通信刚启动时每台只会先看到自己。此时固定 3 号贴盘球器、4 号守门线，
    # 防止两车同冲；若通信已经明确 3 号被罚下，4 号再接管贴防。
    if len(allies) <= 1:
        primary = robots.get(AUSTRALIA.Team.RobotIds[0])
        if (_robot_id() != AUSTRALIA.Team.RobotIds[0]
                and not (isinstance(primary, dict) and primary.get("available") is False)):
            return "goal"
        return "front"
    threat = None
    if isinstance(observation, dict):
        threat = observation.get("carrier") or observation.get("enemy") or observation.get("ball")
    threat = _field_xy(threat)
    if threat is None:
        threat = [_guard_line_x(), 0.0]
    presser = min(allies, key=lambda item: (
        math.dist([item["x_cm"], item["y_cm"]], threat), item["id"]
    ))
    return "front" if presser["id"] == _robot_id() else "goal"

def _journal_event(event, **details):
    if _MATCH_JOURNAL is None:
        return
    payload = {
        "time": time.time(),
        "event": event,
        "robot": _robot_id(),
        **details,
    }
    try:
        _MATCH_JOURNAL.write(json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n")
        _MATCH_JOURNAL.flush()
    except (OSError, TypeError, ValueError):
        pass

def _journal_control_frame(state):
    """每个控制周期记录感知、朝向和底盘命令，便于回放追球与瞄准错误。"""

    pose = UnitedPosition.FieldRobotPose()
    if not _pose_valid(pose):
        _journal_event("frame", state=state, pose=None, pose_valid=False)
        return
    local_ball = _local_robot_ball_pos()
    ball_state = _effective_ball_state(local_ball)
    ball_local = None
    ball_field = None
    ball_bearing = None
    heading_error = None
    if ball_state == "free":
        try:
            ball_local = [float(local_ball[0]), float(local_ball[1])]
            if all(math.isfinite(value) for value in ball_local):
                theta = math.radians(pose[2])
                ball_field = [
                    pose[0] + ball_local[0] * math.cos(theta) - ball_local[1] * math.sin(theta),
                    pose[1] + ball_local[0] * math.sin(theta) + ball_local[1] * math.cos(theta),
                ]
                ball_bearing = UnitedPosition.Pos2Angle(pose, ball_field) % 360.0
                heading_error = angle_error_deg(pose[2], ball_bearing)
        except (TypeError, ValueError, IndexError):
            ball_local = None
    # chassis.stop() sends a zero command but deliberately retains the last requested
    # vector for its own controller diagnostics.  Never report that stale vector as a
    # live command on strategy paths that have just performed an unconditional stop.
    forced_stop = state in {
        "referee_stop", "no_heading", "no_pose", "invalid_robot_id", "invalid_ball",
        "goal_post_retreat_blocked", "support_invalid_ball", "support_wait", "kick_release",
    }
    if forced_stop:
        command = [0.0, 0.0, 0.0]
        command_source = "strategy_forced_stop"
    else:
        try:
            command = [
                float(getattr(chassis, "_last_vx", 0.0)),
                float(getattr(chassis, "_last_vy", 0.0)),
                float(getattr(chassis, "_last_wz", 0.0)),
            ]
        except (TypeError, ValueError):
            command = None
        command_source = "last_strategy_motion_request"
    measured = _semantic_robot_velocity(pose)
    theta = math.radians(pose[2])
    command_field = None
    if command is not None and all(math.isfinite(value) for value in command):
        command_field = [
            command[0] * math.cos(theta) - command[1] * math.sin(theta),
            command[0] * math.sin(theta) + command[1] * math.cos(theta),
        ]
    enemies = _current_enemies()
    _journal_event(
        "frame",
        state=state,
        pose=[float(pose[0]), float(pose[1]), float(pose[2])],
        dribbler_heading_deg=float(pose[2]) % 360.0,
        dribbler_forward=[math.cos(theta), math.sin(theta)],
        ball_state=ball_state,
        ball_local=ball_local,
        ball_field=ball_field,
        ball_bearing_deg=ball_bearing,
        ball_heading_error_deg=heading_error,
        command_local=command,
        command_source=command_source,
        command_field=command_field,
        measured_velocity=measured,
        active_shot=_ACTIVE_SHOT,
        active_wall_plan=_ACTIVE_WALL_PLAN,
        direct_blocked=_DIRECT_BLOCKED,
        chase_aligned=_CHASE_ALIGNED,
        referee=bool(UnitedPosition.Referee()),
        imu_ready=bool(chassis.IsFieldIMUReady()),
        enemies=enemies,
    )

def _start_match_journal(robot_id):
    global _MATCH_JOURNAL, _MATCH_JOURNAL_PATH

    MATCH_LOG_DIR.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    # A restart can happen within the same wall-clock second.  A nanosecond/PID suffix
    # plus exclusive creation keeps separate matches from silently appending together.
    unique = f"{time.time_ns()}-p{os.getpid()}"
    for attempt in range(100):
        suffix = unique if attempt == 0 else f"{unique}-{attempt}"
        _MATCH_JOURNAL_PATH = MATCH_LOG_DIR / f"duel-{stamp}-{suffix}-R{robot_id}.jsonl"
        try:
            _MATCH_JOURNAL = _MATCH_JOURNAL_PATH.open("x", encoding="utf-8")
            break
        except FileExistsError:
            continue
    else:
        raise RuntimeError("could not allocate a unique duel match journal")
    _journal_event("match_start", file=str(_MATCH_JOURNAL_PATH), attack_direction=ATTACK_DIR)
    logger.info("Robot {} match journal: {}", robot_id, _MATCH_JOURNAL_PATH)

def _stop_match_journal():
    global _MATCH_JOURNAL, _MATCH_JOURNAL_PATH

    if _MATCH_JOURNAL is not None:
        _journal_event("match_end")
        try:
            _MATCH_JOURNAL.close()
        except OSError:
            pass
    _MATCH_JOURNAL = None
    _MATCH_JOURNAL_PATH = None

def _support_cover_step(reference):
    """前压机在队友抢球或持球时前插接应，不能与守门机一起去门线巡逻。"""

    pose = _motion_guard()
    point = _field_xy(reference)
    if pose is None or point is None:
        return False
    target = _carrier_exclusion_target(dict(x_cm=point[0], y_cm=point[1]), pose)
    if target is None:
        return False
    peripheral.Dribble(True)
    _position_step(target, speed_gain=1.5, speed_limit=AUSTRALIA.Tactics.SIM_MAX_SPEED_MPS,
                   near_speed_limit=AUSTRALIA.Tactics.SIM_MAX_SPEED_MPS)
    return True

def _carrier_exclusion_target(holder, pose):
    holder_xy = [float(holder["x_cm"]), float(holder["y_cm"])]
    # 持球者贴近边线时，先选择朝场内的一侧。不能因为队友恰好位于边线外侧，
    # 让它继续把持球者压在墙上。
    inward = -1.0 if holder_xy[1] > 0.0 else 1.0
    relative = -1.0 if pose[1] <= holder_xy[1] else 1.0
    for side in (inward, relative, -relative):
        target = _cooperation_target(
            holder_xy[0] + ATTACK_DIR * AUSTRALIA.Tactics.SUPPORT_COVER_FORWARD_CM,
            holder_xy[1] + side * min(AUSTRALIA.Tactics.SUPPORT_COVER_LATERAL_CM, 34.0),
            0.0 if ATTACK_DIR > 0 else 180.0,
        )
        if target is not None:
            return target
    return None

def _blocks_teammate_carrier(pose, holder):
    try:
        holder_pose = [float(holder["x_cm"]), float(holder["y_cm"]), float(holder["yaw_deg"])]
    except (KeyError, TypeError, ValueError):
        return False
    if not _pose_valid(holder_pose):
        return False
    ball = _held_ball_field_pos(holder_pose)
    if math.dist(pose[:2], ball) <= AUSTRALIA.Tactics.ALLY_CARRIER_CLEARANCE_CM:
        return True
    goal = [ATTACK_DIR * Settings.Field.Long / 2.0, 0.0]
    dx, dy = goal[0] - ball[0], goal[1] - ball[1]
    length_sq = dx * dx + dy * dy
    if length_sq <= 0.01:
        return False
    progress = ((pose[0] - ball[0]) * dx + (pose[1] - ball[1]) * dy) / length_sq
    if not 0.0 <= progress <= 1.0:
        return False
    offset = math.hypot(pose[0] - ball[0] - progress * dx, pose[1] - ball[1] - progress * dy)
    return offset <= AUSTRALIA.Tactics.ALLY_SHOT_LANE_CLEARANCE_CM

def _teammate_controls_visible_ball(holder, ball):
    if ball is None:
        return True
    try:
        holder_pose = [float(holder["x_cm"]), float(holder["y_cm"]), float(holder["yaw_deg"])]
    except (KeyError, TypeError, ValueError):
        return False
    if not _pose_valid(holder_pose):
        return False
    return math.dist(_held_ball_field_pos(holder_pose), ball) <= AUSTRALIA.Tactics.TEAMMATE_HELD_BALL_MATCH_CM

def _yield_to_teammate_carrier(holder):
    pose = _motion_guard()
    if pose is None or not _blocks_teammate_carrier(pose, holder):
        return False
    target = _carrier_exclusion_target(holder, pose)
    if target is None:
        return False
    peripheral.Dribble(False)
    _position_step(target, speed_gain=1.6, arrival_cm=3.0,
                   speed_limit=AUSTRALIA.Tactics.SIM_MAX_SPEED_MPS, near_speed_limit=AUSTRALIA.Tactics.SIM_MAX_SPEED_MPS)
    return True

def _support_step(ball_state):
    global _CHASE_ALIGNED

    pose = UnitedPosition.FieldRobotPose()
    if ball_state == "free":
        _, ball = _sensor_ball_snapshot()
        if ball is None:
            safe_stop(chassis, peripheral)
            return "support_invalid_ball"
        distance = math.dist(pose[:2], ball)
        if distance <= 18.0 or (ATTACK_DIR * ball[0] >= AUSTRALIA.Tactics.SUPPORT_BALL_MIN_X_CM and distance <= AUSTRALIA.Tactics.SUPPORT_CAPTURE_RADIUS_CM):
            chase_ball()
            return "support_capture"

    _CHASE_ALIGNED = False
    yaw = 0.0 if ATTACK_DIR > 0 else 180.0
    target = _safe_target(
        ATTACK_DIR * min(Settings.Field.Long * 0.2, Settings.Field.Long / 2.0 - 55.0),
        AUSTRALIA.Tactics.SUPPORT_SIDE * AUSTRALIA.Tactics.SUPPORT_Y_CM,
        yaw,
    )
    if not _lane_clear(pose, target, _current_enemies(), float(Settings.Vision.ChassisWidth) + 4.0):
        safe_stop(chassis, peripheral)
        return "support_wait"
    peripheral.Dribble(False)
    _position_step(target, speed_gain=0.8)
    return "support_forward"

def _start_shot(name, pose, wall_side=None, wall_plan=None):
    global _ACTIVE_SHOT, _ACTIVE_SHOT_SINCE, _ACTIVE_WALL_PLAN

    plan = None
    if name in {"wall", "emergency_wall"}:
        # 提交弹墙后不因对手短暂站在线路上而取消。目标是把球按已经验证的角度打出去，
        # 进球或制造新的自由球都比原地等待更有价值。
        plan = wall_plan or _wall_shot_plan(
            _held_ball_field_pos(pose), None, wall_side, require_clear=False
        )
        if plan is None or _bank_shot_failure(
                _held_ball_field_pos(pose), *plan, require_clear=False) is not None:
            return False
    _reset_attack()
    _ACTIVE_SHOT = name
    _ACTIVE_SHOT_SINCE = time.monotonic()
    _ACTIVE_WALL_PLAN = plan
    if name in {"wall", "emergency_wall"}:
        _journal_event(
            "wall_commit",
            pose=pose,
            ball=_held_ball_field_pos(pose),
            plan=plan,
            reason="direct_lane_blocked" if name == "wall" else "front_pressure_escape",
        )
    return True

def _front_pressure_enemy(pose):
    theta = math.radians(pose[2])
    threats = []
    for enemy in _current_enemies():
        dx, dy = enemy[0] - pose[0], enemy[1] - pose[1]
        forward = dx * math.cos(theta) + dy * math.sin(theta)
        lateral = -dx * math.sin(theta) + dy * math.cos(theta)
        if 0.0 < forward <= AUSTRALIA.Tactics.FRONT_PRESSURE_DISTANCE_CM and abs(lateral) <= AUSTRALIA.Tactics.SIM_BODY_RADIUS_CM + 6.0:
            threats.append(enemy)
    return _nearest_enemy(threats, pose)[0]

def _confirmed_dribbler_duel_enemy(pose):
    """确认双方盘球器持续正面对顶，排除路过或侧面干扰。"""

    global _BALL_DUEL_SINCE

    ball = _held_ball_field_pos(pose)
    theta = math.radians(pose[2])
    candidates = []
    for enemy in _current_enemies():
        dx, dy = enemy[0] - pose[0], enemy[1] - pose[1]
        forward = dx * math.cos(theta) + dy * math.sin(theta)
        lateral = -dx * math.sin(theta) + dy * math.cos(theta)
        if (0.0 < forward <= 2.0 * AUSTRALIA.Tactics.SIM_BODY_RADIUS_CM + SIM_CATCH_POINT_CM[0]
                and abs(lateral) <= AUSTRALIA.Tactics.SIM_BODY_RADIUS_CM + 6.0
                and math.dist(enemy, ball) <= AUSTRALIA.Tactics.DUEL_BREAKAWAY_CONTACT_CM):
            candidates.append(enemy)
    enemy, _ = _nearest_enemy(candidates, ball)
    if enemy is None:
        _BALL_DUEL_SINCE = None
        return None
    now = time.monotonic()
    if _BALL_DUEL_SINCE is None:
        _BALL_DUEL_SINCE = now
        return None
    return enemy if now - _BALL_DUEL_SINCE >= AUSTRALIA.Tactics.DUEL_BREAKAWAY_CONFIRM_S else None

def _eject_duel_ball():
    """双方盘球器顶住时立即弹球，解除持球僵局并转入自由球争夺。"""

    global _DUEL_EJECT_UNTIL

    if not _fire_shot():
        return False
    _DUEL_EJECT_UNTIL = time.monotonic() + AUSTRALIA.Tactics.DUEL_EJECT_FALLBACK_S
    _reset_attack()
    _reset_cooperation()
    return True

def _start_duel_breakaway_wall(pose, enemy, wall_plan):
    """固定一次后撤目标和一库侧，移动时不来回改方向。"""

    global _WALL_RETREAT_TARGET, _WALL_RETREAT_SINCE

    if wall_plan is None:
        return False
    ball = _held_ball_field_pos(pose)
    route = _bank_route(ball, [_opponent_goal_x(), 0.0], *wall_plan)
    if route is None:
        return False
    dx, dy = pose[0] - enemy[0], pose[1] - enemy[1]
    distance = math.hypot(dx, dy)
    if distance < 1.0:
        theta = math.radians(pose[2])
        dx, dy = -math.cos(theta), -math.sin(theta)
        distance = 1.0
    target = _carry_safe_target(
        pose[0] + AUSTRALIA.Tactics.DUEL_BREAKAWAY_RETREAT_CM * dx / distance,
        pose[1] + AUSTRALIA.Tactics.DUEL_BREAKAWAY_RETREAT_CM * dy / distance,
        route["aim_yaw"],
    )
    if target is None or math.dist(target[:2], pose[:2]) <= AUSTRALIA.Tactics.DUEL_BREAKAWAY_ARRIVAL_CM:
        return False
    if not _start_shot("wall", pose, wall_plan=wall_plan):
        return False
    _WALL_RETREAT_TARGET = target
    _WALL_RETREAT_SINCE = time.monotonic()
    return True

def _execute_duel_breakaway_wall(pose):
    """持球后撤与转向并行；结束后交给普通弹墙执行器立即出球。"""

    global _WALL_RETREAT_TARGET, _WALL_RETREAT_SINCE

    if _WALL_RETREAT_TARGET is None:
        return _execute_wall_attack(pose)
    ball = _held_ball_field_pos(pose)
    route = (_bank_route(ball, [_opponent_goal_x(), 0.0], *_ACTIVE_WALL_PLAN)
             if _ACTIVE_WALL_PLAN is not None else None)
    if route is None:
        _reset_attack()
        return "duel_breakaway_cancel"
    target_distance = math.dist(pose[:2], _WALL_RETREAT_TARGET[:2])
    elapsed = time.monotonic() - _WALL_RETREAT_SINCE
    if target_distance > AUSTRALIA.Tactics.DUEL_BREAKAWAY_ARRIVAL_CM and elapsed < AUSTRALIA.Tactics.DUEL_BREAKAWAY_MAX_S:
        _carry_to(pose, _WALL_RETREAT_TARGET, route["aim_yaw"])
        return "duel_breakaway_wall_retreat"
    _WALL_RETREAT_TARGET = None
    _WALL_RETREAT_SINCE = -math.inf
    return _execute_wall_attack(pose)

def _rushing_pressure_enemy(pose):
    return _detect_high_speed_enemy(reference=pose[:2])

def _shield_pass_step(pose):
    state = _cooperative_attack()
    if state is not None:
        _start_shot("pass", pose)
        return "shield_" + state
    plan = _COOP_PLAN
    if plan is None:
        _reset_attack()
        return _start_carry_attack(pose)
    tolerance = min(3.0, math.degrees(math.atan2(3.0, sum(plan["lengths"]))))
    _turn_held(plan["aim_yaw"], tolerance=tolerance)
    return "shield_pass_bank_aim" if plan.get("cushions", 0) else "shield_pass_direct_aim"

def _macau_lane_open(pose):
    if time.monotonic() < _MACAU_RETRY_AFTER:
        return False
    if abs(pose[1]) < AUSTRALIA.Tactics.MACAU_ENTRY_MIN_ABS_Y_CM:
        return False
    if ATTACK_DIR * pose[0] >= AUSTRALIA.Tactics.MACAU_POCKET_MAX_PROGRESS_CM:
        return False
    side = math.copysign(1.0, pose[1])
    for enemy in _current_enemies():
        if math.dist(enemy, pose[:2]) > AUSTRALIA.Tactics.MACAU_LANE_ENEMY_CLEARANCE_CM:
            continue
        if side * enemy[1] >= AUSTRALIA.Tactics.MACAU_ENTRY_MIN_ABS_Y_CM - 16.0:
            return False
    return True

def _start_carry_attack(pose, preferred=None):
    if preferred is None:
        preferred = "macau" if _macau_lane_open(pose) else "split"
    candidates = ("macau", "split") if preferred == "macau" else ("split",)
    for name in candidates:
        _start_shot(name, pose)
        shot = macau_shoot if name == "macau" else split_shoot
        if shot():
            _reset_attack()
            return name + "_shoot"
        if _ACTIVE_SHOT is not None:
            return name + "_aim"
    _carry_aim_step(pose)
    return "attack_hold_aim"

def _locked_clearance_kick(pose, reason):
    """Finish a committed shot with a straight clearance instead of hesitating."""

    _journal_event(
        "locked_shot_clearance",
        pose=pose,
        plan=_ACTIVE_WALL_PLAN,
        reason=reason,
    )
    # Stop the carrier before the emergency kick so a failed aim does not turn
    # into a sideways dribble.  ``route=None`` deliberately skips goal-mouth
    # validation: once committed, getting the ball out is preferable to losing it.
    chassis.stop()
    if _fire_shot():
        _reset_attack()
        _reset_cooperation()
        return "wall_clearance_kick"
    _reset_attack()
    _reset_cooperation()
    return "wall_clearance_failed"

def _execute_wall_attack(pose):
    global _ACTIVE_WALL_PLAN

    now = time.monotonic()
    ball = _held_ball_field_pos(pose)
    if _ACTIVE_WALL_PLAN is None:
        return _locked_clearance_kick(pose, "missing_locked_plan")
    count, side = _ACTIVE_WALL_PLAN
    # The plan (one cushion and one side) is immutable after commit.  Recompute
    # only the geometric aim from the current local catcher position; do not
    # consult a fresh clear-lane decision or select a replacement side.
    route = _bank_route(ball, [_opponent_goal_x(), 0.0], count, side)
    if route is None:
        return _locked_clearance_kick(pose, "locked_route_geometry_lost")
    if now - _ACTIVE_SHOT_SINCE >= AUSTRALIA.Tactics.WALL_AIM_TIMEOUT_S:
        return _locked_clearance_kick(pose, "locked_aim_timeout")
    if not _turn_held(route["aim_yaw"], _bank_shot_tolerance(ball, count, side)):
        return f"wall_{count}_aim"
    if _fire_shot(route):
        _reset_attack()
        return f"wall_{count}_shoot"
    if _AWAIT_BALL_RELEASE:
        return f"wall_{count}_release"
    return _locked_clearance_kick(pose, "locked_bank_kick_failed")

def _with_ball_step():
    global _DIRECT_BLOCKED, _DIRECT_BLOCKED_SINCE, _WALL_RETRY_AFTER, _BALL_DUEL_SINCE
    global _DUEL_EJECT_UNTIL

    pose = UnitedPosition.FieldRobotPose()
    if not _pose_valid(pose):
        safe_stop(chassis, peripheral)
        return "no_pose"
    now = time.monotonic()
    if _AWAIT_BALL_RELEASE:
        chassis.stop()
        return "kick_release"

    active = _ACTIVE_SHOT
    # 澳门射是已经承诺的边线推进：即使球心接近门线且仍在门柱外侧，
    # 也必须先完成“104cm 到位→原地转身”，不能被通用门柱逃逸状态提前截断。
    if active == "macau":
        if macau_shoot():
            _reset_attack()
            return "macau_shoot"
        if _ACTIVE_SHOT in {"wall", "emergency_wall"}:
            return _execute_wall_attack(pose)
        return "macau_move" if _ACTIVE_SHOT == "macau" else "macau_reset"

    # 弹墙也是已经提交的单向动作，优先级高于通用门柱逃逸和直射检查。
    # 否则同一脚球会被安全分支重新改写，重新产生直射/弹墙横跳。
    if active in {"wall", "emergency_wall"}:
        if _WALL_RETREAT_TARGET is not None:
            return _execute_duel_breakaway_wall(pose)
        return _execute_wall_attack(pose)

    # 球在门柱外侧/紧贴门柱且离门线过近时，任何朝球门的出脚都会先撞框或边墙。
    # 此硬规则必须先于澳门射、直射和弹墙射运行，保证不会“顶着门框射门”。
    if active == "goal_post_retreat":
        return _execute_goal_post_retreat(pose)
    if _goal_post_retreat_required(pose):
        _reset_cooperation()
        if _start_goal_post_retreat(pose):
            return _execute_goal_post_retreat(pose)
        _reset_attack()
        return _start_carry_attack(pose, preferred="split")

    # 直射在其它持球状态之上。这里只处理尚未提交弹墙的状态，避免直射和弹墙
    # 在相邻控制周期之间反复横跳。
    direct_route = _ready_direct_shot_route(pose)
    if direct_route is not None:
        _BALL_DUEL_SINCE = None
        if _ACTIVE_SHOT is not None:
            _reset_attack()
            _reset_cooperation()
        if direct_shoot(route=direct_route):
            _reset_attack()
            _reset_cooperation()
            return "direct_shoot"
        return "direct_aim"

    _DIRECT_BLOCKED = True
    if active == "pass":
        state = _cooperative_attack()
        if state is not None:
            return state
        _reset_attack()
        _reset_cooperation()
        active = None
    if active == "shield_pass":
        if now - _ACTIVE_SHOT_SINCE > AUSTRALIA.Tactics.SHIELD_PASS_TIMEOUT_S:
            _reset_attack()
            return _start_carry_attack(pose)
        return _shield_pass_step(pose)
    if active == "split":
        # 错位射只是在直射和一库都被封后换位，不能抢在一库之前。
        ball = _held_ball_field_pos(pose)
        wall_plan = _ready_wall_shot_plan(ball)
        if wall_plan is not None and now >= _WALL_RETRY_AFTER:
            if _start_shot("wall", pose, wall_plan=wall_plan):
                return _execute_wall_attack(pose)
        if split_shoot():
            _reset_attack()
            return "split_shoot"
        return "split_move" if _ACTIVE_SHOT == "split" else _start_carry_attack(pose)
    if active == "slip":
        _reset_attack()
        active = None

    prepared = _prepared_drop_wall_step(pose)
    if prepared is not None:
        return prepared

    if _DIRECT_BLOCKED_SINCE is None:
        _DIRECT_BLOCKED_SINCE = now
    threat = _front_pressure_enemy(pose)
    ball = _held_ball_field_pos(pose)
    duel_enemy = _confirmed_dribbler_duel_enemy(pose)
    if duel_enemy is not None:
        if now >= _DUEL_EJECT_UNTIL and _eject_duel_ball():
            return "duel_eject_kick"
    wall_side = _escape_wall_side(pose, threat) if threat is not None else None
    # 到这里说明本控制周期没有合法、无遮挡的两柱之间直射路线；普通一库
    # 弹墙只在这个条件成立后提交，提交后由上面的锁定状态机一次性完成。
    wall_plan = _ready_wall_shot_plan(ball, wall_side)
    if duel_enemy is not None and wall_plan is not None and now >= _WALL_RETRY_AFTER:
        if _start_duel_breakaway_wall(pose, duel_enemy, wall_plan):
            _reset_cooperation()
            return _execute_duel_breakaway_wall(pose)
    if wall_plan is not None and now >= _WALL_RETRY_AFTER:
        if _start_shot("wall", pose, wall_plan=wall_plan):
            _reset_cooperation()
            return _execute_wall_attack(pose)
        _WALL_RETRY_AFTER = now + AUSTRALIA.Tactics.COOP_RETRY_S
    # 直射与弹墙都没有可执行线路时，已在空边线通道才允许澳门射。
    if _macau_lane_open(pose):
        return _start_carry_attack(pose, preferred="macau")
    state = _cooperative_attack()
    if state is not None:
        _start_shot("pass", pose)
        return state
    rushing = _rushing_pressure_enemy(pose)
    if rushing is not None and _COOP_PLAN is not None:
        _start_shot("shield_pass", pose)
        return _shield_pass_step(pose)
    return _start_carry_attack(pose)

def _without_ball_offence_step(ball_state):
    if ball_state == "lost":
        peripheral.Dribble(False)
        pose = UnitedPosition.FieldRobotPose()
        target = _safe_target(*_home_target())
        if math.hypot(pose[0] - target[0], pose[1] - target[1]) > 8.0:
            _position_step(target, speed_gain=0.8)
        else:
            _safe_robot_move(0.0, 0.0, 45.0)
        return "search_ball"

    chase_ball()
    return "capture_ball"

def _front_defence_step(observation=None):
    pose = _motion_guard()
    if pose is None:
        return "no_pose"
    ball = _local_robot_ball_pos()
    ball_state = _effective_ball_state(ball)
    if observation is None:
        observation = _defence_observation(ball_state)
    # 对手持球沿边线推进时先切断澳门射通道，其他场景再直接贴盘球器抢断。
    if observation.get("carrier") is not None and observation.get("enemy") is not None:
        if macau_defence(observation):
            return "front_defence_macau_block"
        if radical_defence(observation):
            return "front_defence_challenge"
    if ball_state == "held":
        chassis.stop()
        return "defence_acquired"
    if observation.get("carrier") is None and observation.get("ball") is not None:
        if _free_ball_chaser(observation["ball"]) == _robot_id():
            chase_ball(ball_field=observation["ball"])
            return "front_defence_capture"
        return "front_defence_cover" if _support_cover_step(observation["ball"]) else "defence_idle"
    if observation["enemy"] is not None and radical_defence(observation):
        return "front_defence_challenge"
    if ball_state == "free":
        chase_ball()
        return "front_defence_capture"

    target_yaw = 0.0 if ATTACK_DIR > 0 else 180.0
    peripheral.Dribble(False)
    _position_step(
        [AUSTRALIA.Tactics.FRONT_DEFENCE_X, 0.0, target_yaw],
        speed_gain=1.2,
    )
    return "front_defence_position"

def _solo_defence_step(observation=None):
    """Run the surviving robot's attack-first, goal-aware emergency defence.

    A lone robot cannot leave a dedicated goalkeeper behind.  It therefore
    spends almost all non-shooting time reclaiming the ball, but gives priority
    to a measured incoming shot and to a carrier already aligned to the goal.
    Every input here comes from the local ball/enemy observation; teammate
    health only selects this mode and never supplies tactical simulator truth.
    """

    pose = _motion_guard()
    if pose is None:
        return "solo_no_pose"
    if observation is None:
        observation = _defence_observation(_effective_ball_state())

    ball = _field_xy(observation.get("ball"))
    carrier = _field_xy(observation.get("carrier"))
    enemy = _field_xy(observation.get("enemy"))
    # A moving ball heading through the mouth has to be met before any chase or
    # carrier challenge; a single car has no second line of cover.
    if _incoming_goal_target(observation) is not None:
        return "solo_goal_line_intercept" if intercept_defence(observation) else "solo_defence_idle"

    if carrier is not None:
        # When the carrier is visibly aimed at our goal, occupy the actual shot
        # line.  Else keep the aggressive bump/dislodge behaviour: giving it
        # time merely creates a shot lane the lone defender cannot cover twice.
        if _enemy_shot_lane_target(observation) is not None:
            if _block_carrier_goal_line(observation):
                return "solo_goal_line_block"
        own_goal = [-ATTACK_DIR * Settings.Field.Long / 2.0, 0.0]
        if math.dist(carrier, own_goal) <= AUSTRALIA.Tactics.SOLO_GOAL_THREAT_RADIUS_CM:
            if safe_defence(carrier):
                return "solo_goal_patrol"
        if macau_defence(observation):
            return "solo_macau_block"
        if radical_defence(observation):
            return "solo_carrier_challenge"
        _challenge_enemy(carrier, observation.get("enemy_pose"))
        return "solo_carrier_challenge"

    # A visible free ball is always worth the all-out chase.  There is no
    # teammate to yield to, and recovering it immediately restores attack.
    if ball is not None:
        chase_ball(ball_field=ball)
        return "solo_capture_ball"

    remembered = _remembered_ball_field()
    if remembered is not None:
        chase_ball(ball_field=remembered, contest=enemy is not None)
        return "solo_memory_chase"

    # With neither ball nor carrier visible, park on the legal guard line and
    # scan.  This is the sole non-aggressive state and prevents blind charging
    # into the penalty area after a referee reset or a camera dropout.
    if safe_defence(enemy):
        return "solo_goal_scan"
    return "solo_defence_idle"

def _carrier_ball_reference(observation):
    if not isinstance(observation, dict):
        return None
    ball = _field_xy(observation.get("ball"))
    if ball is not None:
        return ball
    enemy_pose = observation.get("enemy_pose")
    if _pose_valid(enemy_pose):
        return _held_ball_field_pos(enemy_pose)
    return _field_xy(observation.get("carrier"))

def _carrier_goal_line_block_target(observation, chaser_id=None, robots=None):
    ball = _carrier_ball_reference(observation)
    if ball is None:
        return None
    own_goal = [-ATTACK_DIR * Settings.Field.Long / 2.0, 0.0]
    dx, dy = own_goal[0] - ball[0], own_goal[1] - ball[1]
    goal_distance = math.hypot(dx, dy)
    if goal_distance <= AUSTRALIA.Tactics.SIM_BODY_RADIUS_CM:
        return None
    ux, uy = dx / goal_distance, dy / goal_distance
    advance = AUSTRALIA.Tactics.CARRIER_GOAL_LINE_BLOCK_STANDOFF_CM
    chaser = robots.get(chaser_id) if isinstance(robots, dict) and chaser_id is not None else None
    if isinstance(chaser, dict):
        try:
            chaser_x, chaser_y = float(chaser["x_cm"]), float(chaser["y_cm"])
            progress = (chaser_x - ball[0]) * ux + (chaser_y - ball[1]) * uy
            lateral = abs((chaser_x - ball[0]) * uy - (chaser_y - ball[1]) * ux)
        except (KeyError, TypeError, ValueError):
            progress = -math.inf
            lateral = math.inf
        if progress >= 0.0 and lateral <= CARRIER_GOAL_LINE_TEAM_CLEARANCE_CM:
            advance = max(advance, progress + CARRIER_GOAL_LINE_TEAM_CLEARANCE_CM)
    # 到门前时由规则投影在禁区线外，绝不为了封线把机器人推进禁区。
    advance = min(advance, goal_distance - AUSTRALIA.Tactics.SIM_BODY_RADIUS_CM)
    target_x, target_y = ball[0] + ux * advance, ball[1] + uy * advance
    return _safe_target(target_x, target_y, UnitedPosition.Pos2Angle([target_x, target_y], ball))

def _block_carrier_goal_line(observation, chaser_id=None, robots=None):
    if _enemy_shot_lane_target(observation) is None:
        return False
    pose = _motion_guard()
    if pose is None:
        return False
    target = _carrier_goal_line_block_target(observation, chaser_id, robots)
    if target is None:
        return False
    peripheral.Dribble(True)
    _position_step(
        target,
        speed_gain=2.0,
        arrival_cm=AUSTRALIA.Tactics.CARRIER_GOAL_LINE_BLOCK_ARRIVAL_CM,
        speed_limit=AUSTRALIA.Tactics.PRESS_MAX_MPS,
        near_speed_limit=AUSTRALIA.Tactics.PRESS_MAX_MPS,
    )
    return True

def _enemy_shot_lane_target(observation):
    if not isinstance(observation, dict) or observation.get("carrier") is None:
        return None
    enemy_pose = observation.get("enemy_pose")
    if not _pose_valid(enemy_pose):
        return None
    theta = math.radians(enemy_pose[2])
    forward = [math.cos(theta), math.sin(theta)]
    ball = [enemy_pose[0] + SIM_CATCH_POINT_CM[0] * forward[0],
            enemy_pose[1] + SIM_CATCH_POINT_CM[0] * forward[1]]
    own_goal = [-ATTACK_DIR * Settings.Field.Long / 2.0, 0.0]
    to_goal = [own_goal[0] - ball[0], own_goal[1] - ball[1]]
    if forward[0] * to_goal[0] + forward[1] * to_goal[1] <= 0.0 or abs(forward[0]) < 0.05:
        return None
    guard_x = _guard_line_x()
    travel = (guard_x - ball[0]) / forward[0]
    if travel <= 0.0:
        return None
    y = ball[1] + travel * forward[1]
    goal_half = float(Settings.Vision.GoalWidth) / 2.0 + AUSTRALIA.Tactics.SIM_BODY_RADIUS_CM
    if abs(y) > goal_half:
        return None
    return [guard_x, clamp(y, -goal_half, goal_half)]

def _goal_line_defence_step(observation=None):
    if observation is None:
        observation = _defence_observation(_effective_ball_state())
    ball, enemy = observation["ball"], observation["enemy"]
    if _incoming_goal_target(observation) is not None:
        return "goal_line_intercept" if intercept_defence(observation) else "defence_idle"
    # 只要本机是自由球最近者，抢球先于挡射线、摆球接应和门线巡逻。
    if ball is not None and observation.get("carrier") is None and _free_ball_chaser(ball) == _robot_id():
        chase_ball(ball_field=ball)
        return "goal_line_capture"
    # 前压机持续用盘球器争夺对手的球。门线机仅在看见对方已经转到直射窗口时，
    # 才沿“球 -> 我方球门”直线前贴；其他时刻仍在门线横向巡防，不能跟去抢同一个球。
    if observation.get("carrier") is not None:
        robots = _team_robots()
        chaser = _carrier_defence_chaser(observation["carrier"], robots)
        if _block_carrier_goal_line(observation, chaser, robots):
            return "goal_line_direct_block"
    # 没有直射姿态时维持门线职责；摆球接应只能在这之后作为低优先级补充。
    plan = _deadball_plan(observation)
    if plan is not None and plan["supporter"] == _robot_id() and _deadball_support_step(plan):
        return "deadball_stage_ready" if plan["ready"] else "deadball_stage"
    if _predicted_intercept_y(ball, observation["velocity"], _guard_line_x()) is not None:
        return "goal_line_intercept" if intercept_defence(observation) else "defence_idle"
    if safe_defence(ball if ball is not None else enemy):
        return "goal_line_defence"
    return "defence_idle"

def _without_ball_defence_step(role, observation=None):
    if _solo_takeover_active():
        return _solo_defence_step(observation)
    if _defence_unit(role, observation) == "front":
        return _front_defence_step(observation)
    return _goal_line_defence_step(observation)

def _contest_or_defend(observation):
    """处理队友配合、自由球冲抢、短时丢球和防守。"""

    global _CHASE_ALIGNED, _HIDDEN_MACAU_RUNNER, _HIDDEN_MACAU_UNTIL
    global _TEAMMATE_HOLDER_MEMORY, _TEAMMATE_HOLDER_UNTIL

    now = time.monotonic()

    # The support car normally waits for the primary car and runs passing or
    # screening plans.  Once its partner is unavailable it must use the same
    # single-car state machine as the primary, with no stale yield/pass state.
    if _solo_takeover_active():
        _reset_cooperation()
        return _solo_defence_step(observation)

    teammates = _team_robots()
    visible_free_ball = _field_xy(observation.get("ball"))
    teammate_holders = [] if teammates is None else [
        item for ident, item in teammates.items()
        if ident != _robot_id() and item.get("has_ball") is True
    ]
    teammate_holds_visible_ball = (
        len(teammate_holders) == 1
        and _teammate_controls_visible_ball(teammate_holders[0], visible_free_ball)
    )
    if teammate_holds_visible_ball:
        _TEAMMATE_HOLDER_MEMORY = dict(teammate_holders[0])
        _TEAMMATE_HOLDER_UNTIL = now + AUSTRALIA.Tactics.TEAMMATE_HOLD_GRACE_S
    elif visible_free_ball is None and now <= _TEAMMATE_HOLDER_UNTIL:
        # 通信状态和本机视觉并非同一帧到达。短暂遮挡时沿用刚确认的队友球权，
        # 避免无球机在让位、门线防守间反复反向；一旦看到离开队友的自由球立即失效。
        teammate_holders = [dict(_TEAMMATE_HOLDER_MEMORY)] if _TEAMMATE_HOLDER_MEMORY else []
        teammate_holds_visible_ball = len(teammate_holders) == 1
    # 本机相机也能看到队友盘球器前的球，但那不是自由球。只有球已经离开队友
    # 接球窗口时，才把延迟的 has_ball 标志视为过期并重新安排最近者冲抢。
    if len(teammate_holders) == 1 and teammate_holds_visible_ball:
        _CHASE_ALIGNED = False
        holder = teammate_holders[0]
        if _yield_to_teammate_carrier(holder):
            return "teammate_yield"
        context = _cooperation_context()
        if context is not None:
            return _cooperative_support(context)
        _reset_cooperation()
        return "teammate_cover" if _support_cover_step([holder["x_cm"], holder["y_cm"]]) else "defence_idle"

    # 本机已经看见球时，抢球优先于隐藏球推断、澳门切线和贴防角色。即使球旁有对手，
    # 最近者也直接以盘球器冲向球；这样既可能收球，也能立即破坏对方下一脚。
    if visible_free_ball is not None:
        # 已确认的传球飞行仍由接球机执行接球路线；普通射门后的自由球没有该计划，
        # 会立刻走最近者冲抢。
        if teammates is not None and _PASS_FLIGHT is not None:
            state = _cooperative_free_ball(teammates, visible_free_ball, observation.get("velocity"))
            if state is not None:
                return state
        # 明确的队友争球状态优先于瞬时距离排名；没有有效声明时仍按原逻辑选最近者。
        chaser = (_carrier_defence_chaser(visible_free_ball, teammates)
                  if observation.get("carrier") is not None else
                  (_communicated_contest_chaser(visible_free_ball, teammates)
                   or _free_ball_chaser(visible_free_ball, teammates)))
        if chaser in {None, _robot_id()}:
            if _PASS_FLIGHT is None:
                _reset_cooperation()
            chase_ball(
                ball_field=visible_free_ball,
                contest=observation.get("carrier") is not None,
            )
            return "contest_ball" if observation.get("carrier") is not None else "capture_ball"
        # 对方持球、队友正在贴盘球器时，本机保持门线巡防；只有对方已经转入
        # 直射窗口，才沿球门连线贴上去封堵。不能再把两台车都派到争夺点。
        if observation.get("carrier") is not None:
            if _block_carrier_goal_line(observation, chaser, teammates):
                return "contest_direct_block"
            return "contest_goal_patrol" if safe_defence(observation["carrier"]) else "defence_idle"
        assist = _contest_assist_step(visible_free_ball, chaser, teammates, observation)
        if assist is not None:
            return assist
        if _screen_chaser_step(visible_free_ball, chaser, teammates):
            return "chaser_screen"
        # 队友负责最近者抢球时，另一台保持进攻接应，不因一帧内没有可推的对手
        # 又切回门线巡逻，从而制造横向反复和禁区惯性冲入。
        return "free_ball_support" if _support_cover_step(visible_free_ball) else "defence_idle"

    collapse = _hidden_macau_collapse_step(observation)
    if collapse is not None:
        return collapse

    hidden_runner = _hidden_macau_runner(observation)
    if hidden_runner is not None:
        _HIDDEN_MACAU_RUNNER = list(hidden_runner)
        _HIDDEN_MACAU_UNTIL = time.monotonic() + AUSTRALIA.Tactics.HIDDEN_MACAU_COLLAPSE_S
        hidden_observation = dict(observation, carrier=hidden_runner, enemy=hidden_runner)
        if _defence_unit("DP", hidden_observation) == "front" and macau_defence(
                hidden_observation, hidden_ball=True):
            return "contest_hidden_macau_block"
        if _hidden_macau_flank_step(hidden_runner):
            return "contest_hidden_macau_flank"
        return _without_ball_defence_step("DP", hidden_observation)

    # 球被对手底盘遮挡时，仍维持明确分工：最近者持续以盘球器贴住对手争球；
    # 另一台在门线巡防，只有看到对方转到直射窗口才走球-球门连线贴防。
    carrier = _field_xy(observation.get("carrier"))
    if carrier is not None:
        chaser = _carrier_defence_chaser(carrier, teammates)
        if chaser not in {None, _robot_id()}:
            if _block_carrier_goal_line(observation, chaser, teammates):
                return "carrier_direct_block"
            return "carrier_goal_patrol" if safe_defence(carrier) else "defence_idle"
        if macau_defence(observation):
            return "contest_macau_block"
        if radical_defence(observation):
            return "contest_carrier_challenge"
        _challenge_enemy(carrier, observation.get("enemy_pose"))
        return "contest_carrier_challenge"

    remembered = _remembered_ball_field()
    if remembered is not None:
        chase_ball(ball_field=remembered, contest=observation.get("carrier") is not None)
        return "memory_chase"
    _reset_cooperation()
    return _without_ball_defence_step("DP", observation)

def _is_defence_role(role):
    return role in {
        "DP", "DEFENCE", "DEFENDER",
        "DP_FRONT", "DEFENCE_FRONT", "DEFENDER_FRONT",
        "DP_GOAL", "DEFENCE_GOAL", "DEFENDER_GOAL",
    }

def _strategy_step(role):
    global _CHASE_ALIGNED, _LAST_BALL_HELD, _AWAIT_BALL_RELEASE
    global _RADICAL_LAST_ENEMY, _RADICAL_PREVIOUS_ENEMIES, _RADICAL_PREVIOUS_ENEMY_TIME
    global _COOP_RETRY_AFTER, _DUEL_EJECT_UNTIL

    if role == "INVALID":
        _reset_cooperation()
        _reset_attack()
        _reset_defence()
        safe_stop(chassis, peripheral)
        return "invalid_robot_id"
    if not UnitedPosition.Referee():
        _reset_cooperation()
        _reset_attack()
        _reset_defence()
        _CHASE_ALIGNED = False
        _LAST_BALL_HELD = False
        _RADICAL_LAST_ENEMY = None
        _RADICAL_PREVIOUS_ENEMIES = []
        _RADICAL_PREVIOUS_ENEMY_TIME = 0.0
        safe_stop(chassis, peripheral)
        return "referee_stop"
    if not chassis.IsFieldIMUReady():
        _reset_cooperation()
        _reset_attack()
        _reset_defence()
        _CHASE_ALIGNED = False
        safe_stop(chassis, peripheral)
        return "no_heading"

    pose = UnitedPosition.FieldRobotPose()
    if not _pose_valid(pose):
        _reset_cooperation()
        _reset_attack()
        _reset_defence()
        _CHASE_ALIGNED = False
        safe_stop(chassis, peripheral)
        return "no_pose"
    if not _update_attack_direction(pose):
        safe_stop(chassis, peripheral)
        return "invalid_robot_id"

    ball_state, ball_field = _sensor_ball_snapshot()
    if ball_state not in {"held", "free", "lost"}:
        _reset_cooperation()
        _reset_attack()
        _reset_defence()
        safe_stop(chassis, peripheral)
        return "invalid_ball"
    _refresh_wall_shot_cache(pose, ball_state, ball_field)
    _refresh_team_communication(pose, ball_state)
    _sync_teammate_mode()
    _sync_cooperation_availability()
    if ball_state != "held":
        _AWAIT_BALL_RELEASE = False
        _DUEL_EJECT_UNTIL = -math.inf
        if _ACTIVE_SHOT in {"wall", "emergency_wall"}:
            logger.info("Robot {} wall_cancel_lost_ball", _robot_id())
        _reset_attack()
    if _LAST_BALL_HELD != (ball_state == "held"):
        _CHASE_ALIGNED = False
    _LAST_BALL_HELD = ball_state == "held"
    if ball_state == "held":
        _reset_defence()
        if _PASS_FLIGHT is not None:
            if _PASS_FLIGHT["receiver"] == _robot_id():
                _reset_cooperation()
                _COOP_RETRY_AFTER = time.monotonic() + AUSTRALIA.Tactics.COOP_RETRY_S
            elif time.monotonic() - _PASS_FLIGHT["since"] > 0.5:
                _AWAIT_BALL_RELEASE = False
                _reset_cooperation()
                _reset_attack()
    if _AWAIT_BALL_RELEASE and time.monotonic() - _LAST_SHOT_TIME > AUSTRALIA.Tactics.SHOT_RELEASE_TIMEOUT_S:
        _AWAIT_BALL_RELEASE = False
        _reset_cooperation()
        _reset_attack()
    observation = _defence_observation(ball_state)
    if ball_state == "held":
        return _with_ball_step()

    if _is_defence_role(role):
        return _without_ball_defence_step(role, observation)
    return _contest_or_defend(observation)

# END GENERATED AUSTRALIA TACTICAL PORT

# Physical geometry and identity are observation/actuator contracts, not
# tactical choices. Restore them after the generated SIM decision graph has
# initialized its constants and state.
SIM_GOAL_LINE_X_CM = Settings.Field.Long / 2.0
SIM_SIDE_WALL_BALL_CENTER_LIMIT_CM = Settings.Field.Short / 2.0 - AUSTRALIA.Real.REAL_BALL_DIAMETER_CM / 2.0
SIM_END_WALL_BALL_CENTER_LIMIT_CM = Settings.Field.Long / 2.0 - AUSTRALIA.Real.REAL_BALL_DIAMETER_CM / 2.0
SIM_CATCH_POINT_CM = _catch_point()
# Preserve the SIM tactical request value; _safe_robot_move applies the REAL
# physical projection before a command reaches the chassis.

_REAL_TACTICAL_PEER_SAMPLE = None
_REAL_TACTICAL_PEER_STATE = None

def _local_robot_ball_pos():
    """Expose only the physical camera's local ball observation."""

    try:
        ball = UnitedPosition.RobotBallPos()
        if ball is None or len(ball) < 2:
            return list(BALL_NOT_FOUND)
        pair = [ball[0], ball[1]]
        if pair == list(BALL_HELD) or pair == list(BALL_NOT_FOUND):
            return pair
        pair = [float(pair[0]), float(pair[1])]
        return pair if all(math.isfinite(value) for value in pair) else list(BALL_NOT_FOUND)
    except (AttributeError, OSError, RuntimeError, TypeError, ValueError, IndexError):
        return list(BALL_NOT_FOUND)

def _effective_ball_state(ball=None, robots=None):
    del robots
    return _real_ball_state(_local_robot_ball_pos() if ball is None else ball)

def _normalise_real_teammate(frame, stamp):
    """Convert a replay-checked RealTeamLink frame to tactical semantics."""

    global _REAL_TACTICAL_PEER_SAMPLE, _REAL_TACTICAL_PEER_STATE

    if not isinstance(frame, dict) or frame.get("id") != _peer_robot_id():
        return None
    point = _physical_xy(frame.get("signed_xy"))
    yaw = _team_frame_yaw(frame)
    if point is None or yaw is None:
        return None
    try:
        identity = int(frame["id"])
        role = int(frame.get("role", Communication.ROLE_BAN))
        status = int(frame.get("status", Communication.STATUS_NO_BALL))
        flags = int(frame.get("flags", frame.get("reserve", 0)))
        stamp = float(stamp)
    except (KeyError, TypeError, ValueError):
        return None
    key = identity, stamp, point[0], point[1], yaw
    if key == _REAL_TACTICAL_PEER_SAMPLE and isinstance(_REAL_TACTICAL_PEER_STATE, dict):
        return dict(_REAL_TACTICAL_PEER_STATE)

    local_vx = local_vy = spin = math.inf
    previous = _REAL_TACTICAL_PEER_SAMPLE
    if previous is not None and previous[0] == identity:
        elapsed = stamp - previous[1]
        if 0.008 <= elapsed <= AUSTRALIA.Real.REAL_POSE_VELOCITY_MAX_SAMPLE_S:
            field_vx = (point[0] - previous[2]) / 100.0 / elapsed
            field_vy = (point[1] - previous[3]) / 100.0 / elapsed
            theta = math.radians(yaw)
            local_vx = field_vx * math.cos(theta) + field_vy * math.sin(theta)
            local_vy = -field_vx * math.sin(theta) + field_vy * math.cos(theta)
            spin = math.radians(angle_error_deg(previous[4], yaw) / elapsed)
            if not all(math.isfinite(value) for value in (local_vx, local_vy, spin)):
                local_vx = local_vy = spin = math.inf

    available = role != Communication.ROLE_BAN
    peer = {
        "id": identity,
        "x_cm": point[0],
        "y_cm": point[1],
        "yaw_deg": yaw,
        "vx_forward_mps": local_vx,
        "vy_left_mps": local_vy,
        "wz_rad_s": spin,
        "active": available,
        "available": available,
        "has_ball": status == Communication.STATUS_CATCH_BALL,
        "contesting": status == TEAM_STATUS_CONTESTING,
        "dribble_command": bool(flags & TEAM_DRIBBLE_READY_MASK),
        "dribble_available": available,
    }
    _REAL_TACTICAL_PEER_SAMPLE = key
    _REAL_TACTICAL_PEER_STATE = dict(peer)
    return peer

def _team_robots(packet=None):
    """Build the canonical tactical schema from local state and RealTeamLink."""

    del packet
    pose = UnitedPosition.FieldRobotPose()
    if not _pose_valid(pose):
        return None
    ball_state = _effective_ball_state()
    velocity = _real_robot_velocity(pose)
    local_vx = local_vy = spin = math.inf
    if velocity is not None:
        theta = math.radians(float(pose[2]))
        field_vx, field_vy, yaw_rate = map(float, velocity[:3])
        local_vx = (field_vx * math.cos(theta) + field_vy * math.sin(theta)) / 100.0
        local_vy = (-field_vx * math.sin(theta) + field_vy * math.cos(theta)) / 100.0
        spin = math.radians(yaw_rate)
    status = _team_status_code(ball_state)
    local = {
        "id": _robot_id(),
        "x_cm": float(pose[0]),
        "y_cm": float(pose[1]),
        "yaw_deg": float(pose[2]) % 360.0,
        "vx_forward_mps": local_vx,
        "vy_left_mps": local_vy,
        "wz_rad_s": spin,
        "active": True,
        "available": True,
        "has_ball": status == Communication.STATUS_CATCH_BALL,
        "contesting": status == TEAM_STATUS_CONTESTING,
        "dribble_command": bool(getattr(peripheral, "DribbleStatus", False)),
        "dribble_available": True,
    }
    robots = {_robot_id(): local}
    frame, stamp = _TEAM_SAMPLE
    if 0.0 <= time.monotonic() - stamp <= AUSTRALIA.Tactics.TEAM_STATE_MAX_AGE_S:
        peer = _normalise_real_teammate(frame, stamp)
        if peer is not None:
            robots[peer["id"]] = peer
    return robots

def _requested_stop_distance_cm(pose, vx, vy, requested_speed):
    """Estimate stopping distance for the command that is about to be sent."""

    if not _pose_valid(pose):
        return float("inf")
    speed = max(0.0, float(requested_speed))
    if speed <= 1e-6:
        return 0.0
    norm = math.hypot(float(vx), float(vy))
    if norm <= 1e-6:
        return 0.0
    theta = math.radians(float(pose[2]))
    field_vx = (float(vx) * math.cos(theta) - float(vy) * math.sin(theta)) / norm
    field_vy = (float(vx) * math.sin(theta) + float(vy) * math.cos(theta)) / norm
    current = _real_robot_velocity(pose)
    current_forward = 0.0
    if current is not None:
        current_forward = max(
            0.0,
            float(current[0]) * field_vx / 100.0 + float(current[1]) * field_vy / 100.0,
        )
    command_brake = max(float(AUSTRALIA.Tactics.MOTION_COMMAND_BRAKE_MPS2), 1e-3)
    boundary_brake = max(float(AUSTRALIA.Real.REAL_BOUNDARY_BRAKE_MPS2), 1e-3)
    command_latency = max(float(AUSTRALIA.Real.REAL_DIRECT_COMMAND_HORIZON_S), float(AUSTRALIA.Tactics.MOTION_LATENCY_S))
    command_latency += max(0.0, float(AUSTRALIA.Tactics.MOTION_TELEMETRY_DELAY_S))
    command_stop = speed * command_latency + speed * speed / (2.0 * command_brake)
    measured_stop = (
        current_forward * float(AUSTRALIA.Real.REAL_DIRECT_COMMAND_HORIZON_S)
        + current_forward * current_forward / (2.0 * boundary_brake)
    )
    return 100.0 * max(command_stop, measured_stop)

def _requested_motion_speed_limit(pose, vx, vy, requested_speed):
    """Return the highest command speed with requested-speed braking margin."""

    requested = min(max(0.0, float(requested_speed)), AUSTRALIA.Real.REAL_ABSOLUTE_MAX_MPS)
    if not _pose_valid(pose):
        return requested, False
    if requested <= 1e-6:
        return requested, False
    norm = math.hypot(float(vx), float(vy))
    if norm <= 1e-6:
        return requested, False
    theta = math.radians(float(pose[2]))
    field_vx = float(vx) * math.cos(theta) - float(vy) * math.sin(theta)
    field_vy = float(vx) * math.sin(theta) + float(vy) * math.cos(theta)
    direction_norm = math.hypot(field_vx, field_vy)
    field_vx /= direction_norm
    field_vy /= direction_norm
    clearances = []
    try:
        boundary = _rule_approach_clearance_cm(pose, vx, vy)
    except (AttributeError, OSError, RuntimeError, TypeError, ValueError):
        boundary = None
    if boundary is not None and math.isfinite(float(boundary)):
        clearances.append(float(boundary))
    try:
        robot_clearance = float(Settings.Vision.ChassisWidth) * 0.5 + 4.0
    except (AttributeError, TypeError, ValueError):
        robot_clearance = 15.0
    for enemy in _current_enemies():
        try:
            rel_x = float(enemy[0]) - float(pose[0])
            rel_y = float(enemy[1]) - float(pose[1])
        except (TypeError, ValueError, IndexError):
            continue
        along = rel_x * field_vx + rel_y * field_vy
        lateral = abs(rel_x * field_vy - rel_y * field_vx)
        if along > 0.0 and lateral <= robot_clearance:
            clearances.append(along - robot_clearance)
    if not clearances:
        return requested, False
    available = min(clearances) - 2.0
    if _requested_stop_distance_cm(pose, vx, vy, requested) <= available:
        return requested, False
    low, high = 0.0, requested
    for _ in range(16):
        candidate = (low + high) * 0.5
        if _requested_stop_distance_cm(pose, vx, vy, candidate) <= available:
            low = candidate
        else:
            high = candidate
    return low, low + 1e-6 < requested

def _open_path_allows_burst(pose, vx, vy, turn):
    """Allow a short >1.5 m/s burst only on a genuinely open path."""

    if not _pose_valid(pose) or math.hypot(vx, vy) <= AUSTRALIA.Real.REAL_SAFE_FAST_MPS:
        return False
    margin_x = SIM_WALL_CONTACT_X_CM - abs(float(pose[0]))
    margin_y = SIM_WALL_CONTACT_Y_CM - abs(float(pose[1]))
    if min(margin_x, margin_y) < 60.0:
        return False
    try:
        if any(math.dist(pose[:2], enemy) < 70.0 for enemy in _current_enemies()):
            return False
    except (TypeError, ValueError):
        return False
    requested = min(math.hypot(vx, vy), AUSTRALIA.Real.REAL_ABSOLUTE_MAX_MPS)
    allowed, clipped = _requested_motion_speed_limit(pose, vx, vy, requested)
    if clipped or allowed + 1e-6 < requested:
        return False
    return _direct_motion_path_safe(
        pose, vx, vy, turn, max(AUSTRALIA.Real.REAL_DIRECT_COMMAND_HORIZON_S, 0.10)
    )

def _safe_robot_move(vx, vy, turn, recovery=False, speed_limit=AUSTRALIA.Tactics.SIM_MAX_SPEED_MPS,
                     turn_priority=False):
    """Execute a clipped, physically bounded body twist.

    The SIM command is attempted first. If its short-horizon path is unsafe,
    progressively smaller commands are tried before a stop. A no-ball
    ``turn_priority`` command preserves useful heading correction when
    translation is blocked; possession commands remain longitudinal only.
    """

    pose = UnitedPosition.FieldRobotPose()
    try:
        vx, vy, turn, speed_limit = map(float, (vx, vy, turn, speed_limit))
    except (TypeError, ValueError):
        _motion_stop("VISION_STOP")
        return False
    speed = math.hypot(vx, vy)
    requested_speed = speed
    burst_allowed = (
        speed_limit > AUSTRALIA.Real.REAL_SAFE_FAST_MPS
        and not recovery
        and _open_path_allows_burst(pose, vx, vy, turn)
    )
    physical_limit = min(
        AUSTRALIA.Real.REAL_ABSOLUTE_MAX_MPS if burst_allowed else AUSTRALIA.Real.REAL_SAFE_FAST_MPS,
        max(0.0, speed_limit),
    )
    braking_limit, braking_clipped = _requested_motion_speed_limit(
        pose, vx, vy, physical_limit
    ) if not recovery else (physical_limit, False)
    physical_limit = min(physical_limit, braking_limit)
    if speed > 1e-6 and physical_limit <= 1e-6:
        _motion_stop("BOUNDARY_STOP")
        return False
    if physical_limit <= 0.0:
        vx = vy = 0.0
    elif speed > physical_limit:
        scale = physical_limit / speed
        vx *= scale
        vy *= scale
    holding = _effective_ball_state() == "held" and _REAL_HOLD_CONFIRMED
    if holding:
        return _bounded_robot_move(
            pose, vx, vy, turn, holding=True, recovery=recovery,
            motion_reason="POSSESSION_REALIGN" if abs(turn) > 1e-6 else "COMMAND",
            requested_speed=requested_speed,
        )

    candidates = [(vx, vy)]
    if math.hypot(vx, vy) > 1e-6:
        candidates.extend((vx * fraction, vy * fraction) for fraction in (0.80, 0.60, 0.40, 0.20))
    for candidate_vx, candidate_vy in candidates:
        if not _direct_motion_path_safe(
                pose, candidate_vx, candidate_vy, turn,
                AUSTRALIA.Real.REAL_DIRECT_COMMAND_HORIZON_S, recovery=recovery):
            continue
        clipped = math.hypot(candidate_vx, candidate_vy) + 1e-6 < requested_speed
        reason = "BOUNDARY_RECOVERY" if recovery else (
            "BRAKING_CLIP" if braking_clipped and clipped else
            "SPEED_CLIP" if clipped else "COMMAND"
        )
        return _bounded_robot_move(
            pose, candidate_vx, candidate_vy, turn, recovery=recovery,
            motion_reason=reason, requested_speed=requested_speed,
        )

    # No-ball heading correction remains useful when the translation is
    # clipped, but only callers that explicitly prioritize turning may use
    # this fallback. Possession never reaches this branch.
    if (not holding and turn_priority and abs(turn) > 1e-6
            and _direct_motion_path_safe(
            pose, 0.0, 0.0, turn, AUSTRALIA.Real.REAL_DIRECT_COMMAND_HORIZON_S,
            recovery=recovery)):
        return _bounded_robot_move(
            pose, 0.0, 0.0, turn, recovery=recovery,
            motion_reason="TURN_PRIORITY", requested_speed=requested_speed,
        )
    _motion_stop("BOUNDARY_STOP")
    return False

def _rule_safe_robot_move(
        vx, vy, turn, speed_limit=AUSTRALIA.Tactics.SIM_MAX_SPEED_MPS, *, recovery=False,
        turn_priority=False):
    return _safe_robot_move(
        vx, vy, turn, recovery=recovery, speed_limit=speed_limit,
        turn_priority=turn_priority,
    )

def _fire_pass(plan):
    """Preserve the canonical pass transition while using the real kicker."""

    global _LAST_SHOT_TIME, _AWAIT_BALL_RELEASE, _PASS_FLIGHT, _COOP_RETRY_AFTER

    if _AWAIT_BALL_RELEASE:
        return False
    if _effective_ball_state() != "held" or not UnitedPosition.Referee():
        safe_stop(chassis, peripheral)
        return False
    if not _pass_participants_ready(plan, _cooperation_context()):
        _reset_cooperation()
        chassis.stop()
        return False
    try:
        level = int(plan["kick_level"])
    except (KeyError, TypeError, ValueError):
        return False
    if not _fire_shot(level):
        return False
    _PASS_FLIGHT = dict(plan, since=_LAST_SHOT_TIME)
    _COOP_RETRY_AFTER = _LAST_SHOT_TIME + AUSTRALIA.Tactics.COOP_RETRY_S
    return True

def offence_step():
    return _strategy_step("OP")

def defence_step():
    role = _strategy_role()
    return _strategy_step(role if _is_defence_role(role) else "DP")

def _executor_intent_for_state(state):
    """Tag the already-selected SIM action for executor plan invalidation."""

    name = str(state or "").lower()
    for token, intent in (
        ("pass", "PASS"),
        ("direct", "SHOOT_DIRECT"),
        ("wall", "SHOOT_WALL"),
        ("macau", "MACAU"),
        ("split", "SPLIT"),
        ("carry", "CARRY"),
        ("slip", "CARRY"),
    ):
        if token in name:
            return intent
    return None

def _cancel_possession_executor_for_tactical_change():
    global _REAL_REACQUIRE, _REAL_CARRY_RECOVERY

    _REAL_REACQUIRE = None
    _REAL_CARRY_RECOVERY = None
    _set_possession_executor_state("TACTICAL_REPLAN")
    _motion_stop("TACTICAL_STOP")

def logic_step():
    global _TEAM_ACTION_STATE, _LAST_RUNTIME_FAULT_LOG_TIME
    global _TACTICAL_STATE, _TACTICAL_INTENT

    if not _real_mainboard_ready():
        _reset_attack()
        _reset_defence()
        _reset_real_ball_control()
        safe_stop(chassis, peripheral)
        _force_real_lower_board_stop()
        _TEAM_ACTION_STATE = "mainboard_unavailable"
        return _TEAM_ACTION_STATE

    try:
        previous_intent = _TACTICAL_INTENT
        previous_reacquire = _REAL_REACQUIRE
        state = _strategy_step(_strategy_role())
        decision_intent = _executor_intent_for_state(state)
        _TACTICAL_STATE = state
        if (previous_reacquire is not None and _REAL_REACQUIRE is previous_reacquire
                and previous_intent is not None
                and decision_intent is not None and decision_intent != previous_intent):
            _cancel_possession_executor_for_tactical_change()
        elif _REAL_REACQUIRE is not None and decision_intent is not None:
            # A new plan created by this tick owns the new tactical decision;
            # attach that owner after the decision is known so the cleanup
            # check cannot delete it as if it were the previous plan.
            _REAL_REACQUIRE["owner_intent"] = decision_intent
        if decision_intent is not None:
            _TACTICAL_INTENT = decision_intent

        pose = UnitedPosition.FieldRobotPose()
        if _pose_valid(pose):
            ball_state = _effective_ball_state()
            _held_ball_stall_watch(state, ball_state, pose)
            executor_state = _real_ball_control(pose, ball_state)
            if executor_state is not None:
                state = executor_state
    except Exception as error:
        # A physical camera, serial link, or actuator can fail for one frame.
        # Do not terminate the strategy process (which would leave the car in
        # its last commanded state); clear transient tactics and retry after a
        # safe stop on the next control tick.  KeyboardInterrupt/SystemExit
        # are not caught here, so normal shutdown still reaches ``finally``.
        _reset_attack()
        _reset_defence()
        _reset_real_ball_control()
        _reset_attack_direction()
        safe_stop(chassis, peripheral)
        _force_real_lower_board_stop()
        _TEAM_ACTION_STATE = "runtime_fault"
        now = time.monotonic()
        if now - _LAST_RUNTIME_FAULT_LOG_TIME >= AUSTRALIA.Real.CONTROL_OVERRUN_LOG_INTERVAL_S:
            logger.warning("Real strategy tick failed; stopped for retry: {}", error)
            _LAST_RUNTIME_FAULT_LOG_TIME = now
        return _TEAM_ACTION_STATE
    # The following heartbeat carries only this locally observed action state;
    # it never changes the local ball, pose, referee, or obstacle readings.
    _TEAM_ACTION_STATE = state
    return state

def _stop_signal(_signum, _frame):
    _real_emergency_stop()
    raise KeyboardInterrupt

def _real_emergency_stop():
    """Operator-triggered stop path used by SIGTERM/SIGINT handling."""

    try:
        peripheral.Dribble(False)
    except Exception:
        pass
    try:
        emergency = getattr(chassis, "emergency_stop", None)
        if callable(emergency):
            emergency()
        else:
            chassis.stop()
    except Exception:
        try:
            chassis.stop()
        except Exception:
            pass
    _force_real_lower_board_stop()

def _real_command_worker_ready():
    """Return whether the asynchronous physical MainBoard worker is alive."""

    try:
        from LunaPre import runtime
        bits = runtime.Bits
        if bits is None:
            return False
        alive = getattr(bits, "IsThreadAlive", None)
        return True if not callable(alive) else bool(alive())
    except Exception:
        return False

def _real_mainboard_transport_telemetry():
    """Read non-blocking MainBoard freshness data when the adapter exposes it."""

    try:
        from LunaPre import runtime

        bits = runtime.Bits
        getter = getattr(bits, "GetTransportTelemetry", None)
        if callable(getter):
            sample = getter()
            return dict(sample) if isinstance(sample, dict) else None
        # Keep compatibility with an older physical wrapper while making the
        # missing fields explicit in diagnostics. The deployed IceLoongBits
        # adapter provides GetTransportTelemetry above.
        if bits is None:
            return None
        age_names = {
            "last_successful_tx_mono": "tx_age_s",
            "last_successful_rx_mono": "rx_age_s",
            "last_pose_rx_mono": "pose_age_s",
            "last_referee_rx_mono": "referee_age_s",
            "last_link_rx_mono": "link_age_s",
        }
        if not any(hasattr(bits, name) for name in age_names):
            return None
        now = time.monotonic()
        sample = {"schema": 0, "now_mono": now, "thread_alive": _real_command_worker_ready()}
        for name, age_key in age_names.items():
            value = getattr(bits, name, None)
            sample[name] = value
            if value is None:
                sample[age_key] = None
            else:
                try:
                    sample[age_key] = max(0.0, now - float(value))
                except (TypeError, ValueError):
                    sample[age_key] = None
        return sample
    except Exception:
        return None

def _real_mainboard_ready():
    """Require recent MainBoard I/O after physical preflight has succeeded."""

    global _REAL_MAINBOARD_STATE, _REAL_MAINBOARD_STATE_REASON

    ready = _real_command_worker_ready()
    reason = None if ready else "sender_thread_dead"
    telemetry = _real_mainboard_transport_telemetry()
    if isinstance(telemetry, dict):
        for key, samples in _MAINBOARD_FRESHNESS_SAMPLES.items():
            try:
                value = float(telemetry.get(key))
            except (TypeError, ValueError):
                continue
            if math.isfinite(value) and value >= 0.0:
                samples.append(value)
                del samples[:-600]
    if ready and _REAL_DEPLOYMENT_ACTIVE:
        if telemetry is None:
            ready = False
            reason = "transport_telemetry_unavailable"
        elif bool(telemetry.get("closed")):
            ready = False
            reason = "adapter_closed"
        else:
            tx_age = telemetry.get("tx_age_s")
            rx_age = telemetry.get("rx_age_s")
            if tx_age is None or tx_age > AUSTRALIA.Real.REAL_MAINBOARD_TX_TIMEOUT_S:
                ready = False
                reason = "tx_stale" if tx_age is not None else "tx_never_seen"
            elif rx_age is None or rx_age > AUSTRALIA.Real.REAL_MAINBOARD_RX_TIMEOUT_S:
                ready = False
                reason = "rx_stale" if rx_age is not None else "rx_never_seen"
            elif (telemetry.get("pose_age_s") is None
                  or telemetry["pose_age_s"] > AUSTRALIA.Real.REAL_POSE_RX_TIMEOUT_S):
                ready = False
                reason = "pose_stale" if telemetry.get("pose_age_s") is not None else "pose_never_seen"
            elif (telemetry.get("referee_age_s") is None
                  or telemetry["referee_age_s"] > AUSTRALIA.Real.REAL_REFEREE_RX_TIMEOUT_S):
                ready = False
                reason = "referee_stale" if telemetry.get("referee_age_s") is not None else "referee_never_seen"
    if (ready, reason) != (_REAL_MAINBOARD_STATE, _REAL_MAINBOARD_STATE_REASON):
        _REAL_MAINBOARD_STATE = ready
        _REAL_MAINBOARD_STATE_REASON = reason
        logger.info(
            "MAINBOARD {}{}".format(
                "RESTORED" if ready else "LOST",
                "" if reason is None else " reason={}".format(reason),
            )
        )
    return ready

def _mono_age(timestamp, now=None):
    """Return a finite monotonic age or ``None`` for an unavailable sample."""

    if timestamp is None:
        return None
    now = time.monotonic() if now is None else float(now)
    try:
        age = now - float(timestamp)
    except (TypeError, ValueError):
        return None
    return max(0.0, age) if math.isfinite(age) else None

def _distribution(values):
    ordered = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not ordered:
        return {"count": 0, "mean_s": 0.0, "p95_s": 0.0, "p99_s": 0.0, "max_s": 0.0}

    def percentile(fraction):
        index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
        return ordered[index]

    return {
        "count": len(ordered),
        "mean_s": sum(ordered) / len(ordered),
        "p95_s": percentile(0.95),
        "p99_s": percentile(0.99),
        "max_s": ordered[-1],
    }

def _real_runtime_telemetry_snapshot(state=None, now=None):
    """Build one JSON-serialisable, non-blocking HIL telemetry snapshot."""

    now = time.monotonic() if now is None else float(now)
    transport = _real_mainboard_transport_telemetry()
    packet = getattr(UnitedPosition, "vision_data", None)
    producer_timestamp = packet.get("timestamp") if isinstance(packet, dict) else None
    try:
        vision_rx_age = UnitedPosition.VisionRxAge()
    except (AttributeError, OSError, RuntimeError, TypeError, ValueError):
        vision_rx_age = None
    try:
        producer_age = now - float(producer_timestamp)
        producer_age = max(0.0, producer_age) if math.isfinite(producer_age) else None
    except (TypeError, ValueError):
        producer_age = None
    peer, peer_timestamp = _TEAM_SAMPLE
    try:
        referee = bool(UnitedPosition.Referee())
    except Exception:
        referee = None
    try:
        pose = UnitedPosition.FieldRobotPose()
        pose_validity = _pose_valid(pose)
    except Exception:
        pose_validity = False
    ordered_executed = sorted(_MOTION_EXECUTED_SAMPLES)
    p95_index = min(
        len(ordered_executed) - 1,
        max(0, math.ceil(0.95 * len(ordered_executed)) - 1),
    ) if ordered_executed else None
    mean_requested = (
        sum(_MOTION_REQUESTED_SAMPLES) / len(_MOTION_REQUESTED_SAMPLES)
        if _MOTION_REQUESTED_SAMPLES else 0.0
    )
    mean_executed = (
        sum(_MOTION_EXECUTED_SAMPLES) / len(_MOTION_EXECUTED_SAMPLES)
        if _MOTION_EXECUTED_SAMPLES else 0.0
    )
    motion_metrics = dict(_MOTION_METRICS)
    if _MOTION_LAST_COMMAND_MONO is not None:
        open_interval = max(0.0, now - _MOTION_LAST_COMMAND_MONO)
        motion_metrics[
            "moving_time_s" if _MOTION_LAST_EXECUTED_SPEED_MPS > 1e-6
            else "stopped_time_s"
        ] += open_interval
    snapshot = {
        "schema": 1,
        "t_mono": now,
        "state": state,
        "mainboard": transport,
        "mainboard_freshness": {
            key: _distribution(values)
            for key, values in _MAINBOARD_FRESHNESS_SAMPLES.items()
        },
        "worker_alive": _real_command_worker_ready(),
        "pose_valid": pose_validity,
        "referee_play": referee,
        "referee_age_s": None if transport is None else transport.get("referee_age_s"),
        "vision_ready": _REAL_VISION_STATE,
        "vision_rx_age_s": vision_rx_age,
        "vision_producer_age_s": producer_age,
        "vision_packet_interval": _distribution(UnitedPosition.VisionRxIntervals()),
        "team_mode": _TEAMMATE_MODE,
        "team_sample_age_s": _mono_age(peer_timestamp, now),
        "team_sample_present": isinstance(peer, dict),
        "team_generation": _TEAM_LINK_GENERATION,
        "action_state": _TEAM_ACTION_STATE,
        "tactical_state": _TACTICAL_STATE,
        "tactical_intent": _TACTICAL_INTENT,
        "tactical_target": _TACTICAL_TARGET,
        "executor_state": _POSSESSION_EXECUTOR_STATE,
        "kick_pending_release": bool(_AWAIT_BALL_RELEASE),
        "last_kick_age_s": _mono_age(_LAST_SHOT_TIME, now),
        "requested_speed_mps": _MOTION_LAST_REQUESTED_SPEED_MPS,
        "executed_speed_mps": _MOTION_LAST_EXECUTED_SPEED_MPS,
        "speed_limit_reason": _MOTION_LAST_REASON,
        "motion_efficiency": {
            **motion_metrics,
            "mean_requested_speed_mps": mean_requested,
            "mean_executed_speed_mps": mean_executed,
            "p95_executed_speed_mps": (
                ordered_executed[p95_index] if p95_index is not None else 0.0
            ),
            "sample_count": len(ordered_executed),
        },
    }
    return snapshot

def _log_real_runtime_telemetry(state=None, now=None):
    """Emit one bounded-rate JSON line for HIL/fault-injection analysis."""

    global _LAST_REAL_RUNTIME_TELEMETRY_LOG_TIME
    now = time.monotonic() if now is None else float(now)
    if now - _LAST_REAL_RUNTIME_TELEMETRY_LOG_TIME < AUSTRALIA.Real.REAL_RUNTIME_TELEMETRY_LOG_INTERVAL_S:
        return False
    snapshot = _real_runtime_telemetry_snapshot(state=state, now=now)
    logger.info(
        "REAL_TELEMETRY {}",
        json.dumps(snapshot, sort_keys=True, separators=(",", ":")),
    )
    _LAST_REAL_RUNTIME_TELEMETRY_LOG_TIME = now
    return True

def _force_real_lower_board_stop():
    """Leave a zero wheel command behind when the IL worker has died.

    ``IceLoongBits.SetWheelSpeed`` normally only updates the command snapshot;
    the 60 Hz worker is what transmits that snapshot.  If the worker exits,
    calling ``chassis.stop()`` alone therefore cannot replace a previously
    latched non-zero command on the MainBoard.  Write the zero snapshot first,
    then ask the adapter to restart its worker when it supports that lifecycle
    API.  This helper never starts motion and remains tolerant of a CPU-only
    test shim or a closing runtime.
    """

    try:
        from LunaPre import runtime

        bits = runtime.Bits
        setter = getattr(bits, "SetWheelSpeed", None)
        if callable(setter):
            setter(0.0, 0.0, 0.0, 0.0)
        restart = getattr(bits, "RestartThreadIfNeeded", None)
        alive = getattr(bits, "IsThreadAlive", None)
        if callable(restart) and callable(alive):
            try:
                if not bool(alive()):
                    restart()
            except Exception:
                # The zero snapshot is still useful if the adapter is closing
                # or its serial port cannot be recovered.
                pass
    except Exception:
        pass

def _real_vision_ready():
    """Accept only a valid vision dictionary backed by a recent successful UDP receive."""

    global _REAL_VISION_STATE
    packet = getattr(UnitedPosition, "vision_data", None)
    age_getter = getattr(UnitedPosition, "VisionRxAge", None)
    try:
        age = age_getter() if callable(age_getter) else None
        ready = (
            isinstance(packet, dict)
            and isinstance(packet.get("chassis_pos"), list)
            and age is not None
            and math.isfinite(float(age))
            and 0.0 <= float(age) <= AUSTRALIA.Real.REAL_VISION_TIMEOUT_S
        )
    except (TypeError, ValueError):
        ready = False
    if ready != _REAL_VISION_STATE:
        _REAL_VISION_STATE = ready
        logger.info("VISION {}".format("RESTORED" if ready else "LOST"))
    return ready

def _reset_real_vision_freshness():
    """Reset only the edge-triggered log state; Positions owns UDP RX age."""

    global _REAL_VISION_STATE
    _REAL_VISION_STATE = None

def _wait_for_real_vision(timeout_s=3.0):
    """Wait for one actual UDP packet; never emits motion or peripheral IO."""

    deadline = time.monotonic() + float(timeout_s)
    while time.monotonic() < deadline:
        if _real_vision_ready():
            return True
        time.sleep(0.01)
    return False

def _wait_for_mainboard_pose(bits, timeout_s=2.0):
    """Wait briefly for one valid GET_POSE response from the command worker."""

    deadline = time.monotonic() + float(timeout_s)
    while time.monotonic() < deadline:
        try:
            pose = bits.GetPose()
        except (AttributeError, OSError, RuntimeError, TypeError, ValueError):
            pose = None
        if _pose_valid(pose):
            return pose
        time.sleep(0.01)
    return None

def _probe_mainboard_readonly_health(bits):
    """Exercise read-only MainBoard inputs during preflight.

    The command-surface check proves that methods exist; this probe also
    catches a dead/raising adapter before strategy motion starts.  Battery and
    dribbler-current values may legitimately be ``None`` until the slow
    background poll completes, so an absent sample is logged but is not
    confused with an actuator failure.
    """

    referee_getter = getattr(bits, "GetRefereeUnlockPin", None)
    if not callable(referee_getter):
        # The normal path has already rejected this through
        # _require_real_runtime_contract().  Keeping this helper tolerant
        # makes isolated preflight tests and diagnostic shims composable.
        return
    try:
        referee = referee_getter()
    except (AttributeError, OSError, RuntimeError, TypeError, ValueError) as exc:
        raise RuntimeError("MainBoard referee input read failed") from exc
    try:
        bool(referee)
    except Exception as exc:
        raise RuntimeError("MainBoard referee input was not boolean-like") from exc

    for name, label in (
            ("GetBatteryVoltage", "battery voltage"),
            ("GetDribbleCurrent", "dribbler current"),
    ):
        getter = getattr(bits, name, None)
        if not callable(getter):
            continue
        try:
            value = getter()
        except (AttributeError, OSError, RuntimeError, TypeError, ValueError) as exc:
            raise RuntimeError(f"MainBoard {label} read failed") from exc
        if value is None:
            logger.warning("MainBoard {} sample is not ready during preflight", label)
            continue
        try:
            if not math.isfinite(float(value)):
                raise RuntimeError(f"MainBoard {label} is non-finite")
        except (TypeError, ValueError) as exc:
            raise RuntimeError(f"MainBoard {label} is not numeric") from exc

def _require_real_referee_interlock():
    """Reject bench-motion overrides before a physical match strategy starts."""

    enabled = [
        name for name in REAL_FORBIDDEN_MOTION_OVERRIDE_ENV
        if os.environ.get(name, "").strip().lower() in {"1", "true", "yes", "on"}
    ]
    if enabled:
        raise RuntimeError(
            "strategy_real refuses referee-bypass motion overrides: "
            + ", ".join(enabled)
        )

def _require_real_runtime_contract(bits, *, team_link=True):
    """Check the physical adapter surface before any strategy tick."""

    required_bits = [
        "SetRoomSize", "GetPose", "SetWheelSpeed", "Kick",
        "SetKickPowerLevel", "GetRefereeUnlockPin", "GetBatteryVoltage",
        "GetDribbleCurrent",
    ]
    missing = [name for name in required_bits if not callable(getattr(bits, name, None))]
    if missing:
        raise RuntimeError("physical MainBoard adapter missing: " + ", ".join(missing))
    if not callable(getattr(chassis, "RobotMoveVector", None)):
        raise RuntimeError("physical chassis wheel-vector API is unavailable")
    if not callable(getattr(chassis, "stop", None)) or not callable(getattr(chassis, "CanMove", None)):
        raise RuntimeError("physical chassis safety API is incomplete")
    if not callable(getattr(peripheral, "Dribble", None)):
        raise RuntimeError("physical dribbler API is unavailable")
    if not callable(getattr(UnitedPosition, "FieldRobotPose", None)):
        raise RuntimeError("physical pose facade is unavailable")
    if not callable(getattr(UnitedPosition, "Referee", None)):
        raise RuntimeError("physical referee input is unavailable")

def _referee_summary():
    try:
        return "PLAY" if bool(UnitedPosition.Referee()) else "LOCKED"
    except Exception:
        return "UNKNOWN"

def _team_link_summary():
    if TEAM_LINK_DISABLED or _TEAM_LINK is None:
        return "disabled"
    if _TEAMMATE_MODE == "paired":
        return "paired"
    return "pairing"

def _reproducibility_identity():
    """Return immutable startup identity metadata without changing behavior."""

    try:
        strategy_sha256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    except (OSError, TypeError, ValueError):
        strategy_sha256 = "unknown"
    try:
        result = subprocess.run(
            ["git", "-C", str(SRC_DIR.parent), "rev-parse", "HEAD"],
            check=False,
            capture_output=True,
            text=True,
            timeout=1.0,
        )
        software_commit = result.stdout.strip() if result.returncode == 0 else "unknown"
    except (OSError, subprocess.SubprocessError, TypeError, ValueError):
        software_commit = "unknown"
    return strategy_sha256, software_commit

def _configure_real_operating_mode(*, team_link):
    """Apply only the explicit operating-mode switches.

    ``solo`` and ``team_link`` are intentionally separate from the fixed
    strategy constants.  They select a visible degraded/diagnostic mode and
    never alter speed, kick, boundary, or other tactical parameters.
    """

    global SOLO_TAKEOVER, TEAM_LINK_DISABLED

    SOLO_TAKEOVER = not bool(team_link)
    TEAM_LINK_DISABLED = not bool(team_link)
    if not team_link:
        logger.warning(
            "TEAM LINK DISABLED BY CONFIG; DEGRADED SINGLE-ROBOT TACTIC ACTIVE"
        )

def _log_startup_summary(camera_count, robot_id):
    """Emit one concise deployment summary; state changes are logged later."""

    pose_ok = _pose_valid(UnitedPosition.FieldRobotPose())
    mode = "DEGRADED_SINGLE_ROBOT" if SOLO_TAKEOVER else "PAIRED_COMPETITION"
    strategy_sha256, software_commit = _reproducibility_identity()
    logger.info(
        "REAL BITS=IceLoong MAINBOARD=OK POSE={} VISION={} CAMERAS={}/4 "
        "REFEREE={} TEAM_LINK={} TEAM_LINK_ENABLED={} SOLO={} MODE={} "
        "ROBOT_ID={} ROLE={} FIELD={}x{}mm STRATEGY_SHA256={} SOFTWARE_COMMIT={}",
        "OK" if pose_ok else "INVALID",
        str(getattr(Settings.RoboInfo, "Rec", "")).upper(),
        camera_count,
        _referee_summary(),
        _team_link_summary(),
        not TEAM_LINK_DISABLED,
        SOLO_TAKEOVER,
        mode,
        robot_id,
        _strategy_role(),
        int(Settings.Field.Long * 10.0),
        int(Settings.Field.Short * 10.0),
        strategy_sha256,
        software_commit,
    )

def preflight():
    """Validate the physical setup without starting strategy motion.

    It opens the existing IL resources and starts the UDP vision receiver long
    enough to verify one fresh packet. It never commands wheels, dribbler, or
    kicker. A check-only caller must call ``shutdown_runtime`` afterwards.
    """

    global SOLO_TAKEOVER, TEAM_LINK_DISABLED

    from LunaPre import runtime

    if runtime._selected_backend_name() != "IL":
        raise RuntimeError("strategy_real_australia requires the IceLoong IL backend")
    _require_real_referee_interlock()
    robot_id = int(getattr(Settings.RoboInfo, "number", 1))
    robot_ids = list(AUSTRALIA.Team.RobotIds)
    robot_roles = list(AUSTRALIA.Team.RobotRoles)
    if set(robot_ids) != {1, 2} or len(robot_ids) != len(robot_roles) or len(set(robot_ids)) != len(robot_ids):
        raise ValueError("AUSTRALIA.Team must define IDs 1 and 2 with one role each")
    if robot_id not in robot_ids:
        raise ValueError("RoboInfo.number must match an AUSTRALIA.Team.RobotIds entry")
    team_link = bool(AUSTRALIA.Runtime.TeamLinkEnabled)
    # Safety tuning is a strategy concern; the vision backend and its geometry
    # are owned by LunaPre and the loaded config, never specified here.
    _configure_real_safety()
    # A disabled/unavailable optional radio is a true single-robot mode, not
    # a silent fallback to the SUPPORT role of a two-robot match.
    _configure_real_operating_mode(team_link=team_link)
    _catch_point()
    try:
        ensure_runtime()
        if runtime.Bits is None:
            raise RuntimeError("physical runtime did not expose IceLoong Bits")
        _require_real_runtime_contract(runtime.Bits, team_link=bool(team_link))
        worker_alive = getattr(runtime.Bits, "IsThreadAlive", None)
        if callable(worker_alive) and not worker_alive():
            raise RuntimeError("physical MainBoard command worker is not alive")
        _configure_mainboard_field(runtime.Bits)
        pose = _wait_for_mainboard_pose(runtime.Bits)
        if pose is None:
            raise RuntimeError("MainBoard GET_POSE did not return a valid physical pose")
        _probe_mainboard_readonly_health(runtime.Bits)
        _reset_real_vision_freshness()
        if not UnitedPosition.StartVision():
            raise RuntimeError("physical vision UDP receiver did not start")
        if not _wait_for_real_vision():
            raise RuntimeError("no fresh physical vision UDP packet received")
        if team_link and _team_transport_configured():
            _configure_team_link_polling(runtime.Bits)
        elif team_link:
            TEAM_LINK_DISABLED = True
            SOLO_TAKEOVER = True
            logger.warning(
                "TEAM LINK DISABLED: SendLinkData/GetLinkData API unavailable; "
                "DEGRADED SINGLE-ROBOT TACTIC ACTIVE"
            )
    except Exception:
        # StartVision owns a receiver thread and (on the physical profile) a
        # vision child process; release both when preflight aborts.
        try:
            UnitedPosition.Stop()
        except Exception:
            pass
        shutdown_runtime()
        raise

def main():
    global _REAL_DEPLOYMENT_ACTIVE

    previous_sigterm = signal.getsignal(signal.SIGTERM)
    previous_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGTERM, _stop_signal)
    signal.signal(signal.SIGINT, _stop_signal)
    try:
        preflight()
        # Transport-age deadlines are enforced only in the live physical loop.
        _REAL_DEPLOYMENT_ACTIVE = True
        _start_team_communication()
        if not UnitedPosition.StartVision():
            raise RuntimeError("physical vision UDP receiver did not start")
        if not _real_vision_ready() and not _wait_for_real_vision():
            raise RuntimeError("physical vision did not provide a fresh packet after startup")
        reset_strategy()
        _reset_attack_direction()
        _log_startup_summary(4, _robot_id())
        last_state = None
        next_deadline = time.monotonic()
        last_overrun_log = -math.inf
        last_perf_log = next_deadline
        logic_samples_ms = []
        while True:
            logic_started = time.monotonic()
            if _real_vision_ready():
                state = logic_step()
            else:
                _reset_attack()
                _reset_real_ball_control()
                safe_stop(chassis, peripheral)
                if not _real_mainboard_ready():
                    _force_real_lower_board_stop()
                state = "wait_real_vision"
            logic_elapsed_ms = (time.monotonic() - logic_started) * 1000.0
            if math.isfinite(logic_elapsed_ms) and logic_elapsed_ms >= 0.0:
                logic_samples_ms.append(logic_elapsed_ms)
                if len(logic_samples_ms) > AUSTRALIA.Real.CONTROL_PERF_SAMPLE_WINDOW:
                    del logic_samples_ms[:-AUSTRALIA.Real.CONTROL_PERF_SAMPLE_WINDOW]
            if state != last_state:
                logger.info("Strategy: {}", state)
                last_state = state
            # Absolute deadlines prevent per-tick Python work from stretching
            # the command period.  Drop overdue ticks rather than running a
            # catch-up burst that would replay stale perception decisions.
            next_deadline += AUSTRALIA.Runtime.ControlPeriodSeconds
            now = time.monotonic()
            _log_real_runtime_telemetry(state=state, now=now)
            if now - last_perf_log >= AUSTRALIA.Real.CONTROL_PERF_LOG_INTERVAL_S and logic_samples_ms:
                ordered = sorted(logic_samples_ms)
                p95_index = min(
                    len(ordered) - 1,
                    max(0, math.ceil(0.95 * len(ordered)) - 1),
                )
                logger.info(
                    "Real strategy timing: avg={:.3f}ms p95={:.3f}ms max={:.3f}ms samples={}",
                    sum(ordered) / len(ordered),
                    ordered[p95_index],
                    ordered[-1],
                    len(ordered),
                )
                last_perf_log = now
            if now >= next_deadline:
                missed = int((now - next_deadline) // AUSTRALIA.Runtime.ControlPeriodSeconds) + 1
                next_deadline += missed * AUSTRALIA.Runtime.ControlPeriodSeconds
                if now - last_overrun_log >= AUSTRALIA.Real.CONTROL_OVERRUN_LOG_INTERVAL_S:
                    logger.warning("Real strategy loop missed {} 60Hz tick(s)", missed)
                    last_overrun_log = now
            time.sleep(max(0.0, next_deadline - now))
    except KeyboardInterrupt:
        pass
    finally:
        try:
            # This is intentionally unconditional: Ctrl+C/SIGTERM or a
            # preflight exception may arrive before ``initialized`` flips.
            # The facade calls are fail-closed when no physical resource exists.
            safe_stop(chassis, peripheral, UnitedPosition)
        finally:
            try:
                _stop_team_communication()
            finally:
                try:
                    shutdown_runtime()
                finally:
                    _REAL_DEPLOYMENT_ACTIVE = False
                    signal.signal(signal.SIGTERM, previous_sigterm)
                    signal.signal(signal.SIGINT, previous_sigint)

if __name__ == "__main__":
    main()
