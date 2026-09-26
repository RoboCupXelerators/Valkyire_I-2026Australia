"""ReasonData.config

精简版配置模块：
- 统一模型路径配置
- 移除冗余配置项
- 保持向后兼容
"""

try:
    import ujson as _json  # type: ignore
except ModuleNotFoundError:
    import json as _json
import os
from pathlib import Path

APP_DIR = Path(__file__).parent
DATA_DIR = Path(os.environ.get("XEL_RUNTIME_DATA_DIR", APP_DIR / "data"))
CONFIG_FILE = DATA_DIR / "config.json"
CACHE_FILE = DATA_DIR / "cfg.cache.json"


class ConfigSection:
    """配置分段的通用容器"""
    def __init__(self, data):
        for key, value in data.items():
            if isinstance(value, dict):
                setattr(self, key, ConfigSection(value))
            else:
                setattr(self, key, value)


def ParseJsonToObj():
    parseData = Preference().cfg
    return ConfigSection(parseData)


class Preference:
    """配置管理类：负责读取、写入、迁移配置文件"""
    
    # 默认配置模板（唯一来源）
    DEFAULT_CONFIG = {
        "RoboInfo": {
            "number": 1,
            "type": "OP",           # OP(OffencePlayer) / DP(DefencePlayer)
            "Bit": "SIM",           # SIM / IL
            "Rec": "XV",            # AI(ArisuIntelligence) / TI(TunaIntelligence) / XV(XeleratorVision) / SY(SimYoloVision) / BB(BlobBased)
        },
        "Ports": {
            "IceLoongPort": "/dev/iceloong",
            "ElecMagnet": 3,
            "Dribble": 4,
            "LeftFront": 1,
            "LeftBack": 2,
            "RightBack": 3,
            "RightFront": 4,
        },
        "Vision": {
            # 相机端口映射 (0:Back, 1:Left, 2:Front, 3:Right)
            "CamPorts": [0, 2, 4, 6],
            # 统一模型名称 (不含后缀，各推理库自动选择所需格式)
            # - ArisuIntelligence (AI/Hailo): 使用 .hef
            # - TunaIntelligence/HybridVision (TI/HV): 使用 .pt 和 .engine
            # - LunaIntelligence/SimYoloVision (SY/LI): 使用 .pt 和 .engine
            "ModelName": "ArisuModel_v1.2s",
            # 0=flexible automatic exposure, 1=auto-meter then lock exposure.
            "ExposureMode": 1,
            # -4..4 controls the camera's physical brightness control.
            "ExposureBias": 0,
            # Stable mode's fixed UVC exposure value (10..500). Lower is darker.
            "ManualExposure": 68,
            # 标定文件夹
            "CalibrationFolder": "Calibration",
            # Target dimensions (cm) for operator-facing object semantics.
            "BallDiameter": 4.2,
            "ChassisWidth": 18.0,
            "ChassisHeight": 5.0,
            "GoalWidth": 90.0,
            "GoalHeight": 18.0,
            "MinBallBBoxArea": 40,
            "BallOffsetXCM": 0.0,
            "BallOffsetYCM": 0.0,
            # HybridVision: low-rate search on every camera, high-rate focus on
            # the camera that recently saw the ball.
            "AdaptiveFrameDistribution": True,
            "CameraMinYoloHz": 4.0,
            "CameraFocusYoloHz": 20.0,
            "CameraFocusHoldMs": 250.0,
            "YoloMaxFramesPerTick": 4,
            # 录制配置
            "Record": False,
            "RecordDir": "./Records",
        },
        "Field": {
            "Short": 200,           # 短边长度 (cm)
            "Long": 260,            # 长边长度 (cm)
            "Home": [-70, 0, 0],    # 默认回位点
        },
        "Chassis": {
            "WheelRadiusM": 0.029,  # 轮子半径，单位 m
            "RobotRadiusM": 0.09,   # 机器人中心到轮子的半径，单位 m
            "MaxWheelRPM": 500,     # 电机命令单位 RPM，正方向为单轮顺时针
        },
        "Motion": {
            "CatchVal": [6.8, 0],   # 抱球时的球位置：机器人中心到球心 8.5cm
            "ErrorRange": 10,       # 到达目标的误差范围 (cm)

            "LockBallSlip": {
                "ForwardKp": 0.025,
                "ForwardKi": 0.0,
                "ForwardKd": 0.0,
                "LateralKp": 0.012,
                "LateralKi": 0.0,
                "LateralKd": 0.0,
                "AngularKp": 0.9,
                "AngularKi": 0.0,
                "AngularKd": 0.0,
                "ForwardMaxMps": 0.35,
                "LateralMaxMps": 0.35,
                "AngularMaxDegS": 50.0,
                "TurnOnlyAngleDeg": 70.0,
                "TurnOnlyKp": 0.75,
                "TurnOnlyMaxDegS": 55.0,
                "RearForwardKp": 0.012,
                "RearForwardMaxMps": 0.55,
                "RearLateralKp": 0.016,
                "RearLateralMaxMps": 0.65,
                "RearAngularKp": 1.80,
                "RearAngularMaxDegS": 85.0,
                "LostHoldSeconds": 0.18,
                "NearCreepMaxMps": 0.10,
                "NearDockDecelMps2": 0.5,
                "BallJumpRejectCm": 45.0,
                "BallJumpConfirmFrames": 2
            },
            "RestrictedArea": {
                # 坐标安全区配置，单位 cm。策略只依赖 FieldRobotPose，不依赖雷达原始距离。
                "XLimit": 110.0,
                "YLimit": 80.0,
                "GoalBoxWidth": 25.0,
                "GoalBoxYHalf": 40.0,
                "GoalBoxCornerRadius": 20.0,
                # 1=严格整车不越线，2=澳洲赛规：允许局部越线但禁止整车进入
                "ProtectionLevel": 1,
                "Clearance": 5.0,
                "RecoveryClearance": 16.0,
                "RecoveryExtra": 3.0,
                # 距离禁区越近速度越低：进入减速区开始线性减速，到边界时降到
                # SpeedMinScale 倍，避免雷达/定位延迟冲过线。
                # SpeedSlowZoneCm=0 表示减速区距离按场地尺寸自动推导（推荐）；
                # >0 时用该值(cm)覆盖。
                "SpeedSlowZoneCm": 0,
                "SpeedMinScale": 0.3
            },
        },
        # Australia Open tuning. Unit suffixes in keys are centimetres, seconds, m/s or degrees.
        "AustraliaStrategy": {'Real': {'FIELD_LONG': 243.0,
                  'FIELD_SHORT': 182.0,
                  'RCJA_WHITE_LINE_OFFSET_CM': 25.0,
                  'RCJA_WHITE_LINE_WIDTH_CM': 5.0,
                  'REAL_WALL_CLEARANCE_CM': 0.05,
                  'REAL_DETECTION_TOLERANCE_CM': 5.0,
                  'RCJA_OPEN_WALL_BANK_ALLOWED': True,
                  'REAL_GOAL_WIDTH_CM': 45.0,
                  'REAL_GOAL_HEIGHT_CM': 14.0,
                  'SIDE_MIN_X_CM': 20.0,
                  'SIDE_CONFIRM_S': 0.3,
                  'SIDE_POSITION_TOLERANCE_CM': 5.0,
                  'CONTROL_OVERRUN_LOG_INTERVAL_S': 2.0,
                  'CONTROL_PERF_LOG_INTERVAL_S': 5.0,
                  'CONTROL_PERF_SAMPLE_WINDOW': 300,
                  'REAL_MAINBOARD_RX_TIMEOUT_S': 0.3,
                  'REAL_MAINBOARD_TX_TIMEOUT_S': 0.3,
                  'REAL_POSE_RX_TIMEOUT_S': 0.3,
                  'REAL_REFEREE_RX_TIMEOUT_S': 0.3,
                  'REAL_RUNTIME_TELEMETRY_LOG_INTERVAL_S': 5.0,
                  'CHASE_REALIGN_DEG': 18.0,
                  'REAL_ABSOLUTE_MAX_MPS': 2.0,
                  'REAL_SAFE_FAST_MPS': 1.5,
                  'REAL_CAPTURE_MPS': 1.5,
                  'REAL_ALIGNMENT_MPS': 0.55,
                  'CHASE_MAX_MPS': 3.0,
                  'CHASE_PREDICT_MIN_DISTANCE_CM': 35.0,
                  'REAL_LAST_BALL_MEMORY_S': 0.8,
                  'REAL_MEMORY_SEARCH_SPEED_GAIN': 1.5,
                  'REAL_DIRECT_COMMAND_HORIZON_S': 0.06,
                  'REAL_POSE_VELOCITY_MIN_SAMPLE_S': 0.05,
                  'REAL_POSE_VELOCITY_MAX_SAMPLE_S': 0.4,
                  'REAL_POSE_VELOCITY_MAX_CM_S': 300.0,
                  'REAL_POSE_VELOCITY_MAX_DEG_S': 720.0,
                  'REAL_BOUNDARY_BRAKE_MPS2': 1.8,
                  'REAL_HOLD_CONFIRM_S': 0.25,
                  'REAL_HOLD_LOSS_GRACE_S': 0.15,
                  'REAL_CARRY_MPS': 1.5,
                  'REAL_BACK_CARRY_MPS': 1.5,
                  'REAL_REPOSITION_MPS': 0.75,
                  'REAL_RELEASE_BACK_MPS': 0.35,
                  'REAL_RELEASE_TIMEOUT_S': 0.45,
                  'REAL_REACQUIRE_TIMEOUT_S': 3.5,
                  'REAL_LOST_WAIT_S': 0.3,
                  'REAL_REACQUIRE_RADIUS_CM': 28.0,
                  'REAL_CARRY_RECOVERY_MAX_ATTEMPTS': 2,
                  'REAL_CARRY_RECOVERY_RETRY_S': 0.75,
                  'REAL_CARRY_RECOVERY_ARRIVAL_CM': 7.0,
                  'REAL_HELD_BALL_STALL_TIMEOUT_S': 0.75,
                  'REAL_ARC_COOLDOWN_S': 4.0,
                  'REAL_BACKSPIN_TIMEOUT_S': 22.0,
                  'REAL_MACAU_TIMEOUT_S': 30.0,
                  'REAL_SPLIT_TIMEOUT_S': 18.0,
                  'REAL_WALL_TIMEOUT_S': 16.0,
                  'REAL_SLIP_TIMEOUT_S': 1.6,
                  'REAL_SLIP_FORWARD_CM': 30.0,
                  'REAL_SLIP_LATERAL_CM': 24.0,
                  'REAL_MACAU_PRESSURE_MIN_CM': 30.0,
                  'REAL_MACAU_PRESSURE_MAX_CM': 65.0,
                  'REAL_MACAU_PRESSURE_ESCAPE_CM': 20.0,
                  'REAL_DUEL_CONFIRM_S': 0.18,
                  'REAL_VISION_TIMEOUT_S': 0.18,
                  'REAL_PLATFORM_JETSON_NANO': 'jetson_nano',
                  'REAL_VISION_BACKEND': 'HV',
                  'REAL_VISION_PROFILE': 'nano',
                  'REAL_BALL_DIAMETER_CM': 4.2},
         'Team': {'SHOT_ALIGN_DEG': 5.0,
                  'SHOT_COOLDOWN_S': 1.2,
                  'SHOT_GOAL_MARGIN_CM': 6.0,
                  'SHOT_MIN_ALIGN_DEG': 0.75,
                  'SHOT_BLOCKED_WAIT_S': 0.4,
                  'GOAL_MOUTH_ESCAPE_TRIGGER_CM': 32.0,
                  'GOAL_MOUTH_ESCAPE_RETREAT_CM': 20.0,
                  'HIDDEN_MACAU_FLANK_CLEARANCE_CM': 26.0,
                  'TEAMMATE_VISION_MATCH_CM': 30.0,
                  'REAL_PASS_MIN_DISTANCE_CM': 35.0,
                  'REAL_PASS_MAX_DISTANCE_CM': 135.0,
                  'REAL_PASS_RECEIVER_CLEARANCE_CM': 30.0,
                  'REAL_PASS_RECEIVER_ALIGNMENT_DEG': 35.0,
                  'REAL_PASS_SETUP_TIMEOUT_S': 0.8,
                  'REAL_PASS_FLIGHT_TIMEOUT_S': 1.2,
                  'SUPPORT_RECEIVE_ARRIVAL_CM': 5.0,
                  'REAL_PASS_LEVEL_DISTANCE_CM': (45.0, 75.0, 105.0, 135.0),
                  'DEFENCE_FRONT_ROBOT_ID': 2,
                  'RobotIds': [1, 2],
                  'RobotRoles': ['PRIMARY', 'SUPPORT'],
                  'OpponentRobotIds': [],
                  'ContestStates': ['contest_ball',
                                    'contest_carrier_challenge',
                                    'contest_block_helper',
                                    'contest_push_enemy',
                                    'front_defence_challenge']},
         'Tactics': {'AUSTRALIA_RULE_PROFILE': 'rcja_open_2026',
                     'SIM_PENALTY_RULE_ENABLED': False,
                     'SIM_GOAL_WIDTH_CM': 45.0,
                     'SIM_GOAL_DEPTH_CM': 7.4,
                     'SIM_PENALTY_FRONT_CM': 70.0,
                     'SIM_PENALTY_HALF_WIDTH_CM': 45.0,
                     'SIM_PENALTY_RAW_HALF_WIDTH_CM': 45.0,
                     'ATTACK_DIR': None,
                     'LAST_BALL_MEMORY_S': 0.8,
                     'CHASE_SENSOR_PREDICT_S': 0.12,
                     'SIM_BODY_RADIUS_CM': 11.0,
                     'MOTION_CLEARANCE_CM': 5.0,
                     'GOAL_CLEARANCE_CM': 1.0,
                     'BOUNDARY_CONTACT_EDGE_GUARD_CM': 30.0,
                     'BOUNDARY_CONTACT_APPROACH_CM': 38.0,
                     'TEAMMATE_SENSOR_MATCH_CM': 15.0,
                     'TEAM_STATE_SEND_PERIOD_S': 0.05,
                     'TEAM_STATE_MAX_AGE_S': 0.3,
                     'TEAMMATE_LOSS_CONFIRM_S': 0.75,
                     'TEAMMATE_STARTUP_TAKEOVER_S': 1.0,
                     'TEAMMATE_HELD_BALL_MATCH_CM': 20.0,
                     'MOTION_MIN_MPS': 0.08,
                     'SIM_MAX_SPEED_MPS': 3.0,
                     'SIM_MAX_WHEEL_RPM': 1600.0,
                     'MOTION_BRAKE_MPS2': 1.8,
                     'MOTION_COMMAND_BRAKE_MPS2': 4.0,
                     'MOTION_LATENCY_S': 0.06,
                     'MOTION_TELEMETRY_DELAY_S': 0.18,
                     'MOTION_EMERGENCY_BRAKE_CM': 18.0,
                     'SIM_RULE_EDGE_EPSILON_CM': 0.05,
                     'HIGH_SPEED_ENEMY_CM_S': 35.0,
                     'HIGH_SPEED_ENEMY_MIN_PROGRESS_CM': -55.0,
                     'CHASE_ALIGN_DEG': 10.0,
                     'CHASE_TURN_MAX_DEG_S': 360.0,
                     'CHASE_TURN_GAIN': 5.0,
                     'CHASE_LEAD_MAX_S': 0.25,
                     'CHASE_CAPTURE_KP': 8.0,
                     'SIM_DRIBBLER_CAPTURE_RELATIVE_LIMIT_MPS': 1.6,
                     'CHASE_CAPTURE_SLOWDOWN_DISTANCE_M': 0.98,
                     'DIRECT_CARRY_ALIGN_DEG': 20.0,
                     'SCREEN_THREAT_RADIUS_CM': 85.0,
                     'SCREEN_IMMEDIATE_RADIUS_CM': 45.0,
                     'SCREEN_BALL_CLEARANCE_CM': 26.0,
                     'SCREEN_LANE_CLEARANCE_CM': 28.0,
                     'SCREEN_LATERAL_OFFSET_CM': 18.0,
                     'CONTEST_ASSIST_CONTACT_CM': 46.0,
                     'CONTEST_ASSIST_CHASER_CM': 48.0,
                     'CONTEST_SUPPORT_BACK_CM': 24.0,
                     'CONTEST_SUPPORT_SIDE_CM': 34.0,
                     'CONTEST_HELPER_MIN_CM': 48.0,
                     'CONTEST_HELPER_MAX_CM': 95.0,
                     'CONTEST_HELPER_BLOCK_CM': 38.0,
                     'TEAMMATE_HOLD_GRACE_S': 0.2,
                     'LOCAL_DRIBBLER_CAPTURE_X_MARGIN_CM': 2.0,
                     'LOCAL_DRIBBLER_CAPTURE_Y_MARGIN_CM': 1.5,
                     'HELD_TURN_MAX_DEG_S': 120.0,
                     'TURN_MIN_DEG_S': 16.0,
                     'SHOT_RELEASE_TIMEOUT_S': 0.1,
                     'SHOT_OPENING_ALIGN_MAX_DEG': 12.0,
                     'DIRECT_KICK_POWER_LEVEL': 3,
                     'WALL_KICK_POWER_LEVEL': 4,
                     'SIM_WALL_REBOUND_ENABLED': True,
                     'MACAU_ENTRY_MIN_ABS_Y_CM': 58.0,
                     'MACAU_LANE_ENEMY_CLEARANCE_CM': 58.0,
                     'MACAU_RETRY_S': 1.2,
                     'MACAU_POCKET_MIN_PROGRESS_CM': 98.0,
                     'MACAU_POCKET_MAX_PROGRESS_CM': 108.0,
                     'MACAU_POCKET_TARGET_PROGRESS_CM': 104.0,
                     'MACAU_POCKET_ARRIVAL_CM': 6.0,
                     'MACAU_POCKET_MIN_ABS_Y_CM': 40.0,
                     'MACAU_POCKET_MAX_ABS_Y_CM': 80.0,
                     'MACAU_POCKET_TARGET_ABS_Y_CM': 60.0,
                     'MACAU_SHIELD_SAMPLE_DEG': 15.0,
                     'MACAU_SHIELD_REPLAN_GAIN': 12.0,
                     'MACAU_SHIELD_MAX_STEP_DEG': 36.0,
                     'MACAU_SHIELD_BODY_MARGIN_CM': 2.0,
                     'MACAU_SHIELD_LANE_STEP_CM': 6.0,
                     'MACAU_SHIELD_REAR_PROGRESS_CM': 10.0,
                     'MACAU_DEFENCE_LANE_MIN_CM': 52.0,
                     'MACAU_DEFENCE_LEAD_CM': 32.0,
                     'MACAU_DEFENCE_TRACK_S': 0.45,
                     'MACAU_DEFENCE_MIN_ADVANCE_CM_S': 15.0,
                     'MACAU_DEFENCE_MAX_LATERAL_SHIFT_CM': 16.0,
                     'MACAU_DEFENCE_HOLD_S': 0.35,
                     'HIDDEN_MACAU_COLLAPSE_S': 0.5,
                     'SPLIT_SHIFT_CM': 24.0,
                     'SPLIT_TIMEOUT_S': 4.0,
                     'WALL_AIM_TIMEOUT_S': 1.5,
                     'FRONT_PRESSURE_DISTANCE_CM': 45.0,
                     'DUEL_BREAKAWAY_CONFIRM_S': 0.18,
                     'DUEL_BREAKAWAY_CONTACT_CM': 26.0,
                     'DUEL_BREAKAWAY_RETREAT_CM': 30.0,
                     'DUEL_BREAKAWAY_MAX_S': 0.55,
                     'DUEL_BREAKAWAY_ARRIVAL_CM': 6.0,
                     'DUEL_EJECT_FALLBACK_S': 1.0,
                     'CONTEST_BALL_RADIUS_CM': 26.0,
                     'ENEMY_SAMPLE_MIN_S': 0.05,
                     'ENEMY_SAMPLE_MAX_S': 0.4,
                     'ENEMY_MAX_SPEED_CM_S': 300.0,
                     'SUPPORT_SIDE': -1.0,
                     'SUPPORT_Y_CM': 38.0,
                     'SUPPORT_BALL_MIN_X_CM': -10.0,
                     'SUPPORT_CAPTURE_RADIUS_CM': 60.0,
                     'ALLY_CARRIER_CLEARANCE_CM': 36.0,
                     'ALLY_SHOT_LANE_CLEARANCE_CM': 34.0,
                     'SUPPORT_COVER_FORWARD_CM': 30.0,
                     'SUPPORT_COVER_LATERAL_CM': 55.0,
                     'COOP_MIN_PROGRESS_CM': -35.0,
                     'COOP_SETUP_TIMEOUT_S': 2.4,
                     'COOP_RETRY_S': 2.5,
                     'PASS_READY_S': 0.08,
                     'PASS_READY_DISTANCE_CM': 7.0,
                     'PASS_RECEIVE_ALIGN_DEG': 8.0,
                     'PASS_RECEIVE_SPEED_MPS': 0.65,
                     'PASS_MAX_LAUNCH_MPS': 4.0,
                     'MAX_PASS_CUSHIONS': 1,
                     'PINNED_PASS_CONTACT_CM': 26.0,
                     'PINNED_PASS_MAX_SPEED_MPS': 0.03,
                     'SHOT_MAX_LAUNCH_MPS': 4.0,
                     'PASS_KICK_IMPULSE_MPS': 4.0,
                     'PASS_WALL_RESTITUTION': 0.5,
                     'PASS_RESISTANCE_MODEL': 'rolling',
                     'PASS_CALIBRATED_DECEL_MPS2': 0.3,
                     'PASS_CALIBRATED_DRAG_MPS2': 0.0,
                     'PASS_CALIBRATED_BACKSPIN_MPS': 0.0,
                     'PASS_MAX_AGE_S': 0.35,
                     'PASS_FLIGHT_TIMEOUT_S': 3.0,
                     'SHOT_RECEIVE_SPEED_MPS': 2.0,
                     'PASS_SETUP_ESCAPE_CM': 18.0,
                     'PASS_SETUP_SHIELD_RADIUS_CM': 55.0,
                     'PASS_SETUP_TIMEOUT_S': 1.25,
                     'SHIELD_PASS_TIMEOUT_S': 0.7,
                     'SLIP_SIDE_TIMEOUT_S': 1.6,
                     'SLIP_SIDE_ESCAPE_CM': 30.0,
                     'SLIP_SIDE_LATERAL_CM': 24.0,
                     'SLIP_SIDE_PRESSURE_CM': 34.0,
                     'MAX_CUSHIONS': 1,
                     'FRONT_DEFENCE_X': 0.0,
                     'SOLO_GOAL_THREAT_RADIUS_CM': 68.0,
                     'HELD_ENEMY_OVERLAP_CM': 14.0,
                     'DEFENCE_BALL_ENTER_CM': 10.0,
                     'DEFENCE_RELEASE_DELAY_S': 0.6,
                     'INTERCEPT_HORIZON_S': 1.2,
                     'INTERCEPT_MIN_SPEED_CM_S': 15.0,
                     'INTERCEPT_MAX_SPEED_CM_S': 3000.0,
                     'INTERCEPT_CAPTURE_RADIUS_CM': 24.0,
                     'INTERCEPT_CAPTURE_MAX_SPEED_CM_S': 70.0,
                     'RADICAL_MEMORY_TIME': 0.45,
                     'RADICAL_TRACK_CM': 45.0,
                     'RADICAL_STANDOFF_CM': 28.0,
                     'RADICAL_PREP_FORWARD_CM': 14.0,
                     'RADICAL_PREP_LATERAL_CM': 30.0,
                     'RADICAL_REAR_ENTRY_CM': 12.0,
                     'RADICAL_PREP_ARRIVAL_CM': 2.0,
                     'RADICAL_CONTACT_RUSH_CM': 44.0,
                     'CARRIER_GOAL_LINE_BLOCK_STANDOFF_CM': 52.0,
                     'CARRIER_GOAL_LINE_BLOCK_ARRIVAL_CM': 2.0,
                     'DEADBALL_CONTACT_CM': 25.0,
                     'DEADBALL_SLOT_OCCUPIED_CM': 28.0,
                     'DEADBALL_STAGE_DISTANCE_CM': 33.0,
                     'DEADBALL_CARRIER_MAX_MPS': 0.03,
                     'DEADBALL_GOAL_DEPTH_CM': 18.0,
                     'DEADBALL_SLOT_INSET_CM': 45.0,
                     'DEADBALL_DIRECT_PREEMPT_DEG': 12.0,
                     'RADICAL_APPROACH_SPEED_CM_S': 10.0,
                     'RADICAL_NEAR_MIDFIELD_PROGRESS_CM': -25.0,
                               'PRESS_MAX_MPS': 3.0,
                     'GOAL_BACK_WALL_AIM_MARGIN_CM': 4.5,
                     'GOAL_POST_RETREAT_TRIGGER_CM': 34.0,
                     'GOAL_POST_RETREAT_ARRIVAL_CM': 6.0,
                     'GOAL_POST_PATH_GRID_CM': 10.0,
                     'GOAL_POST_PATH_MAX_DISTANCE_CM': 80.0,
                     'GOAL_POST_PATH_REPLAN_S': 1.2},
         'Runtime': {'ControlPeriodSeconds': 0.016666666666666666,
                     'TeamLinkPollIntervalSeconds': 0.016666666666666666,
                     'Platform': 'jetson_nano',
                     'TeamLinkEnabled': True}},
        "Debug": {
            "FullLog": False,
            "Beta": True,
            "RecordMotion": False,
            "RecordInterval": 1,
            "RecordSessionId": None,
            "RecordPos2Pos": False,
            "Pos2PosRecordDir": "./Records/pos2pos",
        },
    }

    def __init__(self):
        # 确保配置文件存在
        try:
            os.stat(CONFIG_FILE)
        except FileNotFoundError:
            self._create_default_config()

        # 读取配置
        with open(CONFIG_FILE) as f:
            self.cfg = _json.load(f)

        self._validate_legacy_sensor(self.cfg)

        # 检查是否需要迁移旧配置
        if self._needs_migration():
            self._migrate_config()
        self._validate_active_config(self.cfg)

    @staticmethod
    def _active_bit(value) -> str:
        return str(value or "").strip().upper()

    def _validate_active_config(self, config: dict) -> None:
        bit = self._active_bit(config.get("RoboInfo", {}).get("Bit"))
        if bit not in {"SIM", "IL"}:
            raise ValueError(
                "Unsupported RoboInfo.Bit=%r. Active Bits are SIM and IL; "
                "DM2325 is supported on the legacy branch; ArisuBits is retired. "
                "Select SIM or IL before starting LunaPre." % bit
            )

    @staticmethod
    def _validate_legacy_sensor(config: dict) -> None:
        """Reject retired hardware selection before changing its configuration."""
        source = str(config.get("Sensor", {}).get("Source", "auto")).strip().lower()
        backend = config.get("RoboInfo", {}).get("LidarBackend")
        if source not in {"auto", "bits"} or backend is not None:
            raise ValueError(
                "Native lidar configuration requires the legacy branch "
                "(ROS2 / LD07 / LD06). Main uses SIM or IL Bits pose."
            )

    def _create_default_config(self):
        try:
            DATA_DIR.mkdir(parents=True, exist_ok=True)
            with open(CONFIG_FILE, "w") as f:
                _json.dump(self.DEFAULT_CONFIG, f, indent=2)
        except Exception:
            raise FileNotFoundError("无法创建 config.json，请手动创建")

    def _needs_migration(self):
        """检查是否需要从旧配置迁移"""
        def has_extra_key(default, current):
            if not isinstance(default, dict) or not isinstance(current, dict):
                return False
            for key, value in current.items():
                if key not in default:
                    return True
                if has_extra_key(default[key], value):
                    return True
            return False

        for section in self.DEFAULT_CONFIG:
            if section not in self.cfg:
                return True
            for key in self.DEFAULT_CONFIG[section]:
                if key not in self.cfg.get(section, {}):
                    return True
        return has_extra_key(self.DEFAULT_CONFIG, self.cfg)

    def _migrate_config(self):
        """迁移旧配置到新格式"""
        # Reject retired sensor selections before changing the source file.
        with open(CONFIG_FILE, "r") as f:
            old_cfg = _json.load(f)
        self._validate_legacy_sensor(old_cfg)

        # 创建新配置，先保留现有新格式值，再兼容旧字段映射。
        new_cfg = _json.loads(_json.dumps(self.DEFAULT_CONFIG))

        def merge_existing(dst, src):
            for key, value in src.items():
                if key not in dst:
                    continue
                if isinstance(value, dict) and isinstance(dst.get(key), dict):
                    merge_existing(dst[key], value)
                else:
                    dst[key] = value

        merge_existing(new_cfg, old_cfg)
        
        # 迁移映射表
        migrations = {
            # VisionVals -> Vision
            ("VisionVals", "Front"): ("Vision", "CamPorts", lambda v: None),  # 特殊处理
            ("VisionVals", "HailoModelName"): ("Vision", "HailoModelPath"),
            ("VisionVals", "TunaModelName"): ("Vision", "ModelPath"),
            ("VisionVals", "TunaEnginePath"): ("Vision", "EnginePath"),
            ("VisionVals", "CalibrationFolder"): ("Vision", "CalibrationFolder"),
            ("VisionVals", "BallDiameterCM"): ("Vision", "BallDiameter"),
            ("VisionVals", "ChassisWidthCM"): ("Vision", "ChassisWidth"),
            ("VisionVals", "ChassisHeightCM"): ("Vision", "ChassisHeight"),
            ("VisionVals", "BlueGoalWidthCM"): ("Vision", "GoalWidth"),
            ("VisionVals", "Record"): ("Vision", "Record"),
            ("VisionVals", "RecordDir"): ("Vision", "RecordDir"),
            # Bounds -> Field
            ("Bounds", "Short"): ("Field", "Short"),
            ("Bounds", "Long"): ("Field", "Long"),
            # CheckPoints -> Field
            ("CheckPoints", "Home"): ("Field", "Home"),
            # ExpectedVals -> Motion
            ("ExpectedVals", "CatchVal"): ("Motion", "CatchVal"),
            ("ExpectedVals", "ErrorRange"): ("Motion", "ErrorRange"),
            ("ExpectedVals", "MaxSpeedValue"): ("Chassis", "MaxWheelRPM"),
        }

        for (old_sec, old_key), new_path in migrations.items():
            try:
                old_val = old_cfg.get(old_sec, {}).get(old_key)
                if old_val is not None:
                    if len(new_path) == 2:
                        new_sec, new_key = new_path
                        if new_sec in new_cfg and new_key in new_cfg[new_sec]:
                            new_cfg[new_sec][new_key] = old_val
            except Exception:
                continue

        # Only legacy camera mappings override the current Vision.CamPorts.
        vv = old_cfg.get("VisionVals")
        if isinstance(vv, dict):
            cam_ports = [
                vv.get("Back", 0), vv.get("Left", 2),
                vv.get("Front", 4), vv.get("Right", 6),
            ]
            new_cfg["Vision"]["CamPorts"] = list(dict.fromkeys(cam_ports))

        self._validate_active_config(new_cfg)

        # Keep the original only after the complete migrated config validates.
        os.replace(CONFIG_FILE, CACHE_FILE)

        # 写入新配置
        with open(CONFIG_FILE, "w") as f:
            _json.dump(new_cfg, f, indent=2)

        # 更新内存中的配置
        self.cfg = new_cfg

    def write(self, section: str, option: str, value) -> None:
        self.cfg[section][option] = value
        with open(CONFIG_FILE, "w") as f:
            _json.dump(self.cfg, f, indent=2)

    def read(self, section: str, option: str):
        try:
            return self.cfg[section][option]
        except KeyError:
            self.__init__()
            return self.cfg.get(section, {}).get(option)


# 向后兼容的属性访问类
class _CompatVisionVals:
    """向后兼容层：将旧属性名映射到新配置"""
    def __init__(self, settings):
        self._s = settings

    @property
    def Front(self): return self._s.Vision.CamPorts[2] if len(self._s.Vision.CamPorts) > 2 else 4
    @property
    def Right(self): return self._s.Vision.CamPorts[3] if len(self._s.Vision.CamPorts) > 3 else 6
    @property
    def Back(self): return self._s.Vision.CamPorts[0] if len(self._s.Vision.CamPorts) > 0 else 0
    @property
    def Left(self): return self._s.Vision.CamPorts[1] if len(self._s.Vision.CamPorts) > 1 else 2
    @property
    def HailoModelName(self): 
        # 向后兼容：返回 ModelName + .hef 或旧配置 HailoModelPath
        model_name = getattr(self._s.Vision, "ModelName", None)
        if model_name:
            return f"/{model_name}.hef"
        return getattr(self._s.Vision, "HailoModelPath", "/ArisuModel_v1.2s.hef")
    @property
    def TunaModelName(self):
        # 向后兼容：返回 ModelName + .pt 或旧配置 ModelPath
        model_name = getattr(self._s.Vision, "ModelName", None)
        if model_name:
            return f"/{model_name}.pt"
        return getattr(self._s.Vision, "ModelPath", "")
    @property
    def TunaEnginePath(self):
        # 向后兼容：返回 ModelName + .engine 或旧配置 EnginePath
        model_name = getattr(self._s.Vision, "ModelName", None)
        if model_name:
            return f"/{model_name}.engine"
        return getattr(self._s.Vision, "EnginePath", "")
    @property
    def CalibrationFolder(self): return self._s.Vision.CalibrationFolder
    @property
    def BallDiameterCM(self): return self._s.Vision.BallDiameter
    @property
    def ChassisWidthCM(self): return self._s.Vision.ChassisWidth
    @property
    def ChassisHeightCM(self): return self._s.Vision.ChassisHeight
    @property
    def BlueGoalWidthCM(self): return self._s.Vision.GoalWidth
    @property
    def BlueGoalHeightCM(self): return self._s.Vision.GoalHeight
    @property
    def YellowGoalWidthCM(self): return self._s.Vision.GoalWidth
    @property
    def YellowGoalHeightCM(self): return self._s.Vision.GoalHeight
    @property
    def EnableBBoxDistanceEstimate(self): return False
    @property
    def Record(self): return self._s.Vision.Record
    @property
    def RecordDir(self): return self._s.Vision.RecordDir
    @property
    def RecordCams(self): return [0, 1, 2, 3]
    @property
    def RecordWidth(self): return 320
    @property
    def RecordHeight(self): return 320
    @property
    def RecordSegmentSeconds(self): return 60
    @property
    def RecordFPS(self): return 30
    @property
    def RecordCodec(self): return "mp4v"
    @property
    def RecordExt(self): return "mp4"
    @property
    def CamFps(self): return getattr(self._s.Vision, "CamFps", getattr(self._s.Vision, "CaptureFps", 30))
    @property
    def ExposeVal(self): return 100
    @property
    def AutoExpose(self): return 0


class _CompatBounds:
    """向后兼容层：Bounds -> Field"""
    def __init__(self, settings):
        self._s = settings

    @property
    def Short(self): return self._s.Field.Short
    @property
    def Long(self): return self._s.Field.Long
    @property
    def ClsPos(self): return [85, 35, 0]
    @property
    def FarPos(self): return [95, 75, 0]
    @property
    def Distance2Bound(self): return 240
    @property
    def restricted_area_x(self): return 400
    @property
    def restricted_area_y(self): return 800


class _CompatExpectedVals:
    """向后兼容层：ExpectedVals -> Motion"""
    def __init__(self, settings):
        self._s = settings

    @property
    def CatchVal(self): return self._s.Motion.CatchVal
    @property
    def ErrorRange(self): return self._s.Motion.ErrorRange
    @property
    def MaxSpeedValue(self): return self._s.Chassis.MaxWheelRPM
    @property
    def MaxWarnCount(self): return 3


class _CompatSim:
    """向后兼容层：旧代码仍可读 Settings.Sim，但默认值由 XeleratorSimBits 管理。"""
    def __init__(self, settings):
        self._s = settings

    def _get(self, key, default):
        raw_sim = getattr(self._s._raw, "Sim", None)
        return getattr(raw_sim, key, default)

    @property
    def UnityIP(self): return self._get("UnityIP", "127.0.0.1")
    @property
    def RobotId(self): return self._get("RobotId", getattr(self._s.RoboInfo, "number", 1))
    @property
    def CmdPortBase(self): return self._get("CmdPortBase", self._get("PortBase", 20001))
    @property
    def StatusPortBase(self): return self._get("StatusPortBase", self.CmdPortBase + 1)
    @property
    def PortStride(self): return self._get("PortStride", 10)
    @property
    def MultiRobotMode(self): return self._get("MultiRobotMode", "per_port")
    @property
    def SceneControlPort(self): return self._get("SceneControlPort", 20100)
    @property
    def CmdPort(self): return self._get("CmdPort", self.CmdPortBase)
    @property
    def StatusPort(self): return self._get("StatusPort", self.StatusPortBase)
    @property
    def TwistFromWheels(self): return self._get("TwistFromWheels", True)
    @property
    def WheelToV(self): return self._get("WheelToV", 0.0029321531433504737)
    @property
    def WheelToWz(self): return self._get("WheelToWz", 0.0337524905204023)
    @property
    def YawOffsetDeg(self): return self._get("YawOffsetDeg", 0.0)
    @property
    def YawZeroAtStart(self): return self._get("YawZeroAtStart", False)
    @property
    def KickStrength(self): return self._get("KickStrength", 4.0)
    @property
    def DribbleStrengthScale(self): return self._get("DribbleStrengthScale", self._get("DribbleScale", 3.0))
    @property
    def PlanarMoveSpeedScale(self): return self._get("PlanarMoveSpeedScale", self._get("MoveScale", 1.0))
    @property
    def VisionNoiseLevel(self): return self._get("VisionNoiseLevel", 0.0)
    @property
    def VisionNoiseSeed(self): return self._get("VisionNoiseSeed", 0)
    @property
    def UnityToCm(self): return 100.0
    @property
    def OriginXCM(self): return 0.0
    @property
    def OriginYCM(self): return 0.0
    @property
    def PoseScale(self): return 1.0
    @property
    def PoseOffsetXCM(self): return 0.0
    @property
    def PoseOffsetYCM(self): return 0.0
    @property
    def Debug(self): return False


class SettingsProxy:
    """设置代理：提供新配置API和向后兼容"""
    
    def __init__(self):
        self._raw = ParseJsonToObj()
        # 设置向后兼容层
        self.VisionVals = _CompatVisionVals(self)
        self.Bounds = _CompatBounds(self)
        self.ExpectedVals = _CompatExpectedVals(self)
        # Sim需要特殊处理，因为新旧配置混用
        self._sim_compat = _CompatSim(self)

    def __getattr__(self, name):
        # 优先返回原始配置
        if hasattr(self._raw, name):
            return getattr(self._raw, name)
        # Sim向后兼容
        if name == "Sim":
            return self._sim_compat
        raise AttributeError(f"'{type(self).__name__}' object has no attribute '{name}'")

    # 直接暴露常用配置段
    @property
    def RoboInfo(self): return self._raw.RoboInfo
    @property
    def Ports(self): return self._raw.Ports
    @property
    def Vision(self): return self._raw.Vision
    @property
    def Field(self): return self._raw.Field
    @property
    def Chassis(self): return self._raw.Chassis
    @property
    def Motion(self): return self._raw.Motion
    @property
    def Sim(self): return getattr(self._raw, "Sim", self._sim_compat)
    @property
    def Debug(self): return self._raw.Debug
    # 向后兼容别名
    @property
    def CheckPoints(self):
        class _CP:
            Home = self._raw.Field.Home
        return _CP()


Settings: SettingsProxy = SettingsProxy()
