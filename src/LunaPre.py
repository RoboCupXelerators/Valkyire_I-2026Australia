"""LunaPre

本文件提供上层策略常用的封装：
- 🤖 Positions：坐标/目标位置计算（机体坐标系 & 场地坐标系）
- 📡 Communication：队内通信（Wi-Fi / BLE 等）

命名规范（与 SoccerVisonWiki/ProgramingAdvanced.md 对齐）：
- ✅ 对外方法：PascalCase（大驼峰）
- ✅ 变量：snake_case（蛇形）

兼容性：为了不一次性炸掉历史代码，旧接口会保留为“别名/包装”。
"""

from __future__ import annotations

import math
import errno
import os
import pickle
import socket
import subprocess
import sys
import threading
import time
import simple_pid

from ReasonData import logger, Settings

cfg = Settings

from Sensor import Lidar
from chassis import Car, Peripherals, Key  # Universal-Movement-Standard


# When runtime objects fall back to MockBits (no hardware), this flag becomes True.
# It is used to disable heavy subsystems (Vision subprocess, etc.) by default.
RUNTIME_IS_MOCK = False


class _MockBits:
    """在开发机/无外设时用于让模块可导入的 Mock Bits。"""

    def GetYaw(self):
        return 0

    def SetMotor(self, *args, **kwargs):
        return None

    def SetWheelSpeed(self, *args, **kwargs):
        return None

    def VirtualPorts(self, *args, **kwargs):
        return None

    def SetIO(self, *args, **kwargs):
        return None

    def GetKey(self):
        return 0

def _init_runtime(*, allow_mock: bool = True, auto_start_lidar: bool = True):
    """初始化 Bits / chassis / peripheral / lidar 等全局对象。

    目的：
    - 比赛机：按 Settings 初始化真实硬件。
    - 开发机：允许用 Mock 初始化，避免 import 直接失败。
    """

    global Bits, lidar, chassis, compass, peripheral, key, Odometer, Morse, referee

    def _use_mock(reason: str) -> None:
        """Switch module-level runtime objects to mock implementations."""

        global Bits, lidar, chassis, compass, peripheral, key, Odometer, referee, Morse, RUNTIME_IS_MOCK

        logger.warning(f"Using MockBits: {reason}")
        RUNTIME_IS_MOCK = True
        Bits = _MockBits()
        lidar = Lidar(Bits.GetYaw, auto_start=False)
        chassis = Car(Bits.SetWheelSpeed, Bits.GetYaw)
        compass = Bits.GetYaw
        peripheral = Peripherals(Bits.VirtualPorts)
        key = Key(Bits.GetKey)
        referee = None
        Morse = None
        Odometer = None
        return

    # If running on dev machine, allow user to force mock via env
    if os.environ.get("ROBOCUPX_USE_MOCK", "").lower() in {"1", "true", "yes"}:
        _use_mock("ROBOCUPX_USE_MOCK=1")
        return

    try:
        # Simulator runtime: UDP bridge to Unity (SoccerSimulatorForXels)
        # Enable via config: Settings.RoboInfo.Bit == "SIM"
        # Or env override: ROBOCUPX_BITS=SIM
        bits_override = os.environ.get("ROBOCUPX_BITS", "").strip().upper()
        if bits_override:
            try:
                cfg.RoboInfo.Bit = bits_override
            except Exception:
                pass

        if str(cfg.RoboInfo.Bit).upper() == "SIM":
            from utils.XeleratorSimBits import XeleratorSimBits
            from utils.XeleratorSimBits import Lidar as SimLidar
            from vision_utils.XeleratorVision import XeleratorVision

            # Prefer config-driven settings (ReasonData/data/config.json -> Settings.Sim.*)
            # Env vars remain supported as overrides for quick debugging.
            sim_cfg = getattr(cfg, "Sim", None)
            unity_ip = os.environ.get("XEL_UNITY_IP", getattr(sim_cfg, "UnityIP", "127.0.0.1"))

            # robot id: env override -> Settings.Sim.RobotId -> Settings.RoboInfo.number
            robot_id = int(
                os.environ.get(
                    "XEL_ROBOT_ID",
                    str(getattr(sim_cfg, "RobotId", getattr(cfg.RoboInfo, "number", 1))),
                )
            )

            # Multi-robot port mapping:
            # per_port mode: each robot has dedicated cmd/status ports.
            # Default mapping: base + (robot_id-1)*stride.
            multi_mode = str(getattr(sim_cfg, "MultiRobotMode", "per_port")).lower()
            if multi_mode == "per_port":
                cmd_base = int(getattr(sim_cfg, "CmdPortBase", 20001))
                st_base = int(getattr(sim_cfg, "StatusPortBase", 20002))
                stride = int(getattr(sim_cfg, "PortStride", 10))
                default_cmd_port = cmd_base + (max(robot_id, 1) - 1) * stride
                default_status_port = st_base + (max(robot_id, 1) - 1) * stride
            else:
                # single-port / future hub mode
                default_cmd_port = int(getattr(sim_cfg, "CmdPort", 20001))
                default_status_port = int(getattr(sim_cfg, "StatusPort", 20002))

            cmd_port = int(os.environ.get("XEL_CMD_PORT", str(default_cmd_port)))
            status_port = int(os.environ.get("XEL_STATUS_PORT", str(default_status_port)))

            # Simulator tuning (prefer config Debug section, allow env override in CI/dev)
            try:
                # New SIM config lives under Settings.Sim.*.
                # Keep Debug.* fallback for older configs so existing deployments still work.
                sim_twist_from_wheels = bool(
                    getattr(sim_cfg, "TwistFromWheels", getattr(cfg.Debug, "SimTwistFromWheels", True))
                )
                sim_wheel_to_v = float(getattr(sim_cfg, "WheelToV", getattr(cfg.Debug, "SimWheelToV", 0.008)))
                sim_wheel_to_wz = float(getattr(sim_cfg, "WheelToWz", getattr(cfg.Debug, "SimWheelToWz", 0.02)))
                sim_yaw_offset_deg = float(getattr(sim_cfg, "YawOffsetDeg", getattr(cfg.Debug, "SimYawOffsetDeg", 0.0)))
                sim_yaw_zero_at_start = bool(
                    getattr(sim_cfg, "YawZeroAtStart", getattr(cfg.Debug, "SimYawZeroAtStart", False))
                )
                sim_kick_strength = float(getattr(sim_cfg, "KickStrength", 1.0))
                sim_dribble_scale = float(getattr(sim_cfg, "DribbleStrengthScale", 1.0))
                sim_planar_move_scale = float(getattr(sim_cfg, "PlanarMoveSpeedScale", 1.0))
            except Exception:
                sim_twist_from_wheels = True
                sim_wheel_to_v = 0.008
                sim_wheel_to_wz = 0.02
                sim_yaw_offset_deg = 0.0
                sim_yaw_zero_at_start = False
                sim_kick_strength = 1.0
                sim_dribble_scale = 1.0
                sim_planar_move_scale = 1.0

            # Env overrides (optional)
            if os.environ.get("XEL_TWIST_FROM_WHEELS") is None:
                os.environ["XEL_TWIST_FROM_WHEELS"] = "1" if sim_twist_from_wheels else "0"
            os.environ.setdefault("XEL_WHEEL_TO_V", str(sim_wheel_to_v))
            os.environ.setdefault("XEL_WHEEL_TO_WZ", str(sim_wheel_to_wz))
            os.environ.setdefault("XEL_YAW_OFFSET_DEG", str(sim_yaw_offset_deg))
            os.environ.setdefault("XEL_YAW_ZERO_AT_START", "1" if sim_yaw_zero_at_start else "0")
            os.environ.setdefault("XEL_SIM_KICK_STRENGTH", str(sim_kick_strength))
            os.environ.setdefault("XEL_SIM_DRIBBLE_SCALE", str(sim_dribble_scale))
            os.environ.setdefault("XEL_SIM_PLANAR_MOVE_SCALE", str(sim_planar_move_scale))

            Bits = XeleratorSimBits(unity_ip=unity_ip, cmd_port=cmd_port, status_port=status_port, robot_id=robot_id)
            # Simulator lidar: compute wall distances from pose
            lidar = SimLidar(
                Bits.GetYaw,
                get_pose_callable=Bits.GetPose,
                bounds_long_cm=float(getattr(cfg.Bounds, "Long", 260)),
                bounds_short_cm=float(getattr(cfg.Bounds, "Short", 200)),
                auto_start=True,
            )
            chassis = Car(Bits.SetWheelSpeed, Bits.GetYaw)
            compass = Bits.GetYaw
            peripheral = Peripherals(Bits.VirtualPorts)
            key = Key(Bits.GetKey)
            Odometer = None
            referee = None
            Morse = None
            # Provide a lightweight vision provider in SIM (no subprocess)
            try:
                globals()["vision"] = XeleratorVision(bits=Bits)
            except Exception:
                globals()["vision"] = None
            logger.info(f"Simulator Bits loaded: unity={unity_ip}:{cmd_port}, status_port={status_port}, robot_id={robot_id}")
            return

        if cfg.RoboInfo.Bit == "AB":
            logger.error("Initializing Arisu Bit... Nice to See You Again, Arisu!")
            from utils.ArisuBits import ArisBit

            # avoid starting hardware on dev machines
            dev_path = getattr(Settings.Ports, "ArisuPort", "/dev/arisbit")
            if not os.path.exists(str(dev_path)):
                if allow_mock:
                    _use_mock(f"Arisu device not found: {dev_path}")
                    return

            Bits = ArisBit()
            lidar = Lidar(Bits.GetYaw, auto_start=auto_start_lidar)
            chassis = Car(Bits.SetMotor, Bits.GetYaw, Bits.get_motor_encoder)
            compass = Bits.GetYaw
            peripheral = Peripherals(Bits.SetIO)
            key = Key(Bits.GetKey)
            Odometer = None
            referee = None
            Morse = None
            logger.info("Arisu Bit loaded.")
            return

        if cfg.RoboInfo.Bit == "IL":
            from utils.IceLoongBits import IceLoongBits

            dev_path = getattr(Settings.Ports, "IceLoongPort", "/dev/iceloong")
            if not os.path.exists(str(dev_path)):
                if allow_mock:
                    _use_mock(f"IceLoong device not found: {dev_path}")
                    return

            Bits = IceLoongBits(port=dev_path)
            lidar = Lidar(Bits.GetYaw, auto_start=auto_start_lidar)
            peripheral = Peripherals(Bits.VirtualPorts, Bits.GetRefereeUnlockPin)  # TODO
            chassis = Car(Bits.SetWheelSpeed, Bits.GetYaw, Bits.GetRefereeUnlockPin)
            compass = Bits.GetYaw
            key = Bits.GetKey
            referee = Bits.GetRefereeUnlockPin
            Morse = Bits.MosePrint
            Odometer = None  # TODO add odometer support
            logger.info("IceLoongBits Bit loaded.")
            return

        raise ImportError(f"Unknown Bits model: {cfg.RoboInfo.Bit}")

    except Exception as e:
        if not allow_mock:
            logger.error(f"Failed to init runtime: {e}")
            raise

        logger.warning(f"Init runtime failed, fallback to MockBits: {e}")
        _use_mock(str(e))


# 默认初始化：开发机可导入（allow_mock=True），且不自动启动 ROS2 lidar（避免无 ROS 环境卡死）
_init_runtime(allow_mock=False, auto_start_lidar=True)

Beta_Func = False

# Thread start flags (avoid NameError)
SendstatusThreadFuncStarted = False
PeerstatusThreadFuncStarted = False

class Positions:
    def __init__(self):
        # Values for Setup Position System
        self.LidarPos = [0xfff,0xfff,0xfff]  # legacy name
        self.ballPos = [0xfff,0xfff]         # legacy name
        self.BallDistance = 0xfff            # legacy name
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
        self.vision_data = {"ball_pos": [0, 0], "chassis_pos": [], "blue_goal": None, "yellow_goal": None}
        self.vision_process = None
        self.vision_data_stop_event = threading.Event()
        self.get_vision_data_process: threading.Thread | None = None
        self._vision_started = False
        # Vision should be opt-in on dev/no-hardware runtime; otherwise strategy loops may spawn heavy processes.
        pan_P = 2.3
        pan_I = 0.1
        pan_D = 0.02
        self.ResetFlag = 0
        self.Position_X_PID = simple_pid.PID(pan_P, pan_I, pan_D, setpoint=0,output_limits=(-300,300),auto_mode=True)
        self.Position_Y_PID = simple_pid.PID(pan_P, pan_I, pan_D, setpoint=0,output_limits=(-300,300),auto_mode=True)
        enable_vision_env = os.environ.get("ROBOCUPX_ENABLE_VISION", "").lower() in {"1", "true", "yes"}
        self._vision_enabled = True
        if RUNTIME_IS_MOCK or (os.environ.get("ROBOCUPX_USE_MOCK", "").lower() in {"1", "true", "yes"}):
            self._vision_enabled = enable_vision_env

    def StartVision(self) -> bool:
        """Start UDP receiver + Vision subprocess lazily.

        Importing LunaPre should not start external processes automatically.
        Call this when you really need vision data.
        """

        # Simulator fast-path: use lightweight vision adapter (no socket/subprocess)
        v = globals().get("vision", None)
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
                # if we can receive packets, reset restart counter
                restart_count = 0

            except (socket.timeout, TimeoutError):
                if not self._vision_enabled:
                    # vision explicitly disabled
                    self.vision_data = {"ball_pos": [0, 0], "chassis_pos": [], "blue_goal": None, "yellow_goal": None}
                    time.sleep(0.05)
                    continue

                # normal timeout: no vision UDP packets this cycle.
                # Make sure vision subprocess is running.
                if self.vision_process is None or self.vision_process.poll() is not None:
                    # If it keeps dying, stop respawning (prevents resource_tracker leaked semaphores).
                    if restart_count >= max_restart:
                        self._vision_enabled = False
                        logger.warning("Vision disabled: subprocess crashed too many times (max_restart=%s).", max_restart)
                        continue

                    now = time.time()
                    if (now - last_restart_t) < restart_backoff_s:
                        time.sleep(restart_backoff_s)
                    last_restart_t = now
                    project_root = os.path.dirname(os.path.dirname(__file__))
                    self.vision_process = subprocess.Popen([sys.executable, "src/Vision.py"], cwd=project_root)
                    logger.info("Started vision server process")
                    restart_count += 1

                self.vision_data = {"ball_pos": [0, 0], "chassis_pos": [], "blue_goal": None, "yellow_goal": None}

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
        v = globals().get("vision", None)
        if v is not None:
            try:
                return v.get_vision_data()
            except Exception:
                return {"ball_pos": [0, 0], "chassis_pos": [], "blue_goal": None, "yellow_goal": None}

        # ensure vision thread is running
        self.StartVision()
        return self.vision_data

    def _get_ball_pos(self):
        v = globals().get("vision", None)
        if v is not None:
            try:
                return v.get_ball_pos()
            except Exception:
                return [0, 0]

        self.StartVision()
        return self.vision_data["ball_pos"]
    
    def _get_chassis_pos(self):
        v = globals().get("vision", None)
        if v is not None:
            try:
                return v.get_chassis_pos()
            except Exception:
                return []

        self.StartVision()
        return self.vision_data["chassis_pos"]

    def _get_goals_pos(self):
        v = globals().get("vision", None)
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
    
    def Local2Angle(self, AimPos: list) -> int:
        '''
        `非阻塞函数`
        
        AimPos 一个坐标 示例：[0,0]
        '''
        iAimX = AimPos[0]
        iAimY = AimPos[1]
        iLocX, iLocY, _ = self.FieldRobotPos()
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
        return int(iDeltaAngle)

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
        # Simulator fast-path: if Bits can provide relative ball position directly,
        # do not rely on Vision UDP server.
        try:
            if hasattr(Bits, "GetBallPos") and callable(getattr(Bits, "GetBallPos")):
                # NOTE: SIM Bits.GetBallPos() is already the robot-frame relative position in cm
                # (x_forward, y_left). Do NOT subtract pose here.
                self.ballPos = Bits.GetBallPos()
            else:
                # 20251017 already changed X,Y dimension
                self.ballPos = self._get_ball_pos()
        except Exception:
            self.ballPos = self._get_ball_pos()
        # If SIM Bits returns sentinel values, keep them.
        if self.ballPos == [0xDDD, 0xDDD] or self.ballPos == [0xFFF, 0xFFF]:
            self.ballPosOut = self.ballPos
        # elif self.ballPos == [0,0] or abs(self.ballPos[0]) > 95 or abs(self.ballPos[1]) > 150:
        elif self.ballPos == [0,0]:
            self.ballPosOut = [0xfff,0xfff]
        elif abs(self.ballPos[1]-cfg.ExpectedVals.CatchVal[1]) <= cfg.ExpectedVals.ErrorRange/5 and round(abs(self.ballPos[0]-cfg.ExpectedVals.CatchVal[0]),0) <= cfg.ExpectedVals.ErrorRange/20:
            self.ballPosOut = [0xddd,0xddd]
        else:
            self.ballPosOut = self.ballPos
        if Settings.Debug.FullLog:
            logger.debug("[Relative] Ball Position: %s"%self.ballPosOut)
        return self.ballPosOut

    def DirectDistance(self):
        '''
        `非阻塞函数`
        Get the [Direct] Distance From the bound to the Bound, unit mm [Forward,Left,Backward,Right]

        获取四面墙的距离 单位 mm [Forward,Left,Backward,Right]

        #### Args:

        #### Returns:
            RobotPosition: [0,0]
        #### Note:
            `0xfff` stand for `NoPosition`
        '''
        self.BoundsDistance = lidar.GetDists()
        # disabled lidar returns [0xfff,...]
        if self.BoundsDistance == [0xfff,0xfff,0xfff,0xfff]:
            return [0xfff,0xfff,0xfff,0xfff]
        if self.BoundsDistance == [0, 0, 0, 0]:
            self.WarnedLidarCount +=1
            logger.warning("Lidar Not Started! Retry for %s time in %s second"%(self.WarnedLidarCount,self.WaitTime))
            time.sleep(self.WaitTime) # wait 1 second for starting lidar
            if self.WarnedLidarCount >= cfg.ExpectedVals.MaxWarnCount:
                a = subprocess.run(['sudo fuser -k /dev/sllidar'], shell=True, capture_output=True, text=True)
                logger.error(f"No Lidar Data Recieved! Please Check Lidar Modules! Auto-fixing failed with {a.returncode} after udevadm trigger. Stopping chassis to prevent damage.")
                lidar.stop()
                raise RuntimeError("No Lidar Data Recieved! Please Check Lidar Modules!")
            return [0xfff,0xfff,0xfff,0xfff]
        else:
            if self.WarnedLidarCount:
                self.WarnedLidarCount = 0
            else:
                pass
            if Settings.Debug.FullLog:
                logger.success("Lidar System Started!Read [Direct] Robot Distance: %s"%self.BoundsDistance)
        return self.BoundsDistance

    def FieldRobotPos(self, lower: bool | None = False):
        '''
        `非阻塞函数`

        Get the [Absolute] Position of the Robot

        获取[机器人几何中心]相对于[场地几何中心]的位置 [x_forward,y_left,yaw]

        #### Args:
            lower: `bool`, choose to using lower data

        #### Returns:
            RobotPosition: [0,0]
        #### Note:
            `0xfff` stand for `No Position`
        '''
        # Simulator fast-path: if Bits can provide pose, use it.
        try:
            if hasattr(Bits, "GetPose") and callable(getattr(Bits, "GetPose")):
                x, y, yaw = Bits.GetPose()
                # keep same semantics: [x_forward, y_left, yaw_deg]
                # IMPORTANT: Bits.GetPose() already returns field-center based coordinates (cm)
                # after applying Unity origin shift. Do NOT add extra offsets here, otherwise
                # strategy logic (RestrictedArea/CheckPoints/etc.) will be inconsistent.
                self.LidarPos = [round(x, 2), round(y, 2), float(yaw)]
                return self.LidarPos
        except Exception:
            pass

        _distance = self.DirectDistance()
        if _distance == [0xfff,0xfff,0xfff,0xfff]:
            chassis.stop()
            return [0xfff,0xfff,0xfff]
        if (_distance[0]+_distance[2]) < (cfg.Bounds.Long)*self.LidarScale*self.BoundsScale:
            if _distance[0] > _distance[2]:
                Y = cfg.Bounds.Long*self.LidarScale/2 - _distance[0]
            else:
                Y = _distance[2] - cfg.Bounds.Long*self.LidarScale/2
        else:
            Y = ((cfg.Bounds.Long*self.LidarScale/2 - _distance[0]) + (_distance[2] - cfg.Bounds.Long*self.LidarScale/2))/2
        if _distance[1]+_distance[3] < (cfg.Bounds.Short - 50)*self.LidarScale*self.BoundsScale:
            if _distance[1] > _distance[3]:
                X = -(cfg.Bounds.Short*self.LidarScale/2 - _distance[1])
            else:
                X = -(_distance[3] - cfg.Bounds.Short*self.LidarScale/2)
        else:
            X = -((cfg.Bounds.Short*self.LidarScale/2 - _distance[1]) + (_distance[3] - cfg.Bounds.Short*self.LidarScale/2))/2
        self.LidarPos = [Y/10,X/10,compass()]
        logger.info("[Absolute] Robot Position: %s"%self.LidarPos)
        # 20251216 already changed X_forward,Y_left dimension  [tested]
        try:
            if Odometer:# if use odometer datas
                # TODO finish lower Positions
                pass
        except:
            pass
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

    def FieldGoalsPos(self):
        """获取球门在场地坐标系下的位置。"""
        goals = self._get_goals_pos()
        if not goals:
            return {"blue_goal": None, "yellow_goal": None}

        self_pos = self.FieldRobotPos()
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

        ##### retrun a list of the absloute position of all chassis
        #### Returns:
            OutputDistanceList: [c1,c2]
                c1: 
                    chassis X Position
                    chassis Y Position
                    chassis height
                    chassis weidth
        '''
        # TODO 20260503 This method is to be tested.
        ChassisRawList = self._get_chassis_pos()
        SelfX, SelfY, SelfZ = self.FieldRobotPos()
        OutputDistanceList = []
        for c in ChassisRawList:
            cDistance = math.sqrt(c[0]**2 + c[1]**2)
            if c[1] == 0:
                ChassisRltAngle = 0
            try:
                ChassisRltAngle =  -int(math.degrees(math.atan2(c[0],c[1])) - 90)
            except ZeroDivisionError:
                ChassisRltAngle =  0
            ChassisAngleAngle = ChassisRltAngle + SelfZ
            if ChassisAngleAngle > 360:
                ChassisAbsAngle = ChassisAngleAngle - 360
            else:
                ChassisAbsAngle = ChassisAngleAngle
            AbsChassisCache = [
                cDistance * math.sin(math.radians(ChassisAbsAngle)) + SelfY,
                cDistance * math.cos(math.radians(ChassisAbsAngle)) + SelfX,
                c[2],
                c[3]
                ]
            OutputDistanceList.append(AbsChassisCache)
        return OutputDistanceList
    
    # ********* UPPER FUNCTIONS ********
    def RobotBallAngle(self):
        # TODO 20260503 This method is to be tested.
        '''
        `非阻塞函数`

        获取[球]相对于[机器几何中心]的角度

        retrun a relative angle of the ball
        ### Returns:
            ballangle: int | relative ball angle
        '''
        ballX, ballY = self._get_ball_pos()
        try:
            relBallAngle = int(math.degrees(math.atan2(ballY,ballX)))
        except ZeroDivisionError:
            relBallAngle = 0
        return relBallAngle
    
    def FieldBallAngle(self):
        # TODO 20260503 This method is to be tested.
        '''
        `非阻塞函数`

        获取[球]相对于[场地几何中心]的角度

        retrun a absolute angle of the ball
        #### Returns:
            BallAngle: int | absolute ball angle
        '''
        self.BallAngle = (self.RobotBallAngle() + compass())%360
        return self.BallAngle

    def RobotBallDistance(self):
        # TODO 20260503 This method is to be tested.
        '''
        `非阻塞函数`

        获取[球]相对于[机器几何中心]的距离

        Get the [Absolute] Distance from Robot to Ball
        #### Returns:
            BallDistance: int | absolute Distance from Robot to Ball

        #### Note:
            `0xfff` stand for `NoBall`
            `0xddd` stand for `CatchBall`
        '''
        BallX, BallY = self.RobotBallPos()
        if [BallX, BallY] == [0xfff,0xfff]:
            self.BallDistance = 0xfff
        elif [BallX, BallY] == [0xddd,0xddd]:
            self.BallDistance = 0xddd
        else:
            # RobotBallPos is *relative* to robot center in robot frame (cm), so distance
            # is just the norm of that vector (do NOT mix with field pose).
            self.BallDistance = math.sqrt((BallX) ** 2 + (BallY) ** 2)
        if self.FullLog:
            logger.debug("[Absolute] Ball Distance: %s" % self.BallDistance)
        return self.BallDistance

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
        ballX, ballY = self.RobotBallPos()
        if [ballX,ballY] == [0xfff,0xfff]:
            return [0xfff,0xfff]
        if [ballX,ballY] == [0xddd,0xddd]:
            return [0xddd,0xddd]
        else:
            SelfX, SelfY, SelfZ = self.FieldRobotPos()
            # Convert robot-frame relative (x_forward, y_left) into field-frame delta using yaw.
            # yaw=0 => robot forward aligns with +x_forward.
            th = math.radians(float(SelfZ))
            c = math.cos(th)
            s = math.sin(th)
            dx = float(ballX) * c - float(ballY) * s
            dy = float(ballX) * s + float(ballY) * c
            return [SelfX + dx, SelfY + dy]

    def RobotChassisAngle(self):
        # TODO 20260503 This method is to be tested.
        '''
        `非阻塞函数`

        获取[球]相对于[机器人几何中心]的角度列表
        ##### retrun a list of the relative angle from the robot to the chassis
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

    def Pos2Pos(self, AimPos: list, speed_gain: int | None = None) -> bool:
        '''
        `非阻塞函数`

        ### Args:
            iFacingAngle 移动时面对的方向 0~360
            lAimPos 目标坐标位置 如[0,0] 距离越近速度越小
        ### Returns:
            True 到达目标值
            False 未到达目标值
        '''
        iLocX, iLocY, iLocZ = self.FieldRobotPos()
        if (iLocX,iLocY,iLocZ) == (0xfff,0xfff,0xfff):
            chassis.stop()
            Morse("No Position Data!")
            return False
        iAimX,iAimY,iAimZ = AimPos
        iErrorX = (iAimX) - (iLocX)
        iErrorY = (iAimY) - (iLocY)
        iErrorZ = (iAimZ) - (iLocZ)
        if Settings.Debug.Beta:
            if self.ResetFlag > 1000:
                self.Position_X_PID.reset()
                self.Position_Y_PID.reset()
                self.ResetFlag = 0
            else:
                self.ResetFlag += 1
            self.Position_X_PID.setpoint = iAimX
            self.Position_Y_PID.setpoint = iAimY
            iDeltaX = self.Position_X_PID(iLocX)
            iDeltaY = self.Position_Y_PID(iLocY)
            if Settings.Debug.FullLog:
                logger.debug(f"PID setpoints: X={self.Position_X_PID.setpoint}, Y={self.Position_Y_PID.setpoint}")
                logger.debug(f"PID output: iDeltaX={iDeltaX}, iDeltaY={iDeltaY}")
        else:
            iDeltaX = iErrorX
            iDeltaY = iErrorY
        iErrorRange = Settings.ExpectedVals.ErrorRange
        iMovedAngle = (int(math.degrees(math.atan2(iErrorY,iErrorX))))%360
        if iErrorRange/2 > abs(iErrorX) and iErrorRange/2 > abs(iErrorY) and abs(iErrorRange) > iErrorZ:
            chassis.stop()
            self.ResetFlag = 0
            self.Position_X_PID.reset()
            self.Position_Y_PID.reset()
            return True
        elif iErrorRange/2 > abs(iErrorX) and iErrorRange/2 > abs(iErrorY) and not abs(iErrorRange) > abs(iErrorZ):
            self.ResetFlag = 0
            self.Position_X_PID.reset()
            self.Position_Y_PID.reset()
            chassis.FieldTurn(iAimZ)
            logger.debug("Position reached, adjusting orientation to %s degrees." % iAimZ)
        else:
            if speed_gain:
                chassis.FieldMoveVector(iDeltaX*speed_gain,iDeltaY*speed_gain,iAimZ)
            else:
                chassis.FieldMoveVector(iDeltaX,iDeltaY,iAimZ)
            return False

    def Move2Path(self, Posistions: list, iWaitMs: float):
        '''
        `阻塞函数`

        遍历`Posistions`里的每一个点，直到到达最后一个点完成后返回`True`
        
        ### Args:
            Posistions 移动路径列表
            iWaitMs 每次移动后等待时间
        '''
        for i in Posistions:
            while (1):
                if self.Pos2Pos(i):
                    time.sleep(iWaitMs)
                    if i == Posistions[-1]:
                        return True
                    else:
                        break
                else:
                    pass

    def Cover2Start(self):
        '''
        `阻塞函数`

        直到前方被遮挡后才会返回True

        '''
        while 1:
            if 100 < lidar.GetDists()[0] <= 500:
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
        try:
            return Bits.referee_key
        except Exception:
            return False

    def RestrictedArea(self):
        Distance = self.DirectDistance()

        # Simulator / no-lidar fallback: if we don't have valid wall distances,
        # do not run boundary-avoidance logic (it would generate bogus commands).
        if Distance == [0xfff, 0xfff, 0xfff, 0xfff]:
            return True

        if Distance[0] == 0:
            return False
        else:
            x_max = 90
            x_min = -x_max
            y_max = 70
            y_min = -y_max
            rsct_width = 25
            rsct_height = 80
            robo_pos = UnitedPosition.FieldRobotPos()
            robo_abs_pos = [abs(robo_pos[0]), abs(robo_pos[1])]
            rst = True
            if (robo_pos[0] > x_max or robo_pos[0] < x_min
                or (robo_abs_pos[0] > x_max - rsct_width
                    and robo_abs_pos[1] < rsct_height / 2 + 10)):
                UnitedPosition.Pos2Pos([0, robo_pos[1], robo_pos[2]])
                rst = False
            if (robo_pos[1] > y_max or robo_pos[1] < y_min):
                UnitedPosition.Pos2Pos([robo_pos[0], 0, robo_pos[2]])
                rst = False
            return rst

    def Stop(self):
        '''
        ### Function for stopping the vision data thread
        '''
        chassis.stop()
        peripheral.Dribble(False)
        lidar.stop()
        chassis.stop()
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

        if self.vision_process is not None:
            try:
                self.vision_process.terminate()
                self.vision_process.wait(timeout=1.0)
            except Exception:
                pass

        # best-effort join
        try:
            if self.get_vision_data_process is not None:
                self.get_vision_data_process.join(timeout=0.5)
        except Exception:
            pass

UnitedPosition = Positions()

class Communication:
    """蓝牙/Wi-Fi 队内通信封装。

    协议参考 `蓝牙数据协议.csv`：
    - 帧头: 0x23
    - 帧尾: 0x42
    - 字段顺序:
        [帧头, 识别ID, 角色, 状态, x+, y+, x-, y-, 备用, 帧尾]

    说明：
    - 这里保留了一个通用实现，既可以用于串口 BLE，也可以用于 Wi-Fi
      方案中的二次封装。
    - 对于坐标字段，默认采用“正负拆分”的方式：
        signed = positive - negative
      其中每个分量都会被限制到 0~0xFF。
    """

    FRAME_HEAD = 0x23
    FRAME_TAIL = 0x42
    FRAME_LENGTH = 10

    # 角色编码（对齐蓝牙数据协议.csv）
    ROLE_OFFENCE_YELLOW = 0x01
    ROLE_DEFENCE_YELLOW = 0x00
    ROLE_DEFENCE_BLUE = 0x02
    ROLE_OFFENCE_BLUE = 0x03
    ROLE_BAN = 0x04

    # 状态编码
    STATUS_CATCH_BALL = 0x00
    STATUS_CHASING = 0x01
    STATUS_AVOIDING = 0xEE
    STATUS_NO_BALL = 0xFF

    def __init__(self):
        if cfg.Transimission.Method == "WIFI":
            from utils.ReasonBeacon import MisakaNetwork
            self.transimission = MisakaNetwork()
        elif cfg.Transimission.Method == "BLE":
            import serial
            self.transimission = serial.Serial(getattr(Settings.Ports, "BLEPort", "/dev/ble"), 115200, timeout=0.1)
        else:
            raise RuntimeError("None Transmission Method Selected!")

        self.BallFlag = [0, 0]
        self.PeerPosition = [0xFFF, 0xFFF]
        self.FullLog = cfg.Debug.FullLog
        self.RobotID = int(getattr(cfg.RoboInfo, "number", 1))
        self.Role = self._default_role_from_config()
        self.Status = self.STATUS_NO_BALL
        self.LastFrame = None
        self.LastPeerFrame = None

    def _default_role_from_config(self) -> int:
        robo_type = str(getattr(cfg.RoboInfo, "type", "OP")).upper()
        color = str(getattr(cfg.RoboInfo, "Color", getattr(cfg.RoboInfo, "color", "Y"))).upper()

        if robo_type in {"DP", "DEFENCE", "DEFENSE"}:
            return self.ROLE_DEFENCE_BLUE if color.startswith("B") else self.ROLE_DEFENCE_YELLOW
        return self.ROLE_OFFENCE_BLUE if color.startswith("B") else self.ROLE_OFFENCE_YELLOW

    @staticmethod
    def _clamp_byte(value) -> int:
        try:
            value = int(round(float(value)))
        except Exception:
            value = 0
        return max(0, min(0xFF, value))

    @staticmethod
    def _split_signed_xy(x, y) -> tuple[int, int, int, int]:
        """将带符号坐标拆成正负两路字节。"""
        try:
            x = float(x)
        except Exception:
            x = 0.0
        try:
            y = float(y)
        except Exception:
            y = 0.0

        x_pos = Communication._clamp_byte(max(x, 0.0))
        y_pos = Communication._clamp_byte(max(y, 0.0))
        x_neg = Communication._clamp_byte(max(-x, 0.0))
        y_neg = Communication._clamp_byte(max(-y, 0.0))
        return x_pos, y_pos, x_neg, y_neg

    @staticmethod
    def _merge_signed_xy(x_pos, y_pos, x_neg, y_neg) -> list[int]:
        return [int(x_pos) - int(x_neg), int(y_pos) - int(y_neg)]

    def BuildFrame(self, identity_id: int | None = None, role: int | None = None, status: int | None = None,
                   x_pos: int = 0, y_pos: int = 0, x_neg: int = 0, y_neg: int = 0, reserve: int = 0) -> bytes:
        """构造一帧通信报文。"""
        frame = bytes([
            self.FRAME_HEAD,
            self._clamp_byte(self.RobotID if identity_id is None else identity_id),
            self._clamp_byte(self.Role if role is None else role),
            self._clamp_byte(self.Status if status is None else status),
            self._clamp_byte(x_pos),
            self._clamp_byte(y_pos),
            self._clamp_byte(x_neg),
            self._clamp_byte(y_neg),
            self._clamp_byte(reserve),
            self.FRAME_TAIL,
        ])
        self.LastFrame = frame
        return frame

    def ParseFrame(self, payload: bytes | bytearray | list[int]):
        """解析协议帧，返回字典；非法帧返回 None。"""
        if payload is None:
            return None
        if isinstance(payload, list):
            payload = bytes(payload)
        if not isinstance(payload, (bytes, bytearray)):
            return None
        if len(payload) != self.FRAME_LENGTH:
            return None
        if payload[0] != self.FRAME_HEAD or payload[-1] != self.FRAME_TAIL:
            return None

        identity_id = int(payload[1])
        role = int(payload[2])
        status = int(payload[3])
        x_pos, y_pos, x_neg, y_neg, reserve = map(int, payload[4:9])
        signed_xy = self._merge_signed_xy(x_pos, y_pos, x_neg, y_neg)
        data = {
            "id": identity_id,
            "role": role,
            "status": status,
            "x_pos": x_pos,
            "y_pos": y_pos,
            "x_neg": x_neg,
            "y_neg": y_neg,
            "reserve": reserve,
            "signed_xy": signed_xy,
        }
        self.LastPeerFrame = data
        self.PeerPosition = signed_xy
        self.BallFlag = [status, reserve]
        return data

    def UpdateState(self, *, role: int | None = None, status: int | None = None,
                    ball_flag: list[int] | None = None, peer_position: list[int] | None = None):
        """更新本机待发送状态。"""
        if role is not None:
            self.Role = self._clamp_byte(role)
        if status is not None:
            self.Status = self._clamp_byte(status)
        if ball_flag is not None and len(ball_flag) >= 2:
            self.BallFlag = [self._clamp_byte(ball_flag[0]), self._clamp_byte(ball_flag[1])]
        if peer_position is not None and len(peer_position) >= 2:
            self.PeerPosition = [int(peer_position[0]), int(peer_position[1])]

    def PackSelfStatus(self, *, identity_id: int | None = None, role: int | None = None,
                       status: int | None = None, position: list[int] | tuple[int, int] | None = None,
                       reserve: int = 0) -> bytes:
        """根据当前状态和位置打包一帧。"""
        if position is None:
            position = self.PeerPosition
        if isinstance(position, (list, tuple)) and len(position) >= 2:
            x_pos, y_pos, x_neg, y_neg = self._split_signed_xy(position[0], position[1])
        else:
            x_pos, y_pos, x_neg, y_neg = 0, 0, 0, 0
        return self.BuildFrame(
            identity_id=identity_id,
            role=role,
            status=status,
            x_pos=x_pos,
            y_pos=y_pos,
            x_neg=x_neg,
            y_neg=y_neg,
            reserve=reserve,
        )

    def Send(self, payload: bytes | bytearray | list[int] | None = None) -> bool:
        """发送原始帧或当前状态帧。"""
        if payload is None:
            payload = self.PackSelfStatus()

        try:
            if hasattr(self.transimission, "send") and callable(getattr(self.transimission, "send")):
                self.transimission.send(payload)
            elif hasattr(self.transimission, "sendall") and callable(getattr(self.transimission, "sendall")):
                self.transimission.sendall(payload)
            elif hasattr(self.transimission, "write") and callable(getattr(self.transimission, "write")):
                self.transimission.write(payload)
            else:
                raise AttributeError("transimission object has no send/write interface")
            return True
        except Exception as e:
            if self.FullLog:
                logger.warning(f"Communication send failed: {e}")
            return False

    def Receive(self):
        """尝试接收一帧并解析。"""
        try:
            raw = None
            if hasattr(self.transimission, "recv") and callable(getattr(self.transimission, "recv")):
                raw = self.transimission.recv(self.FRAME_LENGTH)
            elif hasattr(self.transimission, "read") and callable(getattr(self.transimission, "read")):
                raw = self.transimission.read(self.FRAME_LENGTH)
            elif hasattr(self.transimission, "receive") and callable(getattr(self.transimission, "receive")):
                raw = self.transimission.receive()
            else:
                raise AttributeError("transimission object has no recv/read/receive interface")

            if not raw:
                return None

            if isinstance(raw, tuple):
                # 某些实现会返回 (addr, data)
                raw = raw[-1]

            if isinstance(raw, list):
                raw = bytes(raw)

            if isinstance(raw, str):
                raw = raw.encode("latin1", errors="ignore")

            # 兼容一次读到多帧的场景，尽量从中截取第一帧合法数据。
            if isinstance(raw, (bytes, bytearray)) and len(raw) >= self.FRAME_LENGTH:
                for idx in range(0, len(raw) - self.FRAME_LENGTH + 1):
                    chunk = raw[idx: idx + self.FRAME_LENGTH]
                    parsed = self.ParseFrame(chunk)
                    if parsed is not None:
                        return parsed
            return self.ParseFrame(raw)
        except Exception as e:
            if self.FullLog:
                logger.warning(f"Communication receive failed: {e}")
            return None

    def SendStatus(self, *, status: int | None = None, role: int | None = None,
                   position: list[int] | tuple[int, int] | None = None, reserve: int = 0) -> bool:
        """发送当前机器人状态。"""
        payload = self.PackSelfStatus(role=role, status=status, position=position, reserve=reserve)
        return self.Send(payload)

    def SendBallFlag(self, ball_flag: list[int] | None = None, *, reserve: int = 0) -> bool:
        """便捷接口：发送球状态（兼容旧变量命名）。"""
        if ball_flag is not None and len(ball_flag) >= 2:
            self.BallFlag = [self._clamp_byte(ball_flag[0]), self._clamp_byte(ball_flag[1])]
        return self.SendStatus(status=self.Status, position=self.PeerPosition, reserve=reserve)

    def Close(self):
        """关闭通信句柄。"""
        try:
            if hasattr(self.transimission, "close") and callable(getattr(self.transimission, "close")):
                self.transimission.close()
        except Exception as e:
            if self.FullLog:
                logger.warning(f"Communication close failed: {e}")

    # Backward-compatible aliases
    def SendMessage(self, *args, **kwargs):
        return self.Send(*args, **kwargs)

    def RecvMessage(self):
        return self.Receive()

    def SetBallFlag(self, ball_flag):
        self.BallFlag = [self._clamp_byte(ball_flag[0]), self._clamp_byte(ball_flag[1])]
        return self.BallFlag

    def SetPeerPosition(self, peer_position):
        self.PeerPosition = [int(peer_position[0]), int(peer_position[1])]
        return self.PeerPosition
