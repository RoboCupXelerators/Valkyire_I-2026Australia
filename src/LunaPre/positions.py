"""Position, localization, vision, and Pos2Pos facade for LunaPre."""

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

cfg = runtime.cfg

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
        self._pos2pos_cfg = self._load_pos2pos_config()
        self.ResetFlag = 0
        self.Position_X_PID = self._build_pos2pos_pid()
        self.Position_Y_PID = self._build_pos2pos_pid()
        self._vision_enabled = runtime.default_vision_enabled()

    def _load_pos2pos_config(self) -> dict:
        motion_cfg = getattr(cfg, "Motion", None)
        pid_cfg = getattr(motion_cfg, "Pos2PosPID", None)
        return {
            "enabled": bool(getattr(pid_cfg, "Enabled", getattr(cfg.Debug, "Beta", True))),
            "kp": float(getattr(pid_cfg, "Kp", 1.8)),
            "ki": float(getattr(pid_cfg, "Ki", 0.03)),
            "kd": float(getattr(pid_cfg, "Kd", 0.08)),
            "output_limit": float(getattr(pid_cfg, "OutputLimit", 220)),
            "sample_time": float(getattr(pid_cfg, "SampleTime", 0.02)),
            "reset_frames": int(getattr(pid_cfg, "ResetFrames", 300)),
            "arrival_scale": float(getattr(pid_cfg, "ArrivalScale", 0.5)),
            "min_speed_gain": float(getattr(pid_cfg, "MinSpeedGain", 1.0)),
            "max_speed_gain": float(getattr(pid_cfg, "MaxSpeedGain", 6.0)),
            "record_enabled": bool(getattr(cfg.Debug, "RecordPos2Pos", False)),
            "record_dir": str(getattr(cfg.Debug, "Pos2PosRecordDir", "./Records/pos2pos")),
        }

    def _build_pos2pos_pid(self):
        limit = abs(float(self._pos2pos_cfg["output_limit"]))
        return simple_pid.PID(
            self._pos2pos_cfg["kp"],
            self._pos2pos_cfg["ki"],
            self._pos2pos_cfg["kd"],
            setpoint=0,
            output_limits=(-limit, limit),
            sample_time=max(0.0, float(self._pos2pos_cfg["sample_time"])),
            auto_mode=True,
        )

    def RefreshPos2PosPID(self):
        """从配置重新加载 Pos2Pos PID，便于真实车现场调参。"""
        self._pos2pos_cfg = self._load_pos2pos_config()
        self.Position_X_PID = self._build_pos2pos_pid()
        self.Position_Y_PID = self._build_pos2pos_pid()
        self.ResetFlag = 0
        logger.info(
            "Pos2Pos PID refreshed: kp={:.3f} ki={:.3f} kd={:.3f} limit={:.1f} sample_time={:.3f} enabled={}",
            self._pos2pos_cfg["kp"],
            self._pos2pos_cfg["ki"],
            self._pos2pos_cfg["kd"],
            self._pos2pos_cfg["output_limit"],
            self._pos2pos_cfg["sample_time"],
            self._pos2pos_cfg["enabled"],
        )

    def _resolve_speed_gain(self, speed_gain: int | float | None) -> float:
        if speed_gain is None:
            return 1.0
        gain = float(speed_gain)
        gain = max(self._pos2pos_cfg["min_speed_gain"], gain)
        gain = min(self._pos2pos_cfg["max_speed_gain"], gain)
        return gain

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
                    project_root = Path(__file__).resolve().parents[2]
                    vision_script = project_root / "src" / "vision_utils" / "Vision.py"
                    if not vision_script.exists():
                        vision_script = project_root / "src" / "Vision.py"
                    self.vision_process = subprocess.Popen([sys.executable, str(vision_script)], cwd=str(project_root))
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
        v = getattr(runtime, "vision", None)
        if v is not None:
            try:
                return v.get_vision_data()
            except Exception:
                return {"ball_pos": [0, 0], "chassis_pos": [], "blue_goal": None, "yellow_goal": None}

        # ensure vision thread is running
        self.StartVision()
        return self.vision_data

    def _get_ball_pos(self):
        v = getattr(runtime, "vision", None)
        if v is not None:
            try:
                return v.get_ball_pos()
            except Exception:
                return [0, 0]

        self.StartVision()
        return self.vision_data["ball_pos"]
    
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
    
    def Local2Angle(self, AimPos: list) -> int:
        '''
        `非阻塞函数`
        
        AimPos 一个坐标 示例：[0,0]
        '''
        iAimX = AimPos[0]
        iAimY = AimPos[1]
        iLocX, iLocY = self.FieldRobotPosition()
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
        try:
            runtime_ball_pos = runtime.get_runtime_ball_pos()
            if runtime_ball_pos is not None:
                self.ballPos = runtime_ball_pos
            else:
                # 20251017 already changed X,Y dimension
                self.ballPos = self._get_ball_pos()
        except Exception:
            self.ballPos = self._get_ball_pos()
        if self.ballPos == [0xDDD, 0xDDD] or self.ballPos == [0xFFF, 0xFFF]:
            self.ballPosOut = self.ballPos
        # elif self.ballPos == [0,0] or abs(self.ballPos[0]) > 95 or abs(self.ballPos[1]) > 150:
        elif self.ballPos == [0,0]:
            self.ballPosOut = [0xfff,0xfff]
        elif runtime.runtime_ball_state_is_authoritative():
            # Some runtime providers already encode possession state, so the
            # legacy vision geometry heuristic should not override it here.
            self.ballPosOut = self.ballPos
        else:
            catch_val = getattr(cfg.Motion, "CatchVal", [8.5, 0])
            catch_x = catch_val[0] if len(catch_val) > 0 else 8.5
            catch_y = catch_val[1] if len(catch_val) > 1 else 0
            error_range = getattr(cfg.Motion, "ErrorRange", 10)
            x_tol = getattr(cfg.Motion, "CatchXTol", error_range / 20)
            y_tol = getattr(cfg.Motion, "CatchYTol", error_range / 5)
            bbox_caught = False
            try:
                ball_debug = self.vision_data.get("ball_debug") if isinstance(self.vision_data, dict) else None
                bbox = ball_debug.get("bbox") if isinstance(ball_debug, dict) else None
                bbox_size = max(float(bbox[0]), float(bbox[1])) if bbox and len(bbox) >= 2 else 0.0
                bbox_caught = (
                    bbox_size >= getattr(cfg.Motion, "CatchBBoxSize", 90)
                    and 0 <= self.ballPos[0] <= catch_x + getattr(cfg.Motion, "CatchBBoxXMargin", 4)
                    and abs(self.ballPos[1] - catch_y) <= getattr(cfg.Motion, "CatchBBoxYTol", 5)
                )
            except Exception:
                bbox_caught = False
            if (
                abs(self.ballPos[1] - catch_y) <= y_tol
                and round(abs(self.ballPos[0] - catch_x), 0) <= x_tol
            ) or bbox_caught:
                self.ballPosOut = [0xddd,0xddd]
            else:
                self.ballPosOut = self.ballPos
        if Settings.Debug.FullLog:
            logger.debug("[Relative] Ball Position: %s"%self.ballPosOut)
        return self.ballPosOut

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
        if ball_pos == [0xFFF, 0xFFF]:
            return "lost"
        if ball_pos == [0xDDD, 0xDDD]:
            return "held"
        return "free"

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
        self.BoundsDistance = runtime.lidar.GetDists()
        # disabled runtime.lidar returns [0xfff,...]
        if self.BoundsDistance == [0xfff,0xfff,0xfff,0xfff]:
            return [0xfff,0xfff,0xfff,0xfff]
        if self.BoundsDistance == [0, 0, 0, 0]:
            self.WarnedLidarCount +=1
            logger.warning("Lidar Not Started! Retry for %s time in %s second"%(self.WarnedLidarCount,self.WaitTime))
            time.sleep(self.WaitTime) # wait 1 second for starting runtime.lidar
            if self.WarnedLidarCount >= cfg.Motion.ErrorRange/3:
                a = subprocess.run(['sudo fuser -k /dev/sllidar'], shell=True, capture_output=True, text=True)
                logger.error(f"No Lidar Data Recieved! Please Check Lidar Modules! Auto-fixing failed with {a.returncode} after udevadm trigger. Stopping runtime.chassis to prevent damage.")
                runtime.lidar.stop()
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

        Legacy alias for ``FieldRobotPose()``.

        获取[机器人几何中心]相对于[场地几何中心]的姿态 [x_forward,y_left,yaw]

        #### Args:
            lower: `bool`, choose to using lower data

        #### Returns:
            RobotPose: [x, y, yaw]
        #### Note:
            `0xfff` stand for `No Position`
        '''
        # Simulator fast-path: if runtime.Bits can provide pose, use it.
        try:
            if hasattr(runtime.Bits, "GetPose") and callable(getattr(runtime.Bits, "GetPose")):
                x, y, yaw = runtime.Bits.GetPose()
                # keep same semantics: [x_forward, y_left, yaw_deg]
                # IMPORTANT: runtime.Bits.GetPose() already returns field-center based coordinates (cm)
                # after applying Unity origin shift. Do NOT add extra offsets here, otherwise
                # strategy logic (RestrictedArea/CheckPoints/etc.) will be inconsistent.
                self.LidarPos = [round(x, 2), round(y, 2), float(yaw)]
                return self.LidarPos
        except Exception:
            pass

        _distance = self.DirectDistance()
        if _distance == [0xfff,0xfff,0xfff,0xfff]:
            runtime.chassis.stop()
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
        self.LidarPos = [Y/10,X/10,runtime.compass()]
        logger.info("[Field] Robot Pose: %s"%self.LidarPos)
        # 20251216 already changed X_forward,Y_left dimension  [tested]
        try:
            if runtime.Odometer:# if use odometer datas
                # TODO finish lower Positions
                pass
        except:
            pass
        return self.LidarPos

    def FieldRobotPose(self, lower: bool | None = False):
        """Return robot field-frame pose ``[x_forward, y_left, yaw]``.

        Use **Pose** when yaw is included. ``FieldRobotPos()`` is kept for
        compatibility and returns the same value.
        """

        try:
            return self.FieldRobotPos(lower)
        except TypeError:
            return self.FieldRobotPos()

    def FieldRobotPosition(self, lower: bool | None = False):
        """Return robot field-frame position ``[x_forward, y_left]``.

        Use **Position** only for coordinates without heading.
        """

        pose = self.FieldRobotPose(lower)
        if pose == [0xfff, 0xfff, 0xfff] or len(pose) < 2:
            return [0xfff, 0xfff]
        return [pose[0], pose[1]]

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
        self.BallAngle = (self.RobotBallAngle() + runtime.compass())%360
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
        ball_pos = self.RobotBallPos()
        BallX, BallY = ball_pos
        ball_state = self.BallState(ball_pos)
        if ball_state == "lost":
            self.BallDistance = 0xfff
        elif ball_state == "held":
            self.BallDistance = 0xddd
        else:
            # RobotBallPos is *relative* to robot center in robot frame (cm), so distance
            # is just the norm of that vector (do NOT mix with field pose).
            self.BallDistance = math.sqrt((BallX) ** 2 + (BallY) ** 2)
        if self.FullLog:
            logger.debug("[RobotFrame] Ball Distance: %s" % self.BallDistance)
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
        ball_pos = self.RobotBallPos()
        ballX, ballY = ball_pos
        ball_state = self.BallState(ball_pos)
        if ball_state == "lost":
            return [0xfff,0xfff]
        if ball_state == "held":
            return [0xddd,0xddd]
        else:
            SelfX, SelfY, SelfZ = self.FieldRobotPose()
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

    @staticmethod
    def _angle_error_deg(current: float, target: float) -> float:
        """Signed shortest yaw error in degrees."""

        return (float(target) - float(current) + 180.0) % 360.0 - 180.0

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
        iLocX, iLocY, iLocZ = self.FieldRobotPose()
        if (iLocX,iLocY,iLocZ) == (0xfff,0xfff,0xfff):
            runtime.chassis.stop()
            runtime.Morse("No Position Data!")
            return False
        iAimX,iAimY,iAimZ = AimPos
        iAimZ = float(iAimZ) % 360.0
        iErrorX = (iAimX) - (iLocX)
        iErrorY = (iAimY) - (iLocY)
        iErrorZ = self._angle_error_deg(iLocZ, iAimZ)
        pid_enabled = bool(self._pos2pos_cfg.get("enabled", getattr(Settings.Debug, "Beta", True)))
        if pid_enabled:
            if self.ResetFlag > self._pos2pos_cfg["reset_frames"]:
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
        iErrorRange = Settings.Motion.ErrorRange
        iArrivalRange = max(1.0, float(iErrorRange) * float(self._pos2pos_cfg["arrival_scale"]))
        resolved_speed_gain = self._resolve_speed_gain(speed_gain)
        iMovedAngle = (int(math.degrees(math.atan2(iErrorY,iErrorX))))%360
        if iArrivalRange > abs(iErrorX) and iArrivalRange > abs(iErrorY) and abs(iErrorRange) > abs(iErrorZ):
            runtime.chassis.stop()
            self.ResetFlag = 0
            self.Position_X_PID.reset()
            self.Position_Y_PID.reset()
            return True
        elif iArrivalRange > abs(iErrorX) and iArrivalRange > abs(iErrorY) and not abs(iErrorRange) > abs(iErrorZ):
            self.ResetFlag = 0
            self.Position_X_PID.reset()
            self.Position_Y_PID.reset()
            runtime.chassis.FieldTurn(iAimZ)
            logger.debug("Position reached, adjusting orientation to %s degrees." % iAimZ)
        else:
            cmd_x = iDeltaX * resolved_speed_gain
            cmd_y = iDeltaY * resolved_speed_gain
            runtime.chassis.FieldMoveVector(cmd_x, cmd_y, iAimZ)
            return False

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

        point_timeout = float(timeout_s) if timeout_s is not None else float(
            getattr(getattr(cfg, "Motion", object()), "Move2PathPointTimeoutS", 12.0)
        )
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
                    runtime.chassis.stop()
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
            if 100 < runtime.lidar.GetDists()[0] <= 500:
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
            return runtime.Bits.referee_key
        except Exception:
            return False

    def _restricted_area_params(self) -> dict:
        """Return coordinate-space safety limits in field centimeters."""

        field_long = float(getattr(cfg.Bounds, "Long", 260))
        field_short = float(getattr(cfg.Bounds, "Short", 200))
        safety_cfg = getattr(getattr(cfg, "Motion", None), "RestrictedArea", None)

        x_limit = float(getattr(safety_cfg, "XLimit", min(90.0, field_long / 2.0 - 40.0)))
        y_limit = float(getattr(safety_cfg, "YLimit", min(70.0, field_short / 2.0 - 30.0)))
        goal_box_width = float(getattr(safety_cfg, "GoalBoxWidth", 25.0))
        goal_box_y_half = float(getattr(safety_cfg, "GoalBoxYHalf", 50.0))
        clearance = float(getattr(safety_cfg, "Clearance", max(3.0, cfg.Motion.ErrorRange / 2.0)))
        recovery_clearance = float(getattr(safety_cfg, "RecoveryClearance", 0.0))
        if recovery_clearance <= 0.0:
            pid_cfg = getattr(getattr(cfg, "Motion", None), "Pos2PosPID", None)
            arrival_scale = float(getattr(pid_cfg, "ArrivalScale", 0.8))
            arrival_range = float(getattr(cfg.Motion, "ErrorRange", 10.0)) * arrival_scale
            recovery_extra = float(getattr(safety_cfg, "RecoveryExtra", 3.0))
            recovery_clearance = clearance + arrival_range + recovery_extra

        return {
            "x_limit": max(1.0, x_limit),
            "y_limit": max(1.0, y_limit),
            "goal_box_x_min": max(0.0, x_limit - goal_box_width),
            "goal_box_y_half": max(0.0, min(goal_box_y_half, y_limit)),
            "clearance": max(0.0, clearance),
            "recovery_clearance": max(clearance, recovery_clearance),
        }

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    def ProjectSafePose(self, pose: list | tuple | None):
        """Project an unsafe field pose to the nearest legal coordinate.

        Returns:
            list | None: a safe `[x, y, yaw]` target when correction is needed,
            otherwise `None`.
        """

        if not pose or len(pose) < 3 or list(pose[:3]) == [0xfff, 0xfff, 0xfff]:
            return None

        x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
        params = self._restricted_area_params()
        x_limit = params["x_limit"]
        y_limit = params["y_limit"]
        goal_box_x_min = params["goal_box_x_min"]
        goal_box_y_half = params["goal_box_y_half"]
        clearance = params["clearance"]
        recovery_clearance = params["recovery_clearance"]

        guard_x = max(0.0, x_limit - clearance)
        guard_y = max(0.0, y_limit - clearance)
        recover_x = max(0.0, x_limit - recovery_clearance)
        recover_y = max(0.0, y_limit - recovery_clearance)

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

        in_goal_box = abs(target_x) > goal_box_x_min and abs(target_y) < goal_box_y_half
        if in_goal_box:
            sign_x = 1.0 if target_x >= 0 else -1.0
            sign_y = 1.0 if target_y >= 0 else -1.0

            retreat_x = sign_x * max(0.0, goal_box_x_min - recovery_clearance)
            sidestep_limit = max(0.0, y_limit - recovery_clearance)
            sidestep_y = sign_y * min(sidestep_limit, goal_box_y_half + recovery_clearance)

            retreat_cost = abs(target_x - retreat_x)
            sidestep_cost = abs(target_y - sidestep_y)
            sidestep_has_room = sidestep_limit > goal_box_y_half
            if retreat_cost <= sidestep_cost or not sidestep_has_room:
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
        runtime.chassis.stop()
        runtime.peripheral.Dribble(False)
        runtime.lidar.stop()
        runtime.chassis.stop()
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
