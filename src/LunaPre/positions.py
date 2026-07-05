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
from .strategy_tools import BALL_HELD, BALL_NOT_FOUND, angle_error_deg, clamp

cfg = runtime.cfg

POS2POS_KP = 0.018
POS2POS_KI = 0.00005
POS2POS_KD = 0.0008
POS2POS_OUTPUT_LIMIT_MPS = 1.10
POS2POS_MIN_SPEED_MPS = 0.08
POS2POS_ARRIVAL_SCALE = 0.5
ROBOT_BALL_MAX_DISTANCE_CM = 240.0
ROBOT_BALL_MAX_ABS_X_CM = 260.0
ROBOT_BALL_MAX_ABS_Y_CM = 220.0

class Positions:
    def __init__(self):
        # Values for Setup Position System
        self.LidarPos = [0xfff,0xfff,0xfff]
        self.ballPos = [0xfff,0xfff]
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
        self._pos2pos_record_enabled = bool(getattr(cfg.Debug, "RecordPos2Pos", False))
        self._pos2pos_record_dir = str(getattr(cfg.Debug, "Pos2PosRecordDir", "./Records/pos2pos"))
        self.Position_X_PID = self._build_pos2pos_pid()
        self.Position_Y_PID = self._build_pos2pos_pid()
        self._vision_enabled = runtime.default_vision_enabled()

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

    def _resolve_speed_gain(self, speed_gain: int | float | None) -> float:
        if speed_gain is None:
            return 1.0
        return clamp(float(speed_gain), 0.2, 6.0)

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
        return round(iDeltaAngle,2)

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
        if self.ballPos == BALL_HELD or self.ballPos == BALL_NOT_FOUND:
            self.ballPosOut = self.ballPos
        elif self.ballPos == [0, 0]:
            self.ballPosOut = BALL_NOT_FOUND
        elif self._robot_ball_out_of_bounds(self.ballPos):
            self.ballPosOut = BALL_NOT_FOUND
        else:
            catch_x, catch_y = cfg.Motion.CatchVal
            x_tol = cfg.Motion.ErrorRange / 20
            y_tol = cfg.Motion.ErrorRange / 5
            if (
                abs(self.ballPos[1] - catch_y) <= y_tol
                and round(abs(self.ballPos[0] - catch_x), 0) <= x_tol
            ):
                self.ballPosOut = BALL_HELD
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
        if ball_pos == BALL_NOT_FOUND:
            return "lost"
        if ball_pos == BALL_HELD:
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

    def FieldRobotPose(self, lower: bool | None = False):
        '''
        `非阻塞函数`

        获取[机器人几何中心]相对于[场地几何中心]的姿态 [x_forward,y_left,yaw]

        #### Args:
            lower: `bool`, choose to using lower data

        #### Returns:
            RobotPose: [x, y, yaw]
        #### Note:
            `0xfff` stand for `No Position`
        '''
        pose = runtime.get_field_pose()
        if pose is not None:
            self.LidarPos = pose
            return self.LidarPos

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

    def Pos2Pos(self, AimPos: list, speed_gain: int | float | None = None) -> bool:
        """非阻塞到点控制。

        AimPos 为 [x_cm, y_cm, yaw_deg]。每次调用只发一帧速度：
        距离没到就平移并朝向目标 yaw；位置到了但角度没到就原地转向。
        """
        loc_x, loc_y, loc_yaw = self.FieldRobotPose()
        if [loc_x, loc_y, loc_yaw] == [0xfff, 0xfff, 0xfff]:
            runtime.chassis.stop()
            runtime.Morse("No Position Data!")
            return False

        aim_x, aim_y, aim_yaw = AimPos
        aim_yaw = float(aim_yaw) % 360.0
        err_x = float(aim_x) - float(loc_x)
        err_y = float(aim_y) - float(loc_y)
        err_yaw = angle_error_deg(loc_yaw, aim_yaw)
        dist = math.hypot(err_x, err_y)
        arrival_cm = max(1.0, float(Settings.Motion.ErrorRange) * 0.5)

        if dist <= arrival_cm:
            if abs(err_yaw) <= float(Settings.Motion.ErrorRange):
                runtime.chassis.stop()
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
        cmd_speed = math.hypot(cmd_x, cmd_y)
        max_speed = clamp(0.35 * gain, 0.20, POS2POS_OUTPUT_LIMIT_MPS)
        if cmd_speed > max_speed:
            scale = max_speed / cmd_speed
            cmd_x *= scale
            cmd_y *= scale
        elif cmd_speed < POS2POS_MIN_SPEED_MPS:
            cmd_x = err_x / dist * POS2POS_MIN_SPEED_MPS
            cmd_y = err_y / dist * POS2POS_MIN_SPEED_MPS
        runtime.chassis.FieldMoveVector(cmd_x, cmd_y, aim_yaw)
        return False

    def _path_hits_restricted_area(self, start, target, samples: int = 12) -> bool:
        if not start or not target or len(start) < 3 or len(target) < 3:
            return False
        for idx in range(1, max(2, samples)):
            t = idx / float(max(2, samples))
            pose = [
                float(start[0]) + (float(target[0]) - float(start[0])) * t,
                float(start[1]) + (float(target[1]) - float(start[1])) * t,
                float(start[2]) + angle_error_deg(start[2], target[2]) * t,
            ]
            if self.IsPoseInRestrictedArea(pose):
                return True
        return False

    def _navigate_waypoint(self, start, target):
        safe_target = self.ProjectSafePose(target)
        if safe_target is not None:
            target = safe_target
        if not self._path_hits_restricted_area(start, target):
            return target

        params = self._restricted_area_params()
        y_room = max(0.0, params["y_limit"] - params["recovery_clearance"])
        y_detour = min(y_room, params["goal_box_y_half"] + params["recovery_clearance"])
        if y_detour <= params["goal_box_y_half"]:
            return target

        side = 1.0
        if abs(float(start[1])) > 1.0:
            side = 1.0 if float(start[1]) > 0 else -1.0
        elif abs(float(target[1])) > 1.0:
            side = 1.0 if float(target[1]) > 0 else -1.0

        candidates = [
            [float(start[0]), side * y_detour, float(target[2])],
            [float(target[0]), side * y_detour, float(target[2])],
            [0.0, side * y_detour, float(target[2])],
            [float(start[0]), -side * y_detour, float(target[2])],
            [float(target[0]), -side * y_detour, float(target[2])],
        ]
        for waypoint in candidates:
            if self.ProjectSafePose(waypoint) is None and not self._path_hits_restricted_area(start, waypoint):
                return waypoint
        return target

    def Navigate2(self, AimPos: list, speed_gain: int | float | None = None) -> bool:
        """非阻塞简单导航。

        和 Pos2Pos 的区别：Navigate2 会先检查直线路径是否穿过限制区；
        如果会穿过，就先给一个绕到限制区上下方的中间点。
        """
        start = self.FieldRobotPose()
        if start == [0xfff, 0xfff, 0xfff]:
            runtime.chassis.stop()
            runtime.Morse("No Position Data!")
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
        return runtime.get_referee_status()

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
            arrival_range = float(getattr(cfg.Motion, "ErrorRange", 10.0)) * POS2POS_ARRIVAL_SCALE
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
