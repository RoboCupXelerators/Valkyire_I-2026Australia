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

    global Bits, lidar, chassis, compass, peripheral, key, Odometer

    def _use_mock(reason: str) -> None:
        """Switch module-level runtime objects to mock implementations."""

        global Bits, lidar, chassis, compass, peripheral, key, Odometer, RUNTIME_IS_MOCK

        logger.warning(f"Using MockBits: {reason}")
        RUNTIME_IS_MOCK = True
        Bits = _MockBits()
        lidar = Lidar(Bits.GetYaw, auto_start=False)
        chassis = Car(Bits.SetWheelSpeed, Bits.GetYaw)
        compass = Bits.GetYaw
        peripheral = Peripherals(Bits.VirtualPorts)
        key = Key(Bits.GetKey)
        Odometer = None
        return

    # If running on dev machine, allow user to force mock via env
    if os.environ.get("ROBOCUPX_USE_MOCK", "").lower() in {"1", "true", "yes"}:
        _use_mock("ROBOCUPX_USE_MOCK=1")
        return

    try:
        if cfg.RoboInfo.Bit == "AB":
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
            peripheral = Peripherals(Bits.VirtualPorts)  # TODO
            chassis = Car(Bits.SetWheelSpeed, Bits.GetYaw)
            compass = Bits.GetYaw
            # key = Key(Bits.GetKey)
            # Odometer = Bits.Odometer  # TODO add odometer support
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
        self.vision_data = {"ball_pos": [0, 0], "chassis_pos": []}
        self.vision_process = None
        self.vision_data_stop_event = threading.Event()
        self.get_vision_data_process: threading.Thread | None = None
        self._vision_started = False
        # Vision should be opt-in on dev/no-hardware runtime; otherwise strategy loops may spawn heavy processes.
        enable_vision_env = os.environ.get("ROBOCUPX_ENABLE_VISION", "").lower() in {"1", "true", "yes"}
        self._vision_enabled = True
        if RUNTIME_IS_MOCK or (os.environ.get("ROBOCUPX_USE_MOCK", "").lower() in {"1", "true", "yes"}):
            self._vision_enabled = enable_vision_env

    def StartVision(self) -> bool:
        """Start UDP receiver + Vision subprocess lazily.

        Importing LunaPre should not start external processes automatically.
        Call this when you really need vision data.
        """

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
                # if we can receive packets, reset restart counter
                restart_count = 0

            except (socket.timeout, TimeoutError):
                if not self._vision_enabled:
                    # vision explicitly disabled
                    self.vision_data = {"ball_pos": [0, 0], "chassis_pos": []}
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

                self.vision_data = {"ball_pos": [0, 0], "chassis_pos": []}

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
        # ensure vision thread is running
        self.StartVision()
        return self.vision_data

    # Backward-compatible alias
    def get_vision_data(self):
        return self.GetVisionData()

    def _get_ball_pos(self):
        self.StartVision()
        return self.vision_data["ball_pos"]
    
    def _get_chassis_pos(self):
        self.StartVision()
        return self.vision_data["chassis_pos"]
    
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
        # 20251017 already changed X,Y dimension    
        self.ballPos = self._get_ball_pos()
        if self.ballPos == [0,0] or abs(self.ballPos[0]) > 95 or abs(self.ballPos[1]) > 150:
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
        Get the [Direct] Distance From the bound to the Bound

        获取四面墙的距离 

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
                logger.error("No Lidar Data Recieved! Please Check Lidar Modules!")
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
            if Bits.Odometer:# if use odometer datas
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
        # 20251017 already changed X,Y dimension
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
        '''
        `非阻塞函数`

        获取[球]相对于[机器几何中心]的角度

        retrun a relative angle of the ball
        ### Returns:
            ballangle: int | relative ball angle
        '''
        ballX, ballY = self._get_ball_pos()
        if ballY == 0:
            relBallAngle = 0
        try:
            relBallAngle = int(math.degrees(math.atan2(ballX,ballY)))
        except ZeroDivisionError:
            relBallAngle = 0
        return relBallAngle
    
    def FieldBallAngle(self):
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
        x, y, _ = self.FieldRobotPos()
        if [BallX, BallY] == [0xfff,0xfff]:
            self.BallDistance = 0xfff
        elif [BallX, BallY] == [0xddd,0xddd]:
            self.BallDistance = 0xddd
        else:
            self.BallDistance = math.sqrt((BallX - x) ** 2 + (BallY - y) ** 2)
        if self.FullLog:
            logger.debug("[Absolute] Ball Distance: %s" % self.BallDistance)
        return self.BallDistance

    def FieldBallPos(self):
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
            AbsBallPositon = [SelfX + ballX, SelfY + ballY]
            return AbsBallPositon

    def RobotChassisAngle(self):
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

    def FieldChassisAngle(self):
        '''
        `非阻塞函数`

        获取[其他机器]相对于[场地几何中心]的角度列表
        ##### retrun a list of the absolute angles of the chassis
        #### Returns:
            [a1,a2,a3,etc]
        '''
        OutputAngles = []
        CompassCache = compass()
        for i in self.RobotChassisAngle():
            ChassisAngleCache = i + CompassCache
            if ChassisAngleCache > 360:
                OutputAngles.append(int(ChassisAngleCache)%360)
            else:
                OutputAngles.append(int(ChassisAngleCache))
        return OutputAngles

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
            return False
        if iLocX < 0:
            kX = -1
        else:
            kX = 1
        if iLocY < 0:
            kY = -1
        else:
            kY = 1
        iAimX,iAimY,iAimZ = AimPos
        iDeltaX = (iAimX) - (iLocX)
        iDeltaY = (iAimY) - (iLocY)
        iDeltaZ = (iAimZ) - (iLocZ)
        RestrictedX = Settings.Bounds.FarPos[0]
        RestrictedY = Settings.Bounds.FarPos[1]
        try:
            try:
                Slope = iDeltaX/iDeltaY
            except ZeroDivisionError:
                Slope = 1
            if Slope*RestrictedY > RestrictedX:
                iAimX = (RestrictedX - 3)*kX
            if Slope/RestrictedX > RestrictedY:
                iAimY = (RestrictedY - 3)*kY
        except:
            iAimX,iAimY,iAimZ = AimPos
        iErrorRange = Settings.ExpectedVals.ErrorRange
        iMovedAngle = (int(math.degrees(math.atan2(iDeltaY,iDeltaX))))%360
        if iErrorRange/2 > abs(iDeltaX) and iErrorRange/2 > abs(iDeltaY) and abs(iErrorRange) > iDeltaZ:
            chassis.stop()
            return True
        elif iErrorRange/2 > abs(iDeltaX) and iErrorRange/2 > abs(iDeltaY) and not abs(iErrorRange) > abs(iDeltaZ):
            chassis.FieldTurn(iAimZ)
        else:
            if speed_gain:
                chassis.FieldMoveVetor(iDeltaX*speed_gain,iDeltaY*speed_gain,iAimZ)
                # chassis.AbsMoveAngle(AimPos[2],iMovedAngle,speed_gain)
            else:
                chassis.FieldMoveVetor(iDeltaX,iDeltaY,iAimZ)
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
                    if i == Posistions[-1]:
                        return True
                    else:
                        time.sleep(iWaitMs)
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

    def referee2Start(self):
        pass

    def RestrictedArea(self):
        Distance = self.DirectDistance()
        
        # border = 240
        # restricted_area_x = 400
        # restricted_area_y = 800

        if Distance[0] == 0:
            return False
        else:
            if abs(Distance[1] - Distance[3])<Settings.Bounds.restricted_area_y:
                if Distance[0]<Settings.Bounds.restricted_area_x:
                    chassis.FieldMoveAngle(0,180,300)
                    return False
                elif Distance[2]<Settings.Bounds.restricted_area_x:
                    chassis.FieldMoveAngle(0,0,300)
                    return False
            if Distance[0]<Settings.Bounds.Distance2Bound:
                chassis.FieldMoveAngle(0,180,300)
                return False
            if Distance[2]<Settings.Bounds.Distance2Bound:
                chassis.FieldMoveAngle(0,0,300)
                return False
            if Distance[1]<Settings.Bounds.Distance2Bound:
                chassis.FieldMoveAngle(0,270,-300)
                return False
            if Distance[3]<Settings.Bounds.Distance2Bound:
                chassis.FieldMoveAngle(0,90,-300)
                return False
            else:
                return True
            
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

    # Backward-compatible alias (legacy code used lower-case stop())
    def stop(self):
        return self.Stop()

UnitedPosition = Positions()

class Communication:
    def __init__(self):
        if cfg.Transimission.Method == "WIFI":
            from utils.ReasonBeacon import MisakaNetwork
            self.transimission = MisakaNetwork()
        elif cfg.Transimission.Method == "BLE":
            pass
        else:
            raise RuntimeError("None Transmission Method Selected!")
        self.BallFlag = [0,0]
        self.PeerPosition = [0xfff,0xfff]
        self.FullLog = cfg.Debug.FullLog

    # Communicate Functions
    def BallOwner(self):# function for judging whether catch the ball
        '''
        ### function for judging whether catch the ball
        #### no return value, this function will change the global variable `BallFlag`
        '''
        BallX, BallY = UnitedPosition.FieldBallPos()
        if [BallX,BallY] == [0xddd,0xddd]:
            self.BallFlag[0] = 1
        else:
            self.BallFlag[0] = 0
        if self.FullLog:
            logger.debug("BallFlag Satus: %s"%self.BallFlag)
        else:
            pass

    def SendStatusThreadFunc(self): # send role and the status of the ball
        '''
        ### Warning: This function will stuck the main thread
        ### use `Sendstatus()` instead
        '''
        while (1):
            SelfPosition = UnitedPosition.FieldRobotPos()
            try:
                # make sure all the values are integer
                ball_self = int(self.BallFlag[0]) if isinstance(self.BallFlag[0], (int, float, str)) else 0
                pos_x = int(SelfPosition[0])
                pos_y = int(SelfPosition[1])
                msg = f"BS:{ball_self};PX:{pos_x};PY:{pos_y}"
                self.transimission.Send(msg)
                if self.FullLog:
                    logger.success(f"Sent message: {msg}")  # Debug message
            except Exception as e:
                logger.error(f"Send error: {e}")
                raise e
            time.sleep(0.03) # in case too fast

    def SendStatus(self):
        '''
        ### Function for starting Send Status Thread
        #### return `True` if the Thread is already started
        '''
        global SendstatusThreadFuncStarted
        if not SendstatusThreadFuncStarted:
            SendstatusThread = threading.Thread(target=self.SendStatusThreadFunc)
            SendstatusThread.daemon = True  # Set as a daemon thread, will auto finished after the main thread finished
            SendstatusThread.start()
            SendstatusThreadFuncStarted = True
        return True

    def PeerStatusThreadFunc(self):
        '''
        ### Warning: This function will stuck main thread
        ### use `Peerstatus()` instead
        '''
        while (1):
            MessageCache = self.transimission.MessageCache
            if MessageCache == None:
                if self.FullLog:
                    logger.warning("No Messages")
                time.sleep(1)
            if MessageCache:
                try:
                    # setup default val
                    self.PeerPosition = [0xfff,0xfff]
                    self.BallFlag[1] = 0
                    PeerPositionX, PeerPositionY = 0xfff, 0xfff
                    for part in MessageCache.split(";"):
                        if part.startswith("BS:"):
                            self.BallFlag[1] = int(part.split(":")[1])  # convert into int
                        if part.startswith("PX:"):
                            PeerPositionX = int(float(part.split(":")[1]))  # process float number
                        if part.startswith("PY:"):
                            PeerPositionY = int(float(part.split(":")[1]))  # process float number
                    self.PeerPosition = [PeerPositionX, PeerPositionY]
                except Exception as e:
                    self.BallFlag = [0, 0]
                    self.PeerPosition = [0xfff, 0xfff]
                    logger.error(f"Failed to parse message '{MessageCache}': {e}")
                MessageCache = None
            time.sleep(0.02)

    def PeerStatus(self):# Start the Prase Thread
        '''
        ### Function for starting Prase Status Thread
        #### return `True` if the Thread is already started
        #### This thread will update the BallFlag
        '''
        global PeerstatusThreadFuncStarted
        if not PeerstatusThreadFuncStarted:
            PeerstatusThread = threading.Thread(target=self.PeerStatusThreadFunc)
            PeerstatusThread.daemon = True  # 设置为守护线程，主线程结束时自动结束
            PeerstatusThread.start()
            PeerstatusThreadFuncStarted = True

    def StartConncetion(self):
        '''
        ### Function for starting Send&Prase Status Thread
        '''
        if not PeerstatusThreadFuncStarted and not SendstatusThreadFuncStarted:
            self.SendStatus()
            self.PeerStatus()
        else:
            pass

