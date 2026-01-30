#ROS2 libs
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.signals import SignalHandlerOptions

from ReasonData import Settings, logger, LOG_FILE, Preference, DATA_DIR, ROS_ENV
from pathlib import Path
import time, math, queue, threading, os, signal, subprocess

import numpy as np

def GetLineStandardEquation(Line):

    x1, y1, x2, y2 = Line
    
    A = y2 - y1
    B = x1 - x2
    C = x2*y1 - x1*y2

    return A, B, C

def GetLineDistance(Line, Point=(0,0)):
    A, B, C = GetLineStandardEquation(Line)
    Distance = (A * Point[0] + B * Point[1] + C) / math.sqrt(A**2 + B**2)

    return Distance

def GetLineTheta(Line):
    if Line[0] == Line[2]:
        return 90
    
    Slope = (Line[3] - Line[1]) / (Line[2] - Line[0])

    if Slope > 0:
        Theta = math.atan(Slope)
    else:
        Theta = math.atan(Slope) + math.pi
    
    return math.degrees(Theta)

def RoundThresholdJudger(Value, Round, MiddleValue, Offset):
    Value = Value % Round
    
    LowerThreshold = (MiddleValue - Offset) % Round
    UpperThreshold = (MiddleValue + Offset) % Round
    
    if LowerThreshold <= UpperThreshold:
        return LowerThreshold <= Value <= UpperThreshold

    else:
        return Value >= LowerThreshold or Value <= UpperThreshold

# TODO Beta
class DirDistance:
    __slots__ = ('angle', 'distance')  # 避免 dict overhead
    def __init__(self):
        self.angle = 0.0
        self.distance = 0.0

    def set(self, angle, distance):
        self.angle = angle
        self.distance = distance

# 初始化一次


class ROSLidarParser(Node):
    def __init__(self, Queue):
        '''
        解析sllidar_ros2的雷达数据
        '''
        if Settings.Debug.Beta == True:
            if Settings.RoboInfo.LidarType == 's1':
                NumRanges = 1080
            elif Settings.RoboInfo.LidarType == 's2':
                NumRanges = 3240
            self.dirDistance = np.zeros((NumRanges, 2), dtype=np.float32)
        else:
            self.dirDistance = []
        self.Queue = Queue
        super().__init__('sllidar_node')

        oQos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        if Settings.Debug.Beta == True:
            self.subscription = self.create_subscription(
                LaserScan,
                '/scan',
                self._scanCallback_beta,
                qos_profile=oQos)
        else:
            self.subscription = self.create_subscription(
                LaserScan,
                '/scan',
                self.scanCallback,
                qos_profile=oQos)

    # TODO beta
    def _scanCallback_beta(self, msg):
        AngleMin = msg.angle_min
        AngleIncrement = msg.angle_increment
        Ranges = np.array(msg.ranges, dtype=np.float32)

        angles = (np.degrees(AngleMin + np.arange(len(Ranges))*AngleIncrement) + 180) % 360
        self.dirDistance[:,0] = angles
        self.dirDistance[:,1] = Ranges

        self.Queue.put(self.dirDistance)

    def scanCallback(self, msg):
        self.dirDistance.clear()
        ScanTime = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        AngleMin = msg.angle_min
        AngleMax = msg.angle_max
        AngleIncrement = msg.angle_increment
        RangeMin = msg.range_min
        RangeMax = msg.range_max
        Ranges = msg.ranges
        NumRanges = len(Ranges)
        for i in range(NumRanges):
            Angle = AngleMin + i * AngleIncrement
            # TODO 高频小对象地狱
            self.dirDistance.append(((math.degrees(Angle)+180) % 360, Ranges[i]))
        self.Queue.put(self.dirDistance)

class Lidar:
    def start_sllidar_driver(self, enforce_settings: bool = True, bashrc: str = ROS_ENV):
        """
        使用 subprocess.Popen 启动 sllidar_ros2 驱动并将输出重定向到统一的项目 log 目录
        如果 enforce_settings=True，则强制使用 Settings 中的配置（LidarID、LidarType）
        返回 Popen 对象并更新 self.lidar_process / self.lidar_log_files / self.current_launch / self.current_domain
        """
        bashrc = os.path.expanduser(bashrc)

        # Read values from Settings (live) when enforcing
        if enforce_settings:
            try:
                domain = int(Preference().read('Ports', 'LidarID'))
            except Exception:
                domain = 99
            try:
                lidar_type = Preference().read('RoboInfo', 'LidarType')
            except Exception:
                lidar_type = getattr(Settings.RoboInfo, 'LidarType', 's2')
            if str(lidar_type).lower() == 's1':
                launch_file = 'sllidar_s1_launch.py'
            else:
                launch_file = 'sllidar_s2_launch.py'
        else:
            # fallback to current attributes
            domain = getattr(self, 'domainID', Settings.Ports.LidarID)
            launch_file = getattr(self, 'current_launch', 'sllidar_s2_launch.py')

        cmd = f'source {bashrc} && export ROS_DOMAIN_ID={domain} && exec ros2 launch sllidar_ros2 {launch_file} serial_port:=/dev/sllidar'

        # place logs into ReasonData data directory under logs/sllidar
        log_dir = Path(DATA_DIR) / 'logs' / 'sllidar'
        os.makedirs(str(log_dir), exist_ok=True)
        # filename: <launch_without_ext>_d<domain>.out/.err
        out_path = str(log_dir / f'{launch_file.replace(".py","")}_d{domain}.out')
        err_path = str(log_dir / f'{launch_file.replace(".py","")}_d{domain}.err')

        print(f"Starting ROS2 launch (domain={domain}, launch={launch_file}): {cmd}")

        try:
            out = open(out_path, 'ab')
            err = open(err_path, 'ab')
            p = subprocess.Popen(
                ['/bin/bash', '-lc', cmd],
                preexec_fn=os.setsid,
                stdout=out,
                stderr=err
            )
            print(f"ROS 2 driver started with PID: {p.pid}, stdout->{out_path}, stderr->{err_path}")
            self.lidar_log_files = (out, err)
            self.lidar_process = p
            self.current_launch = launch_file
            self.current_domain = domain
            return p
        except FileNotFoundError:
            print("Error: /bin/bash not found. Check your system path.")
            return None
        except Exception as e:
            print(f"An error occurred while launching ROS 2 driver: {e}")
            raise e

    def stop_process_group(self, process):
        """
        通过发送 SIGINT/SIGTERM 信号来终止整个进程组 (包括子进程)
        """
        if process is None or process.poll() is not None:
            print("Process is already stopped or was not started.")
            return

        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            print(f"Sent SIGINT to process group {os.getpgid(process.pid)}. Waiting for termination...")
            process.wait(timeout=5)
        except ProcessLookupError:
            print("Process or process group not found.")
        except subprocess.TimeoutExpired:
            print("Process did not terminate gracefully. Sending SIGKILL...")
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)

        print("ROS 2 driver process stopped.")

    def stop(self):
        """Stop the sllidar driver and close log files"""
        if hasattr(self, 'lidar_process') and self.lidar_process is not None:
            self.stop_process_group(self.lidar_process)
            self.lidar_process = None
        if hasattr(self, 'lidar_log_files'):
            try:
                for f in self.lidar_log_files:
                    try:
                        f.flush()
                        f.close()
                    except Exception:
                        pass
            except Exception:
                pass


    def __init__(self,GetYaw=None):

        self.GetYaw = GetYaw
        self.domainID = Settings.Ports.LidarID

        self.dirNormalizedDistance = [0,0,0,0]

        self.dirLidarQueue = queue.Queue()

        self.ParseLidarThread = threading.Thread(target=self.ParseLidar)
        self.ParseLidarThread.daemon = True
        self.ParseLidarThread.start()

        self.LidarPositioningThread = threading.Thread(target=self.LidarNormalize)
        self.LidarPositioningThread.daemon = True
        self.LidarPositioningThread.start()

        # 自动启动 sllidar 驱动（强制基于 Settings）
        try:
            self.lidar_process = None
            self.lidar_log_files = None
            # initialize current settings values
            try:
                self.domainID = int(Preference().read('Ports', 'LidarID'))
            except Exception:
                self.domainID = int(Settings.Ports.LidarID)
            self.current_launch = None
            self.current_domain = None

            p = self.start_sllidar_driver(enforce_settings=True)
            if p is None:
                logger.error("Warning: failed to start sllidar driver")

            # start settings watcher thread to update on config changes
            self._watcher_thread = threading.Thread(target=self._settings_watcher)
            self._watcher_thread.daemon = True
            self._watcher_thread.start()
        except Exception as e:
            raise(f"Exception while starting sllidar driver: {e}")

    def ParseLidar(self):
        try:
            logger.info(f"Initializing ROS parser with domain_id={self.domainID}")
            rclpy.init(domain_id=self.domainID, signal_handler_options=SignalHandlerOptions(0))
            Parser = ROSLidarParser(self.dirLidarQueue)
            rclpy.spin(Parser)
        except Exception as e:
            logger.error(f"ParseLidar exception: {e}")
    
    def LidarNormalize(self):
        step = 2
        frameCount = 0
        lastTime = time.time()
        
        try:
            while(1):
                Ranges = self.dirLidarQueue.get()
                lines = []
                points = []
                for Element in Ranges:
                    if 0 < Element[1] < 2.5:
                        X = Element[1] * math.sin(math.radians(Element[0]))
                        Y = Element[1] * math.cos(math.radians(Element[0]))
                        points.append((X, Y, Element[0]))
                
                if points:
                    c = 0
                    for i in range(0,len(points),step):
                        c +=1
                        if i + 1 < len(points):
                            Line = (points[i][0], points[i][1], points[i+1][0], points[i+1][1])
                            Theta = GetLineTheta(Line)
                            LidarAVGTheta = (points[i][2] + points[i+1][2]) / 2
                            Distance = GetLineDistance(Line)
                            lines.append((points[i][0], points[i][1], points[i+1][0], points[i+1][1], Distance, Theta, LidarAVGTheta))
                    frameCount += 1
                    CurrentTime = time.time()
                    if CurrentTime - lastTime >= 1.0:
                        if Settings.Debug.FullLog:
                            logger.debug(f"FPS: {frameCount}")
                            logger.debug(f"Points: {c}")
                            frameCount = 0
                            lastTime = CurrentTime

                    Compass = self.GetYaw()
                    CompassFull = Compass

                    if Compass > 180:
                        Compass = Compass - 180

                    HorizontalLines = []
                    VerticalLines = []

                    if lines:
                        for Line in lines:
                            if RoundThresholdJudger(Line[5], 180, Compass, 20):
                                HorizontalLines.append(Line)
                            elif RoundThresholdJudger(Line[5], 180, Compass + 90, 20):
                                VerticalLines.append(Line)
                            
                        # print(Compass,len(HorizontalLines),len(VerticalLines))
                        Distances = [[],[],[],[]]
                        LidarDists = [0,0,0,0]
                    
                        if HorizontalLines and VerticalLines:
                            for Line in HorizontalLines:
                                if RoundThresholdJudger(Line[6], 360, -(CompassFull), 90):
                                    Distance = round(Line[4], 3)
                                    Distances[0].append(Distance)
                                elif RoundThresholdJudger(Line[6], 360, -(CompassFull + 180), 90):
                                    Distance = round(Line[4], 3)
                                    Distances[2].append(Distance)
                            for Line in VerticalLines:
                                if RoundThresholdJudger(Line[6], 360, -(CompassFull + 90), 90):
                                    Distance = round(Line[4], 3)
                                    Distances[1].append(Distance)
                                elif RoundThresholdJudger(Line[6], 360, -(CompassFull + 270), 90):
                                    Distance = round(Line[4], 3)
                                    Distances[3].append(Distance)
                            

                            for i in range(4):
                                if len(Distances[i]) > 5:
                                    Distances[i].sort()
                                    iNum = int(len(Distances[i])/100*85)
                                    import numpy
                                    LidarDists[i] = int(numpy.nan_to_num(Distances[i][iNum]) * 1000)
                                else:
                                    LidarDists[i] = 0

                        self.dirNormalizedDistance = LidarDists
        except Exception as e:
            self.LidarNormalize()

    def GetDists(self):
        '''
        返回一个列表[前，左，后，右]
        '''
        return self.dirNormalizedDistance
    
    def GetFullData(self):
        return self.dirLidarQueue.get()

    def _settings_watcher(self):
        """Background thread: watch config (via Preference) and restart lidar driver / parser if LidarID or LidarType changes."""
        while True:
            # read fresh Preference each loop so that external changes are seen
            try:
                desired_domain = int(Preference().read('Ports', 'LidarID'))
            except Exception:
                desired_domain = self.domainID
            try:
                desired_type = Preference().read('RoboInfo', 'LidarType')
                desired_launch = 'sllidar_s1_launch.py' if str(desired_type).lower() == 's1' else 'sllidar_s2_launch.py'
            except Exception:
                desired_launch = self.current_launch

            if (self.current_domain != desired_domain) or (self.current_launch != desired_launch):
                logger.info(f"Lidar settings changed: domain {self.current_domain}->{desired_domain}, launch {self.current_launch}->{desired_launch}")
                # restart lidar driver to apply new settings
                try:
                    self.stop()
                except Exception:
                    pass
                try:
                    p = self.start_sllidar_driver(enforce_settings=True)
                    if p is None:
                        logger.warning("Failed to restart sllidar driver after settings change")
                except Exception as e:
                    logger.error(f"Error restarting sllidar driver: {e}")

                # if domain changed, restart ROS parser as well
                if self.domainID != desired_domain:
                    try:
                        rclpy.shutdown()
                    except Exception:
                        pass
                    self.domainID = desired_domain
                    try:
                        self.ParseLidarThread = threading.Thread(target=self.ParseLidar)
                        self.ParseLidarThread.daemon = True
                        self.ParseLidarThread.start()
                    except Exception as e:
                        logger.error(f"Failed to restart ParseLidarThread: {e}")

            time.sleep(5)
    
    def __exit__(self):
        self.stop_process_group(self.stop)
