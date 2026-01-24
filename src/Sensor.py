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
from scipy.optimize import least_squares

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



class ROSLidarParser(Node):
    def __init__(self, Queue):
        '''
        解析sllidar_ros2的雷达数据
        '''
        self.dirDistance = []
        self.Queue = Queue
        super().__init__('sllidar_node')

        oQos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scanCallback,
            qos_profile=oQos)
        
        
    def scanCallback(self, msg):
        self.dirDistance = []
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

        cmd = f'source {bashrc} && export ROS_DOMAIN_ID={domain} && exec ros2 launch sllidar_ros2 {launch_file}'

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
                    for i in range(0,len(points),step):
                        if i + 1 < len(points):
                            Line = (points[i][0], points[i][1], points[i+1][0], points[i+1][1])
                            Theta = GetLineTheta(Line)
                            LidarAVGTheta = (points[i][2] + points[i+1][2]) / 2
                            Distance = GetLineDistance(Line)
                            lines.append((points[i][0], points[i][1], points[i+1][0], points[i+1][1], Distance, Theta, LidarAVGTheta))

                    frameCount += 1
                    CurrentTime = time.time()
                    if CurrentTime - lastTime >= 1.0:
                        # if Settings.Debug.FullLog:
                        logger.debug(f"FPS: {frameCount}")
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
            raise e
            self.LidarNormalize()

    def GetDists(self):
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

# class LidarWithoutYaw:
#     def start_sllidar_driver(self, enforce_settings: bool = True, bashrc: str = ROS_ENV):
#         """
#         使用 subprocess.Popen 启动 sllidar_ros2 驱动并将输出重定向到统一的项目 log 目录
#         如果 enforce_settings=True，则强制使用 Settings 中的配置（LidarID、LidarType）
#         返回 Popen 对象并更新 self.lidar_process / self.lidar_log_files / self.current_launch / self.current_domain
#         """
#         bashrc = os.path.expanduser(bashrc)

#         # Read values from Settings (live) when enforcing
#         if enforce_settings:
#             try:
#                 domain = int(Preference().read('Ports', 'LidarID'))
#             except Exception:
#                 domain = 99
#             try:
#                 lidar_type = Preference().read('RoboInfo', 'LidarType')
#             except Exception:
#                 lidar_type = getattr(Settings.RoboInfo, 'LidarType', 's2')
#             if str(lidar_type).lower() == 's1':
#                 launch_file = 'sllidar_s1_launch.py'
#             else:
#                 launch_file = 'sllidar_s2_launch.py'
#         else:
#             # fallback to current attributes
#             domain = getattr(self, 'domainID', Settings.Ports.LidarID)
#             launch_file = getattr(self, 'current_launch', 'sllidar_s2_launch.py')

#         cmd = f'source {bashrc} && export ROS_DOMAIN_ID={domain} && exec ros2 launch sllidar_ros2 {launch_file}'

#         # place logs into ReasonData data directory under logs/sllidar
#         log_dir = Path(DATA_DIR) / 'logs' / 'sllidar'
#         os.makedirs(str(log_dir), exist_ok=True)
#         # filename: <launch_without_ext>_d<domain>.out/.err
#         out_path = str(log_dir / f'{launch_file.replace(".py","")}_d{domain}.out')
#         err_path = str(log_dir / f'{launch_file.replace(".py","")}_d{domain}.err')

#         print(f"Starting ROS2 launch (domain={domain}, launch={launch_file}): {cmd}")

#         try:
#             out = open(out_path, 'ab')
#             err = open(err_path, 'ab')
#             p = subprocess.Popen(
#                 ['/bin/bash', '-lc', cmd],
#                 preexec_fn=os.setsid,
#                 stdout=out,
#                 stderr=err
#             )
#             print(f"ROS 2 driver started with PID: {p.pid}, stdout->{out_path}, stderr->{err_path}")
#             self.lidar_log_files = (out, err)
#             self.lidar_process = p
#             self.current_launch = launch_file
#             self.current_domain = domain
#             return p
#         except FileNotFoundError:
#             print("Error: /bin/bash not found. Check your system path.")
#             return None
#         except Exception as e:
#             print(f"An error occurred while launching ROS 2 driver: {e}")
#             raise e

#     def stop_process_group(self, process):
#         """
#         通过发送 SIGINT/SIGTERM 信号来终止整个进程组 (包括子进程)
#         """
#         if process is None or process.poll() is not None:
#             print("Process is already stopped or was not started.")
#             return

#         try:
#             os.killpg(os.getpgid(process.pid), signal.SIGINT)
#             print(f"Sent SIGINT to process group {os.getpgid(process.pid)}. Waiting for termination...")
#             process.wait(timeout=5)
#         except ProcessLookupError:
#             print("Process or process group not found.")
#         except subprocess.TimeoutExpired:
#             print("Process did not terminate gracefully. Sending SIGKILL...")
#             os.killpg(os.getpgid(process.pid), signal.SIGKILL)

#         print("ROS 2 driver process stopped.")

#     def stop(self):
#         """Stop the sllidar driver and close log files"""
#         if hasattr(self, 'lidar_process') and self.lidar_process is not None:
#             self.stop_process_group(self.lidar_process)
#             self.lidar_process = None
#         if hasattr(self, 'lidar_log_files'):
#             try:
#                 for f in self.lidar_log_files:
#                     try:
#                         f.flush()
#                         f.close()
#                     except Exception:
#                         pass
#             except Exception:
#                 pass


#     def __init__(self):
#         self.logger = logger

#         self.domainID = Settings.Ports.LidarID

#         self.dirNormalizedDistance = [0,0,0,0]

#         self.dirLidarQueue = queue.Queue()

#         self.ParseLidarThread = threading.Thread(target=self.ParseLidar)
#         self.ParseLidarThread.daemon = True
#         self.ParseLidarThread.start()

#         self.LidarPositioningThread = threading.Thread(target=self.LidarPosition)
#         self.LidarPositioningThread.daemon = True
#         self.LidarPositioningThread.start()

#         try:
#             self.lidar_process = None
#             self.lidar_log_files = None
#             # initialize current settings values
#             try:
#                 self.domainID = int(Preference().read('Ports', 'LidarID'))
#             except Exception:
#                 self.domainID = int(Settings.Ports.LidarID)
#             self.current_launch = None
#             self.current_domain = None

#             p = self.start_sllidar_driver(enforce_settings=True)
#             if p is None:
#                 logger.error("Warning: failed to start sllidar driver")

#             # start settings watcher thread to update on config changes
#             self._watcher_thread = threading.Thread(target=self._settings_watcher)
#             self._watcher_thread.daemon = True
#             self._watcher_thread.start()
#         except Exception as e:
#             raise(f"Exception while starting sllidar driver: {e}")
        

#     def _settings_watcher(self):
#         """Background thread: watch config (via Preference) and restart lidar driver / parser if LidarID or LidarType changes."""
#         while True:
#             # read fresh Preference each loop so that external changes are seen
#             try:
#                 desired_domain = int(Preference().read('Ports', 'LidarID'))
#             except Exception:
#                 desired_domain = self.domainID
#             try:
#                 desired_type = Preference().read('RoboInfo', 'LidarType')
#                 desired_launch = 'sllidar_s1_launch.py' if str(desired_type).lower() == 's1' else 'sllidar_s2_launch.py'
#             except Exception:
#                 desired_launch = self.current_launch

#             if (self.current_domain != desired_domain) or (self.current_launch != desired_launch):
#                 logger.info(f"Lidar settings changed: domain {self.current_domain}->{desired_domain}, launch {self.current_launch}->{desired_launch}")
#                 # restart lidar driver to apply new settings
#                 try:
#                     self.stop()
#                 except Exception:
#                     pass
#                 try:
#                     p = self.start_sllidar_driver(enforce_settings=True)
#                     if p is None:
#                         logger.warning("Failed to restart sllidar driver after settings change")
#                 except Exception as e:
#                     logger.error(f"Error restarting sllidar driver: {e}")

#                 # if domain changed, restart ROS parser as well
#                 if self.domainID != desired_domain:
#                     try:
#                         rclpy.shutdown()
#                     except Exception:
#                         pass
#                     self.domainID = desired_domain
#                     try:
#                         self.ParseLidarThread = threading.Thread(target=self.ParseLidar)
#                         self.ParseLidarThread.daemon = True
#                         self.ParseLidarThread.start()
#                     except Exception as e:
#                         logger.error(f"Failed to restart ParseLidarThread: {e}")

#             time.sleep(5)
    
#     def ParseLidar(self):
#         try:
#             logger.info(f"Initializing ROS parser with domain_id={self.domainID}")
#             rclpy.init(domain_id=self.domainID, signal_handler_options=SignalHandlerOptions(0))
#             Parser = ROSLidarParser(self.dirLidarQueue)
#             rclpy.spin(Parser)
#         except Exception as e:
#             logger.error(f"ParseLidar exception: {e}")

#     def lidar_to_cartesian(self,ranges, min_dist=0.1, max_dist=3.0):
#         """
#         将 (angle_deg, distance) 转为机器人坐标系下的 2D 点 (x_front, y_left)
#         """
#         points = []
#         for angle_deg, dist in ranges:
#             if not (min_dist <= dist <= max_dist):
#                 continue
#             angle_rad = np.deg2rad(angle_deg)
#             # ROS2: 0°=+X, 90°=+Y → 标准极坐标
#             x = dist * np.cos(angle_rad)
#             y = dist * np.sin(angle_rad)
#             points.append([x, y])
#         return np.array(points)  # shape (N, 2)

#     def detect_corners(self,points, eps=0.1, min_samples=5):
#         """
#         简化版：直接找最远的4个点作为墙角候选（适用于空旷场地）
#         更鲁棒的方法可用 Hough 直线检测 + 求交点
#         """
#         if len(points) < 4:
#             return None
        
#         # 计算到原点的距离
#         dists = np.linalg.norm(points, axis=1)
#         # 取最远的 8 个点（避免重复）
#         idx = np.argsort(dists)[-8:]
#         candidates = points[idx]
        
#         # 聚类去重（简单版：按象限分组）
#         corners = []
#         quadrants = [
#             (lambda x, y: x > 0 and y > 0),  # Q1: +X, +Y
#             (lambda x, y: x < 0 and y > 0),  # Q2: -X, +Y
#             (lambda x, y: x < 0 and y < 0),  # Q3: -X, -Y
#             (lambda x, y: x > 0 and y < 0),  # Q4: +X, -Y
#         ]
        
#         for quad in quadrants:
#             pts_in_quad = [p for p in candidates if quad(p[0], p[1])]
#             if pts_in_quad:
#                 # 取该象限中最远的点
#                 pts = np.array(pts_in_quad)
#                 d = np.linalg.norm(pts, axis=1)
#                 corners.append(pts[np.argmax(d)])
#             else:
#                 # 若某象限无点，用均值估计（可选）
#                 corners.append([0, 0])
        
#         return np.array(corners)  # (4, 2)

#     def estimate_pose(self, observed_corners, world_corners):
#         """
#         通过 4 对点匹配，求解机器人位姿 [x, y, yaw]
#         observed_corners: 机器人坐标系下的墙角 (4, 2)
#         world_corners:    世界坐标系下的墙角 (4, 2)
#         返回: [x, y, yaw_rad]
#         """
#         def residuals(params, src, dst):
#             tx, ty, theta = params
#             cos_t = np.cos(theta)
#             sin_t = np.sin(theta)
#             R = np.array([[cos_t, -sin_t],
#                         [sin_t,  cos_t]])
#             # 将世界点变换到机器人坐标系
#             transformed = (R @ (dst.T - np.array([[tx], [ty]]))).T
#             return (src - transformed).ravel()
        
#         # 初始猜测：原点
#         x0 = [0.0, 0.0, 0.0]
#         result = least_squares(residuals, x0, args=(observed_corners, world_corners))
#         return result.x  # [x, y, yaw]

#     # ===== 主函数 =====
#     def localize_robot(self, ranges, field_length=1.8, field_width=1.2):
#         # 1. 转换为笛卡尔坐标
#         points = self.lidar_to_cartesian(ranges)
#         if len(points) < 10:
#             raise ValueError("点云太少")
        
#         # 2. 检测墙角
#         obs_corners = self.detect_corners(points)
#         if obs_corners is None:
#             raise ValueError("无法检测墙角")
        
#         # 3. 定义世界墙角
#         world_corners = np.array([
#             [ field_length/2,  field_width/2],
#             [-field_length/2,  field_width/2],
#             [-field_length/2, -field_width/2],
#             [ field_length/2, -field_width/2],
#         ])
        
#         # 4. 估计位姿
#         x, y, yaw_rad = self.estimate_pose(obs_corners, world_corners)
        
#         # 5. 转换为 ROS2 标准输出: [x_forward, y_left, z_yaw_rad]
#         # 注意：我们的坐标系已对齐，无需额外变换
#         return [x, y, math.degrees(yaw_rad)]
    
#     def LidarPosition(self):
#         Ranges = self.dirLidarQueue.get()
#         self.dist = self.localize_robot(Ranges)
