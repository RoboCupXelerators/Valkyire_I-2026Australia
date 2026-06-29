"""Legacy ROS2 lidar scan driver."""

from __future__ import annotations

import math
import os
import queue
import signal
import subprocess
import threading
import time
from pathlib import Path

from ReasonData import DATA_DIR, ROS_ENV, Preference, Settings, logger

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        QoSDurabilityPolicy,
        QoSHistoryPolicy,
        QoSProfile,
        QoSReliabilityPolicy,
    )
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import LaserScan

    ROS_AVAILABLE = True
except Exception:  # pragma: no cover - depends on external ROS2 env
    rclpy = None
    Node = object  # type: ignore
    LaserScan = object  # type: ignore
    QoSProfile = object  # type: ignore
    QoSReliabilityPolicy = object  # type: ignore
    QoSHistoryPolicy = object  # type: ignore
    QoSDurabilityPolicy = object  # type: ignore
    SignalHandlerOptions = object  # type: ignore
    ROS_AVAILABLE = False

try:
    import numpy as np  # type: ignore

    NUMPY_AVAILABLE = True
except ModuleNotFoundError:
    np = None  # type: ignore
    NUMPY_AVAILABLE = False


def _lidar_type_from_settings() -> str:
    try:
        return str(Preference().read("RoboInfo", "LidarType")).lower()
    except Exception:
        return str(getattr(Settings.RoboInfo, "LidarType", "s2")).lower()


def _launch_file_for_type(lidar_type: str) -> str:
    lidar_type_key = str(lidar_type).lower()
    if lidar_type_key == "s1":
        return "sllidar_s1_launch.py"
    if lidar_type_key == "s2":
        return "sllidar_s2_launch.py"
    return "sllidar_s3_launch.py"


class _ROSLidarParser(Node):
    def __init__(self, scan_queue):
        if Settings.Debug.Beta == True and NUMPY_AVAILABLE:
            lidar_type = _lidar_type_from_settings()
            NumRanges = 1080 if lidar_type == "s1" else 3240
            self.dirDistance = np.zeros((NumRanges, 2), dtype=np.float32)
        else:
            self.dirDistance = []
        self.Queue = scan_queue
        super().__init__("sllidar_node")

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        if Settings.Debug.Beta == True and NUMPY_AVAILABLE:
            self.subscription = self.create_subscription(LaserScan, "/scan", self._scanCallback_beta, qos_profile=qos)
        else:
            self.subscription = self.create_subscription(LaserScan, "/scan", self.scanCallback, qos_profile=qos)

    def _scanCallback_beta(self, msg):
        if not NUMPY_AVAILABLE:
            return
        AngleMin = msg.angle_min
        AngleIncrement = msg.angle_increment
        Ranges = np.array(msg.ranges, dtype=np.float32)

        angles = (np.degrees(AngleMin + np.arange(len(Ranges)) * AngleIncrement) + 180) % 360
        self.dirDistance[:, 0] = angles
        self.dirDistance[:, 1] = Ranges
        self._publish(self.dirDistance)

    def scanCallback(self, msg):
        self.dirDistance.clear()
        AngleMin = msg.angle_min
        AngleIncrement = msg.angle_increment
        Ranges = msg.ranges
        for i in range(len(Ranges)):
            Angle = AngleMin + i * AngleIncrement
            self.dirDistance.append(((math.degrees(Angle) + 180) % 360, Ranges[i]))
        self._publish(tuple(self.dirDistance))

    def _publish(self, scan):
        try:
            while self.Queue.full():
                self.Queue.get_nowait()
            self.Queue.put_nowait(scan)
        except queue.Full:
            pass
        except queue.Empty:
            try:
                self.Queue.put_nowait(scan)
            except queue.Full:
                pass


class ROS2LidarDriver:
    """Acquire scan frames from the legacy ROS2 sllidar stack."""

    def __init__(self, *, domain_id: int | None = None, bashrc: str | None = ROS_ENV, serial_port: str = "/dev/ttyUSB1"):
        self.domain_id = domain_id
        self.bashrc = bashrc
        self.serial_port = serial_port
        self.lidar_process = None
        self.lidar_log_files = None
        self.current_launch = None
        self.current_domain = None
        self._thread = None
        self._watcher_thread = None
        self._stop_event = threading.Event()

    def start(self, scan_queue):
        if not ROS_AVAILABLE or rclpy is None:
            logger.warning("ROS2/rclpy not available, skip starting lidar driver.")
            return False
        if not self.bashrc:
            logger.warning("ROS_ENV is None, skip starting lidar driver.")
            return False

        self._stop_event.clear()
        self._thread = threading.Thread(target=self.run_forever, args=(scan_queue,))
        self._thread.daemon = True
        self._thread.start()

        process = self.start_sllidar_driver(enforce_settings=True, bashrc=self.bashrc)
        if process is None:
            logger.error("Warning: failed to start sllidar driver")

        self._watcher_thread = threading.Thread(target=self._settings_watcher)
        self._watcher_thread.daemon = True
        self._watcher_thread.start()
        return True

    def stop(self):
        self._stop_event.set()
        if self.lidar_process is not None:
            self.stop_process_group(self.lidar_process)
            self.lidar_process = None
        if self.lidar_log_files:
            for file_obj in self.lidar_log_files:
                try:
                    file_obj.flush()
                    file_obj.close()
                except Exception:
                    pass
            self.lidar_log_files = None
        try:
            if rclpy is not None:
                rclpy.shutdown()
        except Exception:
            pass

    def run_forever(self, scan_queue):
        try:
            domain = self._desired_domain()
            self.domain_id = domain
            logger.info(f"Initializing ROS parser with domain_id={domain}")
            rclpy.init(domain_id=domain, signal_handler_options=SignalHandlerOptions(0))
            parser = _ROSLidarParser(scan_queue)
            rclpy.spin(parser)
        except Exception as e:
            if not self._stop_event.is_set():
                logger.error(f"ROS lidar parser exception: {e}")

    def _desired_domain(self) -> int:
        if self.domain_id is not None:
            return int(self.domain_id)
        try:
            return int(Preference().read("Ports", "LidarID"))
        except Exception:
            return int(Settings.Ports.LidarID)

    def start_sllidar_driver(self, enforce_settings: bool = True, bashrc: str | None = ROS_ENV):
        if not ROS_AVAILABLE:
            logger.warning("ROS2/rclpy not available, skip starting lidar driver.")
            return None

        if not bashrc:
            logger.warning("ROS_ENV is None, skip starting lidar driver.")
            return None

        bashrc = os.path.expanduser(str(bashrc))
        if enforce_settings:
            domain = self._desired_domain()
            launch_file = _launch_file_for_type(_lidar_type_from_settings())
        else:
            domain = self.domain_id or Settings.Ports.LidarID
            launch_file = self.current_launch or "sllidar_s2_launch.py"

        cmd = (
            f"source {bashrc} && export ROS_DOMAIN_ID={domain} && exec ros2 launch "
            f"sllidar_ros2 {launch_file} serial_port:={self.serial_port}"
        )

        log_dir = Path(DATA_DIR) / "logs" / "sllidar"
        os.makedirs(str(log_dir), exist_ok=True)
        out_path = str(log_dir / f'{launch_file.replace(".py", "")}_d{domain}.out')
        err_path = str(log_dir / f'{launch_file.replace(".py", "")}_d{domain}.err')

        print(f"Starting ROS2 launch (domain={domain}, launch={launch_file}): {cmd}")

        try:
            out = open(out_path, "ab")
            err = open(err_path, "ab")
            process = subprocess.Popen(
                ["/bin/bash", "-lc", cmd],
                preexec_fn=os.setsid,
                stdout=out,
                stderr=err,
            )
            print(f"ROS 2 driver started with PID: {process.pid}, stdout->'{out_path}', stderr->'{err_path}'")
            self.lidar_log_files = (out, err)
            self.lidar_process = process
            self.current_launch = launch_file
            self.current_domain = domain
            return process
        except FileNotFoundError:
            print("Error: /bin/bash not found. Check your system path.")
            return None
        except Exception as e:
            print(f"An error occurred while launching ROS 2 driver: {e}")
            raise e

    def stop_process_group(self, process):
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

    def _settings_watcher(self):
        while not self._stop_event.wait(5):
            desired_domain = self._desired_domain()
            desired_launch = _launch_file_for_type(_lidar_type_from_settings())

            if (self.current_domain != desired_domain) or (self.current_launch != desired_launch):
                logger.info(
                    f"Lidar settings changed: domain {self.current_domain}->{desired_domain}, "
                    f"launch {self.current_launch}->{desired_launch}"
                )
                try:
                    if self.lidar_process is not None:
                        self.stop_process_group(self.lidar_process)
                        self.lidar_process = None
                except Exception:
                    pass
                try:
                    process = self.start_sllidar_driver(enforce_settings=True, bashrc=self.bashrc)
                    if process is None:
                        logger.warning("Failed to restart sllidar driver after settings change")
                except Exception as e:
                    logger.error(f"Error restarting sllidar driver: {e}")
