"""Lidar sensor computation facade.

``sensor.py`` owns the stable LunaPre ``Lidar`` API and the geometry that turns
raw scan frames into four wall distances. Hardware/process details live in
driver modules:

- ``lidar_driver_rplidar``: direct serial RPLidar access via rplidar-roboticia.
- ``lidar_driver_ros2``: legacy ROS2 /scan access.
- ``lidar_driver_ldlidar``: direct serial LDROBOT LD06 access via pyserial.
"""

from __future__ import annotations

import importlib
import math
import queue
import threading
import time

from ReasonData import Settings, logger

from .strategy_tools import BALL_NOT_FOUND


LIDAR_DISABLED_DISTS = [BALL_NOT_FOUND[0], BALL_NOT_FOUND[0], BALL_NOT_FOUND[0], BALL_NOT_FOUND[0]]
LIDAR_ZERO_DISTS = [0, 0, 0, 0]
DEFAULT_SCAN_QUEUE_SIZE = 3
DEFAULT_LIDAR_BACKEND = "ros2"
MAX_POINT_CLOUD_POINTS = 360


def GetLineStandardEquation(Line):
    x1, y1, x2, y2 = Line

    A = y2 - y1
    B = x1 - x2
    C = x2 * y1 - x1 * y2

    return A, B, C


def GetLineDistance(Line, Point=(0, 0)):
    """Return the non-negative perpendicular distance from ``Point`` to ``Line``."""
    A, B, C = GetLineStandardEquation(Line)
    Distance = (A * Point[0] + B * Point[1] + C) / math.sqrt(A**2 + B**2)

    return abs(Distance)


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

    return Value >= LowerThreshold or Value <= UpperThreshold


class DirDistance:
    __slots__ = ("angle", "distance")

    def __init__(self):
        self.angle = 0.0
        self.distance = 0.0

    def set(self, angle, distance):
        self.angle = angle
        self.distance = distance


def _resolve_backend(backend: str | None) -> str:
    backend_value = backend
    if backend_value in (None, "", "None"):
        backend_value = getattr(getattr(Settings, "RoboInfo", object()), "LidarBackend", None)
    if backend_value in (None, "", "None"):
        backend_value = getattr(getattr(Settings, "Lidar", object()), "Backend", None)
    if backend_value in (None, "", "None"):
        backend_value = DEFAULT_LIDAR_BACKEND

    backend_key = str(backend_value).strip().lower()
    if backend_key in {"rplidar", "direct"}:
        lidar_type = str(
            getattr(getattr(Settings, "RoboInfo", object()), "LidarType", "")
        ).strip().lower()
        if lidar_type == "s3":
            # The bundled rplidar-roboticia package only understands the
            # legacy scan response. S3 requires Slamtec's newer scan-mode
            # negotiation, which is provided by the official ROS2 package.
            logger.warning("RPLIDAR S3 selected with the legacy backend; using the official ROS2 driver.")
            return "ros2"
        return "rplidar"
    if backend_key in {"ros", "ros2"}:
        return "ros2"
    if backend_key in {"ldlidar", "ld06", "ldrobot", "ydlidar", "ld07"}:
        # ``ydlidar`` was the historical misspelling; keep it readable for
        # old config files while the canonical backend name is ``ldlidar``.
        return "ldlidar"
    logger.warning(f"Unknown configured lidar backend={backend_key!r}; using {DEFAULT_LIDAR_BACKEND}.")
    return DEFAULT_LIDAR_BACKEND


def _load_driver_class(backend: str):
    if backend == "ros2":
        module = importlib.import_module(".lidar_driver_ros2", package=__package__)
        return module.ROS2LidarDriver
    if backend == "ldlidar":
        module = importlib.import_module(".lidar_driver_ldlidar", package=__package__)
        lidar_type = str(
            getattr(getattr(Settings, "RoboInfo", object()), "LidarType", "ld06")
        ).strip().lower()
        configured_backend = str(
            getattr(getattr(Settings, "RoboInfo", object()), "LidarBackend", "")
        ).strip().lower()
        if lidar_type == "ld07" or (
            configured_backend == "ydlidar" and lidar_type not in {"ld06", "ld07"}
        ):
            return module.LD07Driver
        return module.LDLidarDriver
    module = importlib.import_module(".lidar_driver_rplidar", package=__package__)
    return module.RPLidarDriver


def _scan_frame_is_empty(frame) -> bool:
    if frame is None:
        return True
    try:
        return len(frame) == 0
    except TypeError:
        return False


def _is_valid_scan_distance(distance) -> bool:
    """Return True for finite positive ranges used as one-revolution samples."""
    try:
        value = float(distance)
    except (TypeError, ValueError):
        return False
    return math.isfinite(value) and value > 0.0


class Lidar:
    """Legacy LunaPre lidar API backed by a pluggable scan driver."""

    def __init__(
        self,
        GetYaw=None,
        *,
        auto_start: bool = True,
        backend: str | None = None,
        driver=None,
        queue_size: int = DEFAULT_SCAN_QUEUE_SIZE,
        **driver_kwargs,
    ):
        self.GetYaw = GetYaw
        self.backend = _resolve_backend(backend)
        self.driver = driver
        self.running = False
        self.disabled = not auto_start
        self._stop_event = threading.Event()
        self.LidarPositioningThread = None
        self.dirLidarQueue = queue.Queue(maxsize=max(1, int(queue_size)))
        self.dirNormalizedDistance = list(LIDAR_DISABLED_DISTS if self.disabled else LIDAR_ZERO_DISTS)
        self.last_raw_scan_count = 0
        self.last_scan_point_count = 0
        # Latest valid scan points in robot coordinates, in metres.
        self.last_point_cloud = ()
        self.last_line_count = 0

        if self.driver is None and auto_start:
            try:
                DriverClass = _load_driver_class(self.backend)
                self.driver = DriverClass(**driver_kwargs)
            except Exception as e:
                logger.error(f"Failed to initialize lidar driver ({self.backend}): {e}")
                self.disabled = True
                self.dirNormalizedDistance = list(LIDAR_DISABLED_DISTS)

        if self.disabled:
            logger.warning(
                "Lidar disabled (backend=%s, auto_start=%s, driver=%s)."
                % (self.backend, auto_start, type(self.driver).__name__ if self.driver else None)
            )
            return

        self.running = True
        if self.driver is not None:
            started = self.driver.start(self.dirLidarQueue)
            if started is False:
                self.disabled = True
                self.running = False
                self.dirNormalizedDistance = list(LIDAR_DISABLED_DISTS)
                return

        self.LidarPositioningThread = threading.Thread(target=self.LidarNormalize)
        self.LidarPositioningThread.daemon = True
        self.LidarPositioningThread.start()

    def _is_running(self) -> bool:
        if self._stop_event.is_set():
            return False
        return bool(self.running)

    def _current_yaw(self) -> float | None:
        if not callable(self.GetYaw):
            return None
        try:
            return float(self.GetYaw())
        except Exception:
            return None

    def stop(self):
        """Stop the sensor facade and its active scan driver."""
        self.running = False
        self._stop_event.set()
        if self.driver is not None:
            try:
                self.driver.stop()
            except Exception:
                pass

    def ParseLidar(self):
        """Compatibility hook; drivers own acquisition in the refactored path."""
        if self.driver is not None and hasattr(self.driver, "run_forever"):
            self.driver.run_forever(self.dirLidarQueue)

    def LidarNormalize(self):
        step = 2
        frameCount = 0
        lastTime = time.time()

        while self._is_running():
            try:
                Ranges = self.dirLidarQueue.get(timeout=0.2)
                if _scan_frame_is_empty(Ranges):
                    self.last_point_cloud = ()
                    continue
                lines = []
                points = []
                point_cloud = []
                point_cloud_stride = max(
                    1,
                    math.ceil(len(Ranges) / MAX_POINT_CLOUD_POINTS),
                )
                raw_point_count = 0
                for scan_index, Element in enumerate(Ranges):
                    distance = Element[1]
                    if not _is_valid_scan_distance(distance):
                        continue
                    raw_point_count += 1
                    keep_for_cloud = scan_index % point_cloud_stride == 0
                    keep_for_near_field = distance < 2.5
                    if not (keep_for_cloud or keep_for_near_field):
                        continue
                    # Lidar angles use 0 degrees for the robot forward axis
                    # and increase toward the robot's left side. Keep the
                    # normalized scan in the same forward/left frame used by
                    # the radar view and the yaw-based wall classification.
                    angle_rad = math.radians(Element[0])
                    X = distance * math.cos(angle_rad)
                    Y = distance * math.sin(angle_rad)
                    if keep_for_cloud:
                        point_cloud.append((X, Y))
                    if keep_for_near_field:
                        points.append((X, Y, Element[0]))

                point_count = len(points)
                line_pair_count = 0
                self.last_raw_scan_count = raw_point_count
                self.last_scan_point_count = point_count
                self.last_point_cloud = tuple(
                    (float(x), float(y)) for x, y in point_cloud
                )

                if points:
                    for i in range(0, len(points), step):
                        line_pair_count += 1
                        if i + 1 < len(points):
                            if points[i][0] == points[i + 1][0] and points[i][1] == points[i + 1][1]:
                                continue
                            Line = (points[i][0], points[i][1], points[i + 1][0], points[i + 1][1])
                            Theta = GetLineTheta(Line)
                            LidarAVGTheta = (points[i][2] + points[i + 1][2]) / 2
                            Distance = GetLineDistance(Line)
                            lines.append((
                                points[i][0],
                                points[i][1],
                                points[i + 1][0],
                                points[i + 1][1],
                                Distance,
                                Theta,
                                LidarAVGTheta,
                            ))
                self.last_line_count = len(lines)

                frameCount += 1
                CurrentTime = time.time()
                if CurrentTime - lastTime >= 1.0:
                    if Settings.Debug.FullLog:
                        logger.debug(
                            "Lidar FPS: %s | points=%s | raw=%s | line_pairs=%s"
                            % (frameCount, point_count, raw_point_count, line_pair_count)
                        )
                    frameCount = 0
                    lastTime = CurrentTime

                if points:

                    Compass = self._current_yaw()
                    if Compass is None or not math.isfinite(Compass):
                        # A stale/unavailable heading must not masquerade as
                        # 0 degrees. Keep the last valid normalized frame.
                        continue
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

                        Distances = [[], [], [], []]
                        LidarDists = [0, 0, 0, 0]

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
                                    iNum = int(len(Distances[i]) / 100 * 85)
                                    value = Distances[i][iNum]
                                    LidarDists[i] = int(value * 1000) if math.isfinite(value) else 0
                                else:
                                    LidarDists[i] = 0

                        self.dirNormalizedDistance = LidarDists
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"LidarNormalize exception: {e}")
                time.sleep(0.05)

    def GetDists(self):
        """返回一个列表 [前, 左, 后, 右]，单位 mm。"""
        return self.dirNormalizedDistance

    def GetPointCount(self):
        """返回最近一帧雷达过滤后的有效点数量。"""
        return self.last_scan_point_count

    def GetFullData(self):
        return self.dirLidarQueue.get()

    def GetNearbyObjects(self):
        """Return near-field LDROBOT clusters in robot coordinates, if supported."""
        detector = getattr(self.driver, "detect_objects", None)
        if not callable(detector):
            return []
        try:
            return detector()
        except Exception as error:
            logger.warning(f"Nearby lidar object detection failed: {error}")
            return []

    def __exit__(self, exc_type=None, exc=None, tb=None):
        self.stop()
