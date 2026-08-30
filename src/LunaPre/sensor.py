"""LunaPre sensor facade.

``Sensor`` is the semantic observation boundary used by LunaPre.  It selects
between a Bits pose provider and native scan providers, exposes the historical
four wall-distance API, and owns the lidar-only geometric pose helper.  The
lower-level ``Lidar`` class remains available for callers that need raw scan
frames, but it is not a localization source itself.
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
DEFAULT_SENSOR_SOURCE = "auto"
SENSOR_SOURCES = frozenset({"auto", "bits", "ros2", "ld07"})


def coerce_pose(pose):
    """Return a finite ``(x_cm, y_cm, yaw_deg)`` tuple or ``None``.

    Bits implementations are allowed to report an unavailable pose with
    ``None``.  Keeping validation here gives runtime and sensor callers one
    contract and prevents malformed payloads from turning into a false origin.
    """

    if pose is None:
        return None
    try:
        x_cm, y_cm, yaw_deg = pose
        values = (float(x_cm), float(y_cm), float(yaw_deg))
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in values):
        return None
    return values


def _resolve_source(source: str | None) -> str:
    """Resolve the configured semantic source without selecting hardware."""

    value = source
    if value in (None, "", "None"):
        try:
            value = getattr(Settings.Sensor, "Source", None)
        except AttributeError:
            value = None
    if value in (None, "", "None"):
        value = DEFAULT_SENSOR_SOURCE

    key = str(value).strip().lower()
    if key not in SENSOR_SOURCES:
        allowed = ", ".join(sorted(SENSOR_SOURCES))
        raise ValueError(f"Sensor.Source must be one of: {allowed}; got {value!r}")
    return key


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
        source = _resolve_source(None)
        backend_value = source if source in {"ros2", "ld07"} else DEFAULT_LIDAR_BACKEND

    backend_key = str(backend_value).strip().lower()
    if backend_key in {"ros", "ros2"}:
        return "ros2"
    if backend_key in {"ld07", "ldlidar", "ld06", "ldrobot", "ydlidar"}:
        # Keep existing LDROBOT configuration readable while making LD07 the
        # only active direct-serial source name.
        return "ld07"
    if backend_key in {"rplidar", "direct"}:
        raise ValueError(
            "Direct RPLidar is no longer a LunaPre sensor backend; "
            "configure Sensor.Source=ros2 or Sensor.Source=ld07."
        )
    raise ValueError(
        f"Unknown lidar backend={backend_key!r}; supported backends are ros2 and ld07."
    )


def _load_driver_class(backend: str):
    if backend == "ros2":
        module = importlib.import_module(".lidar_driver_ros2", package=__package__)
        return module.ROS2LidarDriver
    if backend == "ld07":
        module = importlib.import_module(".lidar_driver_ld07", package=__package__)
        return module.LD07Driver
    raise ValueError(f"Unsupported lidar backend: {backend}")


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


def _valid_wall_distances(distances) -> list[float] | None:
    """Normalize a native [front, left, back, right] millimetre frame."""

    if distances is None:
        return None
    try:
        values = [float(value) for value in distances]
    except (TypeError, ValueError):
        return None
    if len(values) != 4 or not all(math.isfinite(value) and value >= 0.0 for value in values):
        return None
    if values == [float(value) for value in LIDAR_DISABLED_DISTS] or not any(values):
        return None
    return values


def _pose_wall_distances(pose, *, bounds_long_cm: float, bounds_short_cm: float) -> list[float] | None:
    """Ray-cast a valid Bits pose against the rectangular field in millimetres."""

    normalized = coerce_pose(pose)
    if normalized is None:
        return None
    x_cm, y_cm, yaw_deg = normalized
    half_long = float(bounds_long_cm) / 2.0
    half_short = float(bounds_short_cm) / 2.0
    if not all(math.isfinite(value) and value > 0.0 for value in (half_long, half_short)):
        return None

    def _ray_distance_cm(dx: float, dy: float) -> float:
        candidates = []
        epsilon = 1e-9
        if abs(dx) > epsilon:
            for bound_x in (-half_long, half_long):
                t = (bound_x - x_cm) / dx
                hit_y = y_cm + t * dy
                if t >= 0.0 and -half_short - epsilon <= hit_y <= half_short + epsilon:
                    candidates.append(t)
        if abs(dy) > epsilon:
            for bound_y in (-half_short, half_short):
                t = (bound_y - y_cm) / dy
                hit_x = x_cm + t * dx
                if t >= 0.0 and -half_long - epsilon <= hit_x <= half_long + epsilon:
                    candidates.append(t)
        return max(0.0, min(candidates)) if candidates else 0.0

    heading = math.radians(yaw_deg)
    forward = (math.cos(heading), math.sin(heading))
    left = (-forward[1], forward[0])
    directions = (forward, left, (-forward[0], -forward[1]), (forward[1], -forward[0]))
    return [_ray_distance_cm(dx, dy) * 10.0 for dx, dy in directions]


class Sensor:
    """Semantic LunaPre sensor service.

    ``Source`` is intentionally limited to ``auto``, ``bits``, ``ros2`` and
    ``ld07``.  ``auto`` tries a valid Bits pose first, then the active ROS2
    driver, then the active LD07 driver.  The runtime owns driver construction;
    this facade only chooses observations that are already available.
    """

    def __init__(
        self,
        *,
        pose_provider=None,
        yaw_provider=None,
        lidar=None,
        source: str | None = None,
        bounds_long_cm: float | None = None,
        bounds_short_cm: float | None = None,
    ):
        self._pose_provider = pose_provider
        self._yaw_provider = yaw_provider
        self._lidar = lidar
        self._source = _resolve_source(source)
        self.bounds_long_cm = float(
            bounds_long_cm
            if bounds_long_cm is not None
            else getattr(getattr(Settings, "Bounds", object()), "Long", 260.0)
        )
        self.bounds_short_cm = float(
            bounds_short_cm
            if bounds_short_cm is not None
            else getattr(getattr(Settings, "Bounds", object()), "Short", 200.0)
        )
        self.last_source: str | None = None

    @property
    def Source(self) -> str:
        """Configured semantic source, using the historical capitalized name."""

        return self._source

    @Source.setter
    def Source(self, value: str) -> None:
        self._source = _resolve_source(value)

    @property
    def lidar(self):
        """Expose the raw scan facade for diagnostics only."""

        return self._lidar

    @property
    def running(self):
        """Compatibility view of the raw scan worker state."""

        return bool(getattr(self._lidar, "running", getattr(self._lidar, "_running", False)))

    @property
    def disabled(self):
        """Compatibility view of whether a native scan source is unavailable."""

        return bool(getattr(self._lidar, "disabled", not self.running))

    @property
    def driver(self):
        """Expose the active raw driver for diagnostics without making it a source API."""

        return getattr(self._lidar, "driver", None)

    @property
    def backend(self):
        """Raw native backend name, when a scan facade is present."""

        return getattr(self._lidar, "backend", None)

    @property
    def last_point_cloud(self):
        """Latest native point cloud in robot coordinates, in metres."""

        return getattr(self._lidar, "last_point_cloud", ())

    @property
    def last_line_count(self):
        return int(getattr(self._lidar, "last_line_count", 0))

    @property
    def last_scan_point_count(self):
        return int(getattr(self._lidar, "last_scan_point_count", 0))

    @property
    def last_raw_scan_count(self):
        return int(getattr(self._lidar, "last_raw_scan_count", 0))

    def _pose(self):
        if not callable(self._pose_provider):
            return None
        try:
            return coerce_pose(self._pose_provider())
        except Exception:
            return None

    def _native_distances(self, source: str) -> list[float] | None:
        if self._lidar is None:
            return None
        backend = str(getattr(self._lidar, "backend", "")).strip().lower()
        if backend != source:
            return None
        getter = getattr(self._lidar, "GetDists", None)
        if not callable(getter):
            return None
        try:
            return _valid_wall_distances(getter())
        except Exception:
            return None

    def _record_source(self, source: str | None) -> None:
        if source == self.last_source:
            return
        if source is None:
            logger.warning("Sensor has no valid observation source")
        elif self.last_source is not None:
            logger.info(f"Sensor source switched: {self.last_source} -> {source}")
        self.last_source = source

    def _candidate_sources(self) -> tuple[str, ...]:
        if self.Source == "auto":
            return ("bits", "ros2", "ld07")
        return (self.Source,)

    def _geometric_pose_from_distances(self, distances):
        if distances is None or not callable(self._yaw_provider):
            return None
        try:
            yaw_deg = float(self._yaw_provider())
        except Exception:
            return None
        if not math.isfinite(yaw_deg):
            return None

        forward, left, back, right = distances
        long_mm = self.bounds_long_cm * 10.0
        short_mm = self.bounds_short_cm * 10.0
        if forward + back < long_mm:
            x_forward_mm = long_mm / 2.0 - forward if forward > back else back - long_mm / 2.0
        else:
            x_forward_mm = ((long_mm / 2.0 - forward) + (back - long_mm / 2.0)) / 2.0
        if left + right < (self.bounds_short_cm - 50.0) * 10.0:
            y_left_mm = -(
                short_mm / 2.0 - left if left > right else right - short_mm / 2.0
            )
        else:
            y_left_mm = -((short_mm / 2.0 - left) + (right - short_mm / 2.0)) / 2.0
        return coerce_pose((x_forward_mm / 10.0, y_left_mm / 10.0, yaw_deg))

    def GetPose(self):
        """Return the configured field pose as ``(x_cm, y_cm, yaw_deg)`` or ``None``."""

        for source in self._candidate_sources():
            pose = self._pose() if source == "bits" else self._geometric_pose_from_distances(
                self._native_distances(source)
            )
            if pose is not None:
                self._record_source(source)
                return pose
        self._record_source(None)
        return None

    def DirectDistance(self) -> list[float]:
        """Return four wall ranges in millimetres [front, left, back, right]."""

        for source in self._candidate_sources():
            if source == "bits":
                distances = _pose_wall_distances(
                    self._pose(),
                    bounds_long_cm=self.bounds_long_cm,
                    bounds_short_cm=self.bounds_short_cm,
                )
            else:
                distances = self._native_distances(source)
            if distances is not None:
                self._record_source(source)
                return list(distances)
        self._record_source(None)
        return list(LIDAR_DISABLED_DISTS)

    def DirectDistanceSnapshot(self) -> list[float]:
        """Return the latest semantic wall-distance frame without side effects."""

        return self.DirectDistance()

    def GetDists(self) -> list[float]:
        """Compatibility alias for callers that historically held ``runtime.lidar``."""

        return self.DirectDistance()

    def GetGeometricPose(self):
        """Return the active wall-distance estimate for diagnostics, or ``None``."""

        pose = self._geometric_pose_from_distances(self.DirectDistance())
        return list(pose) if pose is not None else None

    def GetPointCloud(self):
        """Return the latest raw scan cloud in robot coordinates, in metres."""

        cloud = getattr(self._lidar, "last_point_cloud", ())
        try:
            return list(cloud)
        except TypeError:
            return []

    def GetNearbyObjects(self):
        """Return native near-field clusters when the active scan driver supports them."""

        detector = getattr(self._lidar, "GetNearbyObjects", None)
        if not callable(detector):
            return []
        try:
            return detector()
        except Exception as error:
            logger.warning(f"Nearby lidar object detection failed: {error}")
            return []

    def stop(self) -> None:
        """Stop the underlying scan facade, if it owns one."""

        stopper = getattr(self._lidar, "stop", None)
        if callable(stopper):
            stopper()


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
