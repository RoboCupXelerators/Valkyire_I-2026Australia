"""Shared vision preprocessing utilities.

This module is used by multiple vision backends (Hailo / BPU / Blob).

Configuration comes from ``merged_settings`` so backend selection, camera
preprocessing, and projection all see the same system config plus optional
vision overlay. Runtime logging and data-directory ownership remain in
``ReasonData``.
"""

from pathlib import Path

_REASON_DATA_DIR_FALLBACK = Path(__file__).resolve().parents[1] / "ReasonData" / "data"

# All active vision modules import this singleton rather than constructing a
# second Settings object from ReasonData or VisionDatas.
from .merged_settings import Settings
from .modern_calibration import ModernCalibration, ModernCalibrationError

try:
    from ReasonData import logger, DATA_DIR as REASON_DATA_DIR  # type: ignore
except Exception:  # pragma: no cover
    try:
        from src.ReasonData import logger, DATA_DIR as REASON_DATA_DIR  # type: ignore
    except Exception:  # pragma: no cover
        import logging

        logger = logging.getLogger(__name__)
        if not logger.handlers:
            logger.addHandler(logging.NullHandler())
        REASON_DATA_DIR = _REASON_DATA_DIR_FALLBACK
import threading
import queue
import time
import cv2
import numpy as np
import pathlib
from dataclasses import dataclass
import atexit
import signal
import datetime

APP_DIR = Path(__file__).parent

# Local (legacy) data dir shipped with vision_utils.
_VISION_LOCAL_DATA_DIR = APP_DIR / "VisionDatas" / "data"

# Prefer ReasonData's runtime data dir when available, so calibration/config/logs
# live under ONE source of truth: `src/ReasonData/data`.
#
# This fixes a long-standing issue where calibration was accidentally read from:
#   /Software/src/vision_utils/VisionDatas/data/Calibration/
# instead of:
#   /Software/src/ReasonData/data/Calibration/
DATA_DIR = Path(REASON_DATA_DIR) if REASON_DATA_DIR is not None else _VISION_LOCAL_DATA_DIR

MODEL_DIR = APP_DIR / "models"


def _norm_rel_folder(folder: str | None, default: str) -> str:
    """Normalize a folder configured like '/Calibration/' to a relative name.

    Many legacy configs include leading/trailing slashes. When using pathlib,
    a leading '/' would turn it into an absolute path, so we must strip it.
    """

    s = (folder or default).strip()
    # turn '', '/', '////' into default
    if not s or set(s) == {"/"}:
        s = default
    # '/Calibration/' -> 'Calibration'
    s = s.strip("/")
    return s or default.strip("/")


def _is_jetson_nano() -> bool:
    """Best-effort detection of Jetson Nano.

    We *intentionally* only auto-enable hwaccel for Nano because other platforms
    may not have the same GStreamer elements or behave differently.
    """
    try:
        import os

        if os.path.exists("/etc/nv_tegra_release"):
            # All Jetsons have this, but we only enable for Nano.
            # Try model file first.
            model_paths = [
                "/proc/device-tree/model",
                "/sys/firmware/devicetree/base/model",
            ]
            for p in model_paths:
                try:
                    if os.path.exists(p):
                        s = open(p, "rb").read().decode("utf-8", errors="ignore").lower()
                        if "jetson nano" in s:
                            return True
                except Exception:
                    pass
    except Exception:
        pass
    return False


def _build_gst_v4l2_pipelines(
    dev_index: int,
    width: int,
    height: int,
    fps: int,
    codec: str = "mjpg",
) -> list[str]:
    """Build candidate gstreamer pipeline strings for Jetson Nano.

    Notes:
      - For USB MJPEG cameras, `v4l2src ! image/jpeg ! jpegdec` is CPU.
        Some systems have `nvjpegdec`/`nvv4l2decoder mjpeg=1`, but availability
        depends on JetPack build.
      - We keep this conservative: try a Nano-friendly path first, and rely on
        runtime fallback to legacy V4L2 if it fails.
    """
    codec = (codec or "mjpg").lower()
    if codec in ("mjpg", "mjpeg", "jpeg"):
        # MJPEG cameras: different JetPack images may provide different decoders.
        # Try in order:
        #   1) nvv4l2decoder mjpeg=1 (V4L2 HW decode, best when available)
        #   2) nvjpegdec (NVJPEG HW decode, often available)
        #   3) jpegdec (CPU fallback but still via gstreamer; may still help piping)
        base = f"v4l2src device=/dev/video{dev_index} ! image/jpeg,width={width},height={height},framerate={fps}/1 ! "
        tail = " nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1 sync=false"
        return [
            base + "nvv4l2decoder mjpeg=1 !" + tail,
            base + "nvjpegdec !" + tail,
            base + "jpegdec !" + tail,
        ]
    else:
        # Raw YUYV (no decode step)
        return [
            (
                f"v4l2src device=/dev/video{dev_index} ! "
                f"video/x-raw,format=YUY2,width={width},height={height},framerate={fps}/1 ! "
                "nvvidconv ! video/x-raw,format=BGRx ! "
                "videoconvert ! video/x-raw,format=BGR ! appsink drop=1 sync=false"
            )
        ]


def _try_open_gstreamer(dev_index: int, width: int, height: int, fps: int, codec: str, logger):
    """Try to open a camera with multiple gstreamer pipelines, return (cap, pipeline) or (None, None)."""
    for pipeline in _build_gst_v4l2_pipelines(dev_index, width, height, fps, codec=codec):
        try:
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if cap is not None and cap.isOpened():
                return cap, pipeline
        except Exception:
            pass
    return None, None
@dataclass
class _RecordConfig:
    enabled: bool
    record_dir: Path
    segment_seconds: int
    fps: int
    codec: str
    ext: str
    cams: list[int] | None = None
    # Writer output size (width, height). If None, use source frame size.
    record_size: tuple[int, int] | None = None


class SegmentedMultiCamRecorder:
    """Write multi-camera frames to segmented video files.

    - One file per camera.
    - Automatically rotates (closes current writers and opens new ones)
      every `segment_seconds`.

    This is designed to be fed by an external camera reading thread
    (i.e. VisionPreUntil.ReadCams), so it only handles writing.
    """

    def __init__(self, logger, cam_ports: list[int], cfg: _RecordConfig):
        self.logger = logger
        self.cam_ports = cam_ports
        self.cfg = cfg

        cam_count = len(self.cam_ports)
        if self.cfg.cams is None:
            self.cams = list(range(cam_count))
        else:
            # sanitize
            self.cams = sorted({int(i) for i in self.cfg.cams if 0 <= int(i) < cam_count})
        if not self.cams:
            self.logger.warning("Recording enabled but RecordCams is empty; nothing will be recorded")

        # index-aligned with camera indices; disabled cams keep None.
        self._writers: list[cv2.VideoWriter | None] = [None] * cam_count
        self._segment_start_ts: float | None = None
        self._frame_size: tuple[int, int] | None = None  # (w, h)
        self._current_paths_tmp: list[Path | None] = [None] * cam_count
        self._current_paths_final: list[Path | None] = [None] * cam_count

        # Prepare directories: {RecordDir}/{cam_index}
        for cam_index in self.cams:
            (self.cfg.record_dir / str(cam_index)).mkdir(parents=True, exist_ok=True)

    def _fourcc(self) -> int:
        codec = (self.cfg.codec or "mp4v").strip()
        if len(codec) != 4:
            self.logger.warning(f"RecordCodec should be 4 chars, got '{codec}', fallback to mp4v")
            codec = "mp4v"
        return cv2.VideoWriter_fourcc(*codec)

    def _open_writers(self, frame_size: tuple[int, int]):
        self._close_writers()
        self._frame_size = frame_size
        # File name prefix: YYYY-MM-DD-HH-MM
        # Add a per-segment increasing number to avoid collision.
        ts_prefix = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M")
        if not hasattr(self, "_segment_index"):
            self._segment_index = 0
        self._segment_index += 1
        seg_no = int(self._segment_index)
        fourcc = self._fourcc()

        # Try open with preferred codec/ext; if fails, fallback.
        # Fallback order:
        #   1) mp4/mp4v (preferred; might fail on some OpenCV builds)
        #   2) mp4/avc1 (H.264; might fail without extra deps)
        #   3) avi/MJPG (usually works almost everywhere)
        candidates: list[tuple[str, str]] = []
        candidates.append((self.cfg.codec, self.cfg.ext))
        # add a couple common fallbacks (avoid duplicates)
        for c in [("mp4v", "mp4"), ("avc1", "mp4"), ("MJPG", "avi")]:
            if (c[0], c[1]) not in candidates:
                candidates.append(c)

        chosen: tuple[str, str] | None = None
        # Recording output size can be configured independently from camera capture size.
        w, h = frame_size
        if self.cfg.record_size is not None:
            w, h = self.cfg.record_size
        for codec, ext in candidates:
            try_fourcc = cv2.VideoWriter_fourcc(*codec)
            # Use a unique probe file name to avoid collision with real segment.
            probe_cam = self.cams[0] if self.cams else 0
            test_path = self.cfg.record_dir / str(probe_cam) / f"{ts_prefix}-{seg_no}._probe_.{ext}"
            test_writer = cv2.VideoWriter(str(test_path), try_fourcc, float(self.cfg.fps), (w, h))
            ok = test_writer.isOpened()
            test_writer.release()
            try:
                # remove empty probe file if created
                if test_path.exists():
                    test_path.unlink(missing_ok=True)  # py>=3.8
            except Exception:
                pass
            if ok:
                chosen = (codec, ext)
                break

        if chosen is None:
            chosen = (self.cfg.codec, self.cfg.ext)
            self.logger.warning(
                f"No working codec found among {candidates}; will still try codec={chosen[0]} ext={chosen[1]}"
            )

        # Persist chosen for the rest of the run
        self.cfg.codec, self.cfg.ext = chosen
        fourcc = cv2.VideoWriter_fourcc(*self.cfg.codec)

        # IMPORTANT:
        # MP4 usually becomes playable only after writer.release() (moov atom is written at the end).
        # To avoid users opening an in-progress file and thinking it is "stuck",
        # we write to "*.tmp.ext" and rename to "*.ext" when the segment is finalized.
        cam_count = len(self.cam_ports)
        self._current_paths_tmp = [None] * cam_count
        self._current_paths_final = [None] * cam_count
        self._writers = [None] * cam_count

        for cam_index in self.cams:
            out_dir = self.cfg.record_dir / str(cam_index)
            final_path = out_dir / f"{ts_prefix}-{seg_no}.{self.cfg.ext}"
            tmp_path = out_dir / f"{ts_prefix}-{seg_no}.tmp.{self.cfg.ext}"
            writer = cv2.VideoWriter(str(tmp_path), fourcc, float(self.cfg.fps), (w, h))
            if not writer.isOpened():
                self.logger.warning(
                    f"VideoWriter open failed: {tmp_path} codec={self.cfg.codec} fps={self.cfg.fps} size={frame_size}"
                )
                # IMPORTANT: store None so `any(self._writers)` reflects actual availability.
                try:
                    writer.release()
                except Exception:
                    pass
                self._writers[cam_index] = None
                self._current_paths_tmp[cam_index] = None
                self._current_paths_final[cam_index] = None
            else:
                self._writers[cam_index] = writer
                self._current_paths_tmp[cam_index] = tmp_path
                self._current_paths_final[cam_index] = final_path
        self._segment_start_ts = time.time()
        self.logger.info(
            f"Recording started: dir={self.cfg.record_dir} segment={self.cfg.segment_seconds}s codec={self.cfg.codec} ext={self.cfg.ext}"
        )

    def _close_writers(self):
        for w in getattr(self, "_writers", []):
            if w is None:
                continue
            try:
                w.release()
            except Exception:
                pass
        # keep structure
        self._writers = [None] * len(self.cam_ports)
        self._segment_start_ts = None

        # Finalize segment by renaming tmp files to final names.
        try:
            for tmp_path, final_path in zip(self._current_paths_tmp, self._current_paths_final):
                try:
                    if tmp_path is None or final_path is None:
                        continue
                    if tmp_path.exists():
                        tmp_path.rename(final_path)
                except Exception as e:
                    self.logger.warning(f"Finalize record segment rename failed: {tmp_path} -> {final_path}: {e}")
        finally:
            self._current_paths_tmp = [None] * len(self.cam_ports)
            self._current_paths_final = [None] * len(self.cam_ports)

    def maybe_rotate(self):
        if self._segment_start_ts is None:
            return
        if time.time() - self._segment_start_ts >= float(self.cfg.segment_seconds):
            # rotation will happen on next write with known size
            self._close_writers()

    def write_frames(self, frames: list[np.ndarray | None]):
        """Write a batch of frames (one per camera)."""

        if not self.cfg.enabled:
            return

        # Determine source frame size from the first valid frame.
        if self._frame_size is None:
            for fr in frames:
                if fr is not None:
                    h, w = fr.shape[:2]
                    self._frame_size = (w, h)
                    break

        if self._frame_size is None:
            # no frames yet
            return

        if not any(self._writers):
            self._open_writers(self._frame_size)

        self.maybe_rotate()
        if not any(self._writers):
            # rotated (closed) above; reopen now
            self._open_writers(self._frame_size)

        # Write each camera frame.
        # Precompute resize target (writer size)
        target_size: tuple[int, int] | None = None
        if self.cfg.record_size is not None:
            target_size = self.cfg.record_size

        for cam_index in self.cams:
            if cam_index >= len(frames):
                continue
            writer = self._writers[cam_index]
            if writer is None:
                continue
            fr = frames[cam_index]
            if fr is None:
                continue
            if target_size is not None:
                try:
                    fr = cv2.resize(fr, target_size)
                except Exception as e:
                    self.logger.warning(f"Record resize failed cam={cam_index} target={target_size}: {e}")
            try:
                writer.write(fr)
            except Exception as e:
                self.logger.warning(f"VideoWriter.write failed cam={cam_index}: {e}")

    def close(self):
        self._close_writers()

class VisionPreUntil:
    '''
    Base vision pre-utility.

    Notes
    -----
    Historically this class always initialized 4 V4L2 cameras and started
    background threads immediately. For unit-tests / offline inference we also
    support disabling camera initialization via `init_cams=False`.
    '''

    # Process-level registry so SIGINT/SIGTERM/atexit can release cameras even
    # when callers forget to call close(), or when a capture thread is stuck in
    # a blocking VideoCapture.read().
    _active_instances = set()
    _lifecycle_lock = threading.RLock()
    _process_hooks_installed = False
    _prev_sigint = None
    _prev_sigterm = None

    @classmethod
    def _install_process_hooks(cls) -> None:
        """Register atexit + signal cleanup once per process."""
        with cls._lifecycle_lock:
            if cls._process_hooks_installed:
                return
            cls._process_hooks_installed = True
            try:
                atexit.register(cls._cleanup_all_instances)
            except Exception:
                pass

            def _make_handler(previous):
                def _handler(signum, frame):
                    try:
                        cls._cleanup_all_instances(join_timeout=0.3)
                    except Exception:
                        pass
                    if callable(previous):
                        try:
                            previous(signum, frame)
                            return
                        except Exception:
                            pass
                    if signum == signal.SIGINT:
                        try:
                            signal.default_int_handler(signum, frame)
                            return
                        except Exception:
                            pass
                    raise SystemExit(128 + int(signum))

                return _handler

            try:
                cls._prev_sigint = signal.getsignal(signal.SIGINT)
                signal.signal(signal.SIGINT, _make_handler(cls._prev_sigint))
            except Exception:
                # signal may fail in non-main threads / embedded interpreters
                pass
            try:
                cls._prev_sigterm = signal.getsignal(signal.SIGTERM)
                signal.signal(signal.SIGTERM, _make_handler(cls._prev_sigterm))
            except Exception:
                pass

    @classmethod
    def _register_instance(cls, instance) -> None:
        cls._install_process_hooks()
        with cls._lifecycle_lock:
            cls._active_instances.add(instance)

    @classmethod
    def _unregister_instance(cls, instance) -> None:
        with cls._lifecycle_lock:
            cls._active_instances.discard(instance)

    @classmethod
    def _cleanup_all_instances(cls, join_timeout: float = 0.5) -> None:
        with cls._lifecycle_lock:
            instances = list(cls._active_instances)
        for instance in instances:
            try:
                instance.close(join_timeout=join_timeout)
            except Exception:
                pass

    def __init__(self, init_cams: bool = True, cam_ports=None):
        # Allow subclasses to pre-create a stop event.
        # If not provided, create one so we can shutdown gracefully in tests.
        if not hasattr(self, "_stop_event") or self._stop_event is None:
            self._stop_event = threading.Event()

        # Guard against re-entrant/double close (common on signal + atexit + __del__).
        self._close_lock = threading.RLock()
        self._closed = False
        self._cams_lock = threading.RLock()

        self.cfg = Settings
        self.logger = logger

        # TODO put into settings
        # self.CamPorts = [8,10,1,5]
        self.CamPorts = cam_ports or list(getattr(Settings.Vision, "CamPorts", [0, 2, 4, 6]))
        self.StopRecord = 0

        self.Cams = []
        # Always keep 4 slots, even when cameras are not initialized.
        self.Frames = [None, None, None, None]
        self.Videos = []

        self.BallPos = [0,0]

        # thread lifecycle (for clean shutdown)
        self.ReadCamsThread = None
        self.VideoRecordThread = None

        # recorder (optional)
        self._recorder: SegmentedMultiCamRecorder | None = None

        if init_cams:
            self.InitCam(self.CamPorts)

        self.ReadCamTF = 0
        self.PreProcessTF = 0
        self.InferTF = 0
        self.PostProcessTF = 0
        self.PreProcessMs = 0.0
        self.InferMs = 0.0
        self.InferBatchMs = 0.0
        self.InferBatchSize = 0
        self.PostProcessMs = 0.0


        if init_cams:
            self.ReadCamsThread = threading.Thread(target=self.ReadCams)
            self.ReadCamsThread.daemon = True
            self.ReadCamsThread.start()
            # Always track live camera owners so unexpected process exit can
            # still release V4L2/GStreamer handles.
            self._register_instance(self)

        # Recording initialization (isolated; does not affect non-record runs)
        vision_vals = getattr(self.cfg, "VisionVals", object())
        vision_cfg = getattr(self.cfg, "Vision", object())
        if getattr(vision_vals, "Record", getattr(vision_cfg, "Record", False)):
            # Default: store recordings next to ReasonData log file (ReasonData/data).
            record_dir_val = str(getattr(vision_vals, "RecordDir", getattr(vision_cfg, "RecordDir", "./Records")))
            record_dir_path = Path(record_dir_val)
            if not record_dir_path.is_absolute() and REASON_DATA_DIR is not None:
                # Normalize "./xxx" -> "xxx" so it becomes {DATA_DIR}/xxx
                if record_dir_val.startswith("./"):
                    record_dir_path = Path(record_dir_val[2:])
                record_dir_path = Path(REASON_DATA_DIR) / record_dir_path

            self._recorder = SegmentedMultiCamRecorder(
                logger=self.logger,
                cam_ports=self.CamPorts,
                cfg=_RecordConfig(
                    enabled=True,
                    record_dir=record_dir_path,
                    cams=list(getattr(vision_vals, "RecordCams", [0, 1, 2, 3])),
                    record_size=(
                        int(getattr(vision_vals, "RecordWidth", 320)),
                        int(getattr(vision_vals, "RecordHeight", 320)),
                    ),
                    segment_seconds=int(getattr(vision_vals, "RecordSegmentSeconds", 60)),
                    fps=int(getattr(vision_vals, "RecordFPS", 30)),
                    codec=str(getattr(vision_vals, "RecordCodec", "mp4v")),
                    ext=str(getattr(vision_vals, "RecordExt", "mp4")),
                ),
            )

            self.VideoRecordThread = threading.Thread(target=self.VideoRecord)
            self.VideoRecordThread.daemon = True
            self.VideoRecordThread.start()


    def close(self, join_timeout: float = 1.0):
        """Stop background threads and release camera/video resources.

        Important ordering note:
        OpenCV `VideoCapture.read()` can block indefinitely. We therefore
        release cameras *before* joining the reader thread so a stuck read is
        interrupted by the handle teardown whenever the backend allows it.
        """
        close_lock = getattr(self, "_close_lock", None)
        if close_lock is None:
            close_lock = threading.RLock()
            self._close_lock = close_lock

        with close_lock:
            if getattr(self, "_closed", False):
                return
            self._closed = True

            self.stop()

            # Release cameras first so blocked reader threads can observe errors
            # and exit instead of holding device nodes forever.
            self._release_cameras()

            for t in (getattr(self, "ReadCamsThread", None), getattr(self, "VideoRecordThread", None)):
                try:
                    if t is not None and t.is_alive():
                        t.join(timeout=join_timeout)
                except Exception:
                    pass

            # release videos / segment headers
            try:
                self.CloseVideo()
            except Exception:
                pass

            self._unregister_instance(self)

    def _release_cameras(self) -> None:
        """Release all owned VideoCapture handles and clear the camera list."""
        cams_lock = getattr(self, "_cams_lock", None)
        if cams_lock is None:
            cams_lock = threading.RLock()
            self._cams_lock = cams_lock

        with cams_lock:
            cams = list(getattr(self, "Cams", []) or [])
            self.Cams = []

        for cam in cams:
            try:
                if cam is None:
                    continue
                cam.release()
            except Exception:
                pass

    def _camera_snapshot(self) -> list:
        """Thread-safe snapshot of current camera handles."""
        cams_lock = getattr(self, "_cams_lock", None)
        if cams_lock is None:
            return list(getattr(self, "Cams", []) or [])
        try:
            with cams_lock:
                return list(getattr(self, "Cams", []) or [])
        except Exception:
            return list(getattr(self, "Cams", []) or [])

    def _configured_exposure_mode(
        self,
        legacy_auto_exposure: float = 0,
    ) -> int:
        """Return normalized exposure mode from settings."""

        vision_vals = getattr(self.cfg, "VisionVals", object())
        exposure_mode = getattr(vision_vals, "ExposureMode", None)
        if exposure_mode is None:
            legacy_auto_exposure = getattr(
                vision_vals,
                "AutoExposure",
                legacy_auto_exposure,
            )
            try:
                exposure_mode = 1 if int(legacy_auto_exposure) == 1 else 0
            except (TypeError, ValueError):
                exposure_mode = 0
        else:
            try:
                exposure_mode = 1 if int(exposure_mode) == 1 else 0
            except (TypeError, ValueError):
                exposure_mode = 0

        return exposure_mode

    def _configured_exposure_bias(self) -> int:
        """Return stable-mode exposure compensation in whole EV stops."""

        value = getattr(getattr(self.cfg, "VisionVals", object()), "ExposureBias", 0)
        try:
            return max(-4, min(4, int(value)))
        except (TypeError, ValueError):
            return 0

    def _apply_camera_exposure(
        self,
        cam,
        port: int,
        exposure_mode: int,
        exposure_bias: int,
    ) -> dict:
        """Apply one exposure profile to an already-opened V4L2 camera.

        Flexible mode keeps continuous auto exposure. Stable mode meters once,
        then locks a compensation-adjusted value derived from that same meter
        reading. Reapplying a bias cannot compound exposure changes.
        """

        if cam is None:
            return {"port": int(port), "success": False, "error": "camera unavailable"}

        metered_exposure = None
        target_exposure = None
        try:
            if exposure_mode == 1:
                # Always meter afresh before locking. ExposureBias is a
                # persistent EV offset relative to the current environment,
                # rather than an adjustment relative to a prior lock.
                # This camera's proven auto/manual transition is 0 -> 1.
                # Value 3 leaves its prior manual exposure influencing the
                # metering loop, which makes a persistent Bias compound over
                # later launches.
                auto_exposure_set = cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.0)
                # Feed the sensor enough frames for its auto-exposure loop
                # to settle before locking the result. This is the
                # programmatic equivalent of changing 0 -> 1 manually.
                # Some UVC devices report their lower bound (5) for several
                # seconds immediately after enabling auto exposure, so keep
                # metering beyond the initial warm-up when necessary.
                warmup_deadline = time.monotonic() + 3.0
                settle_deadline = warmup_deadline + 7.0
                reader = getattr(self, "ReadCamsThread", None)
                while True:
                    if reader is not None and reader.is_alive():
                        # The existing reader supplies frames to the camera;
                        # do not read concurrently from this thread.
                        time.sleep(0.1)
                    else:
                        ok, _frame = cam.read()
                        if not ok:
                            time.sleep(0.02)
                    metered_exposure = float(cam.get(cv2.CAP_PROP_EXPOSURE))
                    now = time.monotonic()
                    if now >= warmup_deadline and metered_exposure > 5.0:
                        break
                    if now >= settle_deadline:
                        break

                if metered_exposure <= 5.0:
                    exposure_set = True
                    self.logger.warning(
                        f"Camera exposure meter not settled: /dev/video{port} "
                        f"metered={metered_exposure}; keeping auto exposure enabled "
                        f"and not applying bias={exposure_bias}"
                    )
                else:
                    target_exposure = max(
                        1.0,
                        round(float(metered_exposure) * (2.0 ** int(exposure_bias))),
                    )
                    exposure_set = cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)
                    time.sleep(0.05)
                    exposure_set = bool(
                        exposure_set and cam.set(cv2.CAP_PROP_EXPOSURE, target_exposure)
                    )
            else:
                auto_exposure_set = cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.0)
                exposure_set = True
            actual_auto_exposure = float(cam.get(cv2.CAP_PROP_AUTO_EXPOSURE))
            actual_exposure = float(cam.get(cv2.CAP_PROP_EXPOSURE))
        except Exception as e:
            self.logger.warning(f"Camera exposure apply failed: /dev/video{port}: {e}")
            return {"port": int(port), "success": False, "error": str(e)}

        success = bool(auto_exposure_set and exposure_set)
        if not success:
            self.logger.warning(
                f"Camera exposure control unsupported: /dev/video{port} "
                f"mode={exposure_mode}"
            )
        else:
            self.logger.info(
                f"Camera exposure configured: /dev/video{port} "
                f"mode={exposure_mode} auto={actual_auto_exposure} "
                f"exposure={actual_exposure} bias={exposure_bias} "
                f"metered={metered_exposure}"
            )
        return {
            "port": int(port),
            "success": success,
            "mode": exposure_mode,
            "auto_exposure": actual_auto_exposure,
            "exposure": actual_exposure,
            "metered_exposure": metered_exposure,
            "target_exposure": target_exposure,
        }

    def ApplyExposureSettings(
        self,
        exposure_mode: int | None = None,
        exposure_bias: int | None = None,
    ) -> list[dict]:
        """Apply exposure settings to active cameras without restarting vision."""

        configured_mode = self._configured_exposure_mode()
        configured_bias = self._configured_exposure_bias()
        if exposure_mode is None:
            exposure_mode = configured_mode
        else:
            try:
                exposure_mode = 1 if int(exposure_mode) == 1 else 0
            except (TypeError, ValueError):
                exposure_mode = configured_mode
        if exposure_bias is None:
            exposure_bias = configured_bias
        else:
            try:
                exposure_bias = max(-4, min(4, int(exposure_bias)))
            except (TypeError, ValueError):
                exposure_bias = configured_bias
        # Keep the in-memory singleton coherent with the setting just saved by
        # the control panel, including any later camera reopen.
        for section_name in ("Vision", "VisionVals"):
            section = getattr(self.cfg, section_name, None)
            if section is not None:
                setattr(section, "ExposureMode", exposure_mode)
                setattr(section, "ExposureBias", exposure_bias)
                setattr(section, "AutoExposure", 1 if exposure_mode == 1 else 0)

        ports = list(getattr(self, "CamPorts", []) or [])
        results = []
        for index, cam in enumerate(self._camera_snapshot()):
            port = ports[index] if index < len(ports) else index
            result = self._apply_camera_exposure(cam, port, exposure_mode, exposure_bias)
            results.append(result)
        return results

    def _safe_cam_read(self, cam):
        """Read one frame, tolerating released/invalid captures during shutdown."""
        if cam is None:
            return False, None
        try:
            if hasattr(cam, "isOpened") and not cam.isOpened():
                return False, None
        except Exception:
            return False, None
        try:
            return cam.read()
        except Exception:
            return False, None

    def reopen_cameras(self, cam_ports=None, **init_kwargs) -> None:
        """Force-release then re-open cameras.

        Useful after an unclean previous run or when a capture path gets stuck.
        """
        ports = list(cam_ports) if cam_ports is not None else list(getattr(self, "CamPorts", []) or [])
        self.CamPorts = ports
        self._release_cameras()
        if not ports:
            return
        try:
            self.InitCam(ports, **init_kwargs)
        except Exception as e:
            self.logger.error(f"reopen_cameras failed: {e}")

        # Ensure a reader is running after a forced reopen.
        thread = getattr(self, "ReadCamsThread", None)
        if thread is None or not thread.is_alive():
            if not hasattr(self, "_stop_event") or self._stop_event is None:
                self._stop_event = threading.Event()
            else:
                try:
                    self._stop_event.clear()
                except Exception:
                    self._stop_event = threading.Event()
            self._closed = False
            self.ReadCamsThread = threading.Thread(target=self.ReadCams, daemon=True)
            self.ReadCamsThread.start()
            self._register_instance(self)

    def __del__(self):
        try:
            self.close(join_timeout=0.2)
        except Exception:
            pass

    def _init_calibration(self):
        """Load only pixel2cm_v2 raw-pixel-to-ground-plane calibrations."""

        projectors: list[ModernCalibration | None] = []
        calibration_errors: list[str] = []

        vision_vals = getattr(self.cfg, "VisionVals", object())
        vision_cfg = getattr(self.cfg, "Vision", object())
        calib_sub = _norm_rel_folder(
            getattr(vision_vals, "CalibrationFolder", getattr(vision_cfg, "CalibrationFolder", None)),
            "Calibration",
        )
        calib_dir = DATA_DIR / calib_sub
        for port in self.CamPorts:
            npz_path = calib_dir / f"CalibrationData{port}.npz"
            try:
                projectors.append(ModernCalibration.load(npz_path))
            except Exception as e:
                self.logger.error(f"Modern calibration required for camera port {port} ({npz_path}): {e}")
                projectors.append(None)
                calibration_errors.append(str(e))

        self.CalibrationProjectors = projectors
        self.CalibrationModes = ["modern" if projector is not None else "missing" for projector in projectors]
        self.CalibrationImageSizes = [None if projector is None else projector.image_size for projector in projectors]
        self.CalibrationErrors = calibration_errors

    def reload_calibration(self):
        self._init_calibration()
        return {"modes": list(getattr(self, "CalibrationModes", [])), "errors": list(getattr(self, "CalibrationErrors", []))}


    def ApplyPerspectiveTransform(self, X, Y, Matrix):
        """Apply a 3x3 perspective matrix on a pixel point."""
        point = np.array([X, Y, 1], dtype=np.float64)
        transformed = Matrix @ point
        transformed /= transformed[2]
        return int(transformed[1]), int(transformed[0])

    def _apply_homography_float(self, X, Y, Matrix):
        point = np.array([float(X), float(Y), 1.0], dtype=np.float64)
        transformed = Matrix @ point
        if abs(float(transformed[2])) < 1e-12:
            return float(X), float(Y)
        transformed /= transformed[2]
        return float(transformed[0]), float(transformed[1])

    @staticmethod
    def _undistort_point(X, Y, K, dist):
        pt = np.array([[[float(X), float(Y)]]], dtype=np.float64)
        und = cv2.undistortPoints(pt, K, dist, P=K)
        return float(und[0, 0, 0]), float(und[0, 0, 1])


    def Pixel2CM(self, X, Y, CamIndex: int, frame_size: tuple[int, int] | None = None):
        """Project a raw camera pixel to the calibrated robot-centred ground plane."""

        if not hasattr(self, "CalibrationProjectors"):
            self._init_calibration()
        try:
            projector = self.CalibrationProjectors[int(CamIndex)]
            if projector is None:
                raise ModernCalibrationError(f"camera {CamIndex} has no modern calibration")
            return projector.pixel_to_cm(X, Y, frame_size=frame_size)
        except Exception as exc:
            raise ModernCalibrationError(f"Pixel2CM failed (CamIndex={CamIndex}, X={X}, Y={Y}): {exc}") from exc


    def CM2Pixel(self, X, Y, CamIndex: int):
        """Project a robot-centred ground-plane point into raw camera pixels."""

        if not hasattr(self, "CalibrationProjectors"):
            self._init_calibration()
        try:
            projector = self.CalibrationProjectors[int(CamIndex)]
            if projector is None:
                raise ModernCalibrationError(f"camera {CamIndex} has no modern calibration")
            return projector.cm_to_pixel(X, Y)
        except Exception as exc:
            raise ModernCalibrationError(f"CM2Pixel failed (CamIndex={CamIndex}, X={X}, Y={Y}): {exc}") from exc


    def InitVideo(self):
        """Backward-compatible stub.

        Recording is now handled by SegmentedMultiCamRecorder.
        Kept to avoid breaking external code that may call InitVideo/CloseVideo.
        """
        return

    def CloseVideo(self):
        """Backward-compatible stub. See InitVideo."""
        if getattr(self, "_recorder", None) is not None:
            try:
                self._recorder.close()
            except Exception:
                pass

    def VideoRecord(self):
        """Recording loop.

        The actual file writing is handled by `SegmentedMultiCamRecorder`.
        We keep this thread separate from camera reading/inference to avoid
        blocking the pipeline when filesystem is slow.
        """
        fps = int(getattr(getattr(self.cfg, "VisionVals", object()), "RecordFPS", 30))
        period = 1.0 / float(max(1, fps))
        while not self._stop_event.is_set():
            if getattr(self, "_recorder", None) is not None:
                try:
                    # copy to avoid race with ReadCams updating the list
                    frames = list(getattr(self, "Frames", []))
                    self._recorder.write_frames(frames)
                except Exception as e:
                    self.logger.warning(f"Recorder write_frames failed: {e}")
            time.sleep(period)


    def InitCam(self, CamPorts, Width=640, Height=480, AutoExposure=0, Exposure=157, Brightness=0, Contrast=32, Saturation=64):
        """Initialize cameras.

        Backward compatible behavior:
          - still exposes `self.Cams` as a list of cv2.VideoCapture
          - still supports all existing callers without changes

        New behavior (isolated by platform/config):
          - When Settings.VisionVals.CaptureBackend == 'auto' and device is Jetson Nano,
            try to open cameras via GStreamer pipeline (hw accel).
          - If GStreamer fails, fallback to legacy V4L2 VideoCapture.
        """

        # Read config with safe defaults (old configs won't have these keys)
        backend = getattr(getattr(self.cfg, "VisionVals", object()), "CaptureBackend", "auto")
        codec = getattr(getattr(self.cfg, "VisionVals", object()), "CaptureCodec", "mjpg")
        Width = int(getattr(getattr(self.cfg, "VisionVals", object()), "CamWidth", Width))
        Height = int(getattr(getattr(self.cfg, "VisionVals", object()), "CamHeight", Height))
        fps = int(getattr(getattr(self.cfg, "VisionVals", object()), "CamFps", 30))
        exposure_mode = self._configured_exposure_mode(AutoExposure)
        exposure_bias = self._configured_exposure_bias()
        exposure_results = []

        use_gst = False
        if isinstance(backend, str):
            b = backend.lower()
            if b == "gstreamer":
                use_gst = True
            elif b == "v4l2":
                use_gst = False
            else:  # auto
                use_gst = _is_jetson_nano()

        for Port in CamPorts:
            Cam = None

            # Try gstreamer first (Jetson Nano only by default)
            if use_gst:
                try:
                    Cam, pipeline = _try_open_gstreamer(Port, Width, Height, fps, codec=codec, logger=self.logger)
                    if Cam is None:
                        self.logger.warning(
                            f"GStreamer open failed for /dev/video{Port} codec={codec}, fallback to V4L2"
                        )
                    else:
                        self.logger.info(
                            f"Camera opened with GStreamer: /dev/video{Port} codec={codec} {Width}x{Height}@{fps}"
                        )
                        if pipeline:
                            self.logger.debug(f"GStreamer pipeline: {pipeline}")
                except Exception as e:
                    self.logger.warning(f"GStreamer init exception for /dev/video{Port}: {e}. Fallback to V4L2")
                    Cam = None

            # Legacy V4L2 backend
            if Cam is None:
                Cam = cv2.VideoCapture(Port, cv2.CAP_V4L2)
                Cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                Cam.set(cv2.CAP_PROP_FRAME_WIDTH, Width)
                Cam.set(cv2.CAP_PROP_FRAME_HEIGHT, Height)
                result = self._apply_camera_exposure(
                    Cam,
                    Port,
                    exposure_mode,
                    exposure_bias,
                )
                exposure_results.append(result)
                Cam.set(cv2.CAP_PROP_BRIGHTNESS, Brightness)
                Cam.set(cv2.CAP_PROP_CONTRAST, Contrast)
                Cam.set(cv2.CAP_PROP_SATURATION, Saturation)
                if not Cam.isOpened():
                    self.logger.error(f"Camera open failed: /dev/video{Port} (index={Port})")
                    try:
                        Cam.release()
                    except Exception:
                        pass
                    Cam = None

            cams_lock = getattr(self, "_cams_lock", None)
            if cams_lock is None:
                self.Cams.append(Cam)
            else:
                with cams_lock:
                    self.Cams.append(Cam)

    def ReadCams(self):
        FrameCount = 0
        warn_flag = False
        LastTime = time.time()
        fail_counts = []
        while not self._stop_event.is_set():
            cams = self._camera_snapshot()
            if not cams:
                # Cameras were released during shutdown/reopen; wait for stop or reinit.
                self._stop_event.wait(0.02)
                continue

            if len(fail_counts) != len(cams):
                fail_counts = [0] * len(cams)

            Frames = []
            any_ok = False
            for idx, Cam in enumerate(cams):
                ret, Frame = self._safe_cam_read(Cam)
                if (not ret) or Frame is None:
                    fail_counts[idx] += 1
                    Frames.append(None)
                    if fail_counts[idx] in (10, 30, 100):
                        port = self.CamPorts[idx] if idx < len(self.CamPorts) else idx
                        self.logger.warning(f"Camera read failed /dev/video{port} count={fail_counts[idx]}")
                else:
                    any_ok = True
                    fail_counts[idx] = 0
                    Frames.append(Frame)

            # Keep a stable 4-slot Frames list for consumers.
            while len(Frames) < 4:
                Frames.append(None)
            self.Frames = Frames[: max(4, len(Frames))]
            FrameCount += 1
            CurrentTime = time.time()
            if CurrentTime - LastTime >= 1.0:
                if self.cfg.Debug.FullLog and not warn_flag:
                    warn_flag = True
                    self.logger.debug("Camera FPS: "+str(FrameCount))
                FrameCount = 0
                LastTime = CurrentTime
            self.ReadCamTF = self.ReadCamTF + 1

            # If all cameras are failing, avoid a busy-loop that will eat CPU and
            # starve inference threads.
            if not any_ok:
                self._stop_event.wait(0.01)



    # def BinaryObjectDetection(self):
    #     二值化检测对方
    #     Frame = self.Frames[0]
    #     Pos = [self.GetPos()[0], self.GetPos()[1]]
    #     Yaw = self.GetPos()[2]
    #     DistToCorner = int(math.sqrt((Pos[0] - 10) ** 2 + (Pos[1] - 13) ** 2))
    #     VisionAngle = math.degrees(math.atan2(Pos[0] - 10, Pos[1] - 13))
    #     Theta = VisionAngle - Yaw
    #     VisionCornerX = int(DistToCorner * math.sin(math.radians(Theta)))
    #     VisionCornerY = int(DistToCorner * math.cos(math.radians(Theta)))
    #     VisionCornerX, VisionCornerY = self.CM2Pixel(VisionCornerX, VisionCornerY, 0)
    #     print(VisionCornerX, VisionCornerY)

    def stop(self):
        """Request background threads to stop (non-blocking)."""
        try:
            self._stop_event.set()
        except Exception:
            pass

    def __exit__(self, *args, **kwargs):
        self.close(join_timeout=0.5)
