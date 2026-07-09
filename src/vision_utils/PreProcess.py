"""Shared vision preprocessing utilities.

This module is used by multiple vision backends (Hailo / BPU / Blob).

Compatibility note:
- The main system config is `src/ReasonData/data/config.json`.
- `vision_utils` historically also shipped its own config `vision_utils/VisionDatas/data/vision.json`.

To reduce "config split" issues, we prefer importing `ReasonData.Settings/logger`
when available, and fallback to `vision_utils.VisionDatas`.
"""

from pathlib import Path

_REASON_DATA_DIR_FALLBACK = Path(__file__).resolve().parents[1] / "ReasonData" / "data"

# Settings/logger import order:
#  1) ReasonData (installed package usage)
#  2) src.ReasonData (running from repo root, imports as "src.*")
#  3) merged_settings (vision-only merged config; always provides VisionVals defaults)
#  4) legacy VisionDatas (vision-only config)
try:
    from ReasonData import Settings, logger, DATA_DIR as REASON_DATA_DIR  # type: ignore
except Exception:  # pragma: no cover
    try:
        from src.ReasonData import Settings, logger, DATA_DIR as REASON_DATA_DIR  # type: ignore
    except Exception:  # pragma: no cover
        try:
            # merged_settings does not expose logger; keep legacy logger
            from .merged_settings import Settings  # type: ignore
            try:
                from .VisionDatas import logger
            except Exception:
                import logging

                logger = logging.getLogger(__name__)
            REASON_DATA_DIR = _REASON_DATA_DIR_FALLBACK  # type: ignore
        except Exception:  # pragma: no cover
            try:
                from .VisionDatas import Settings, logger
            except Exception:
                from .merged_settings import Settings  # type: ignore
                import logging

                logger = logging.getLogger(__name__)
            REASON_DATA_DIR = _REASON_DATA_DIR_FALLBACK  # type: ignore
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


def _npz_text(data, key: str, default: str = "") -> str:
    if key not in data.files:
        return default
    try:
        return str(np.atleast_1d(data[key])[0])
    except Exception:
        return default


def _npz_bool(data, key: str, default: bool = False) -> bool:
    if key not in data.files:
        return default
    try:
        value = np.atleast_1d(data[key])[0]
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)
    except Exception:
        return default


def _modern_pose_from_npz(data):
    H = np.asarray(data["H"], dtype=np.float64)
    rvec = np.asarray(data["rvec"], dtype=np.float64) if "rvec" in data.files else None
    tvec = np.asarray(data["tvec"], dtype=np.float64) if "tvec" in data.files else None
    if rvec is not None and rvec.size == 0:
        rvec = None
    if tvec is not None and tvec.size == 0:
        tvec = None

    if (
        _npz_text(data, "origin") == "robot_center"
        and _npz_text(data, "coordinate_convention") != "robot_center_v2"
        and "origin_offset_cm" in data.files
    ):
        offset = np.asarray(data["origin_offset_cm"], dtype=np.float64).reshape(-1)
        if offset.size >= 2:
            dy_mm = 2.0 * float(offset[1]) * 10.0
            new_to_old = np.array(
                [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, -dy_mm],
                    [0.0, 0.0, 1.0],
                ],
                dtype=np.float64,
            )
            H = H @ new_to_old
            if rvec is not None and tvec is not None:
                R, _ = cv2.Rodrigues(rvec)
                d = np.array([[0.0], [-dy_mm], [0.0]], dtype=np.float64)
                tvec = np.asarray(tvec, dtype=np.float64).reshape(3, 1) + R @ d

    return H, rvec, tvec


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
    def __init__(self, init_cams: bool = True, cam_ports=None):
        # Allow subclasses to pre-create a stop event.
        # If not provided, create one so we can shutdown gracefully in tests.
        if not hasattr(self, "_stop_event") or self._stop_event is None:
            self._stop_event = threading.Event()

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

        # Recording initialization (isolated; does not affect non-record runs)
        vision_vals = getattr(self.cfg, "VisionVals", object())
        vision_cfg = getattr(self.cfg, "Vision", object())
        if getattr(vision_vals, "Record", getattr(vision_cfg, "Record", False)):
            # Ensure mp4/avi headers are finalized on exit.
            atexit.register(self.close)
            try:
                # Cleanup on SIGINT/SIGTERM (best effort)
                def _sig_cleanup_and_exit(signum, frame):
                    try:
                        self.stop()
                        try:
                            self.CloseVideo()
                        except Exception:
                            pass
                    finally:
                        if signum == signal.SIGINT:
                            signal.default_int_handler(signum, frame)
                        else:
                            raise SystemExit(0)

                signal.signal(signal.SIGINT, _sig_cleanup_and_exit)
                signal.signal(signal.SIGTERM, _sig_cleanup_and_exit)
            except Exception:
                # signal may fail in some embedded/threads env; atexit still helps.
                pass

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
        """Stop background threads and release camera/video resources."""
        self.stop()

        for t in (getattr(self, "ReadCamsThread", None), getattr(self, "VideoRecordThread", None)):
            try:
                if t is not None and t.is_alive():
                    t.join(timeout=join_timeout)
            except Exception:
                pass

        # release cameras
        try:
            for Cam in getattr(self, "Cams", []):
                try:
                    Cam.release()
                except Exception:
                    pass
        except Exception:
            pass

        # release videos
        try:
            self.CloseVideo()
        except Exception:
            pass


    def __del__(self):
        try:
            self.close(join_timeout=0.2)
        except Exception:
            pass

    def _init_calibration(self):
        """Load per-camera calibration used by Pixel2CM/CM2Pixel.

        Expected file: {DATA_DIR}/Calibration/CalibrationData{port}.npz
        with keys:
        - matrix: 3x3 perspective transform
        - p2c: (k, hb, vb)
        """

        perspective_matrices: list[np.ndarray] = []
        p2ck: list[float] = []
        p2chb: list[float] = []
        p2cvb: list[float] = []
        calibration_ks: list[np.ndarray | None] = []
        calibration_dists: list[np.ndarray | None] = []
        calibration_hs: list[np.ndarray | None] = []
        calibration_rvecs: list[np.ndarray | None] = []
        calibration_tvecs: list[np.ndarray | None] = []
        calibration_modes: list[str] = []
        calibration_undistort_points: list[bool] = []

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
                data = np.load(str(npz_path))
                matrix = data["matrix"]
                p2c = data["p2c"]

                perspective_matrices.append(matrix)
                # Be tolerant to p2c shape (sometimes it's nested).
                p2ck.append(float(p2c[0]))
                p2chb.append(float(p2c[1]))
                p2cvb.append(float(p2c[2]))
                if {"K", "dist", "H"}.issubset(set(data.files)) or (
                    "version" in data.files and int(np.atleast_1d(data["version"])[0]) >= 2
                ):
                    H, rvec, tvec = _modern_pose_from_npz(data)
                    calibration_ks.append(np.asarray(data["K"], dtype=np.float64))
                    calibration_dists.append(np.asarray(data["dist"], dtype=np.float64))
                    calibration_hs.append(H)
                    calibration_rvecs.append(rvec)
                    calibration_tvecs.append(tvec)
                    calibration_modes.append("modern")
                    calibration_undistort_points.append(_npz_bool(data, "undistort_points", True))
                else:
                    calibration_ks.append(None)
                    calibration_dists.append(None)
                    calibration_hs.append(None)
                    calibration_rvecs.append(None)
                    calibration_tvecs.append(None)
                    calibration_modes.append("legacy")
                    calibration_undistort_points.append(False)
            except Exception as e:
                self.logger.warning(
                    f"Calibration load failed for camera port {port} ({npz_path}): {e}. "
                    "Fallback to identity Pixel2CM."
                )
                perspective_matrices.append(np.eye(3, dtype=np.float64))
                p2ck.append(1.0)
                p2chb.append(0.0)
                p2cvb.append(0.0)
                calibration_ks.append(None)
                calibration_dists.append(None)
                calibration_hs.append(None)
                calibration_rvecs.append(None)
                calibration_tvecs.append(None)
                calibration_modes.append("missing")
                calibration_undistort_points.append(False)

        self.PerspectiveMatrices = perspective_matrices
        self.P2CK = p2ck
        self.P2CHB = p2chb
        self.P2CVB = p2cvb
        self.CalibrationKs = calibration_ks
        self.CalibrationDists = calibration_dists
        self.CalibrationHs = calibration_hs
        self.CalibrationRvecs = calibration_rvecs
        self.CalibrationTvecs = calibration_tvecs
        self.CalibrationModes = calibration_modes
        self.CalibrationUndistortPoints = calibration_undistort_points

    def reload_calibration(self):
        self._init_calibration()
        return {"modes": list(getattr(self, "CalibrationModes", []))}


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


    def Pixel2CM(self, X, Y, CamIndex: int):
        """Convert pixel coordinate to cm coordinate for a specific camera.

        Notes:
        - This is shared by Hailo/BPU/Blob pipelines.
        - If calibration wasn't loaded, returns (X, Y) as-is.
        """

        if not getattr(self, "PerspectiveMatrices", None):
            self._init_calibration()

        try:
            cam_i = int(CamIndex)
            if (
                0 <= cam_i < len(self.CalibrationModes)
                and self.CalibrationModes[cam_i] == "modern"
                and self.CalibrationHs[cam_i] is not None
            ):
                u, v = float(X), float(Y)
                undistort = (
                    cam_i < len(getattr(self, "CalibrationUndistortPoints", []))
                    and self.CalibrationUndistortPoints[cam_i]
                    and self.CalibrationKs[cam_i] is not None
                    and self.CalibrationDists[cam_i] is not None
                )
                if undistort:
                    u, v = self._undistort_point(X, Y, self.CalibrationKs[cam_i], self.CalibrationDists[cam_i])
                world_x_mm, world_y_mm = self._apply_homography_float(u, v, np.linalg.inv(self.CalibrationHs[cam_i]))
                return round(world_x_mm / 10.0, 2), round(world_y_mm / 10.0, 2)

            X, Y = self.ApplyPerspectiveTransform(X, Y, self.PerspectiveMatrices[cam_i])
            X = round(self.P2CK[cam_i] * X + self.P2CHB[cam_i], 2)
            Y = round(-self.P2CK[cam_i] * Y + self.P2CVB[cam_i], 2)
            return X, Y
        except Exception as e:
            self.logger.warning(f"Pixel2CM failed (CamIndex={CamIndex}, X={X}, Y={Y}): {e}")
            return X, Y


    def CM2Pixel(self, X, Y, CamIndex: int):
        """Convert cm coordinate back to pixel coordinate for a specific camera."""

        if not getattr(self, "PerspectiveMatrices", None):
            self._init_calibration()

        try:
            cam_i = int(CamIndex)
            if (
                0 <= cam_i < len(self.CalibrationModes)
                and self.CalibrationModes[cam_i] == "modern"
            ):
                undistort = (
                    cam_i < len(getattr(self, "CalibrationUndistortPoints", []))
                    and self.CalibrationUndistortPoints[cam_i]
                    and self.CalibrationKs[cam_i] is not None
                    and self.CalibrationDists[cam_i] is not None
                )
                obj = np.array([[[float(X) * 10.0, float(Y) * 10.0, 0.0]]], dtype=np.float64)
                rvec = self.CalibrationRvecs[cam_i] if cam_i < len(self.CalibrationRvecs) else None
                tvec = self.CalibrationTvecs[cam_i] if cam_i < len(self.CalibrationTvecs) else None
                if undistort and rvec is not None and tvec is not None:
                    img, _ = cv2.projectPoints(obj, rvec, tvec, self.CalibrationKs[cam_i], self.CalibrationDists[cam_i])
                    return int(round(float(img[0, 0, 0]))), int(round(float(img[0, 0, 1])))

                H = self.CalibrationHs[cam_i] if cam_i < len(self.CalibrationHs) else None
                if H is not None:
                    px_x, px_y = self._apply_homography_float(float(X) * 10.0, float(Y) * 10.0, H)
                    return int(round(px_x)), int(round(px_y))

            p2ck = self.P2CK[cam_i]
            p2chb = self.P2CHB[cam_i]
            p2cvb = self.P2CVB[cam_i]

            X = (X - p2chb) / p2ck
            Y = -(Y - p2cvb) / p2ck
            X, Y = self.ApplyPerspectiveTransform(X, Y, np.linalg.inv(self.PerspectiveMatrices[cam_i]))
            return int(X), int(Y)
        except Exception as e:
            self.logger.warning(f"CM2Pixel failed (CamIndex={CamIndex}, X={X}, Y={Y}): {e}")
            return int(X), int(Y)


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


    def InitCam(self, CamPorts, Width=640, Height=480, AutoExposure=3, Exposure=157, Brightness=0, Contrast=32, Saturation=64):
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
                # Exposure controls may not be supported on all cameras.
                Cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, AutoExposure)
                Cam.set(cv2.CAP_PROP_EXPOSURE, Exposure)
                Cam.set(cv2.CAP_PROP_BRIGHTNESS, Brightness)
                Cam.set(cv2.CAP_PROP_CONTRAST, Contrast)
                Cam.set(cv2.CAP_PROP_SATURATION, Saturation)
                if not Cam.isOpened():
                    self.logger.error(f"Camera open failed: /dev/video{Port} (index={Port})")

            self.Cams.append(Cam)

    def ReadCams(self):
        FrameCount = 0
        warn_flag = False
        LastTime = time.time()
        fail_counts = [0] * len(self.Cams)
        while not self._stop_event.is_set():
            Frames = []
            any_ok = False
            for idx, Cam in enumerate(self.Cams):
                ret, Frame = Cam.read()
                if (not ret) or Frame is None:
                    fail_counts[idx] += 1
                    Frames.append(None)
                    if fail_counts[idx] in (10, 30, 100):
                        self.logger.warning(f"Camera read failed /dev/video{self.CamPorts[idx]} count={fail_counts[idx]}")
                else:
                    any_ok = True
                    fail_counts[idx] = 0
                    Frames.append(Frame)
            self.Frames = Frames
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
                time.sleep(0.01)



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
