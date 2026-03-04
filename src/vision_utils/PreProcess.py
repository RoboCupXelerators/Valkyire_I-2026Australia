"""Shared vision preprocessing utilities.

This module is used by multiple vision backends (Hailo / BPU / Blob).

Compatibility note:
- The main system config is `src/ReasonData/data/config.json`.
- `vision_utils` historically also shipped its own config `vision_utils/VisionDatas/data/vision.json`.

To reduce "config split" issues, we prefer importing `ReasonData.Settings/logger`
when available, and fallback to `vision_utils.VisionDatas`.
"""

from pathlib import Path

try:
    # Prefer the main system config
    from ReasonData import Settings, logger, DATA_DIR as REASON_DATA_DIR  # type: ignore
except Exception:  # pragma: no cover
    from .VisionDatas import Settings, logger
    REASON_DATA_DIR = None  # type: ignore
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
DATA_DIR = APP_DIR / "VisionDatas" / "data"
MODEL_DIR = APP_DIR / "models"


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
    with hailo only
    '''
    def __init__(self):
        # Allow subclasses to pre-create a stop event (e.g. ArisuIntelligence).
        # If not provided, create one so we can shutdown gracefully in tests.
        if not hasattr(self, "_stop_event"):
            self._stop_event = threading.Event()

        self.cfg = Settings
        self.logger = logger

        #TODO put into settings
        # self.CamPorts = [8,10,1,5]
        self.CamPorts = [
                    Settings.VisionVals.Front,
                    Settings.VisionVals.Right,
                    Settings.VisionVals.Back,
                    Settings.VisionVals.Left,
                    ]
        self.StopRecord = 0

        self.Cams = []
        self.Frames = []
        self.Videos = []

        self.BallPos = [0,0]

        # Calibration for Pixel<->CM mapping (used by BlobBased / other detectors).
        # If calibration files are missing, we fallback to identity mapping.
        self._init_calibration()

        self.InitCam(self.CamPorts)

        self.ReadCamTF = 0
        self.PreProcessTF = 0
        self.InferTF = 0
        self.PostProcessTF = 0


        self.ReadCamsThread = threading.Thread(target=self.ReadCams)
        self.ReadCamsThread.daemon = True
        self.ReadCamsThread.start()

        time.sleep(3)

        # Start recording thread only when enabled (zero overhead otherwise).
        self._recorder: SegmentedMultiCamRecorder | None = None
        if getattr(self.cfg.VisionVals, "Record", False):
            # Ensure we finalize mp4/avi headers on exit.
            # Without a clean release(), mp4 often becomes "moov atom not found" and unplayable.
            atexit.register(self.close)
            try:
                # Important: do not swallow Ctrl+C.
                # We cleanup, then delegate to Python's default handlers
                # so the process can actually exit.
                def _sig_cleanup_and_exit(signum, frame):
                    try:
                        # Keep the signal handler lightweight to avoid deadlocks
                        # (e.g. releasing VideoCapture while another thread is blocking on read()).
                        self.stop()
                        # Finalize current segment so MP4 is playable.
                        try:
                            self.CloseVideo()
                        except Exception:
                            pass
                    finally:
                        if signum == signal.SIGINT:
                            # Default behavior: raise KeyboardInterrupt
                            signal.default_int_handler(signum, frame)
                        else:
                            # For SIGTERM: exit cleanly
                            raise SystemExit(0)

                signal.signal(signal.SIGINT, _sig_cleanup_and_exit)
                signal.signal(signal.SIGTERM, _sig_cleanup_and_exit)
            except Exception:
                # signal may fail in some embedded/threads env; atexit still helps.
                pass

            # Default: store recordings next to ReasonData log file (ReasonData/data).
            # If user provides an absolute path, use it. If it's relative (including ./Records),
            # treat it as relative to ReasonData's DATA_DIR, not current working directory.
            record_dir_val = str(getattr(self.cfg.VisionVals, "RecordDir", "./Records"))
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
                    cams=list(getattr(self.cfg.VisionVals, "RecordCams", [0, 1, 2, 3])),
                    record_size=(
                        int(getattr(self.cfg.VisionVals, "RecordWidth", 320)),
                        int(getattr(self.cfg.VisionVals, "RecordHeight", 320)),
                    ),
                    segment_seconds=int(getattr(self.cfg.VisionVals, "RecordSegmentSeconds", 60)),
                    fps=int(getattr(self.cfg.VisionVals, "RecordFPS", 30)),
                    codec=str(getattr(self.cfg.VisionVals, "RecordCodec", "mp4v")),
                    ext=str(getattr(self.cfg.VisionVals, "RecordExt", "mp4")),
                ),
            )

            self.VideoRecordThread = threading.Thread(target=self.VideoRecord)
            self.VideoRecordThread.daemon = True
            self.VideoRecordThread.start()


    def _init_calibration(self):
        """Load per-camera calibration used by Pixel2CM/CM2Pixel.

        Expected file: {DATA_DIR}/Calibration/CalibrationData{port}.npz
        with keys:
        - matrix: 3x3 perspective transform
        - p2c: (k, hb, vb)
        """

        self.PerspectiveMatrices: list[np.ndarray] = []
        self.P2CK: list[float] = []
        self.P2CHB: list[float] = []
        self.P2CVB: list[float] = []

        calib_dir = DATA_DIR / "Calibration"
        for port in self.CamPorts:
            npz_path = calib_dir / f"CalibrationData{port}.npz"
            try:
                data = np.load(str(npz_path))
                matrix = data["matrix"]
                p2c = data["p2c"]

                self.PerspectiveMatrices.append(matrix)
                # Be tolerant to p2c shape (sometimes it's nested).
                self.P2CK.append(float(p2c[0]))
                self.P2CHB.append(float(p2c[1]))
                self.P2CVB.append(float(p2c[2]))
            except Exception as e:
                self.logger.warning(
                    f"Calibration load failed for camera port {port} ({npz_path}): {e}. "
                    "Fallback to identity Pixel2CM."
                )
                self.PerspectiveMatrices.append(np.eye(3, dtype=np.float64))
                self.P2CK.append(1.0)
                self.P2CHB.append(0.0)
                self.P2CVB.append(0.0)


    def ApplyPerspectiveTransform(self, X, Y, Matrix):
        """Apply a 3x3 perspective matrix on a pixel point."""
        point = np.array([X, Y, 1], dtype=np.float64)
        transformed = Matrix @ point
        transformed /= transformed[2]
        return int(transformed[1]), int(transformed[0])


    def Pixel2CM(self, X, Y, CamIndex: int):
        """Convert pixel coordinate to cm coordinate for a specific camera.

        Notes:
        - This is shared by Hailo/BPU/Blob pipelines.
        - If calibration wasn't loaded, returns (X, Y) as-is.
        """

        if not getattr(self, "PerspectiveMatrices", None):
            self._init_calibration()

        try:
            X, Y = self.ApplyPerspectiveTransform(X, Y, self.PerspectiveMatrices[CamIndex])
            X = round(self.P2CK[CamIndex] * X + self.P2CHB[CamIndex], 2)
            Y = round(-self.P2CK[CamIndex] * Y + self.P2CVB[CamIndex], 2)
            return X, Y
        except Exception as e:
            self.logger.warning(f"Pixel2CM failed (CamIndex={CamIndex}, X={X}, Y={Y}): {e}")
            return X, Y


    def CM2Pixel(self, X, Y, CamIndex: int):
        """Convert cm coordinate back to pixel coordinate for a specific camera."""

        if not getattr(self, "PerspectiveMatrices", None):
            self._init_calibration()

        try:
            p2ck = self.P2CK[CamIndex]
            p2chb = self.P2CHB[CamIndex]
            p2cvb = self.P2CVB[CamIndex]

            X = (X - p2chb) / p2ck
            Y = -(Y - p2cvb) / p2ck
            X, Y = self.ApplyPerspectiveTransform(X, Y, np.linalg.inv(self.PerspectiveMatrices[CamIndex]))
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
        """Continuously write frames to segmented video files."""

        # If recording is disabled, the thread should never be started, but keep safe.
        if getattr(self, "_recorder", None) is None:
            return

        # Aim for ~30 fps write loop by default.
        loop_sleep = 1.0 / max(1, int(getattr(self.cfg.VisionVals, "RecordFPS", 30)))
        while not self._stop_event.is_set():
            frames = getattr(self, "Frames", [])
            if frames:
                try:
                    self._recorder.write_frames(frames)
                except Exception as e:
                    self.logger.warning(f"Recorder error: {e}")
            time.sleep(loop_sleep)


    def InitCam(self,CamPorts,Width=640, Height=480, AutoExposure=3, Exposure=157, Brightness=0, Contrast=32, Saturation=64):
        for Port in CamPorts:
            Cam = cv2.VideoCapture(Port,cv2.CAP_V4L2)
            Cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            Cam.set(cv2.CAP_PROP_FRAME_WIDTH, Width)
            Cam.set(cv2.CAP_PROP_FRAME_HEIGHT, Height)
            Cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, AutoExposure)
            Cam.set(cv2.CAP_PROP_EXPOSURE, Exposure)
            Cam.set(cv2.CAP_PROP_BRIGHTNESS, Brightness)
            Cam.set(cv2.CAP_PROP_CONTRAST, Contrast)
            Cam.set(cv2.CAP_PROP_SATURATION, Saturation)
            self.Cams.append(Cam)
    
    def ReadCams(self):
        FrameCount = 0
        LastTime = time.time()
        while not self._stop_event.is_set():
            Frames = []
            for Cam in self.Cams:
                ok, frame = Cam.read()
                Frame = frame if ok else None
                Frames.append(Frame)
            self.Frames = Frames
            FrameCount += 1
            CurrentTime = time.time()
            if CurrentTime - LastTime >= 1.0:
                if self.cfg.Debug.FullLog:
                    self.logger.debug("Camera FPS: "+str(FrameCount))
                FrameCount = 0
                LastTime = CurrentTime
            self.ReadCamTF = self.ReadCamTF + 1
        


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

    def close(self):
        """Stop threads and release camera/video resources."""
        self.stop()
        for Cam in getattr(self, "Cams", []):
            try:
                Cam.release()
            except Exception:
                pass
        try:
            self.CloseVideo()
        except Exception:
            pass

    def stop(self):
        """Request background threads to stop (non-blocking)."""
        try:
            self._stop_event.set()
        except Exception:
            pass

    def __exit__(self, *args, **kwargs):
        self.close()

    def __del__(self):
        # Best-effort cleanup for short-lived scripts.
        try:
            self.close()
        except Exception:
            pass
