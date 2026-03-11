from .VisionDatas import Settings, logger, Path
import threading
import queue
import time
import cv2
import numpy as np
import pathlib

APP_DIR = Path(__file__).parent
DATA_DIR = APP_DIR / "VisionDatas" / "data"
MODEL_DIR = APP_DIR / "models"


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
        self.cfg = Settings
        self.logger = logger

        # TODO put into settings
        # self.CamPorts = [8,10,1,5]
        self.CamPorts = cam_ports or [
            Settings.VisionVals.Front,
            Settings.VisionVals.Right,
            Settings.VisionVals.Back,
            Settings.VisionVals.Left,
        ]
        self.StopRecord = 0

        self.Cams = []
        # Always keep 4 slots, even when cameras are not initialized.
        self.Frames = [None, None, None, None]
        self.Videos = []

        self.BallPos = [0,0]

        # thread lifecycle (for clean shutdown)
        self._stop_event = threading.Event()
        self.ReadCamsThread = None
        self.VideoRecordThread = None

        if init_cams:
            self.InitCam(self.CamPorts)

        self.ReadCamTF = 0
        self.PreProcessTF = 0
        self.InferTF = 0
        self.PostProcessTF = 0


        if init_cams:
            self.ReadCamsThread = threading.Thread(target=self.ReadCams)
            self.ReadCamsThread.daemon = True
            self.ReadCamsThread.start()

            time.sleep(3)

            self.VideoRecordThread = threading.Thread(target=self.VideoRecord)
            self.VideoRecordThread.daemon = True
            self.VideoRecordThread.start()


    def close(self, join_timeout: float = 1.0):
        """Stop background threads and release camera/video resources."""
        try:
            self._stop_event.set()
        except Exception:
            pass

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


    def InitVideo(self):
        Videos = []
        if not self.cfg.VisionVals.Record:
            return
        for i in range(4):
            Time = int(time.time())
            Video = cv2.VideoWriter('./Records/' + str(i) + '/' + str(Time) + '.mp4', cv2.VideoWriter_fourcc(*'avc1'), 30, (640, 480))
            Videos.append(Video)
        self.Videos = Videos
    
    def CloseVideo(self):
        for Video in self.Videos:
            Video.release()
        self.Videos = []
    
    def VideoRecord(self):
        while not self._stop_event.is_set():
            if not self.Videos:
                self.InitVideo()
            elif int(time.time()) % 30 == 0:
                self.CloseVideo()
            
            for i in range(4):
                if self.Videos:
                    Frame = self.Frames[i]
                    if Frame is not None:
                        self.Videos[i].write(Frame)
            time.sleep(0.03)


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
                if self.cfg.Debug.FullLog:
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

    def __exit__(self):
        for Cam in self.Cams:
            Cam.release()
        self.CloseVideo()
