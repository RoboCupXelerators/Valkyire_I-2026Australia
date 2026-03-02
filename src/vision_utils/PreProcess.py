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
    from ReasonData import Settings, logger  # type: ignore
except Exception:  # pragma: no cover
    from .VisionDatas import Settings, logger
import threading
import queue
import time
import cv2
import numpy as np
import pathlib

APP_DIR = Path(__file__).parent
DATA_DIR = APP_DIR / "VisionDatas" / "data"
MODEL_DIR = APP_DIR / "models"

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
        try:
            self._stop_event.set()
        except Exception:
            pass
        for Cam in getattr(self, "Cams", []):
            try:
                Cam.release()
            except Exception:
                pass
        try:
            self.CloseVideo()
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
