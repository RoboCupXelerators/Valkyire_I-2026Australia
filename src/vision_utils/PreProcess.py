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
