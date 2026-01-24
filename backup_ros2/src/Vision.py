# ROS2-adapted backup of Vision.py (selected parts)
import math, time
import threading
import queue

import cv2
import numpy as np

from utils.ReasonData import Settings, logger, DATA_DIR
from utils.coords import camera_to_robot, angle_from_xy


class ArisuIntelligence:
    def __init__(self, GetPos=None):
        self.GetPos = GetPos
        self.cfg = Settings
        self.logger = logger
        self.CamPorts = [6, 4, 2, 0]
        self.BallPos = [0, 0]  # x_forward, y_left
        # ... (omitted initialization identical to original)

    def FindBall(self):
        while True:
            Balls = []
            for i in range(4):
                Blobs = []
                Frame = self.Frames[i]
                Blobs = self.FindBlobs(Frame, self.OrangeThreshold)
                if Blobs:
                    MaxBlob = self.FindMaxBlob(Blobs)
                    X = MaxBlob[4]
                    Y = MaxBlob[1] + MaxBlob[3]
                    X, Y = self.Pixel2CM(X, Y, i)
                    Balls.append((X, Y, i))

            if Balls:
                X = Balls[0][0]
                Y = Balls[0][1]
                CamIndex = Balls[0][2]
                x_forward, y_left = camera_to_robot(X, Y, CamIndex)
                self.BallPos = [x_forward, y_left]
            else:
                self.BallPos = [0, 0]

            time.sleep(0.03)

    def GetBallPos(self):
        return self.BallPos
