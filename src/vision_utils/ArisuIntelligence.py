from .PreProcess import Settings, logger, MODEL_DIR, VisionPreUntil, threading, queue, time, cv2, np
from hailo_platform import VDevice, HailoSchedulingAlgorithm

class ArisuIntelligence(VisionPreUntil):
    '''
    with hailo only
    '''
    
    def __init__(self):


        self.logger = logger


        self.PreProcessTF = 0
        self.InferTF = 0

        self.HailoParams = VDevice.create_params()
        self.HailoParams.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN

        # Runtime flags
        self._stop_event = threading.Event()
        self._model_ready = threading.Event()
        self._hailo_init_error: str | None = None

        self.YOLOQueue = queue.Queue(maxsize=1)

        self.MergedChassisList = []
        self.ChassisQueue = queue.Queue(maxsize=1)
        self.BallQueue = queue.Queue(maxsize=1)
        # Goals: keep interface compatible with TunaIntelligence for dashboard.
        # Format: [[cam, cls, center_x, bottom_y, w, h, conf], ...]
        # cls: 1 BlueGoal, 2 YellowGoal
        self.GoalQueue = queue.Queue(maxsize=1)

        super().__init__()

        self.InitConfiguredModelThread = threading.Thread(target=self.InitConfiguredModel)
        self.InitConfiguredModelThread.daemon = True
        self.InitConfiguredModelThread.start()

        self.ModelPreProcessThread = threading.Thread(target=self.ModelPreProcess)
        self.ModelPreProcessThread.daemon = True
        self.ModelPreProcessThread.start()

        self.ModelInferThread = threading.Thread(target=self.ModelInfer)
        self.ModelInferThread.daemon = True
        self.ModelInferThread.start()


    def Resize(self, Frame, TargetSize=(640, 640)):
        Height, Width = Frame.shape[:2]
        TargetHeight, TargetWidth = TargetSize
        
        Scale = min(TargetWidth/Width, TargetHeight/Height)
        
        NewWidth = int(Width * Scale)
        NewHeight = int(Height * Scale)
        
        ResizedImage = cv2.resize(Frame, (NewWidth, NewHeight))
        
        PaddedImage = np.zeros((TargetHeight, TargetWidth, 3), dtype=np.uint8)
        
        YOffset = (TargetHeight - NewHeight) // 2
        XOffset = (TargetWidth - NewWidth) // 2
        
        PaddedImage[YOffset:YOffset+NewHeight, XOffset:XOffset+NewWidth] = ResizedImage
        
        return PaddedImage
   

    def InitConfiguredModel(self):
        """Init hailo model.

        Important:
        - Hailo device may be occupied by another process.
        - If init fails, DO NOT crash the whole vision server; retry with backoff.
        - Keep the VDevice/ConfiguredInferModel contexts alive while other threads use it.
        """

        backoff_s = 1.0
        while not self._stop_event.is_set():
            try:
                self._model_ready.clear()
                self._hailo_init_error = None

                with VDevice(self.HailoParams) as hat:
                    infer_model = hat.create_infer_model(str(MODEL_DIR))
                    infer_model.set_batch_size(4)
                    self.InputShape = infer_model.input().shape
                    self.OutputShape = infer_model.output().shape

                    logger.success("Hailo InferModel created")

                    with infer_model.configure() as configured:
                        self.ConfiguredInferModel = configured
                        self._model_ready.set()

                        # Hold context until stop requested; if device is lost, an exception
                        # may propagate to inference thread; we'll re-init there.
                        while not self._stop_event.is_set():
                            time.sleep(0.5)

            except Exception as e:
                # Device busy / out of devices are common in field.
                self._hailo_init_error = str(e)
                self._model_ready.clear()
                logger.warning(f"Hailo init failed: {e} (will retry in {backoff_s:.1f}s)")
                time.sleep(backoff_s)
                backoff_s = min(backoff_s * 1.5, 10.0)

    def ModelPreProcess(self):
        '''
        input: int8
        output: float32
        '''
        while(1):
            # Wait until model is ready; if Hailo is busy, InitConfiguredModel will retry.
            if not self._model_ready.wait(timeout=0.5):
                time.sleep(0.05)
                continue

            BindingsList = []
            for i in range(4):
                try:
                    configured = getattr(self, "ConfiguredInferModel", None)
                    if configured is None:
                        continue
                    Bindings = configured.create_bindings()
                except Exception as e:
                    # Model got invalidated (device reset, etc.). Force re-init.
                    logger.warning(f"create_bindings failed: {e}. Will re-init hailo model")
                    self._model_ready.clear()
                    time.sleep(0.2)
                    continue
                OutputBuffer = np.empty(self.OutputShape, dtype=np.float32)
                Frame = self.Frames[i]
                if Frame is not None:
                    Frame = self.Resize(Frame, (640, 640))
                    # TODO 大块内存偶尔分配
                    Frame = cv2.cvtColor(Frame, cv2.COLOR_BGR2RGB)
                    # cv2.imshow(f"Frame {i}", Frame)
                    # Frame = Frame.astype(np.uint8)
                    # Frame = np.ascontiguousarray(Frame, dtype=np.uint8)
                    # print(Frame.shape)
                    Bindings.input().set_buffer(Frame)
                    Bindings.output().set_buffer(OutputBuffer)
                    BindingsList.append(Bindings)

            # No valid camera frame yet: keep waiting instead of sending an empty
            # batch to Hailo. This avoids spurious run() timeouts during startup
            # or when a camera temporarily drops out.
            if not BindingsList:
                time.sleep(0.02)
                continue

            if self.YOLOQueue.full():
                self.YOLOQueue.empty()
                # print(self.YOLOQueue.get())
                continue
            else:
                self.PreProcessTF = self.PreProcessTF + 1
                self.YOLOQueue.put(BindingsList)
            time.sleep(0.01)
                
    def ModelInfer(self):
        FrameCount = 0
        LastTime = time.time()
        while(1):
            if not self._model_ready.wait(timeout=0.5):
                time.sleep(0.05)
                continue

            FrameCount += 1
            CurrentTime = time.time()
            if Settings.Debug.FullLog:
                if CurrentTime - LastTime >= 1.0:
                    self.logger.debug(f"Hailo Process FPS: {FrameCount}")
                    FrameCount = 0
                    LastTime = CurrentTime
            BindingsList = self.YOLOQueue.get()
            self.InferTF = self.InferTF + 1
            # 确保在传递给 BindingsList 之前执行此操作
            try:
                configured = getattr(self, "ConfiguredInferModel", None)
                if configured is None:
                    continue
                if not BindingsList:
                    continue
                configured.run(BindingsList, 2000)
            except Exception as e:
                # DO NOT recurse (will explode the stack). Mark model not-ready so init thread re-acquires device.
                self.logger.error(f"Hailo inference error: {e}. Marking model as not-ready and waiting for re-init.")
                self._model_ready.clear()
                time.sleep(0.2)
                continue
            BallOutputs = []
            ChassisOutputs = []
            GoalOutputs = []
            CameraIndex = 0
            for Bindings in BindingsList:
                OutputBuffer = Bindings.output().get_buffer()
                if OutputBuffer[0].shape[0] > 0:
                    for _ball in OutputBuffer[0]:
                        if _ball[4] < 0.4:
                            continue
                        Ball_Y_Min = int(_ball[0] * 640) - 80 + 15
                        Ball_X_Min = int(_ball[1] * 640) + 15
                        Ball_Y_Max = int(_ball[2] * 640) - 80 -15
                        Ball_X_Max = int(_ball[3] * 640) -15
                        Ball_Bottom_Y = Ball_Y_Max
                        Ball_Width = Ball_X_Max - Ball_X_Min
                        Ball_Height = Ball_Y_Max - Ball_Y_Min
                        Ball_Center_X = int((Ball_X_Min + Ball_X_Max) / 2)
                        Ball_Confidence = _ball[4]
                        BallOutputs.append([
                            CameraIndex,
                            Ball_Center_X, 
                            Ball_Bottom_Y, 
                            Ball_Width, 
                            Ball_Height, 
                            Ball_Confidence])
                if OutputBuffer[3].shape[0] > 0:
                    for chassis in OutputBuffer[3]:
                        if chassis[4] < 0.5:
                            continue
                        Chassis_Y_Min = int(chassis[0] * 640) - 80 
                        Chassis_X_Min = int(chassis[1] * 640)
                        Chassis_Y_Max = int(chassis[2] * 640) - 80 
                        Chassis_X_Max = int(chassis[3] * 640)
                        Chassis_Bottom_Y = Chassis_Y_Max
                        Chassis_Width = Chassis_X_Max - Chassis_X_Min
                        Chassis_Height = Chassis_Y_Max - Chassis_Y_Min
                        Chassis_Center_X = int((Chassis_X_Min + Chassis_X_Max) / 2)
                        Chassis_Confidence = chassis[4]
                        ChassisOutputs.append([
                            CameraIndex,
                            Chassis_Center_X,
                            Chassis_Bottom_Y, 
                            Chassis_Width, 
                            Chassis_Height, 
                            Chassis_Confidence])

                # Goals: OutputBuffer[1] -> BlueGoal, OutputBuffer[2] -> YellowGoal
                # Keep top-1 (by conf) for each cam; if both exist, pick higher.
                best_goal = None
                try:
                    if OutputBuffer[1].shape[0] > 0:
                        for g in OutputBuffer[1]:
                            if g[4] < 0.4:
                                continue
                            # g: [ymin,xmin,ymax,xmax,conf] normalized
                            y1 = int(g[0] * 640) - 80
                            x1 = int(g[1] * 640)
                            y2 = int(g[2] * 640) - 80
                            x2 = int(g[3] * 640)
                            conf = float(g[4])
                            cx = int((x1 + x2) / 2)
                            bottom_y = int(y2)
                            w = int(x2 - x1)
                            h = int(y2 - y1)
                            cand = [CameraIndex, 1, cx, bottom_y, w, h, conf]
                            if best_goal is None or conf > best_goal[6]:
                                best_goal = cand
                except Exception:
                    pass
                try:
                    if OutputBuffer[2].shape[0] > 0:
                        for g in OutputBuffer[2]:
                            if g[4] < 0.4:
                                continue
                            y1 = int(g[0] * 640) - 80
                            x1 = int(g[1] * 640)
                            y2 = int(g[2] * 640) - 80
                            x2 = int(g[3] * 640)
                            conf = float(g[4])
                            cx = int((x1 + x2) / 2)
                            bottom_y = int(y2)
                            w = int(x2 - x1)
                            h = int(y2 - y1)
                            cand = [CameraIndex, 2, cx, bottom_y, w, h, conf]
                            if best_goal is None or conf > best_goal[6]:
                                best_goal = cand
                except Exception:
                    pass
                if best_goal is not None:
                    GoalOutputs.append(best_goal)
                CameraIndex += 1
                # Keep latest in size=1 queues
                if self.ChassisQueue.full():
                    try:
                        _ = self.ChassisQueue.get_nowait()
                    except Exception:
                        pass
                if self.BallQueue.full():
                    try:
                        _ = self.BallQueue.get_nowait()
                    except Exception:
                        pass
                if self.GoalQueue.full():
                    try:
                        _ = self.GoalQueue.get_nowait()
                    except Exception:
                        pass

                self.ChassisQueue.put(ChassisOutputs)
                self.BallQueue.put(BallOutputs)
                self.GoalQueue.put(GoalOutputs)
            # print(ChassisList)
        
            # print(Output0)
            # time.sleep(0.01)

