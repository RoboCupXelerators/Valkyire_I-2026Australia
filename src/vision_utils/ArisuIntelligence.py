from .PreProcess import Settings, logger, MODEL_DIR, VisionPreUntil, threading, queue, time, cv2, np
from .modern_calibration import model_bbox_to_raw
from hailo_platform import VDevice, HailoSchedulingAlgorithm

class ArisuIntelligence(VisionPreUntil):
    '''
    with hailo only
    '''
    
    def __init__(self):


        self.logger = logger


        self.PreProcessTF = 0
        self.InferTF = 0
        self.PreProcessMs = 0.0
        self.InferMs = 0.0
        self.InferBatchMs = 0.0
        self.InferBatchSize = 0

        self.HailoParams = VDevice.create_params()
        self.HailoParams.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN

        # Runtime flags
        self._warned = False
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
        # BBox queues for dashboard overlay (same as TunaIntelligence)
        # Format: [[cam, x1, y1, x2, y2, conf], ...] in 640x640 space
        self.BallBBoxQueue = queue.Queue(maxsize=1)
        self.ChassisBBoxQueue = queue.Queue(maxsize=1)
        # Goal BBox queue for dashboard overlay
        # Format: [[cam, cls, x1, y1, x2, y2, conf], ...] in 640x640 space
        # cls: 1 BlueGoal, 2 YellowGoal
        self.GoalBBoxQueue = queue.Queue(maxsize=1)

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
                    # 从统一 ModelName 配置获取模型名称，自动拼接 .hef 后缀
                    model_name = getattr(Settings.Vision, "ModelName", "ArisuModel_v1.2s")
                    # 向后兼容：如果旧配置存在 HailoModelPath，优先使用
                    hailo_path_legacy = getattr(Settings.Vision, "HailoModelPath", None)
                    if hailo_path_legacy and hailo_path_legacy.startswith("/"):
                        model_path = MODEL_DIR / hailo_path_legacy.lstrip("/")
                    else:
                        model_path = MODEL_DIR / f"{model_name}.hef"
                    infer_model = hat.create_infer_model(str(model_path))
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
            preprocess_t0 = time.time()
            # Wait until model is ready; if Hailo is busy, InitConfiguredModel will retry.
            if not self._model_ready.wait(timeout=0.5):
                time.sleep(0.05)
                continue

            BindingsList = []
            BindingMeta = []
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
                    source_size = (int(Frame.shape[1]), int(Frame.shape[0]))
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
                    BindingMeta.append((i, source_size))

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
                self.PreProcessMs = (time.time() - preprocess_t0) * 1000.0
                self.YOLOQueue.put((BindingsList, BindingMeta))
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
            queued = self.YOLOQueue.get()
            if isinstance(queued, tuple) and len(queued) == 2:
                BindingsList, BindingMeta = queued
            else:
                BindingsList = queued
                BindingMeta = [(index, None) for index in range(len(BindingsList))]
            self.InferTF = self.InferTF + 1
            # 确保在传递给 BindingsList 之前执行此操作
            try:
                configured = getattr(self, "ConfiguredInferModel", None)
                if configured is None:
                    continue
                if not BindingsList:
                    continue
                infer_t0 = time.time()
                configured.run(BindingsList, 2000)
                self.InferBatchSize = len(BindingsList)
                self.InferBatchMs = (time.time() - infer_t0) * 1000.0
                self.InferMs = self.InferBatchMs / max(1, self.InferBatchSize)
            except Exception as e:
                # DO NOT recurse (will explode the stack). Mark model not-ready so init thread re-acquires device.
                self.logger.error(f"Hailo inference error: {e}. Marking model as not-ready and waiting for re-init.")
                self._model_ready.clear()
                time.sleep(0.2)
                continue
            BallOutputs = [[i, 0, 0, 0, 0, 0.0, 0, 0] for i in range(4)]
            ChassisOutputs = [[i, 0, 0, 0, 0, 0.0, 0, 0] for i in range(4)]
            GoalOutputs = [[i, -1, 0, 0, 0, 0, 0.0, 0, 0] for i in range(4)]
            # BBox outputs for dashboard overlay (in 640x640 space)
            BallBBoxOutputs = [[i, 0, 0, 0, 0, 0.0] for i in range(4)]  # [cam, x1, y1, x2, y2, conf]
            ChassisBBoxOutputs = [[i, 0, 0, 0, 0, 0.0] for i in range(4)]
            # Goal BBox: [cam, cls, x1, y1, x2, y2, conf], cls: 1=BlueGoal, 2=YellowGoal
            GoalBBoxOutputs = [[i, -1, 0, 0, 0, 0, 0.0] for i in range(4)]
            for Bindings, (CameraIndex, source_size) in zip(BindingsList, BindingMeta):
                OutputBuffer = Bindings.output().get_buffer()
                if OutputBuffer[0].shape[0] > 0:
                    for _ball in OutputBuffer[0]:
                        if _ball[4] < 0.4:
                            continue
                        # Raw bbox in 640x640 space (before the -80 offset used for position calculation)
                        # Hailo output is normalized [ymin, xmin, ymax, xmax, conf]
                        raw_y1 = int(_ball[0] * 640)
                        raw_x1 = int(_ball[1] * 640)
                        raw_y2 = int(_ball[2] * 640)
                        raw_x2 = int(_ball[3] * 640)
                        Ball_Confidence = float(_ball[4])
                        # Store bbox in 640x640 space for dashboard
                        BallBBoxOutputs[CameraIndex] = [CameraIndex, raw_x1, raw_y1, raw_x2, raw_y2, Ball_Confidence]
                        
                        if source_size is not None:
                            x1, y1, x2, y2 = model_bbox_to_raw((raw_x1, raw_y1, raw_x2, raw_y2), source_size)
                            BallOutputs[CameraIndex] = [
                                CameraIndex,
                                int(round((x1 + x2) / 2.0)),
                                int(round(y2)),
                                max(0, int(round(x2 - x1))),
                                max(0, int(round(y2 - y1))),
                                Ball_Confidence,
                                int(source_size[0]),
                                int(source_size[1]),
                            ]
                if OutputBuffer[3].shape[0] > 0:
                    for chassis in OutputBuffer[3]:
                        if chassis[4] < 0.5:
                            continue
                        # Raw bbox in 640x640 space
                        raw_y1 = int(chassis[0] * 640)
                        raw_x1 = int(chassis[1] * 640)
                        raw_y2 = int(chassis[2] * 640)
                        raw_x2 = int(chassis[3] * 640)
                        Chassis_Confidence = float(chassis[4])
                        # Store bbox in 640x640 space for dashboard
                        ChassisBBoxOutputs[CameraIndex] = [CameraIndex, raw_x1, raw_y1, raw_x2, raw_y2, Chassis_Confidence]
                        
                        if source_size is not None and Chassis_Confidence > ChassisOutputs[CameraIndex][5]:
                            x1, y1, x2, y2 = model_bbox_to_raw((raw_x1, raw_y1, raw_x2, raw_y2), source_size)
                            ChassisOutputs[CameraIndex] = [
                                CameraIndex,
                                int(round((x1 + x2) / 2.0)),
                                int(round(y2)),
                                max(0, int(round(x2 - x1))),
                                max(0, int(round(y2 - y1))),
                                Chassis_Confidence,
                                int(source_size[0]),
                                int(source_size[1]),
                            ]

                # Goals: OutputBuffer[1] -> BlueGoal, OutputBuffer[2] -> YellowGoal
                # Keep top-1 (by conf) for each cam; if both exist, pick higher.
                best_goal = None
                best_goal_cls = -1
                best_goal_bbox = None  # (raw_x1, raw_y1, raw_x2, raw_y2, conf)
                
                # Debug: log OutputBuffer structure for goal detection
                if not self._warned:
                    if Settings.Debug.FullLog and CameraIndex == 0:
                        self.logger.debug(f"OutputBuffer len={len(OutputBuffer)}, shapes={[OutputBuffer[i].shape for i in range(min(4, len(OutputBuffer)))]}")
                    self._warned = True

                try:
                    if len(OutputBuffer) > 1 and OutputBuffer[1].shape[0] > 0:
                        for g in OutputBuffer[1]:
                            if g[4] < 0.25:  # Lower threshold for goals
                                continue
                            # g: [ymin,xmin,ymax,xmax,conf] normalized
                            # Raw bbox in 640x640 space (for dashboard overlay)
                            raw_y1 = int(g[0] * 640)
                            raw_x1 = int(g[1] * 640)
                            raw_y2 = int(g[2] * 640)
                            raw_x2 = int(g[3] * 640)
                            conf = float(g[4])
                            x1, y1, x2, y2 = model_bbox_to_raw((raw_x1, raw_y1, raw_x2, raw_y2), source_size)
                            cand = [CameraIndex, 1, int(round((x1 + x2) / 2.0)), int(round(y2)), max(0, int(round(x2 - x1))), max(0, int(round(y2 - y1))), conf, int(source_size[0]), int(source_size[1])]
                            if best_goal is None or conf > best_goal[6]:
                                best_goal = cand
                                best_goal_cls = 1
                                best_goal_bbox = (raw_x1, raw_y1, raw_x2, raw_y2, conf)
                except Exception:
                    pass
                try:
                    if len(OutputBuffer) > 2 and OutputBuffer[2].shape[0] > 0:
                        for g in OutputBuffer[2]:
                            if g[4] < 0.25:  # Lower threshold for goals
                                continue
                            # Raw bbox in 640x640 space (for dashboard overlay)
                            raw_y1 = int(g[0] * 640)
                            raw_x1 = int(g[1] * 640)
                            raw_y2 = int(g[2] * 640)
                            raw_x2 = int(g[3] * 640)
                            conf = float(g[4])
                            x1, y1, x2, y2 = model_bbox_to_raw((raw_x1, raw_y1, raw_x2, raw_y2), source_size)
                            cand = [CameraIndex, 2, int(round((x1 + x2) / 2.0)), int(round(y2)), max(0, int(round(x2 - x1))), max(0, int(round(y2 - y1))), conf, int(source_size[0]), int(source_size[1])]
                            if best_goal is None or conf > best_goal[6]:
                                best_goal = cand
                                best_goal_cls = 2
                                best_goal_bbox = (raw_x1, raw_y1, raw_x2, raw_y2, conf)
                except Exception:
                    pass
                if best_goal is not None:
                    GoalOutputs[CameraIndex] = best_goal
                    # Store goal bbox for dashboard overlay
                    if best_goal_bbox is not None:
                        GoalBBoxOutputs[CameraIndex] = [CameraIndex, best_goal_cls, 
                                                        best_goal_bbox[0], best_goal_bbox[1],
                                                        best_goal_bbox[2], best_goal_bbox[3],
                                                        best_goal_bbox[4]]
            # Put outputs into queues AFTER processing all cameras
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
            
            # Put bbox outputs into queues for dashboard overlay
            if self.BallBBoxQueue.full():
                try:
                    _ = self.BallBBoxQueue.get_nowait()
                except Exception:
                    pass
            if self.ChassisBBoxQueue.full():
                try:
                    _ = self.ChassisBBoxQueue.get_nowait()
                except Exception:
                    pass
            if self.GoalBBoxQueue.full():
                try:
                    _ = self.GoalBBoxQueue.get_nowait()
                except Exception:
                    pass
            self.BallBBoxQueue.put(BallBBoxOutputs)
            self.ChassisBBoxQueue.put(ChassisBBoxOutputs)
            self.GoalBBoxQueue.put(GoalBBoxOutputs)
            # print(ChassisList)
        
            # print(Output0)
            # time.sleep(0.01)
