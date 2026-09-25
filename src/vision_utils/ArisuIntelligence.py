from __future__ import annotations
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
        self._model_reset_event = threading.Event()
        self._configured_model_lock = threading.Lock()
        self._configured_model_generation = 0
        self.ConfiguredInferModel = None
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
        # Preserve the transform from the inference batch for the dashboard.
        # Camera sizes are not guaranteed to match across Jetson and Raspberry Pi.
        self._bbox_coordinate_spaces = [None, None, None, None]
        self._bbox_coordinate_spaces_lock = threading.Lock()
        # Keep the original frame that produced the published Hailo bboxes.
        # The control panel must not draw a completed batch on a newer frame.
        self._overlay_frames = [None, None, None, None]
        self._overlay_frames_lock = threading.Lock()

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

    @staticmethod
    def _bbox_coordinate_space(Frame, TargetSize=(640, 640)):
        """Return the exact integer letterbox transform used for one frame."""
        Height, Width = Frame.shape[:2]
        TargetHeight, TargetWidth = TargetSize
        Scale = min(TargetWidth / Width, TargetHeight / Height)
        NewWidth = int(Width * Scale)
        NewHeight = int(Height * Scale)
        return {
            "source_width": int(Width),
            "source_height": int(Height),
            "input_width": int(TargetWidth),
            "input_height": int(TargetHeight),
            "resized_width": int(NewWidth),
            "resized_height": int(NewHeight),
            "pad_x": int((TargetWidth - NewWidth) // 2),
            "pad_y": int((TargetHeight - NewHeight) // 2),
        }

    def GetBBoxCoordinateSpaces(self):
        """Return the latest detector transform for each camera slot."""
        with self._bbox_coordinate_spaces_lock:
            return [dict(space) if isinstance(space, dict) else None
                    for space in self._bbox_coordinate_spaces]

    def GetOverlayFrame(self, CameraIndex):
        """Return the source frame paired with the latest Hailo bbox batch."""
        try:
            CameraIndex = int(CameraIndex)
            if not 0 <= CameraIndex < len(self._overlay_frames):
                return None
        except (TypeError, ValueError):
            return None
        with self._overlay_frames_lock:
            return self._overlay_frames[CameraIndex]

    @staticmethod
    def _bbox_640_to_frame(BBox, FrameShape):
        """Map one 640-square letterbox bbox to raw camera pixels."""
        X1, Y1, X2, Y2 = (float(value) for value in BBox[:4])
        Height, Width = FrameShape[:2]
        Scale = min(640.0 / Width, 640.0 / Height)
        NewWidth = int(Width * Scale)
        NewHeight = int(Height * Scale)
        PadX = (640 - NewWidth) // 2
        PadY = (640 - NewHeight) // 2
        X1 = int(round((X1 - PadX) * Width / NewWidth))
        Y1 = int(round((Y1 - PadY) * Height / NewHeight))
        X2 = int(round((X2 - PadX) * Width / NewWidth))
        Y2 = int(round((Y2 - PadY) * Height / NewHeight))
        return (
            max(0, min(Width, X1)),
            max(0, min(Height, Y1)),
            max(0, min(Width, X2)),
            max(0, min(Height, Y2)),
        )

    @staticmethod
    def _bbox_frame_to_640(BBox, FrameShape):
        """Map raw camera pixels into the 640-square detector letterbox."""
        X1, Y1, X2, Y2 = (float(value) for value in BBox[:4])
        Height, Width = FrameShape[:2]
        Scale = min(640.0 / Width, 640.0 / Height)
        NewWidth = int(Width * Scale)
        NewHeight = int(Height * Scale)
        PadX = (640 - NewWidth) // 2
        PadY = (640 - NewHeight) // 2
        return (
            int(round(X1 * NewWidth / Width + PadX)),
            int(round(Y1 * NewHeight / Height + PadY)),
            int(round(X2 * NewWidth / Width + PadX)),
            int(round(Y2 * NewHeight / Height + PadY)),
        )

    @classmethod
    def _refine_colour_bbox_for_overlay(cls, Frame, BBox, Lower, Upper):
        """Move a display-only bbox to the matching coloured object."""
        if Frame is None:
            return BBox
        try:
            Height, Width = Frame.shape[:2]
            if Height < 2 or Width < 2:
                return BBox
            HSV = cv2.cvtColor(Frame, cv2.COLOR_BGR2HSV)
            Mask = cv2.inRange(HSV, np.array(Lower), np.array(Upper))
            Kernel = np.ones((5, 5), dtype=np.uint8)
            Mask = cv2.morphologyEx(Mask, cv2.MORPH_CLOSE, Kernel)

            Seed = cls._bbox_640_to_frame(BBox, Frame.shape)
            SX1, SY1, SX2, SY2 = Seed
            SeedArea = max(1, (SX2 - SX1) * (SY2 - SY1))
            SeedColourRatio = float(np.count_nonzero(Mask[SY1:SY2, SX1:SX2])) / SeedArea
            if SeedColourRatio >= 0.02:
                return BBox

            Contours = cv2.findContours(Mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            Best = None
            SeedWidth = max(1, SX2 - SX1)
            for Contour in Contours:
                X, Y, W, H = cv2.boundingRect(Contour)
                Area = W * H
                if Area < max(30, int(Width * Height * 0.0002)):
                    continue
                Overlap = max(0, min(SX2, X + W) - max(SX1, X))
                HorizontalOverlap = Overlap / max(1, min(SeedWidth, W))
                if HorizontalOverlap < 0.55:
                    continue
                # Keep the widest strongly overlapping target-colour region.
                Score = HorizontalOverlap * 10.0 + min(W, SeedWidth) / SeedWidth + Area / (Width * Height)
                if Best is None or Score > Best[0]:
                    Best = (Score, X, Y, X + W, Y + H)
            if Best is None:
                return BBox

            RX1, RY1, RX2, RY2 = cls._bbox_frame_to_640(Best[1:], Frame.shape)
            return (RX1, RY1, RX2, RY2, BBox[4])
        except Exception:
            return BBox

    @classmethod
    def _refine_goal_bbox_for_overlay(cls, Frame, ClassId, BBox):
        """Use the goal colour in the source frame to repair Hailo UI bboxes."""
        if ClassId == 1:
            return cls._refine_colour_bbox_for_overlay(
                Frame, BBox, (90, 70, 45), (135, 255, 255))
        if ClassId == 2:
            return cls._refine_colour_bbox_for_overlay(
                Frame, BBox, (18, 80, 80), (42, 255, 255))
        return BBox

    @classmethod
    def _refine_ball_bbox_for_overlay(cls, Frame, BBox):
        """Return a validated orange-ball bbox, or None for non-ball colour.

        Skin and other field objects can be orange. Unlike goals, a ball must
        have a compact near-circular contour before it is allowed to affect
        either the dashboard or the robot-frame position calculation.
        """
        if Frame is None:
            return None
        try:
            Height, Width = Frame.shape[:2]
            HSV = cv2.cvtColor(Frame, cv2.COLOR_BGR2HSV)
            # Require bright, saturated orange. This deliberately excludes the
            # less-saturated skin tones that caused false ball inputs.
            Mask = cv2.inRange(HSV, np.array((8, 140, 140)), np.array((25, 255, 255)))
            Mask = cv2.morphologyEx(
                Mask, cv2.MORPH_CLOSE, np.ones((3, 3), dtype=np.uint8))
            SX1, _SY1, SX2, _SY2 = cls._bbox_640_to_frame(BBox, Frame.shape)
            SeedWidth = max(1, SX2 - SX1)
            Best = None
            for Contour in cv2.findContours(Mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]:
                Area = cv2.contourArea(Contour)
                if Area < max(20.0, Width * Height * 0.00005):
                    continue
                X, Y, W, H = cv2.boundingRect(Contour)
                Aspect = W / max(1, H)
                if not 0.65 <= Aspect <= 1.55:
                    continue
                Perimeter = cv2.arcLength(Contour, True)
                Circularity = 4.0 * np.pi * Area / max(1.0, Perimeter * Perimeter)
                if Circularity < 0.72:
                    continue
                Overlap = max(0, min(SX2, X + W) - max(SX1, X))
                HorizontalOverlap = Overlap / max(1, min(SeedWidth, W))
                if HorizontalOverlap < 0.55:
                    continue
                Score = Circularity * 10.0 + HorizontalOverlap + min(W, H) / max(Width, Height)
                if Best is None or Score > Best[0]:
                    Best = (Score, X, Y, X + W, Y + H)
            if Best is None:
                return None
            RX1, RY1, RX2, RY2 = cls._bbox_frame_to_640(Best[1:], Frame.shape)
            return (RX1, RY1, RX2, RY2, BBox[4])
        except Exception:
            return None

    @classmethod
    def _refine_chassis_bbox_for_overlay(cls, Frame, BBox):
        """Relocate a line false-positive to a plausible chassis region.

        Chassis has no stable single colour. The field is predominantly green
        and the common false-positive is a dark boundary line, so scan the
        matching horizontal strip above the Hailo box for a dense neutral/bright
        foreground region. This is intentionally display-only.
        """
        if Frame is None:
            return BBox
        try:
            Height, Width = Frame.shape[:2]
            SX1, SY1, SX2, SY2 = cls._bbox_640_to_frame(BBox, Frame.shape)
            BoxWidth = SX2 - SX1
            BoxHeight = SY2 - SY1
            if BoxWidth < 8 or BoxHeight < 8:
                return BBox

            HSV = cv2.cvtColor(Frame, cv2.COLOR_BGR2HSV)
            Saturation = HSV[:, :, 1]
            Value = HSV[:, :, 2]
            # Metallic/white chassis panels are neutral or bright. Dark lines
            # and saturated green carpet are excluded before scoring windows.
            Foreground = (((Saturation < 90) & (Value >= 60)) | (Value >= 185)).astype(np.uint8)
            Foreground = cv2.morphologyEx(
                Foreground, cv2.MORPH_CLOSE, np.ones((7, 7), dtype=np.uint8))

            SearchEnd = max(0, SY1 - max(6, BoxHeight // 8))
            if SearchEnd < 1:
                return BBox
            Integral = cv2.integral(Foreground)

            def density(Top):
                Bottom = min(Height, Top + BoxHeight)
                Area = max(1, (SX2 - SX1) * (Bottom - Top))
                Count = (
                    Integral[Bottom, SX2]
                    - Integral[Top, SX2]
                    - Integral[Bottom, SX1]
                    + Integral[Top, SX1]
                )
                return float(Count) / Area

            CurrentDensity = density(SY1)
            BestTop = None
            BestDensity = 0.0
            for Top in range(0, SearchEnd + 1, max(1, BoxHeight // 12)):
                CandidateDensity = density(Top)
                if CandidateDensity > BestDensity:
                    BestDensity = CandidateDensity
                    BestTop = Top

            # Do not move a credible model box. A new candidate needs a clear
            # foreground signal and must be materially better than the line.
            if (BestTop is None or BestDensity < 0.12
                    or BestDensity < CurrentDensity + 0.08):
                return BBox
            return (*cls._bbox_frame_to_640(
                (SX1, BestTop, SX2, min(Height, BestTop + BoxHeight)),
                Frame.shape,
            ), BBox[4])
        except Exception:
            return BBox

    def _configured_model_snapshot(self):
        with self._configured_model_lock:
            return self.ConfiguredInferModel, self._configured_model_generation

    def _clear_configured_model(self, expected=None):
        """Unpublish a configured model without touching a newer generation."""
        with self._configured_model_lock:
            if expected is not None and self.ConfiguredInferModel is not expected:
                return False
            self.ConfiguredInferModel = None
            self._model_ready.clear()
            return True

    def _request_model_reinit(self, expected=None):
        """Make the init thread close an invalid Hailo context and recreate it."""
        # A late failure from an old batch must not tear down a newly published
        # context.  Only signal the init thread when this call actually removed
        # the configured model it intended to invalidate.
        if self._clear_configured_model(expected):
            self._model_reset_event.set()
   

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
                self._clear_configured_model()
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
                        with self._configured_model_lock:
                            self.ConfiguredInferModel = configured
                            self._configured_model_generation += 1
                            self._model_reset_event.clear()
                            self._model_ready.set()
                        backoff_s = 1.0

                        # The inference worker sets _model_reset_event when the
                        # Hailo runtime invalidates this context.  Exit the
                        # context before publishing a replacement, otherwise a
                        # stale configured object can still receive run_async.
                        while not self._stop_event.is_set():
                            if self._model_reset_event.wait(timeout=0.5):
                                break
                        self._clear_configured_model(configured)
                # A broken context must be fully released before reopening the
                # device.  This also prevents rapid reset loops from making a
                # PCIe Hailo appear to disappear.
                if self._model_reset_event.is_set() and not self._stop_event.is_set():
                    self._stop_event.wait(1.0)

            except Exception as e:
                # Device busy / out of devices are common in field.
                self._hailo_init_error = str(e)
                self._clear_configured_model()
                logger.warning(f"Hailo init failed: {e} (will retry in {backoff_s:.1f}s)")
                time.sleep(backoff_s)
                backoff_s = min(backoff_s * 1.5, 10.0)

    def ModelPreProcess(self):
        '''
        input: int8
        output: float32
        '''
        while not self._stop_event.is_set():
            preprocess_t0 = time.time()
            # Wait until model is ready; if Hailo is busy, InitConfiguredModel will retry.
            if not self._model_ready.wait(timeout=0.5):
                time.sleep(0.05)
                continue

            configured, generation = self._configured_model_snapshot()
            if configured is None:
                continue

            BindingRecords = []
            for i in range(4):
                Frame = self.Frames[i]
                if Frame is None:
                    continue
                try:
                    Bindings = configured.create_bindings()
                except Exception as e:
                    # Model got invalidated (device reset, etc.). The init
                    # thread owns recreation after this old context is closed.
                    logger.warning(f"create_bindings failed: {e}. Will re-init hailo model")
                    self._request_model_reinit(configured)
                    BindingRecords = []
                    break
                try:
                    OutputBuffer = np.empty(self.OutputShape, dtype=np.float32)
                    SourceFrame = Frame
                    CoordinateSpace = self._bbox_coordinate_space(SourceFrame, (640, 640))
                    Frame = self.Resize(Frame, (640, 640))
                    Frame = cv2.cvtColor(Frame, cv2.COLOR_BGR2RGB)
                    Bindings.input().set_buffer(Frame)
                    Bindings.output().set_buffer(OutputBuffer)
                    # Keep the original slot: sparse Hailo batches must not make a
                    # later camera's bbox appear in an earlier control-panel tile.
                    BindingRecords.append((i, Bindings, CoordinateSpace, SourceFrame))
                except Exception as e:
                    logger.warning(f"Hailo input binding failed: {e}. Will re-init hailo model")
                    self._request_model_reinit(configured)
                    BindingRecords = []
                    break

            # No valid camera frame yet: keep waiting instead of sending an empty
            # batch to Hailo. This avoids spurious run() timeouts during startup
            # or when a camera temporarily drops out.
            if not BindingRecords:
                time.sleep(0.02)
                continue

            current_configured, current_generation = self._configured_model_snapshot()
            if configured is not current_configured or generation != current_generation:
                continue
            try:
                if self.YOLOQueue.full():
                    try:
                        self.YOLOQueue.get_nowait()
                    except queue.Empty:
                        pass
                self.PreProcessTF = self.PreProcessTF + 1
                self.PreProcessMs = (time.time() - preprocess_t0) * 1000.0
                self.YOLOQueue.put_nowait((configured, generation, BindingRecords))
            except queue.Full:
                continue
            time.sleep(0.01)
                
    def ModelInfer(self):
        FrameCount = 0
        LastTime = time.time()
        while not self._stop_event.is_set():
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
            try:
                configured, generation, BindingRecords = self.YOLOQueue.get(timeout=0.2)
            except queue.Empty:
                continue
            self.InferTF = self.InferTF + 1
            try:
                current_configured, current_generation = self._configured_model_snapshot()
                if configured is not current_configured or generation != current_generation:
                    continue
                if not BindingRecords:
                    continue
                infer_t0 = time.time()
                configured.run([record[1] for record in BindingRecords], 2000)
                self.InferBatchSize = len(BindingRecords)
                self.InferBatchMs = (time.time() - infer_t0) * 1000.0
                self.InferMs = self.InferBatchMs / max(1, self.InferBatchSize)
            except Exception as e:
                # Do not reuse a Hailo context after run_async has failed.
                self.logger.error(f"Hailo inference error: {e}. Marking model as not-ready and waiting for re-init.")
                self._request_model_reinit(configured)
                continue
            BallOutputs = []
            ChassisOutputs = []
            GoalOutputs = []
            # BBox outputs for dashboard overlay (in 640x640 space)
            BallBBoxOutputs = [[i, 0, 0, 0, 0, 0.0] for i in range(4)]  # [cam, x1, y1, x2, y2, conf]
            ChassisBBoxOutputs = [[i, 0, 0, 0, 0, 0.0] for i in range(4)]
            # Goal BBox: [cam, cls, x1, y1, x2, y2, conf], cls: 1=BlueGoal, 2=YellowGoal
            GoalBBoxOutputs = [[i, -1, 0, 0, 0, 0, 0.0] for i in range(4)]
            for CameraIndex, Bindings, _CoordinateSpace, SourceFrame in BindingRecords:
                source_size = None
                if isinstance(_CoordinateSpace, dict):
                    source_size = (
                        int(_CoordinateSpace["source_width"]),
                        int(_CoordinateSpace["source_height"]),
                    )
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
                        refined_ball_bbox = self._refine_ball_bbox_for_overlay(
                            SourceFrame,
                            (raw_x1, raw_y1, raw_x2, raw_y2, Ball_Confidence),
                        )
                        # Fail closed: a non-circular orange region must never
                        # become a ball coordinate merely because Hailo emitted
                        # a candidate nearby.
                        if refined_ball_bbox is None:
                            continue
                        BallBBoxOutputs[CameraIndex] = [
                            CameraIndex,
                            refined_ball_bbox[0], refined_ball_bbox[1],
                            refined_ball_bbox[2], refined_ball_bbox[3],
                            refined_ball_bbox[4],
                        ]
                        
                        if source_size is not None:
                            Ball_X_Min, Ball_Y_Min, Ball_X_Max, Ball_Y_Max = model_bbox_to_raw(
                                refined_ball_bbox[:4], source_size
                            )
                        else:
                            Ball_X_Min = refined_ball_bbox[0]
                            Ball_Y_Min = refined_ball_bbox[1] - 80
                            Ball_X_Max = refined_ball_bbox[2]
                            Ball_Y_Max = refined_ball_bbox[3] - 80
                        Ball_Bottom_Y = Ball_Y_Max
                        Ball_Width = Ball_X_Max - Ball_X_Min
                        Ball_Height = Ball_Y_Max - Ball_Y_Min
                        Ball_Center_X = int((Ball_X_Min + Ball_X_Max) / 2)
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
                        # Raw bbox in 640x640 space
                        raw_y1 = int(chassis[0] * 640)
                        raw_x1 = int(chassis[1] * 640)
                        raw_y2 = int(chassis[2] * 640)
                        raw_x2 = int(chassis[3] * 640)
                        Chassis_Confidence = float(chassis[4])
                        overlay_chassis_bbox = self._refine_chassis_bbox_for_overlay(
                            SourceFrame,
                            (raw_x1, raw_y1, raw_x2, raw_y2, Chassis_Confidence),
                        )
                        # Store bbox in 640x640 space for dashboard.
                        ChassisBBoxOutputs[CameraIndex] = [
                            CameraIndex,
                            overlay_chassis_bbox[0], overlay_chassis_bbox[1],
                            overlay_chassis_bbox[2], overlay_chassis_bbox[3],
                            overlay_chassis_bbox[4],
                        ]
                        
                        if source_size is not None:
                            x1, y1, x2, y2 = model_bbox_to_raw(
                                (raw_x1, raw_y1, raw_x2, raw_y2), source_size
                            )
                        else:
                            x1, y1, x2, y2 = raw_x1, raw_y1 - 80, raw_x2, raw_y2 - 80
                        ChassisOutputs.append([
                            CameraIndex,
                            int(round((x1 + x2) / 2.0)),
                            int(round(y2)),
                            max(0, int(round(x2 - x1))),
                            max(0, int(round(y2 - y1))),
                            Chassis_Confidence,
                        ])

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
                            cand = [CameraIndex, 1, int(round((x1 + x2) / 2.0)), int(round(y2)), max(0, int(round(x2 - x1))), max(0, int(round(y2 - y1))), conf]
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
                            cand = [CameraIndex, 2, int(round((x1 + x2) / 2.0)), int(round(y2)), max(0, int(round(x2 - x1))), max(0, int(round(y2 - y1))), conf]
                            if best_goal is None or conf > best_goal[6]:
                                best_goal = cand
                                best_goal_cls = 2
                                best_goal_bbox = (raw_x1, raw_y1, raw_x2, raw_y2, conf)
                except Exception:
                    pass
                if best_goal is not None:
                    GoalOutputs.append(best_goal)
                    # The Hailo model can occasionally classify a dark field line
                    # as a goal.  Correct only the dashboard rectangle from the
                    # same source frame; GoalOutputs remains the raw model result
                    # used by robot control.
                    if best_goal_bbox is not None:
                        overlay_goal_bbox = self._refine_goal_bbox_for_overlay(
                            SourceFrame,
                            best_goal_cls,
                            best_goal_bbox,
                        )
                        GoalBBoxOutputs[CameraIndex] = [CameraIndex, best_goal_cls, 
                                                        overlay_goal_bbox[0], overlay_goal_bbox[1],
                                                        overlay_goal_bbox[2], overlay_goal_bbox[3],
                                                        overlay_goal_bbox[4]]
            CoordinateSpaces = [None, None, None, None]
            OverlayFrames = [None, None, None, None]
            for CameraIndex, _Bindings, CoordinateSpace, SourceFrame in BindingRecords:
                if 0 <= CameraIndex < len(CoordinateSpaces):
                    CoordinateSpaces[CameraIndex] = CoordinateSpace
                    OverlayFrames[CameraIndex] = SourceFrame
            with self._bbox_coordinate_spaces_lock:
                self._bbox_coordinate_spaces = CoordinateSpaces
            with self._overlay_frames_lock:
                self._overlay_frames = OverlayFrames

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
