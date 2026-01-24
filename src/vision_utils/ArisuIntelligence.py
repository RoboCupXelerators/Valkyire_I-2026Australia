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

        self.YOLOQueue = queue.Queue(maxsize=1)

        self.MergedChassisList = []
        self.ChassisQueue = queue.Queue(maxsize=1)
        self.BallQueue = queue.Queue(maxsize=1)

        super().__init__()

        self.InitConfiguredModelThread = threading.Thread(target=self.InitConfiguredModel)
        self.InitConfiguredModelThread.daemon = True
        self.InitConfiguredModelThread.start()

        # 报错会阻塞程序 try&except
        time.sleep(2)


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
        with VDevice(self.HailoParams) as Hat:
            InferModel = Hat.create_infer_model(str(MODEL_DIR)+"/yolov8s.hef")
            InferModel.set_batch_size(4)
            logger.success("InferModel created")
            self.InputShape = InferModel.input().shape
            self.OutputShape = InferModel.output().shape
            with InferModel.configure() as ConfiguredInferModel:
                self.ConfiguredInferModel = ConfiguredInferModel
                time.sleep(114514)

    def ModelPreProcess(self):
        '''
        input: int8
        output: float32
        '''
        while(1):
            BindingsList = []
            for i in range(4):
                Bindings = self.ConfiguredInferModel.create_bindings()
                OutputBuffer = np.empty(self.OutputShape, dtype=np.float32)
                Frame = self.Frames[i]
                if Frame is not None:
                    Frame = self.Resize(Frame, (640, 640))
                    Frame = cv2.cvtColor(Frame, cv2.COLOR_BGR2RGB)
                    # cv2.imshow(f"Frame {i}", Frame)
                    # Frame = Frame.astype(np.uint8)
                    # Frame = np.ascontiguousarray(Frame, dtype=np.uint8)
                    # print(Frame.shape)
                    Bindings.input().set_buffer(Frame)
                    Bindings.output().set_buffer(OutputBuffer)
                    BindingsList.append(Bindings)
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
            FrameCount += 1
            CurrentTime = time.time()
            if CurrentTime - LastTime >= 1.0:
                self.logger.debug(f"Hailo Process FPS: {FrameCount}")
                FrameCount = 0
                LastTime = CurrentTime
            BindingsList = self.YOLOQueue.get()
            self.InferTF = self.InferTF + 1
            # 确保在传递给 BindingsList 之前执行此操作
            try:
                self.ConfiguredInferModel.run(BindingsList, 1000)
            except Exception as e:
                self.logger.error(f"Hailo Inference Error: {e}, automatically restarting inference thread.")
                self.ModelInfer()
            BallOutputs = []
            ChassisOutputs = []
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
                CameraIndex += 1
                self.ChassisQueue.put(ChassisOutputs)
                self.BallQueue.put(BallOutputs)
            # print(ChassisList)
        
            # print(Output0)
            # time.sleep(0.01)

