from .PreProcess import Settings, logger, MODEL_DIR, VisionPreUntil, threading, queue, time, cv2, np

import numpy as np
import cv2
from typing import Dict, List

import numpy as np
import cv2


class TunaVision(VisionPreUntil):
    '''
    support for RDK x5 BPU
    '''
    def __init__(self):
        super().__init__()
        import bpu_infer_lib
        self.inf = bpu_infer_lib.Infer(False)
        self.inf.load_model(str(MODEL_DIR / "yolov8n.bin"))


        self.YOLOQueue = queue.Queue(maxsize=1)
        self.ChassisQueue = queue.Queue(maxsize=1)
        self.BallQueue = queue.Queue(maxsize=1)


        self.YoloProcessThread = threading.Thread(target=self.YoloProcess)
        self.YoloProcessThread.daemon = True
        self.YoloProcessThread.start()

        self.ModelInferThread = threading.Thread(target=self.ModelInfer)
        self.ModelInferThread.daemon = True
        self.ModelInferThread.start()


    def bgr2nv12_opencv(self, image):
        height, width = image.shape[0], image.shape[1]
        area = height * width
        yuv420p = cv2.cvtColor(image, cv2.COLOR_BGR2YUV_I420).reshape((area * 3 // 2,))
        y = yuv420p[:area] # Y分量：前area个元素
        uv_planar = yuv420p[area:].reshape((2, area // 4)) # UV分量：后面的元素，每2个元素分别为U和V分量
        uv_packed = uv_planar.transpose((1, 0)).reshape((area // 2,)) # 将UV分量交替排列为交错的UV格式
        nv12 = np.zeros_like(yuv420p) # 创建与原YUV数据形状相同的空数组用于存放NV12格式数据
        nv12[:height * width] = y # 将Y分量直接赋值到NV12数组的前部
        nv12[height * width:] = uv_packed  # 将交错的UV分量赋值到NV12数组的后部
        return nv12

    def YoloProcess(self):
        while(1):
            BindingsList = []
            for i in range(4):
                Frame = self.Frames[i]
                if Frame is not None:
                    BindingsList.append(self.bgr2nv12_opencv(Frame))
            if self.YOLOQueue.full():
                pass
            else:
                self.PreProcessTF = self.PreProcessTF + 1
                self.YOLOQueue.put(BindingsList)
            time.sleep(0.01)
        
    def ModelInfer(self):
        while(1):
            print(1)
            FrameCount = 0
            LastTime = time.time()
            # while(1):
            FrameCount += 1
            CurrentTime = time.time()
            if CurrentTime - LastTime >= 1.0:
                logger.success(f"Hailo Process FPS: {FrameCount}")
                FrameCount = 0
                LastTime = CurrentTime
            BindingsList = self.YOLOQueue.get()
            self.InferTF = self.InferTF + 1
            # 确保在传递给 BindingsList 之前执行此操作
            BallOutputs = []
            ChassisList = []
            count = 0
            for Bindings in BindingsList:
                flag = 0
                self.inf.read_input(Bindings, 0)
                self.inf.forward(True) 
                self.inf.get_output()
                # TODO finish data after-process
                # breakpoint()
                
                # breakpoint()
                # self.infer.outputs.data
                # x_start, x_end, y_start, y_end, confidence, class_id = detection
                # BallOutputs.append([CenterX*self.IMAGE_SIZE/10, BottomY*self.IMAGE_SIZE/10, BallWidth*self.IMAGE_SIZE/10, BallHeight*self.IMAGE_SIZE/10, confidence])
                # BallOutputs.append([0,0,0,0,0])
                    

                # # 0 512000
                # # 1 409600
                # # 2 128000
                # # 3 102400
                # # 4 32000
                # # 5 25600
                # ChassisList.append(self.multi_scale_yolov8_post_process(bpu_raw_outputs_dict))
            logger.info(BallOutputs)
            self.ChassisQueue.put(ChassisList)
            self.BallQueue.put(BallOutputs)

    def PostProcess(self, method=1):
        """
        后处理函数
        Args:
            method: 选择使用的方法
                - '0': 使用get_output
                - '1': 使用get_infer_res_np_float32
        """
        if method == 1 :
            # 方法1：使用get_infer_res_np_float32获取原始输出并处理
            print("\n=== 方法1: 使用get_infer_res_np_float32 ===")
            s_pred = self.inf.get_infer_res_np_float32(0)
            m_pred = self.inf.get_infer_res_np_float32(1)
            l_pred = self.inf.get_infer_res_np_float32(2)
            print(f"原始输出: {s_pred.shape = }  {m_pred.shape = }  {l_pred.shape = }")

            # reshape
            s_pred = s_pred.reshape([-1, (5 + self.nc)])
            m_pred = m_pred.reshape([-1, (5 + self.nc)])
            l_pred = l_pred.reshape([-1, (5 + self.nc)])
            print(f"Reshape后: {s_pred.shape = }  {m_pred.shape = }  {l_pred.shape = }")

            # classify: 利用numpy向量化操作完成阈值筛选
            s_raw_max_scores = np.max(s_pred[:, 5:], axis=1)
            s_max_scores = 1 / ((1 + np.exp(-s_pred[:, 4]))*(1 + np.exp(-s_raw_max_scores)))
            s_valid_indices = np.flatnonzero(s_max_scores >= self.conf)
            s_ids = np.argmax(s_pred[s_valid_indices, 5:], axis=1)
            s_scores = s_max_scores[s_valid_indices]

            m_raw_max_scores = np.max(m_pred[:, 5:], axis=1)
            m_max_scores = 1 / ((1 + np.exp(-m_pred[:, 4]))*(1 + np.exp(-m_raw_max_scores)))
            m_valid_indices = np.flatnonzero(m_max_scores >= self.conf)
            m_ids = np.argmax(m_pred[m_valid_indices, 5:], axis=1)
            m_scores = m_max_scores[m_valid_indices]

            l_raw_max_scores = np.max(l_pred[:, 5:], axis=1)
            l_max_scores = 1 / ((1 + np.exp(-l_pred[:, 4]))*(1 + np.exp(-l_raw_max_scores)))
            l_valid_indices = np.flatnonzero(l_max_scores >= self.conf)
            l_ids = np.argmax(l_pred[l_valid_indices, 5:], axis=1)
            l_scores = l_max_scores[l_valid_indices]

            # 特征解码
            s_dxyhw = 1 / (1 + np.exp(-s_pred[s_valid_indices, :4]))
            s_xy = (s_dxyhw[:, 0:2] * 2.0 + self.s_grid[s_valid_indices,:] - 1.0) * self.strides[0]
            s_wh = (s_dxyhw[:, 2:4] * 2.0) ** 2 * self.s_anchors[s_valid_indices, :]
            s_xyxy = np.concatenate([s_xy - s_wh * 0.5, s_xy + s_wh * 0.5], axis=-1)

            m_dxyhw = 1 / (1 + np.exp(-m_pred[m_valid_indices, :4]))
            m_xy = (m_dxyhw[:, 0:2] * 2.0 + self.m_grid[m_valid_indices,:] - 1.0) * self.strides[1]
            m_wh = (m_dxyhw[:, 2:4] * 2.0) ** 2 * self.m_anchors[m_valid_indices, :]
            m_xyxy = np.concatenate([m_xy - m_wh * 0.5, m_xy + m_wh * 0.5], axis=-1)

            l_dxyhw = 1 / (1 + np.exp(-l_pred[l_valid_indices, :4]))
            l_xy = (l_dxyhw[:, 0:2] * 2.0 + self.l_grid[l_valid_indices,:] - 1.0) * self.strides[2]
            l_wh = (l_dxyhw[:, 2:4] * 2.0) ** 2 * self.l_anchors[l_valid_indices, :]
            l_xyxy = np.concatenate([l_xy - l_wh * 0.5, l_xy + l_wh * 0.5], axis=-1)

            # 大中小特征层阈值筛选结果拼接
            xyxy = np.concatenate((s_xyxy, m_xyxy, l_xyxy), axis=0)
            scores = np.concatenate((s_scores, m_scores, l_scores), axis=0)
            ids = np.concatenate((s_ids, m_ids, l_ids), axis=0)

        elif method == 0:
            # 方法2：使用get_output获取输出
            print("\n=== 方法2: 使用get_output ===")
            if not self.inf.get_output():
                raise RuntimeError("获取输出失败")

            classes_scores = self.inf.outputs[0].data  # (1, 80, 80, 18)
            bboxes = self.inf.outputs[1].data         # (1, 40, 40, 18)
            print(f"classes_scores: shape={classes_scores.shape}")
            print(f"bboxes: shape={bboxes.shape}")

            # 直接使用4D数据
            # 每个网格有3个anchor，每个anchor预测6个值(4个框坐标+1个objectness+1个类别)
            batch, height, width, channels = classes_scores.shape
            num_anchors = 3
            pred_per_anchor = 6

            scores_list = []
            boxes_list = []
            ids_list = []
            # 处理每个网格点
            for h in range(height):
                for w in range(width):
                    for a in range(num_anchors):
                        # 获取当前anchor的预测值
                        start_idx = int(a * pred_per_anchor)
                        box = classes_scores[0, h, w, start_idx:start_idx+4].copy()  # 框坐标
                        obj_score = float(classes_scores[0, h, w, start_idx+4])      # objectness
                        cls_score = float(classes_scores[0, h, w, start_idx+5])      # 类别分数
                        # sigmoid激活
                        obj_score = 1 / (1 + np.exp(-obj_score))
                        cls_score = 1 / (1 + np.exp(-cls_score))
                        score = obj_score * cls_score

                        # 如果分数超过阈值，保存这个预测
                        if score >= self.conf:
                            # 解码框坐标
                            box = 1 / (1 + np.exp(-box))  # sigmoid
                            cx = float((box[0] * 2.0 + w - 0.5) * self.strides[0])
                            cy = float((box[1] * 2.0 + h - 0.5) * self.strides[0])
                            w_pred = float((box[2] * 2.0) ** 2 * self.anchors[0][a*2])
                            h_pred = float((box[3] * 2.0) ** 2 * self.anchors[0][a*2+1])

                            # 转换为xyxy格式
                            x1 = cx - w_pred/2
                            y1 = cy - h_pred/2
                            x2 = cx + w_pred/2
                            y2 = cy + h_pred/2

                            boxes_list.append([x1, y1, x2, y2])
                            scores_list.append(float(score))  # 确保是标量
                            ids_list.append(0)  # 假设只有一个类别
            if boxes_list:
                xyxy = np.array(boxes_list, dtype=np.float32)
                scores = np.array(scores_list, dtype=np.float32)
                ids = np.array(ids_list, dtype=np.int32)
            else:
                xyxy = np.array([], dtype=np.float32).reshape(0, 4)
                scores = np.array([], dtype=np.float32)
                ids = np.array([], dtype=np.int32)

        else:
            raise ValueError("method must be 0 or 1")

        # NMS处理
        indices = cv2.dnn.NMSBoxes(xyxy.tolist(), scores.tolist(), self.conf, self.iou)

        if len(indices) > 0:
            indices = np.array(indices).flatten()
            self.bboxes = (xyxy[indices] * np.array([self.x_scale, self.y_scale, self.x_scale, self.y_scale])).astype(np.int32)
            self.scores = scores[indices]
            self.ids = ids[indices]
        else:
            print("No detections after NMS")
            self.bboxes = np.array([], dtype=np.int32).reshape(0, 4)
            self.scores = np.array([], dtype=np.float32)
            self.ids = np.array([], dtype=np.int32)