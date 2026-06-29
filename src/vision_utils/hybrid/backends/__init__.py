"""Hardware backend namespace for HybridVision.

The first implementation keeps TensorRT/PyTorch loading behind
`hybrid.yolo_detector.YoloDetector`; this package is reserved for backend
specialization without changing the fusion layer API.
"""
