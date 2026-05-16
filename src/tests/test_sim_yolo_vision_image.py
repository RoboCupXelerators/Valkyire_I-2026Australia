"""Test SimYoloVision with a static image (test1.jpg).

This script loads an image, runs YOLO inference using SimYoloVision's
inference logic, and draws bounding boxes on the detected objects.
"""

import sys
import os

# Add src to path
src_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, src_dir)

import cv2
import numpy as np
from pathlib import Path

# Class names and colors for visualization
CLASS_NAMES = {
    0: "Ball",
    1: "BlueGoal",
    2: "YellowGoal",
    3: "Chassis"
}

CLASS_COLORS = {
    0: (0, 255, 0),      # Ball - Green
    1: (255, 0, 0),      # BlueGoal - Blue
    2: (0, 255, 255),    # YellowGoal - Yellow
    3: (0, 0, 255),      # Chassis - Red
}


def resize_letterbox(frame, target_size=(640, 640)):
    """Letterbox resize to target size with padding."""
    height, width = frame.shape[:2]
    target_height, target_width = target_size

    scale = min(target_width / width, target_height / height)
    new_width = int(width * scale)
    new_height = int(height * scale)

    resized = cv2.resize(frame, (new_width, new_height))
    padded = np.zeros((target_height, target_width, 3), dtype=np.uint8)

    y_offset = (target_height - new_height) // 2
    x_offset = (target_width - new_width) // 2
    padded[y_offset:y_offset + new_height, x_offset:x_offset + new_width] = resized

    return padded, scale, x_offset, y_offset


def draw_detections(image, detections, scale, x_offset, y_offset):
    """Draw bounding boxes on the image.
    
    Args:
        image: Original image (BGR)
        detections: List of (cls, conf, x1, y1, x2, y2) in 640x640 space
        scale: Scale factor from letterbox resize
        x_offset: X offset from letterbox resize
        y_offset: Y offset from letterbox resize
    
    Returns:
        Image with drawn bounding boxes
    """
    result = image.copy()
    
    for det in detections:
        cls, conf, x1, y1, x2, y2 = det
        
        # Convert from 640x640 space back to original image space
        orig_x1 = int((x1 - x_offset) / scale)
        orig_y1 = int((y1 - y_offset) / scale)
        orig_x2 = int((x2 - x_offset) / scale)
        orig_y2 = int((y2 - y_offset) / scale)
        
        # Clamp to image bounds
        h, w = result.shape[:2]
        orig_x1 = max(0, min(orig_x1, w))
        orig_y1 = max(0, min(orig_y1, h))
        orig_x2 = max(0, min(orig_x2, w))
        orig_y2 = max(0, min(orig_y2, h))
        
        # Get class info
        cls_name = CLASS_NAMES.get(int(cls), f"cls_{int(cls)}")
        color = CLASS_COLORS.get(int(cls), (128, 128, 128))
        
        # Draw bounding box
        cv2.rectangle(result, (orig_x1, orig_y1), (orig_x2, orig_y2), color, 2)
        
        # Draw label
        label = f"{cls_name}: {conf:.2f}"
        label_size, baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(
            result,
            (orig_x1, orig_y1 - label_size[1] - 10),
            (orig_x1 + label_size[0], orig_y1),
            color,
            -1
        )
        cv2.putText(
            result,
            label,
            (orig_x1, orig_y1 - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )
    
    return result


def main():
    """Main test function."""
    # Paths
    vision_utils_dir = Path(src_dir) / "vision_utils"
    test_image_path = vision_utils_dir / "test2.jpg"
    model_path = vision_utils_dir / "models" / "yolov8nForX5.pt"
    output_path = vision_utils_dir / "test2_detected.jpg"
    
    # Check if test image exists
    if not test_image_path.exists():
        print(f"[ERROR] Test image not found: {test_image_path}")
        return False
    
    print(f"[INFO] Loading image: {test_image_path}")
    
    # Load image
    image = cv2.imread(str(test_image_path))
    if image is None:
        print(f"[ERROR] Failed to load image: {test_image_path}")
        return False
    
    print(f"[INFO] Image size: {image.shape[1]}x{image.shape[0]}")
    
    # Resize to 640x640 with letterbox
    image_640, scale, x_offset, y_offset = resize_letterbox(image, (640, 640))
    
    # Load YOLO model
    print(f"[INFO] Loading YOLO model: {model_path}")
    try:
        from ultralytics import YOLO
    except ImportError:
        print("[ERROR] ultralytics not installed. Install with: pip install ultralytics")
        return False
    
    if not model_path.exists():
        print(f"[WARN] Model not found: {model_path}")
        print("[INFO] Trying to use default yolov8n.pt...")
        model = YOLO("yolov8n.pt", task="detect")
    else:
        model = YOLO(str(model_path), task="detect")
    
    # Run inference
    print("[INFO] Running inference...")
    results = model.predict(image_640, verbose=False, conf=0.05, imgsz=640)
    
    # Parse detections
    detections = []
    for r in results:
        if r.boxes is None or len(r.boxes) == 0:
            continue
        
        boxes = r.boxes
        xyxy = boxes.xyxy.detach().cpu().numpy()
        conf = boxes.conf.detach().cpu().numpy()
        cls = boxes.cls.detach().cpu().numpy()
        
        for (x1, y1, x2, y2), c, k in zip(xyxy, conf, cls):
            detections.append((int(k), float(c), float(x1), float(y1), float(x2), float(y2)))
            cls_name = CLASS_NAMES.get(int(k), f"cls_{int(k)}")
            print(f"[DETECT] {cls_name}: conf={c:.2f}, bbox=({int(x1)},{int(y1)},{int(x2)},{int(y2)})")
    
    if not detections:
        print("[WARN] No detections found")
    else:
        print(f"[INFO] Found {len(detections)} detections")
    
    # Draw detections on original image
    result_image = draw_detections(image, detections, scale, x_offset, y_offset)
    
    # Save result
    cv2.imwrite(str(output_path), result_image)
    print(f"[INFO] Result saved to: {output_path}")
    
    # Also show the result if possible
    try:
        cv2.imshow("Detection Result", result_image)
        print("[INFO] Press any key to close the window...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    except Exception as e:
        print(f"[INFO] Could not display image (headless mode?): {e}")
    
    return True


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)