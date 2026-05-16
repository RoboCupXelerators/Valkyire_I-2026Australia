"""Test LunaIntelligence (SimYoloVision) with Unity simulator.

This test connects to the Unity simulator's MJPEG camera streams,
runs YOLO inference, and outputs detection results.
"""

import sys
import os
import time

# Add src to path
src_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, src_dir)

def main():
    """Main test function."""
    print("=" * 60)
    print("LunaIntelligence (SimYoloVision) Simulator Test")
    print("=" * 60)
    
    # Import the vision module
    try:
        from vision_utils.LunaIntelligence import SimYoloVision
        print("[INFO] LunaIntelligence imported successfully")
    except ImportError as e:
        print(f"[ERROR] Failed to import LunaIntelligence: {e}")
        return False
    
    # Create vision instance
    print("\n[INFO] Creating SimYoloVision instance...")
    try:
        vision = SimYoloVision(
            model_path=os.path.join(src_dir, "vision_utils", "models", "yolov8nForX5.pt"),
            conf_ball=0.30,
            conf_chassis=0.50,
            start_threads=True,
        )
        print("[INFO] SimYoloVision instance created")
    except Exception as e:
        print(f"[ERROR] Failed to create SimYoloVision: {e}")
        return False
    
    # Wait for frames to be captured
    print("\n[INFO] Waiting for camera frames (5 seconds)...")
    time.sleep(5.0)
    
    # Run detection loop
    print("\n[INFO] Running detection loop (10 iterations)...")
    print("-" * 60)
    
    for i in range(10):
        print(f"\n--- Iteration {i+1}/10 ---")
        
        # Get frames status
        frames = vision.Frames
        frames_status = [f"Cam{j}: {'OK' if f is not None else 'None'}" for j, f in enumerate(frames)]
        print(f"[INFO] Frames: {', '.join(frames_status)}")
        
        # Get ball position
        ball_pos = vision.get_ball_pos()
        print(f"[DETECT] Ball position: {ball_pos}")
        
        # Get chassis positions
        chassis_pos = vision.get_chassis_pos()
        if chassis_pos:
            print(f"[DETECT] Chassis detections: {len(chassis_pos)}")
            for c in chassis_pos:
                print(f"        - Cam{c[0]}: cx={c[1]}, bottom_y={c[2]}, conf={c[5]:.2f}")
        
        # Get goal positions
        goals = vision.get_goals_pos()
        if goals.get("blue_goal"):
            print(f"[DETECT] Blue goal: {goals['blue_goal']}")
        if goals.get("yellow_goal"):
            print(f"[DETECT] Yellow goal: {goals['yellow_goal']}")
        
        # Get ball bboxes
        ball_bboxes = vision.GetBallBBoxes()
        valid_bboxes = [b for b in ball_bboxes if b is not None]
        if valid_bboxes:
            print(f"[DETECT] Ball bboxes: {len(valid_bboxes)} valid")
            for b in valid_bboxes:
                print(f"        - Cam{b[0]}: ({b[1]},{b[2]},{b[3]},{b[4]}) conf={b[5]:.2f}")
        
        # Get goal bboxes
        goal_bboxes = vision.GetGoalBBoxes()
        valid_goal_bboxes = [g for g in goal_bboxes if g is not None]
        if valid_goal_bboxes:
            print(f"[DETECT] Goal bboxes: {len(valid_goal_bboxes)} valid")
            for g in valid_goal_bboxes:
                cls_name = "BlueGoal" if g[1] == 1 else "YellowGoal"
                print(f"        - Cam{g[0]}: {cls_name} ({g[2]},{g[3]},{g[4]},{g[5]}) conf={g[6]:.2f}")
        
        # Performance counters
        print(f"[PERF] ReadCamTF={vision.ReadCamTF}, PreProcessTF={vision.PreProcessTF}, InferTF={vision.InferTF}")
        
        time.sleep(0.5)
    
    print("\n" + "-" * 60)
    print("[INFO] Test completed")
    
    # Cleanup
    print("\n[INFO] Closing vision instance...")
    vision.close()
    print("[INFO] Done")
    
    return True


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)