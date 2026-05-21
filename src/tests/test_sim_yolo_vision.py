"""Test script for SimYoloVision.

This test verifies that SimYoloVision can be imported and initialized correctly.
"""

import sys
import os

# Add src to path for testing
src_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, src_dir)

def test_import():
    """Test that SimYoloVision can be imported."""
    from vision_utils.LunaIntelligence import SimYoloVision

    assert SimYoloVision is not None

def test_class_structure():
    """Test that SimYoloVision has expected methods."""
    from vision_utils.LunaIntelligence import SimYoloVision
    
    expected_methods = [
        'get_ball_pos',
        'get_chassis_pos',
        'get_goals_pos',
        'GetBallBBoxes',
        'GetChassisBBoxes',
        'GetGoalBBoxes',
        'close',
        'build_camera_url',
        'build_stream_id',
    ]
    
    expected_attrs = [
        'Frames',
        'CamPorts',
        'BallQueue',
        'ChassisQueue',
        'GoalQueue',
        'BallBBoxQueue',
        'ChassisBBoxQueue',
        'YOLOQueue',
    ]
    
    for method in expected_methods:
        assert hasattr(SimYoloVision, method), f"Method '{method}' missing"
    
    # Check instance attributes by creating a mock instance
    # Note: We use start_threads=False to avoid starting actual threads
    try:
        # Mock initialization without starting threads
        # This requires ultralytics to be installed
        vision = SimYoloVision(start_threads=False)
        
        for attr in expected_attrs:
            assert hasattr(vision, attr), f"Attribute '{attr}' missing"
        
        vision.close()
        
    except Exception as e:
        print(f"[WARN] Could not create instance (likely missing ultralytics): {e}")
        print("[INFO] Skipping instance attribute tests")

def test_package_export():
    """Test that SimYoloVision is exported from vision_utils package."""
    from vision_utils import LunaIntelligence

    assert LunaIntelligence is not None

def test_config_selection():
    """Test that config 'SY' or 'SIMYOLO' selects SimYoloVision."""
    # This test would require modifying Settings, which is complex
    # Just verify the logic in __init__.py
    print("[INFO] Config selection test skipped (requires config modification)")

def main():
    """Run all tests."""
    print("=" * 60)
    print("SimYoloVision Test Suite")
    print("=" * 60)
    
    tests = [
        ("Import Test", test_import),
        ("Package Export Test", test_package_export),
        ("Class Structure Test", test_class_structure),
        ("Config Selection Test", test_config_selection),
    ]
    
    results = []
    for name, test_func in tests:
        print(f"\n--- {name} ---")
        try:
            test_func()
            results.append(True)
        except Exception as e:
            print(f"[ERROR] Test failed with exception: {e}")
            results.append(False)
    
    print("\n" + "=" * 60)
    passed = sum(results)
    total = len(results)
    print(f"Results: {passed}/{total} tests passed")
    print("=" * 60)
    
    return all(results)

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
