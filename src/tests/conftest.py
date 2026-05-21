"""Pytest collection guard for script-style diagnostics.

Several files in this directory are operational smoke/diagnostic scripts. They
require Unity, real robot hardware, camera streams, network services, or user
input, so importing them during normal pytest collection can fail before any
unit test runs. Keep them runnable with `python -m ...`, but out of default
pytest discovery.
"""

collect_ignore = [
    "test_basicfuc_real.py",
    "test_luna_intelligence.py",
    "test_match_recording.py",
    "test_mjpeg.py",
    "test_mjpeg_stream.py",
    "test_mlp_control.py",
    "rplidar_test.py",
    "test_sim_heading_fix.py",
    "test_sim_yolo_vision_image.py",
    "test_training_env.py",
    "unity_basic_test.py",
]
