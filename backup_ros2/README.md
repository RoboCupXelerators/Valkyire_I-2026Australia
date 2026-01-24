Backup (ROS2 convention)

This folder contains a ROS2-coordinate-system-adapted backup of selected source files.
Convention used:
- x: forward
- y: left
- yaw: around +z, positive counter-clockwise (CCW)

Files:
- src/Vision.py (modified to expose BallPos as [x_forward, y_left])
- src/LunaPre.py (uses angle_from_xy helper)

Note: these are backup copies and not wired into the main project. They include small helper functions at the top to transform camera coordinates into robot-frame coordinates following the ROS2 convention. Review and adapt further before integrating into the main codebase.
