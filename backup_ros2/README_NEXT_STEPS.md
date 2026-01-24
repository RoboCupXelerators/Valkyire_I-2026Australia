Next steps after backup creation:

1) Validate helper mapping:
   - Confirm camera_to_robot mapping for each camera index with a small test or known landmarks.
2) Replace other strategy files to use angle_from_xy and camera_to_robot where appropriate.
3) Add unit tests for angle_from_xy and coordinate conversions.
4) Integrate into ROS2 node later: publish/subscribe geometry_msgs/Twist and sensor_msgs/Imu conventions.

If you want, I can proceed to apply the mapping changes to other files (LunaPre, strategies) automatically.
