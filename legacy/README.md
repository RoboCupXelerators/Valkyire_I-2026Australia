# Legacy Hardware Archive

This directory is intentionally outside the active runtime, dependency lists,
configuration schema, and pytest collection.

- `hardware/arisubits/` preserves the retired ArisuBits serial adapter.
- `hardware/dm2325/` preserves the retired DM2325 driver, teleop, hardware
  scripts, tests, and engineering records.
- `sensors/rplidar/` preserves the removed direct Python RPLidar driver.

These files are historical reference only. New robot configurations must use
`RoboInfo.Bit=SIM` or `IL` and `Sensor.Source=auto`, `bits`, `ros2`, or `ld07`.
