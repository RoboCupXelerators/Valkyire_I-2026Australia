# Archived DM2325 Control Panel Surface

The active control panel no longer exposes the DM2325-specific protocol and
watchdog surface. The historical pytest coverage remains in
`tests/test_control_panel_dm_chassis.py` beside this note.

Removed server routes:

- `GET /api/control/telemetry`
- `POST /api/control/lease`
- `POST /api/control/profile`

Removed server helpers included `control_get_telemetry`, `_control_imu_telemetry`,
`control_renew_lease`, `control_set_profile`, and the 30 ms C-board command
heartbeat. The active control panel only exposes generic enable, speed, move,
move-vector, kick, and dribble actions for SIM and IceLoong runtime backends.
