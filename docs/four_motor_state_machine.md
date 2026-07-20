# Four-Motor State Machine

source=design

The firmware state sequence is `BOOT -> DISABLED -> ARMING -> READY -> RUNNING -> STOPPING -> DISABLED`; faults enter `FAULT`.

- Startup validates unique IDs and rank-3 geometry and never enables a motor.
- ARMING enables FL, FR, RL, RR sequentially and requires a new healthy Enabled feedback frame after each command.
- READY continuously sends safe zero targets while accepting fresh chassis commands.
- RUNNING applies speed, acceleration, jerk, inverse kinematics, and common wheel saturation.
- STOP returns RUNNING to READY with zero targets while motors remain Enabled.
- DISABLE enters STOPPING, sends at least 200 ms of zeros, sends four Disable frames, and requires fresh Disabled confirmation. One bounded confirmation retry occurs after 75 ms; no unbounded retry exists.
- FAULT clears targets, sends zero and Disable to all wheels, preserves the first runtime fault, records subsequent fault and count separately, and requires explicit clear.
- User emergency stop enters `FAULT(USER_ESTOP=8)` even when invoked from Disabled; it cannot be bypassed by Arm.

Any stale/offline/fault wheel, host watchdog expiry, CAN failure, or invalid configuration triggers the unified safety path.
