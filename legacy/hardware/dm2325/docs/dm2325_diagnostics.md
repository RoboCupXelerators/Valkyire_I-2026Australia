# DM-S2325 Diagnostics

source=design

Each wheel must retain independent wrapped/unwrapped position, speed, torque, motor and rotor temperatures, online state, motor state, fault code, feedback age, receive count, invalid count, and stale count. Chassis velocity and odometry are reconstructed from feedback through forward kinematics; command values are never substituted as feedback.

Advisory diagnostic codes are `WHEEL_STALL_SUSPECTED`, `WHEEL_CONTACT_LOSS_SUSPECTED`, `WHEEL_DRAG_SUSPECTED`, `SLIP_SUSPECTED`, `MAPPING_ERROR_SUSPECTED`, `CHASSIS_IMPACT_OR_BLOCKAGE`, `TEMPERATURE_WARNING`, and `TEMPERATURE_FAULT`.

`diagnose_wheel()` evaluates real target, speed, torque, temperature, and residual values. It distinguishes near-zero high-torque stall, partial-speed high-torque drag, high-speed low-torque contact loss, sign-opposed mapping errors, residual-based slip, and warning/fault temperatures. `diagnose_chassis()` consumes four values in canonical `FL, FR, RL, RR` order and reports multi-wheel blockage, mapping, and temperature findings. Thresholds remain explicitly `TEMPORARY`; diagnostics never rewrite geometry, signs, gains, or persistent motor parameters.

Thresholds remain temporary until measured data exists. A single advisory event never changes gains, IDs, geometry, or persistent motor parameters. Fault-grade temperature, stale feedback, motor fault, CAN bus-off, or watchdog expiry causes the unified zero-and-disable path.

The FR direction retry is a concrete stall example: commanded velocity `+0.3 rad/s`, measured velocity at the zero baseline, stable torque approximately `0.237 Nm`, fresh feedback, Enabled state, and no motor fault for 10 seconds. It is recorded as `WHEEL_STALL_SUSPECTED`, not as a CAN, mapping, or control-mode failure.
