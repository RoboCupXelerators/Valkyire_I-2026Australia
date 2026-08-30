# DM-S2325 Capabilities

source=design

The backend uses MIT Enable/Disable and fields `p_des`, `v_des`, `kp`, `kd`, and `t_ff`, plus independent position, velocity, torque, temperature, motor state, fault, online, and feedback-age telemetry. Initial velocity control fixes `p_des=0`, `kp=0`, `t_ff=0`, and uses the previously verified conservative `kd`.

The architecture retains full MIT fields for future damping, friction, start-up, and load compensation. High position stiffness, torque feedforward, position locking, mode changes, and automatic persistent-parameter changes are explicitly disabled until separate evidence and approval exist.
