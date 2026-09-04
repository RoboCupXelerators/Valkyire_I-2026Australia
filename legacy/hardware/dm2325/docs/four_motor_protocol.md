# Four-Motor Protocol

source=design

Pi and C board use framed USB CDC messages with CRC16, sequence matching, a 250-byte payload limit, and typed ACK/NACK responses. Chassis commands are `ARM`, `TWIST`, `STOP`, `DISABLE`, `EMERGENCY_STOP`, `CLEAR_FAULT`, and status queries. Production motor order is always `FL, FR, RL, RR`.

`EMERGENCY_STOP` is a zero-payload command distinct from normal Disable. It clears all targets, sends safe zero/Disable output, and latches runtime fault `USER_ESTOP=8`. `CLEAR_FAULT` is accepted only with a zero-length payload and fresh Disabled feedback from all four motors.

The 248-byte chassis status contains board state and CAN diagnostics followed by four independent 52-byte wheel records. Each wheel includes ESC/Master ID, state flags, fault code, temperatures, wrapped and unwrapped position, velocity, torque, raw fields, feedback sequence/tick/age, RX/invalid/stale counters, and last raw CAN ID. Global fields include control sequence/ticks, CAN TX failures, command skew, scheduler lateness, runtime fault, and ID mismatch count.

Parameter replies are accepted only while a matching operation is pending and only when ESC ID, Master ID, opcode, and register all match. Ordinary feedback bytes are never classified by the presence of `0x33`, `0x55`, or `0xAA` alone.
