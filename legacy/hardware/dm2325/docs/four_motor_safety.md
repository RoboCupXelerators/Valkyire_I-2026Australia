# Four-Motor Safety

source=design

Nonzero hardware output is compile-time gated while drive directions and motor signs are unconfirmed. Production startup is Disabled and never auto-arms. Persistent motor IDs, mode, baud rate, and zero point are not modified.

Real nonzero tests are limited to low speed, two seconds by default, five seconds maximum, and one attempt per direction. No failed nonzero test is repeated automatically. Long-duration evidence comes from Mock, simulation, feedback replay, Disabled soak, or zero output.

`stop` zeros the target while retaining Enable. `disable` zeros for at least 200 ms and confirms all four Disabled. `shutdown` stops, disables, closes CDC, and releases resources. `emergency_stop` rejects new motion, clears pending targets, and enters FAULT. No finally block is allowed to restore a prior nonzero command.

The hardware zero-output E-stop acceptance exercised `DISABLED -> FAULT(USER_ESTOP=8) -> explicit ClearFault -> DISABLED`. Throughout the test, `online_mask=0x0F`, `enabled_mask=0`, `fault_mask=0`, and `CAN_ESR=0`. Evidence is in `docs/four_motor_emergency_stop_disabled_test.txt`, with build and SWD verify logs in `docs/cboard_emergency_stop_build.txt` and `docs/cboard_emergency_stop_flash.txt`.
