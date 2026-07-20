# Legacy Software Compatibility

source=software-test

`Car` remains the strategy-facing facade. `RobotMoveVector` applies explicit legacy-to-SI scales before calling the DM chassis twist backend; `RobotMoveTwist` is the native SI API. Field movement, angle movement, turning, stop, disable, emergency stop, and shutdown retain their public names. Legacy wheel order conversion is explicit at the boundary; the new core uses only FL, FR, RL, RR.

SIM, AB, IL, and DM2325 runtime selection coexist. The DM path hides IDs, MIT fields, Enable frames, and CAN details from strategy code and requires explicit Arm. The low-speed `python -m teleop` entry supports W/S/A/D/Q/E, Space stop, ESC shutdown, a 150 ms deadman timeout, and conservative `0.02 m/s` / `0.10 rad/s` defaults.

Compatibility is covered by Car, legacy order, runtime strategy, teleop, and existing offence/defence/navigation tests. Real strategy motion remains disabled until hardware direction gates pass.
