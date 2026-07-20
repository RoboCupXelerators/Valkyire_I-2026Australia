# Four-Wheel Chassis Acceptance Report

source=engineering-summary

Status: **IN PROGRESS - NOT FINAL ACCEPTANCE**

This report deliberately keeps every incomplete physical gate visible. Runtime
authority is `~/Software`; the Pi backend is
`src/utils/DM2325Bits.py`. Historical firmware and hardware evidence was
migrated byte-for-byte from `~/Software-refactor/docs` into `~/Software/docs`.

1. Main workspace: `~/Software`; no runtime staging dependency found in the DM backend.
2. Build result: current-tree clean Release build passed with GCC `13.2.1` and CMake `3.28.3`; 14 third-party/newlib warning lines were retained.
3. ELF/BIN/MAP SHA256: `24bd6e...3f9`, `c9c12a...f2b`, `6405b1...3f1`; see `cboard_four_motor_build_current.txt`.
4. SWD verify: the current BIN is byte-identical to the last flashed `build-cboard-imu-final6-clean` BIN; its log records `Programming Finished`, `Verified OK`, reset, and CDC recovery.
5. Motor mapping: FL `01/11`, FR `02/12`, RL `03/13`, RR `04/14`.
6. 80/100 definition: wheel-center candidates are documented; physical drive axes remain unconfirmed.
7. Wheel positions and drive directions: mathematical candidates only, marked `TEMPORARY_UNCONFIRMED`.
8. Jacobian: recorded in `nonorthogonal_kinematics_derivation.md` and generated golden JSON.
9. Rank: `3`.
10. Condition number: `11.733076488` for the current candidate geometry.
11. Python/C error: 1011 vectors pass; maximum motor-target difference `2.12170974496e-05 rad/s`, maximum C round-trip component error `5.1908527432e-07`.
12. Tests: `343/343` current-tree Python and native C host tests pass.
13. Disabled soak: hardware PASS, 1200/1200, historical raw evidence retained.
14. Zero output: hardware v2 PASS for 30 seconds; historical FAIL retained separately.
15. Single-wheel mapping: all ESCs produced electronic motion evidence; physical direction labels remain pending.
16. Nonzero durations: FL/RL approximately 2 s at `+0.3 rad/s`; approved FR/RR diagnostics approximately 2 s at `+1.0 rad/s`.
17. Nonzero target speeds: as listed above; the earlier user-approved FR 10 s diagnostic is historical and is not a permitted default test.
18. Lifted forward result: not executed.
19. Lifted backward result: not executed.
20. Lifted left result: not executed.
21. Lifted right result: not executed.
22. Lifted clockwise result: not executed.
23. Lifted counterclockwise result: not executed.
24. Forward-kinematics error: 10-minute Mock maximum twist reconstruction error `0.0009287496802161954`.
25. Four-wheel tracking: zero-output evidence available; combined nonzero tracking pending direction confirmation.
26. Command skew: zero-output v2 maximum `130 us`.
27. Scheduler lateness: zero-output v2 maximum `0` in the retained acceptance record.
28. Torque, temperature, diagnostics: per-wheel telemetry and advisory diagnostics implemented; ground-load validation pending.
29. CAN errors: historical accepted Disabled/zero-output runs reported zero; the latest motor-power-off snapshot also has `CAN_ESR=0`, but remains invalid for acceptance because all four feedback channels are offline. An earlier motor-off snapshot transiently reported `0x280030` and is retained in the interim status record.
30. Fault record: historical zero-output first attempt and FR stall evidence retained; no record was overwritten.
31. Watchdog: unit tests and 10-minute Mock injection pass; hardware CDC watchdog evidence is retained separately.
32. Legacy Software: Car facade, SI twist, field movement, strategy, and teleop tests pass in the current suite.
33. Teleop: conservative W/S/A/D/Q/E, Space stop, ESC shutdown, explicit Arm, and 150 ms deadman implemented.
34. Final safety state: latest read-only snapshot is C-board DISABLED but all four motors offline; therefore not accepted as confirmed four-motor Disabled.
35. User visual confirmation: four motors were seen turning; wheel-positive directions were explicitly deferred.
36. Documentation: required documents are indexed in `docs`; lifted and ground reports remain explicitly deferred.
37. Commit: no commit created.
38. Push: no push performed.
39. Remaining calibration: motor signs, physical drive directions, wheel geometry measurements, radius/speed scales, angle/position offsets, lifted combined motion, and all ground motion.

Completion is prohibited until the outstanding direction, lifted-motion, ground
motion, current firmware provenance, and final four-motor online Disabled gates
have direct evidence.
