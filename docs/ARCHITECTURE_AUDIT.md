# Software Architecture Audit

Date: 2026-09-07. Baseline: `9a4a5c1` on `main`.

## Branch Boundary

- `legacy` retains ROS2, LD07/LD06, RPLidar history and DM2325 tooling.
  Commit `a45a43e` restores the DM2325 public driver and runtime factory.
  Handshake does not arm motors; explicit Arm and existing stop/disable remain.
- `main` runs SIM and IL with Bits pose. Native scan drivers, scan workers,
  hardware archive, serial scan probes and ROS setup dependency are removed.
- ArisuBits is removed from main. ArisuIntelligence/Hailo vision is retained.
- Structure changes apply only to main. No remote deployment is performed.

## Findings Resolved

| Finding | Result |
| --- | --- |
| SIM wall geometry was duplicated by its Lidar adapter and Sensor | One shared Bits pose-to-wall calculation in Sensor |
| Sensor wrapped native driver diagnostics and source fallback | Remove native dispatch, scan threads and diagnostic forwarding |
| Car forwarded fourteen private methods to MotionBackend | Call the mixer directly; preserve movement and permission behavior |
| Runtime rewrapped RecordingCar constructor arguments | Construct RecordingCar directly |
| Package lifecycle functions synchronized a second set of runtime globals | Stable proxies follow the current instance across restart |
| Runtime shutdown discarded its serial referee without stopping it | Close the owned referee worker before clearing providers |
| ReasonData setup repeatedly loaded configuration and added another log sink | Initialize configuration and logging in their owning modules |
| Configuration migration overwrote current camera order | Apply camera mapping only when legacy VisionVals exists |
| Migration discarded its original configuration | Keep cfg.cache.json after validating migration |
| Strategy tests patched process-wide time functions and could kill background threads | Isolate each candidate clock in its test fixture |
| Positions repeated distance forwarding and chassis stop | Alias the identical snapshot query; stop each owned function once |
| Service installer required ROS and referenced removed main.py | Start the project Python environment with an explicitly selected entrypoint |
| Control panel offered retired sources and scan diagnostics | Settings and wall preview match the Bits-only runtime |

## Interfaces Kept

`FieldRobotPose()` remains a zero-argument observation query. `DirectDistance()`
and `DirectDistanceSnapshot()` return front/left/back/right millimetres and keep
the unavailable sentinel. Motion APIs, referee gating, simulated messaging,
vision backends, calibration, recording and MLP observation semantics remain.

`runtime.lidar` remains an alias to Sensor for existing callers. The public
`auto_start_lidar` argument is accepted for existing entrypoints and has no
hardware effect on main. Removed native Lidar constructors and diagnostics
belong to legacy. `Sensor.Source=auto/bits` migrates to the sole Bits source;
explicit native source settings are rejected before modifying the file.

RecordingCar and RecordingPositions remain because they implement output and
position capture. MotionBackend retains wheel allocation, filtering and output
limits. Positions retains navigation and vision semantics. These are functional
boundaries, not pass-through layers; merging them would reduce file count while
increasing coupling. The existing vision, MLP, strategy and OTA boundaries remain.

## Validation

- Parsed all 118 remaining Python files; checked shell and browser script syntax.
- Baseline with local networking: 307 passed, 2 failed, 3 skipped, 1 deselected.
  ROS launch ignored the instance setup path; repaired on legacy. A lockball test
  depended on deployment tuning; its fixture now supplies its own turn limits.
- Main: 297 passed, 3 skipped, 1 deselected. Native-only tests remain on legacy;
  the smaller main count is not an exclusion of active features.
- Legacy compatibility: 58 tests passed, including DM protocol/lifecycle,
  runtime assembly, ROS launch/scan handling, LD drivers and sensor contracts.
- Chrome desktop (1440 x 1000) and mobile (390 x 844): overview/settings render,
  retired settings absent, no page script errors, pose endpoint remains available
  without starting hardware.

Skipped tests require PyTorch; the default camera marker is deselected.
Physical ROS/serial devices, motor movement, GPU inference and a live Unity match
were not available for acceptance testing. Unit and transport tests establish
software contracts, not physical hardware acceptance. The full 118-file syntax
scan and core-path review are not a claim that every algorithm is defect-free.
