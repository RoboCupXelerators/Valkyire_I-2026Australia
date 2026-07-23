# Tools

Legacy standalone camera/calibration utilities were removed from this folder.

Current calibration workflow:

- Use the control panel camera calibration page.
- Calibration files are saved under `src/ReasonData/data/Calibration/`.
- Runtime vision code reads calibration from the ReasonData calibration folder.

The `Calibration_*` datasets predate the DM2325 integration and remain here.
The duplicate DM2325 `imu_web` was removed; hardware control and IMU telemetry
are available through `web/control_panel/`. See
`docs/DM2325_ARCHITECTURE.md` for the repository boundary.
