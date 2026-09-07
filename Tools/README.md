# Tools

Legacy standalone camera/calibration utilities were removed from this folder.

Current calibration workflow:

- Use the control panel camera calibration page.
- Calibration files are saved under `src/ReasonData/data/Calibration/`.
- Runtime vision code reads calibration from the ReasonData calibration folder.

The `Calibration_*` datasets predate the DM2325 integration and remain here.
DM2325 hardware material and its runtime are maintained on the `legacy` branch.
