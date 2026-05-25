# Simulator acceptance workflow

This workflow is for the Unity simulator plus the Python strategy stack.

## 1. Start Unity

Open `D:\Github\SoccerSimulatorForXels` in Unity and run `SampleScene`.
If scripts or `SampleScene.unity` were edited while Unity was already open,
stop Play mode, let Unity import/compile, then start Play mode again before
running the Python smoke script.

The scene should expose per-robot ports:

| robot | command | status |
|---:|---:|---:|
| 1 | 20001 | 20002 |
| 2 | 20011 | 20012 |
| 3 | 20021 | 20022 |
| 4 | 20031 | 20032 |

For the current single-robot acceptance path, use robot 1.

Scene-control UDP is available on port `20100` while Play mode is running:

```powershell
python -m src.utils.XeleratorSceneControl ping
python -m src.utils.XeleratorSceneControl set_robot --robot-id 1 --x -70 --y 0 --yaw 0
python -m src.utils.XeleratorSceneControl set_ball --x 55 --y 35
python -m src.utils.XeleratorSceneControl set_robot_tuning --robot-id 0 --linear-scale 0.26 --lateral-scale 0.167 --angular-scale 2.2
```

## 2. Run smoke acceptance

From `D:\Github\Software`:

```powershell
$env:PYTHONPATH="D:\Github\Software\src"
$env:ROBOCUPX_BITS="SIM"
.\venv312\Scripts\python.exe -m src.tests.sim_acceptance
```

Report output:

```text
Records/sim_acceptance/sim_acceptance_*.json
```

This checks simulator connection, short x/y/w motion, a small `Move2Path`
square, and dribble/kick IO.

## 3. Collect simulator motion response

```powershell
$env:PYTHONPATH="D:\Github\Software\src"
$env:ROBOCUPX_BITS="SIM"
.\venv312\Scripts\python.exe -m src.tests.sim_motion_sampling
```

Optional shorter run:

```powershell
$env:SIM_MOTION_AXES="x,w"
$env:SIM_MOTION_LEVELS="12,20"
$env:SIM_MOTION_HOLD_S="0.35"
.\venv312\Scripts\python.exe -m src.tests.sim_motion_sampling
```

CSV output:

```text
Records/sim_motion/sim_motion_sampling_*.csv
```

## 4. Compare real vs simulator

```powershell
.\venv312\Scripts\python.exe -m src.tests.compare_motion_response `
  --real Records\friction\friction_real_sampling_20260519_190045.csv `
  --sim Records\sim_motion\sim_motion_sampling_YYYYMMDD_HHMMSS.csv `
  --current-linear 0.26 `
  --current-lateral 0.167 `
  --current-angular 2.20 `
  --out Records\sim_motion\motion_compare_latest.json
```

The comparison reports median response by axis/level/sign and suggests next
speed-scale adjustments. It uses robust medians because the real CSV contains
occasional localization spikes.

## Current calibration baseline

Unity `SampleScene` currently uses:

- `linearSpeedScale = 0.26`
- `lateralSpeedScale = 0.167`
- `angularSpeedScale = 2.20`
- `linearCommandDeadzone = 0.064`
- `angularCommandDeadzone = 0.24`
- `linearAcceleration = 4.0`
- `linearBrakeAcceleration = 1.8`
- `angularAcceleration = 6.0`
- `angularBrakeAcceleration = 3.0`

Latest motion-response files:

- Final full x/y/w sample: `Records/sim_motion/sim_motion_sampling_20260522_001453.csv`
- Final full comparison: `Records/sim_motion/motion_compare_latest.json`
- y-only lateral check used while choosing `lateralSpeedScale = 0.167`: `Records/sim_motion/motion_compare_y_016.json`

Important: these values are calibrated for the normal strategy path:

```text
chassis.RobotMoveVector -> XeleratorSimBits.SetWheelSpeed -> Unity
```

Do not mix them with direct `SetTwist(rad/s)` calibration without adjusting the
Python bridge scale.

## 5. Tune chase-ball PID

The repeatable simulator sweep places the robot and ball through scene-control,
then runs `BasicFuc.LockBallSlip()` across fixed scenarios:

```powershell
.\venv\Scripts\python.exe scripts\run_sim_lockball_pid_sweep.py `
  --linear-scale 0.26 `
  --lateral-scale 0.167 `
  --angular-scale 2.2
```

Latest result:

```text
Records/sim_motion/lockball_pid_sweep_latest.json
```

Current selected profile is `fast`: 4/4 catches, mean catch time about `0.736s`
on the short simulator sweep. Its gains are written to `Motion.LockBallSlip*`
in `src/ReasonData/data/config.json`.
