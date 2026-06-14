# Multi-Robot Scrimmage Debugging

Unity already has four `XelUdpBridge` endpoints in `SampleScene`:

- R1: command `20001`, status `20002`
- R2: command `20011`, status `20012`
- R3: command `20021`, status `20022`
- R4: command `20031`, status `20032`

Python maps these automatically from `XEL_ROBOT_ID` when `Settings.Sim.MultiRobotMode` is `per_port`.

## Run 1v1

Start Unity Play mode, then run from `Software`:

```powershell
python src\tests\sim_multi_robot_scrimmage.py --robots 1,3 --duration 15
```

This starts one subprocess per robot:

- R1 offence attacks `+X`
- R3 offence attacks `-X`

Outputs go to:

```text
Records/multi_robot_scrimmage/run_YYYYMMDD_HHMMSS/
```

The run writes per-robot JSONL traces, per-robot logs, `scene_status.jsonl`, `summary.json`, and `report.md`.

By default the runner starts with the ball near midfield but slightly on the ally side (`-12,0` cm), runs for up to 15 seconds, and fails if the Unity score is still 0-0. Once a goal is detected it records a short tail and ends the smoke run early; use `--continue-after-goal` for full-duration post-goal rule debugging, or `--allow-no-goal` for lower-level possession/motion debugging.

For a livelier iteration run, ask for multiple goals before early exit:

```powershell
python src\tests\sim_multi_robot_scrimmage.py --robots 1,3 --duration 30 --target-goals 3
```

For this debugging runner, `XEL_SIM_BALL_STATUS_FALLBACK=1` is enabled in each robot subprocess. If the normal UDP vision packet does not include a ball position, the Python side synthesizes `RobotBallPos` from scene-control ball and robot poses. This keeps scrimmage debugging focused on multi-robot control instead of Unity camera/occlusion quirks.

## Run 2v2

```powershell
python src\tests\sim_multi_robot_scrimmage.py --robots 1,2,3,4 --duration 12
```

With `--roles auto`, the default division is:

- R1/R3: `offence`
- R2/R4: `defence`

Override manually when needed:

```powershell
python src\tests\sim_multi_robot_scrimmage.py --robots 1,2,3,4 --roles offence,defence,offence,defence --duration 12
```

## Notes

The current strategy stack uses global `LunaPre` objects, so one process cannot safely own multiple robots. The scrimmage runner intentionally launches one process per robot, matching the real deployment shape.

The active offence strategy now supports attack direction:

```powershell
$env:OFFENCE_ATTACK_SIGN="1"   # attack +X
$env:OFFENCE_ATTACK_SIGN="-1"  # attack -X
```

If `OFFENCE_ATTACK_SIGN` is unset or `0`, SIM robots with id `3` or `4` automatically attack `-X`; ids `1` or `2` attack `+X`.
