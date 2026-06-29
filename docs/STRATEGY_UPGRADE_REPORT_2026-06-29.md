# Strategy Upgrade Report - 2026-06-29

Branch: `refactor/software-architecture-layers`

## Scope

- Added a small shared strategy library at `src/LunaPre/simple_strategy.py`.
- Kept the role entrypoints simple: `offence.py` and `defence.py` call the strategy helpers, then use existing chassis/peripheral commands.
- Did not add environment variables or new complex data structures.
- Preserved the existing fast attack path unless the new if/else plan has a clear reason to take over.

## Offence Strategies

- `direct_shot`: marks clean open-goal shots in trace as direct shots while preserving the existing `quick_shot` runtime state return.
- `wall_setup`: when the front lane is crowded before the deeper shooting zone, drive toward a side-wall rebound lane instead of waiting.
- `wall_shot`: when a close blocker makes the direct lane poor, aim through the wall lane and shoot if yaw is acceptable.
- `sideline_drive`: from a wide lane before the shot window, continue down the side instead of falling into the old wide recovery path.
- `sideline_shot`: from a wide deep lane, shoot at the near-post side target if yaw is acceptable.

## Defence Coverage

- `cover_direct_shot`: guard the center shot lane when the ball is central and not close enough to chase.
- `cover_wall_shot`: shift to the likely rebound side for mid-wide ball positions.
- `cover_sideline_shot`: move toward the wide lane when the attacker is likely to shoot from the side.

The original priority order is preserved: boundary recovery, held-ball clear, danger-ball challenge, close-ball intercept, then shot-family coverage.

## Verification Run

Passed:

```powershell
venv312\Scripts\python.exe -m pytest src\tests\test_offence_boundary.py src\tests\test_defence_strategy.py -q
```

Result: `40 passed in 0.28s`

Unity scene-control smoke was attempted:

```powershell
venv312\Scripts\python.exe -m src.utils.XeleratorSceneControl ping
```

Current result: failed with `ConnectionResetError: [WinError 10054]`, so the Unity scene-control UDP endpoint is not accepting requests from the runner at the moment. The Unity validation commands below are ready to rerun once `ping` returns `ok: true`.

Local diagnostics at the time of this report:

- `Get-NetUDPEndpoint -LocalPort 20100` returned no listener.
- `Test-NetConnection 127.0.0.1 -Port 8090` failed.
- `Test-NetConnection ::1 -Port 8090` succeeded, with Unity listening on IPv6 loopback.
- Unity MCP tool calls timed out, so MCP was reachable at TCP level but not responding to tool requests.

## Reproduction Commands

### Per-strategy commands

`direct_shot`:

```powershell
venv312\Scripts\python.exe -m pytest src\tests\test_offence_boundary.py::test_offence_shoots_immediately_in_quick_shot_zone -q
```

`wall_setup`:

```powershell
venv312\Scripts\python.exe -m pytest src\tests\test_offence_boundary.py::test_offence_sets_up_wall_shot_before_deep_zone -q
```

`wall_shot`:

```powershell
venv312\Scripts\python.exe -m pytest src\tests\test_offence_boundary.py::test_offence_wall_shoots_when_direct_lane_is_blocked -q
```

`sideline_drive`:

```powershell
venv312\Scripts\python.exe -m pytest src\tests\test_offence_boundary.py::test_offence_sideline_drives_before_shot_window -q
```

`sideline_shot`:

```powershell
venv312\Scripts\python.exe -m pytest src\tests\test_offence_boundary.py::test_offence_sideline_shoots_from_wide_deep_lane -q
```

`cover_direct_shot`:

```powershell
venv312\Scripts\python.exe -m pytest src\tests\test_defence_strategy.py::test_defence_covers_direct_shot_when_ball_is_not_chased -q
```

`cover_wall_shot`:

```powershell
venv312\Scripts\python.exe -m pytest src\tests\test_defence_strategy.py::test_defence_covers_wall_shot_when_ball_is_mid_wide -q
```

`cover_sideline_shot`:

```powershell
venv312\Scripts\python.exe -m pytest src\tests\test_defence_strategy.py::test_defence_covers_sideline_shot_when_ball_is_wide -q
```

### 1v0 - direct shot / sideline basics

```powershell
venv312\Scripts\python.exe -m pytest src\tests\test_offence_boundary.py::test_attack_plan_direct_shot_for_1v0_open_lane src\tests\test_offence_boundary.py::test_offence_shoots_immediately_in_quick_shot_zone src\tests\test_offence_boundary.py::test_attack_plan_sideline_drive_from_wide_lane -q
```

### 1v1 - blocker response / wall and sideline

```powershell
venv312\Scripts\python.exe -m pytest src\tests\test_offence_boundary.py::test_offence_wall_shoots_when_direct_lane_is_blocked src\tests\test_offence_boundary.py::test_offence_sets_up_wall_shot_before_deep_zone src\tests\test_offence_boundary.py::test_offence_sideline_drives_before_shot_window src\tests\test_defence_strategy.py::test_defence_covers_direct_shot_when_ball_is_not_chased -q
```

### 2v1 - crowded front lane

```powershell
venv312\Scripts\python.exe -m pytest src\tests\test_offence_boundary.py::test_attack_plan_2v1_prefers_wall_shot_over_waiting src\tests\test_defence_strategy.py::test_defence_cover_plan_handles_direct_wall_and_sideline -q
```

### 2v2 - crowded front lane with extra side robots

```powershell
venv312\Scripts\python.exe -m pytest src\tests\test_offence_boundary.py::test_attack_plan_2v2_still_selects_a_simple_exit_lane src\tests\test_defence_strategy.py::test_defence_covers_sideline_shot_when_ball_is_wide -q
```

### Full related offline suite

```powershell
venv312\Scripts\python.exe -m pytest src\tests\test_offence_boundary.py src\tests\test_defence_strategy.py -q
```

### Unity smoke commands after scene-control is healthy

```powershell
venv312\Scripts\python.exe -m src.utils.XeleratorSceneControl ping
venv312\Scripts\python.exe src\tests\sim_multi_robot_scrimmage.py --robots 1 --roles offence --duration 8 --allow-no-possession --allow-no-goal --output tmp_strategy_1v0.json
venv312\Scripts\python.exe src\tests\sim_multi_robot_scrimmage.py --robots 1,3 --roles offence,defence --duration 12 --allow-no-goal --output tmp_strategy_1v1.json
venv312\Scripts\python.exe src\tests\sim_multi_robot_scrimmage.py --robots 1,2,3 --roles offence,defence,defence --duration 12 --allow-no-goal --output tmp_strategy_2v1.json
venv312\Scripts\python.exe src\tests\sim_multi_robot_scrimmage.py --robots 1,2,3,4 --roles offence,defence,offence,defence --duration 12 --allow-no-goal --output tmp_strategy_2v2.json
```
