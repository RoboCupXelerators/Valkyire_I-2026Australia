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

Unity scene-control smoke was blocked on 2026-06-29, then re-run successfully on 2026-07-01 after scene-control/MCP recovered:

```powershell
venv312\Scripts\python.exe -m src.utils.XeleratorSceneControl ping
```

Current result: `ok: true`.

Earlier local diagnostics while blocked:

- `Get-NetUDPEndpoint -LocalPort 20100` returned no listener.
- `Test-NetConnection 127.0.0.1 -Port 8090` failed.
- `Test-NetConnection ::1 -Port 8090` succeeded, with Unity listening on IPv6 loopback.
- Unity MCP tool calls timed out, so MCP was reachable at TCP level but not responding to tool requests.

Unity smoke results after recovery:

- 1v0: `ok=True`, output `tmp_strategy_1v0.json\run_20260701_123011`, ally 2 - enemy 0, offence states include `quick_shot`, `attack_dodge`, `feint_drive`.
- 1v1: `ok=True`, output `tmp_strategy_1v1.json\run_20260701_123041`, ally 1 - enemy 0, defence states include `cover_direct_shot`, `intercept_lane`, `cover_sideline_shot`.
- 2v1: `ok=True`, output `tmp_strategy_2v1.json\run_20260701_123110`, ally 1 - enemy 0, defence states include `cover_direct_shot`, `cover_wall_shot`, `cover_sideline_shot`, `intercept_lane`, `challenge_ball`, `clear_ball`.
- 2v2: `ok=True`, output `tmp_strategy_2v2.json\run_20260701_123139`, ally 0 - enemy 0, all robot processes exited 0, defence states include `cover_direct_shot`, `cover_wall_shot`, `cover_sideline_shot`, `intercept_lane`.

## 2026-07-01 Control Follow-up

After video review, three control issues were reproduced:

- Wall shots were too eager when the robot was not deep enough or the lane was only mildly blocked.
- Own-half possession could enter shot/carry logic too early.
- In 1v1, the simulator adapter sometimes converted the dribbler recovery window into the `[0xDDD, 0xDDD]` possession sentinel even when Unity scene status still reported `has_ball=False`.

Fixes:

- Tightened `choose_attack_plan()` shot gates so wall shots require a deeper field position and a clearly blocked direct lane.
- Added `own_half_carry`, `near_ball_capture`, `near_ball_align`, and `own_half_wide_clear` branches in `offence.py` to reduce close-ball jitter and prevent own-half random shots.
- Tightened simulator possession inference in `XeleratorSimBits`: the adapter now only emits the possession sentinel when the ball is near the actual dribbler center, not merely inside the wider recovery window.
- Added regression coverage for gentle close-ball capture, sideways alignment, own-half wide clear, and simulator possession semantics.

Final 1v1 one-minute replay:

```powershell
venv312\Scripts\python.exe src\tests\sim_multi_robot_scrimmage.py --robots 1,3 --roles offence,defence --duration 60 --continue-after-goal --allow-no-goal --output tmp_strategy_1v1_controlfix7_replay
```

Result: `ok=True`, output `tmp_strategy_1v1_controlfix7_replay\run_20260701_151902`.

- Score: ally 15 - enemy 0.
- Out-of-bounds / penalty events: 0.
- Lack-of-progress events: 0.
- Multi-possession frames: 0.
- Max holders: 1.
- Holder frames: 90 / 237 scene frames.
- R1 total path length from trace: 2720.069 cm.
- R1 states: `near_ball_acquire`, `chase_ball`, `own_half_carry`, `attack_dodge`, `quick_shot`, `turn_to_ball`.
- R1 shot actions: `direct_shot` only in this run; no `wall_shot` or `sideline_shot` was selected under this 1v1 scene.

Interpretation: the 1v1 control failure is fixed under the current replay command, but the 15-0 score suggests the test scene is still too easy for measuring realistic battle intensity. Use a stricter defender start or longer mixed 2v2 replay before judging final balance.

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
venv312\Scripts\python.exe -m pytest src\tests\test_offence_boundary.py::test_offence_wall_shoots_when_deep_direct_lane_is_blocked -q
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
venv312\Scripts\python.exe -m pytest src\tests\test_offence_boundary.py::test_offence_wall_shoots_when_deep_direct_lane_is_blocked src\tests\test_offence_boundary.py::test_offence_sets_up_wall_shot_before_deep_zone src\tests\test_offence_boundary.py::test_offence_sideline_drives_before_shot_window src\tests\test_defence_strategy.py::test_defence_covers_direct_shot_when_ball_is_not_chased -q
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
venv312\Scripts\python.exe -m pytest src\tests\test_offence_boundary.py src\tests\test_defence_strategy.py src\tests\test_xelerator_sim_bits.py -q
```

### Unity smoke commands after scene-control is healthy

```powershell
venv312\Scripts\python.exe -m src.utils.XeleratorSceneControl ping
venv312\Scripts\python.exe src\tests\sim_multi_robot_scrimmage.py --robots 1 --roles offence --duration 8 --allow-no-possession --allow-no-goal --output tmp_strategy_1v0.json
venv312\Scripts\python.exe src\tests\sim_multi_robot_scrimmage.py --robots 1,3 --roles offence,defence --duration 12 --allow-no-goal --output tmp_strategy_1v1.json
venv312\Scripts\python.exe src\tests\sim_multi_robot_scrimmage.py --robots 1,2,3 --roles offence,defence,defence --duration 12 --allow-no-goal --output tmp_strategy_2v1.json
venv312\Scripts\python.exe src\tests\sim_multi_robot_scrimmage.py --robots 1,2,3,4 --roles offence,defence,offence,defence --duration 12 --allow-no-goal --output tmp_strategy_2v2.json
```
