# Teacher Debug Loop

This folder adds a repeatable loop for debugging the rule teacher before using it as MLP training data.

## Offline Checks

Run from `Software`:

```powershell
python -m pytest src\tests\test_teacher_debug.py -q
```

This validates:

- boundary recovery targets are legal
- visible ball states choose `APPROACH_BALL`
- possession states avoid the goal box before carrying forward
- teacher traces are JSONL replayable
- badcase detection catches illegal targets and state oscillation
- `AVOID_WALL` can command a real recovery target through `SkillExecutor`

## Unity Regression

Start Unity Play mode first, then run:

```powershell
python src\tests\sim_teacher_regression.py
```

Outputs go to:

```text
Records/teacher_debug/run_YYYYMMDD_HHMMSS/
```

Each run writes per-scenario JSONL traces, `summary.json`, and `report.md`.

Useful options:

```powershell
python src\tests\sim_teacher_regression.py --max-scenarios 1
python src\tests\sim_teacher_regression.py --duration-scale 0.5
python src\tests\sim_teacher_regression.py --scenario-file path\to\scenarios.json
```

`has_ball` scenarios are skipped by default because scene control cannot yet force possession. Enable them only after adding a possession hook:

```powershell
$env:SIM_TEACHER_INCLUDE_POSSESSION="1"
python src\tests\sim_teacher_regression.py
```

## Real-Robot Handoff Prompt

Use this prompt for the next AI when running on hardware:

```text
You are continuing the RoboCupX teacher-debug loop on the real robot.
Do not change strategy logic first. Run the offline teacher tests, then run one short real-robot teacher trace with the robot raised or in a safe open area.
Compare the JSONL trace against the latest Unity run in Records/teacher_debug.
Look for: illegal targets, state oscillation, stuck movement, search while ball is visible, and executor commands that do not move the robot as expected.
If real motion diverges, tune only the interface/calibration layer first: ball observation, chassis scaling, wall/boundary distances, and SkillExecutor mapping.
Record every run under Records/teacher_debug_real with a short report.md containing the scenario, starting pose, observed issue, and the exact file changed.
```
