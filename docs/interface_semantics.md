# Interface Semantics

This document is the source of truth for coordinate, pose, sentinel, and live
debug semantics used by strategy code, agent-generated code, and future MLP
interfaces.

## Pose vs Position

Use the names consistently:

| Name | Meaning | Shape |
| --- | --- | --- |
| `Pose` | position plus heading | `[x, y, yaw]` |
| `Position` | coordinates only | `[x, y]` |
| `Robot*` | robot-frame data | relative to robot center |
| `Field*` | field-frame data | relative to field center |
| `Direct*` | raw sensor direction/order | sensor-specific order |

Current public interfaces:

| Interface | Semantics |
| --- | --- |
| `UnitedPosition.FieldRobotPose()` | field-frame robot pose `[x_forward, y_left, yaw_deg]` |
| `UnitedPosition.RobotBallPos()` | robot-frame ball position `[x_forward, y_left]` or sentinel |
| `UnitedPosition.FieldBallPos()` | field-frame ball position `[x_forward, y_left]` or sentinel |
| `UnitedPosition.PredictBallLanding()` | short-horizon field-frame interception prediction dict |
| `UnitedPosition.DirectDistance()` | raw wall distances from lidar |

## Ball State

Legacy ball sentinel values are still supported:

| Value | Semantic state |
| --- | --- |
| `[0xFFF, 0xFFF]` | `lost` |
| `[0xDDD, 0xDDD]` | `held` |
| any other `[x, y]` | `free` |

Use `UnitedPosition.BallState(ball_pos=None)` when branching on ball state.
This avoids scattering raw sentinel comparisons through strategy and MLP code.

## Ball Prediction

`UnitedPosition.PredictBallLanding(horizon_s=0.45)` estimates a short-term
field-frame interception point from recent `FieldBallPos()` samples. The result
is a dict with `valid`, `field_pos`, `current_field_pos`, `velocity_cm_s`,
`speed_cm_s`, `reason`, and confidence fields. Use `field_pos` only when
`valid` is true; otherwise fall back to the current ball position or home
defence. When the caller already has the current field ball observation, pass
it as `field_ball_pos` to avoid a second sensor read in the same control tick.

## Snapshot Discipline

Do not put every derived field into a shared snapshot by default. Keep the core
observation small and let strategy/MLP encoders derive extra fields when needed.

Recommended names for future MLP features:

```python
robot_pose       # [x, y, yaw]
robot_position   # [x, y]
ball_robot_pos   # [x, y] in robot frame, or sentinel
ball_field_pos   # [x, y] in field frame, only if explicitly needed
ball_state       # "lost" / "held" / "free"
```

## Basic Checks

If Unity/real robot is not available, prefer syntax checks and pytest instead
of starting runtime objects:

```bash
python -B -m py_compile src/LunaPre/positions.py src/LunaPre/runtime.py src/LunaPre/sensor.py
../venv/bin/python3 -m pytest
```
