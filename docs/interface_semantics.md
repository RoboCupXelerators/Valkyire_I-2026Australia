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
| `UnitedPosition.FieldRobotPosition()` | field-frame robot position `[x_forward, y_left]` |
| `UnitedPosition.FieldRobotPos()` | legacy alias for `FieldRobotPose()` |
| `UnitedPosition.RobotBallPos()` | robot-frame ball position `[x_forward, y_left]` or sentinel |
| `UnitedPosition.FieldBallPos()` | field-frame ball position `[x_forward, y_left]` or sentinel |
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

## IDLE / Agent Debug Entry

Use `idle_debug.py` when running from IDLE, a local terminal, or an agent test
with a live Unity simulator or real robot. This script has no offline runtime
backend.

```bash
ROBOCUPX_IDLE_BACKEND=SIM python idle_debug.py
ROBOCUPX_IDLE_BACKEND=REAL python idle_debug.py
```

`SIM` expects Unity Play mode with `XelUdpBridge` listening on the configured
UDP ports. `REAL` uses the current `ReasonData` robot configuration as-is.

The script configures the backend before importing `LunaPre`, then reads one
live debug frame through the same public interfaces:

```python
frame = run_step()
```

If Unity/real robot is not available, do not run IDLE runtime debugging. Use
syntax checks instead:

```bash
python -m py_compile src/LunaPre.py src/Sensor.py idle_debug.py
```
