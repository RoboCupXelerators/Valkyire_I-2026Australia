# DM-S2325 Driver Status

This is an interim engineering record. It is not final chassis acceptance.

## Authoritative Runtime

- Workspace: `~/Software`
- Driver: `src/utils/DM2325Bits.py`
- Bus: CAN1 only
- Runtime imports the driver through `utils.DM2325Bits`.
- `~/Software-refactor` is historical material and must not overwrite this driver.

The driver owns the DM protocol codec, CDC frame parser and transport, four-wheel
status model, chassis lifecycle commands, and C-board IMU compatibility facade.

## Wheel Mapping

The only internal wheel order is `FL, FR, RL, RR`.

| Wheel | ESC ID | Master ID |
| --- | ---: | ---: |
| FL | `0x01` | `0x11` |
| FR | `0x02` | `0x12` |
| RL | `0x03` | `0x13` |
| RR | `0x04` | `0x14` |

Physical forward signs and the final IMU-to-robot axis mapping remain deferred
until final assembly. Combined chassis motion must not use guessed signs.

## Safety Semantics

- `all_disabled` requires all four motors online, fresh, not enabled, and free
  of motor faults. Zero masks with missing feedback are not confirmation.
- Offline wheels are included in `stale_mask`.
- `stop()` and `disable()` are idempotent only after a confirmed safe snapshot.
- `shutdown()` attempts Disable even if Stop fails and always closes the port.
- emergency-stop confirmation requires four fresh Disabled motor states and the
  expected latched runtime fault.
- `GetHeadingCandidate()` may expose firmware attitude for diagnostics, but it
  always reports axis mapping and robot direction as unverified and is invalid
  for motion. Motion code must continue to use the stricter `GetYaw()` gate.

## Verification

On 2026-07-13, the complete Software suite passed after the current-tree C
firmware, C host tests, Python/C golden validation, entrypoint checks, and
feedback replay were included:

```text
343 passed in 6.44s
```

A subsequent read-only C-board snapshot reported:

```text
runtime_state=DISABLED (1)
runtime_fault=0
online_mask=0x0
enabled_mask=0x0
fault_mask=0x0
stale_mask=0xF
all_disabled=False
CAN_ESR=0x0 (latest read-only retry; an earlier motor-off snapshot was 0x280030)
FL/FR/RL/RR feedback_age_ms=65535
```

This snapshot does not pass the four-motor Disabled gate. No Enable, nonzero
target, or combined motion command was sent. Resume with a read-only Disabled
check after all four motor power stages are online.
