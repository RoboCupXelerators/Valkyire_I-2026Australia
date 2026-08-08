"""Port contract shared by the Unity simulator and Python control clients."""

from __future__ import annotations

import os


SIM_DEFAULT_SCENE_CONTROL_PORT = 20100
SIM_INSTANCE_PORT_BLOCK_SIZE = 100
SIM_MAX_PORT_OFFSET = 40000


def instance_id() -> int:
    value = os.environ.get("XEL_INSTANCE_ID", "0")
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return 0
    return max(0, parsed)


def port_offset() -> int:
    explicit = os.environ.get("XEL_PORT_OFFSET")
    if explicit is None:
        offset = instance_id() * SIM_INSTANCE_PORT_BLOCK_SIZE
    else:
        try:
            offset = int(explicit)
        except ValueError:
            offset = 0

    if offset < 0 or offset > SIM_MAX_PORT_OFFSET:
        return 0
    return offset


def resolve_port(port: int) -> int:
    """Apply the instance offset to a legacy/base port."""

    base = int(port)
    if base <= 0:
        return base
    resolved = base + port_offset()
    return resolved if resolved <= 65535 else base
