"""kits (compat layer)

历史策略代码（src/Stragety/*）大量依赖 `from kits import *`。
但当前仓库缺失 kits.py，导致策略层无法导入。

本文件提供一个“兼容层”，尽量把旧接口映射到新的 LunaPre / BasicFuc。

注意：
- 本文件以兼容为优先，不代表最佳实践。
- 新代码请直接使用 `from LunaPre import *` 或显式导入所需对象。
"""

from __future__ import annotations

import math
import time

from ReasonData import logger, Settings

# === Runtime objects ===
from LunaPre import UnitedPosition, chassis, peripheral, compass


# ===== Sentinel values (legacy) =====
NO_BALL = [0xFFF, 0xFFF]
CATCH_BALL = [0xDDD, 0xDDD]


# ===== Position / Vision wrappers =====
def GetBallPos():
    """Legacy: return robot-frame ball position.

    Old code sometimes uses [1024,1024] as no-ball; we keep LunaPre sentinel.
    """

    return UnitedPosition.RobotBallPos()


def AbsBallPos():
    """Legacy: absolute(field) ball position."""

    return UnitedPosition.FieldBallPos()


def GetPos():
    """Legacy: return robot field position [x,y,yaw]."""

    return UnitedPosition.FieldRobotPos()


def GetEnemyPos():
    """Legacy placeholder.

    Old strategy expects a single enemy position [x,y] from vision.
    New system returns a list; here we return the nearest/first if any.
    """

    lst = UnitedPosition.FieldChassisPos()
    if not lst:
        return [0xFFF, 0xFFF]
    # each element is [x,y,w,h] currently; take first two
    return [lst[0][0], lst[0][1]]


def AbsChassisAngle():
    return UnitedPosition.FieldChassisAngle()


def AbsBallAngle():
    return UnitedPosition.FieldBallAngle()


def RelBallAngle():
    return UnitedPosition.RobotBallAngle()


def AbsBallDistance():
    return UnitedPosition.RobotBallDistance()


# ===== Movement wrappers =====
def Pos2Pos(pos, *args, **kwargs):
    """Legacy signature shim.

    Old code: Pos2Pos([x,y,yaw], False, 200)
    New code: UnitedPosition.Pos2Pos([x,y,yaw], speed_gain=None)

    We accept variable args and try to map the last numeric argument to speed_gain.
    """

    speed_gain = None
    # heuristics: last positional arg numeric
    if args:
        for a in reversed(args):
            if isinstance(a, (int, float)):
                speed_gain = a
                break
    if "speed_gain" in kwargs:
        speed_gain = kwargs["speed_gain"]
    return UnitedPosition.Pos2Pos(pos, speed_gain=speed_gain)


# ===== Communication legacy globals =====
# Many old strategies treat these as globals.
PeerPosition = [0xFFF, 0xFFF]
BallFlag = [0, 0]


def Peerstatus():
    """Legacy stub.

    Old code expects Peerstatus()[3] etc. We return a tuple structure that
    minimally satisfies common indexing patterns.
    """

    # (unused0, unused1, unused2, peer_pos)
    return (None, None, None, [PeerPosition])


# ===== Peripheral legacy convenience =====
def LockBallSlip():
    # historically provided by BasicFuc; keep here for strategy
    try:
        from BasicFuc import lockball

        return lockball.LockBallSlip()
    except Exception:
        return False


__all__ = [
    "math",
    "time",
    "logger",
    "Settings",
    "UnitedPosition",
    "chassis",
    "peripheral",
    "compass",
    # sentinels
    "NO_BALL",
    "CATCH_BALL",
    # wrappers
    "GetBallPos",
    "AbsBallPos",
    "GetPos",
    "GetEnemyPos",
    "AbsChassisAngle",
    "AbsBallAngle",
    "RelBallAngle",
    "AbsBallDistance",
    "Pos2Pos",
    # legacy globals
    "PeerPosition",
    "BallFlag",
    "Peerstatus",
    "LockBallSlip",
]
