import math

def angle_from_xy(x: float, y: float) -> float:
    """Return angle in degrees [0,360) using ROS2 convention: atan2(y, x), CCW positive.

    x: forward, y: left
    """
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def camera_to_robot(bx: float, by: float, cam_index: int) -> tuple[float, float]:
    """Map camera-local (bx,by) to robot-frame (x_forward, y_left).

    This preserves the mapping logic used in the original Vision.py but returns
    a clearly-documented (x_forward, y_left) pair.

    Note: bx,by are expected to be the intermediate X,Y derived from Pixel2CM.
    The mapping below mirrors the previous per-camera remapping and then applies
    the final rotation that the original code used (final mapping: x = -BY, y = BX).

    If you later change camera mounting/orientation, update this function.
    """
    if cam_index == 0:
        BX = bx
        BY = by
    elif cam_index == 1:
        # previously: BY = -X; BX = Y
        BX = by
        BY = -bx
    elif cam_index == 2:
        BX = -bx
        BY = -by
    elif cam_index == 3:
        BX = -by
        BY = bx
    else:
        BX = bx
        BY = by

    # original final mapping in Vision.py was: BallPos = [-BY, BX]
    x_forward = -BY
    y_left = BX
    return x_forward, y_left
