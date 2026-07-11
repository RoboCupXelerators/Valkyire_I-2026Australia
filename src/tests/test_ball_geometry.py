from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from vision_utils.ball_geometry import (  # noqa: E402
    contact_point_from_bbox_xyxy,
    reconstruct_ball_contact_point,
)


def test_complete_square_bbox_keeps_bottom_edge():
    cx, y2, meta = reconstruct_ball_contact_point(100, 200, 40, 40)
    assert cx == 100
    assert y2 == 200
    assert meta["reconstructed"] is False
    assert meta["occlusion"] == "none"


def test_bottom_occluded_wide_bbox_rebuilds_square_bottom():
    # Visible top remains, height clipped: 40x24 wide rectangle.
    # True square bottom should be y1 + width = 160 + 40 = 200.
    cx, y2, meta = reconstruct_ball_contact_point(100, 184, 40, 24)
    assert cx == 100
    assert y2 == 200
    assert meta["reconstructed"] is True
    assert meta["occlusion"] == "bottom"
    assert meta["raw_bottom_y"] == 184
    assert meta["corr_bottom_y"] == 200


def test_side_occluded_tall_bbox_keeps_bottom():
    cx, y2, meta = reconstruct_ball_contact_point(100, 200, 24, 40)
    assert cx == 100
    assert y2 == 200
    assert meta["occlusion"] == "side"
    assert meta["reconstructed"] is False


def test_xyxy_wrapper_matches_manual_call():
    a = reconstruct_ball_contact_point(120, 210, 50, 30)
    b = contact_point_from_bbox_xyxy(95, 180, 145, 210)
    assert a[0] == b[0]
    assert a[1] == b[1]
    assert a[2]["occlusion"] == b[2]["occlusion"]
