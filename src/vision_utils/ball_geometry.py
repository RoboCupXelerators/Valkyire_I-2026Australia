"""Near-field ball geometry helpers for occlusion-aware contact points.

When the dribbler clips the lower part of the ball, the detector bbox becomes a
wide rectangle instead of a near-square. Using the clipped bottom edge for
Pixel2CM systematically biases the robot-frame ball coordinate by ~1cm near the
catch window.

This module reconstructs a square contact point from the still-visible width
before projection.
"""

from __future__ import annotations

from typing import Any


def reconstruct_ball_contact_point(
    center_x: float,
    bottom_y: float,
    width: float,
    height: float,
    *,
    wide_aspect_thresh: float = 1.15,
    tall_aspect_thresh: float = 1.15,
    max_rebuild_aspect: float = 2.6,
    min_size_px: float = 10.0,
    near_min_size_px: float = 18.0,
) -> tuple[float, float, dict[str, Any]]:
    """Return a ground-contact pixel and debug metadata.

    Args:
        center_x: bbox horizontal center in the same pixel space as bottom_y.
        bottom_y: bbox bottom edge (y2).
        width/height: bbox size in pixels.

    Returns:
        (contact_x, contact_y, meta)
        meta includes occlusion kind and whether reconstruction was applied.
    """
    w = float(max(0.0, width))
    h = float(max(0.0, height))
    cx = float(center_x)
    y2 = float(bottom_y)
    y1 = y2 - h
    meta: dict[str, Any] = {
        "occlusion": "none",
        "reconstructed": False,
        "aspect_wh": 0.0,
        "size_px": 0.0,
        "raw_bottom_y": y2,
        "corr_bottom_y": y2,
    }

    if w < 1.0 or h < 1.0:
        return cx, y2, meta

    aspect_wh = w / max(h, 1e-6)
    meta["aspect_wh"] = round(aspect_wh, 4)
    meta["size_px"] = round(max(w, h), 2)

    # Bottom clipped by dribbler/structure: wide rectangle. Prefer width.
    if (
        aspect_wh >= wide_aspect_thresh
        and aspect_wh <= max_rebuild_aspect
        and w >= min_size_px
        and (w >= near_min_size_px or aspect_wh >= 1.25)
    ):
        size = w
        corr_bottom = y1 + size
        meta.update(
            {
                "occlusion": "bottom",
                "reconstructed": True,
                "size_px": round(size, 2),
                "corr_bottom_y": round(corr_bottom, 2),
            }
        )
        return cx, float(corr_bottom), meta

    # Side clipped: tall rectangle. Keep bottom, recentre with height if needed.
    if (
        (h / max(w, 1e-6)) >= tall_aspect_thresh
        and (h / max(w, 1e-6)) <= max_rebuild_aspect
        and h >= min_size_px
        and (h >= near_min_size_px or (h / max(w, 1e-6)) >= 1.25)
    ):
        size = h
        # Keep measured bottom (ground contact more trustworthy than sides).
        meta.update(
            {
                "occlusion": "side",
                "reconstructed": False,
                "size_px": round(size, 2),
                "corr_bottom_y": round(y2, 2),
            }
        )
        return cx, y2, meta

    return cx, y2, meta


def contact_point_from_bbox_xyxy(
    x1: float,
    y1: float,
    x2: float,
    y2: float,
    **kwargs,
) -> tuple[float, float, dict[str, Any]]:
    """Convenience wrapper for xyxy boxes."""
    w = float(x2) - float(x1)
    h = float(y2) - float(y1)
    cx = (float(x1) + float(x2)) * 0.5
    return reconstruct_ball_contact_point(cx, float(y2), w, h, **kwargs)
