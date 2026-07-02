"""Offline checks for multi-robot scrimmage setup helpers."""

from __future__ import annotations

import json
import os
import sys

_original_bits_env = os.environ.get("ROBOCUPX_BITS")
if _original_bits_env is None:
    os.environ["ROBOCUPX_BITS"] = "SIM"

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from sim_multi_robot_scrimmage import (  # noqa: E402
    SIM_LINEAR_SCALE,
    SIM_WHEEL_MAX_RPM,
    SIM_WHEEL_TO_V_56MM_500RPM,
    _layout_for_round,
    _random_layout,
    _trace_summary,
)

if _original_bits_env is None:
    os.environ.pop("ROBOCUPX_BITS", None)
else:
    os.environ["ROBOCUPX_BITS"] = _original_bits_env


def test_random_layout_is_seeded_and_round_specific():
    first = _layout_for_round("random", [1, 3], (-12.0, 0.0), 20260701, 0)
    again = _layout_for_round("random", [1, 3], (-12.0, 0.0), 20260701, 0)
    next_round = _layout_for_round("random", [1, 3], (-12.0, 0.0), 20260701, 1)

    assert first == again
    assert first != next_round


def test_random_1v1_places_enemy_defender_near_ball_lane():
    layout = _random_layout([1, 3], (-12.0, 0.0), 11)

    r1 = layout[1]
    r3 = layout[3]

    assert r1[0] < -12.0
    assert r3[0] > -12.0
    assert abs(r1[1]) <= 58.0
    assert abs(r3[1]) <= 58.0
    assert -86.0 <= r1[0] <= 86.0
    assert -86.0 <= r3[0] <= 86.0
    assert not (abs(r1[0]) > 65.0 and abs(r1[1]) < 50.0)
    assert not (abs(r3[0]) > 65.0 and abs(r3[1]) < 50.0)
    assert r3[2] == 180.0


def test_sim_linear_scale_matches_56mm_500rpm_wheel_limit():
    target_mps = 3.141592653589793 * 0.056 * 500.0 / 60.0

    assert SIM_LINEAR_SCALE == 1.0
    assert abs(SIM_WHEEL_MAX_RPM * SIM_WHEEL_TO_V_56MM_500RPM - target_mps) < 0.01


def test_trace_summary_reports_total_path_not_displacement(tmp_path):
    trace = tmp_path / "trace.jsonl"
    rows = [
        {"pose": [0.0, 0.0, 0.0], "state": "start"},
        {"pose": [3.0, 4.0, 0.0], "state": "move"},
        {"pose": [0.0, 0.0, 0.0], "state": "return"},
    ]
    trace.write_text("\n".join(json.dumps(row) for row in rows) + "\n", encoding="utf-8")

    summary = _trace_summary(trace)

    assert summary["moved_cm"] == 10.0
    assert summary["displacement_cm"] == 0.0
