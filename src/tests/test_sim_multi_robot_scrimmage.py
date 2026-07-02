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
    _inactive_parking_pose,
    _layout_for_round,
    _place_robots,
    _random_layout,
    _lock_active_robots,
    _scene_summary,
    _trace_summary,
    _uses_controlled_goal_reset,
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


def test_scene_summary_ignores_stale_rule_event_before_reset(tmp_path):
    trace = tmp_path / "scene_status.jsonl"
    rows = [
        {
            "status": {
                "time": 1.0,
                "score": {"ally": 0, "enemy": 0, "last_event": "lack of progress", "last_time": -999},
                "ball": {"x_cm": 0.0, "y_cm": 0.0, "speed_cm_s": 0.0},
                "robots": [],
            }
        },
        {
            "status": {
                "time": 2.0,
                "score": {"ally": 1, "enemy": 0, "last_event": "Ally goal -> Enemy kickoff", "last_time": 2.0},
                "ball": {"x_cm": 110.0, "y_cm": 0.0, "speed_cm_s": 0.0},
                "robots": [],
            }
        },
    ]
    trace.write_text("\n".join(json.dumps(row) for row in rows) + "\n", encoding="utf-8")

    summary = _scene_summary(trace)

    assert summary["event_counts"] == {"Ally goal -> Enemy kickoff": 1}
    assert summary["first_rule_event"] is None


def test_scene_summary_ignores_physically_impossible_pushing_event(tmp_path):
    trace = tmp_path / "scene_status.jsonl"
    rows = [
        {
            "status": {
                "time": 5.0,
                "score": {"ally": 0, "enemy": 0, "last_event": "pushing R1/R3", "last_time": 5.0},
                "ball": {"x_cm": 0.0, "y_cm": 0.0, "speed_cm_s": 0.0},
                "robots": [
                    {"id": 1, "x_cm": 0.0, "y_cm": 0.0, "has_ball": False},
                    {"id": 3, "x_cm": 90.0, "y_cm": 0.0, "has_ball": False},
                ],
            }
        }
    ]
    trace.write_text("\n".join(json.dumps(row) for row in rows) + "\n", encoding="utf-8")

    summary = _scene_summary(trace)

    assert summary["event_counts"] == {}
    assert summary["first_rule_event"] is None


def test_scene_summary_keeps_supported_pushing_event(tmp_path):
    trace = tmp_path / "scene_status.jsonl"
    rows = [
        {
            "status": {
                "time": 5.0,
                "score": {"ally": 0, "enemy": 0, "last_event": "pushing R1/R3", "last_time": 5.0},
                "ball": {"x_cm": 0.0, "y_cm": 0.0, "speed_cm_s": 0.0},
                "robots": [
                    {"id": 1, "x_cm": 0.0, "y_cm": 0.0, "has_ball": False},
                    {"id": 3, "x_cm": 24.0, "y_cm": 0.0, "has_ball": False},
                ],
            }
        }
    ]
    trace.write_text("\n".join(json.dumps(row) for row in rows) + "\n", encoding="utf-8")

    summary = _scene_summary(trace)

    assert summary["event_counts"] == {"pushing R1/R3": 1}
    assert summary["first_rule_event"]["event"] == "pushing R1/R3"


class _FakeSceneControl:
    def __init__(self):
        self.robot_calls = []
        self.dribble_calls = []
        self.reset_count = 0
        self.ball_calls = []

    def reset_score(self):
        self.reset_count += 1

    def set_dribble(self, robot_id, active, strength=0.0):
        self.dribble_calls.append((robot_id, active, strength))

    def set_robot(self, robot_id, x, y, yaw, *, active=True):
        self.robot_calls.append((robot_id, x, y, yaw, active))

    def set_ball(self, x, y, *, reset_velocity=True):
        self.ball_calls.append((x, y, reset_velocity))


def test_place_robots_parks_inactive_robots_away_from_match_layout():
    ctl = _FakeSceneControl()

    _place_robots(ctl, [1, 3], (-12.0, 0.0), _layout_for_round("default", [1, 3], (-12.0, 0.0), 1, 0))

    calls = {robot_id: call for robot_id, *call in ctl.robot_calls}
    assert calls[1][-1] is True
    assert calls[3][-1] is True
    assert calls[2] == [*_inactive_parking_pose(2), False]
    assert calls[4] == [*_inactive_parking_pose(4), False]


def test_lock_active_robots_moves_unexpected_active_robot_to_parking_pose():
    ctl = _FakeSceneControl()
    status = {
        "robots": [
            {"id": 2, "active": True, "x_cm": -70.0, "y_cm": 60.0, "yaw_deg": 0.0},
            {"id": 1, "active": True, "x_cm": -60.0, "y_cm": -30.0, "yaw_deg": 0.0},
            {"id": 3, "active": False, "x_cm": 60.0, "y_cm": 30.0, "yaw_deg": 180.0},
        ]
    }

    correction = _lock_active_robots(ctl, status, [1, 3], set())

    assert correction == {"unexpected_active": [2], "missing_active": [3]}
    parking = _inactive_parking_pose(2)
    assert (2, *parking, False) in ctl.robot_calls


def test_allow_no_goal_runs_still_use_controlled_goal_reset():
    class Args:
        continue_after_goal = False
        allow_no_goal = True

    assert _uses_controlled_goal_reset(Args())
