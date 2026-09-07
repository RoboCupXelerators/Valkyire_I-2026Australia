"""Focused collision-avoidance checks for the coordinated-lanes candidate."""

from __future__ import annotations

import os
import sys
import json
from types import SimpleNamespace

import pytest

os.environ.setdefault("ROBOCUPX_BITS", "SIM")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import candidate_coordinated_lanes as candidate  # noqa: E402


@pytest.fixture(autouse=True)
def isolate_candidate_clock(monkeypatch):
    """Strategy time mocks must not alter live transport/vision threads."""
    monkeypatch.setattr(candidate, "time", SimpleNamespace(**vars(candidate.time)))


@pytest.fixture(autouse=True)
def reset_drop_retrieval_state():
    candidate._drop_approach_marker = None
    candidate._free_approach_marker = None
    candidate._held_since = None
    candidate._corner_release_pending = False
    yield
    candidate._drop_approach_marker = None
    candidate._free_approach_marker = None
    candidate._held_since = None
    candidate._corner_release_pending = False


def _free_ball_patch(monkeypatch, *, pose=(0.0, 0.0, 0.0), local_ball=(30.0, 0.0), field_ball=(30.0, 0.0), enemies=()):
    moves = []
    monkeypatch.setattr(candidate.UnitedPosition, "FieldRobotPose", lambda: list(pose))
    monkeypatch.setattr(candidate.UnitedPosition, "RestrictedArea", lambda: True)
    monkeypatch.setattr(candidate.UnitedPosition, "RobotBallPos", lambda: list(local_ball))
    monkeypatch.setattr(candidate.UnitedPosition, "FieldBallPos", lambda: list(field_ball))
    monkeypatch.setattr(candidate.UnitedPosition, "BallState", lambda _ball: "free")
    monkeypatch.setattr(candidate.UnitedPosition, "FieldChassisPos", lambda: [list(point) for point in enemies])
    monkeypatch.setattr(candidate.UnitedPosition, "ProjectSafePose", lambda _target: None)
    monkeypatch.setattr(candidate.chassis, "FieldMoveVector", lambda *args, **_kwargs: moves.append(args))
    monkeypatch.setattr(candidate.peripheral, "Dribble", lambda _value: None)
    return moves


def _held_ball_patch(monkeypatch, *, pose=(0.0, 0.0, 0.0)):
    dribble_calls = []
    shots = []
    moves = []
    monkeypatch.setattr(candidate.UnitedPosition, "FieldRobotPose", lambda: list(pose))
    monkeypatch.setattr(candidate.UnitedPosition, "RestrictedArea", lambda: True)
    monkeypatch.setattr(candidate.UnitedPosition, "RobotBallPos", lambda: candidate.BALL_HELD)
    monkeypatch.setattr(candidate.UnitedPosition, "BallState", lambda _ball: "held")
    monkeypatch.setattr(candidate.peripheral, "Dribble", lambda value: dribble_calls.append(value))
    monkeypatch.setattr(candidate.peripheral, "ShootBall", lambda: shots.append(True))
    monkeypatch.setattr(candidate.chassis, "FieldMoveVector", lambda *args, **_kwargs: moves.append(args))
    return dribble_calls, shots, moves


def test_physical_teams_receive_complementary_lanes_and_roles(monkeypatch):
    assert candidate.lane_side(1) == candidate.lane_side(2) == 1.0
    assert candidate.lane_side(3) == candidate.lane_side(4) == -1.0
    monkeypatch.setenv("XEL_ROBOT_ID", "3")
    assert candidate.effective_role("defender") == "attacker"
    monkeypatch.setenv("XEL_ROBOT_ID", "4")
    assert candidate.effective_role("attacker") == "defender"


def test_main_trace_is_throttled_and_emits_state_changes(monkeypatch, capsys):
    states = iter(("approach_shoulder", "approach_shoulder", "free_ball_enter"))
    timestamps = iter((10.0, 10.1, 10.2))
    sleeps = []
    monkeypatch.setenv("COORDINATED_LANES_TRACE", "1")
    monkeypatch.setenv("XEL_ROBOT_ID", "3")
    monkeypatch.setattr(candidate, "step", lambda _role: next(states))
    monkeypatch.setattr(candidate.time, "monotonic", lambda: next(timestamps))
    monkeypatch.setattr(candidate.UnitedPosition, "FieldRobotPose", lambda: [1.0, 2.0, 90.0])
    monkeypatch.setattr(candidate.UnitedPosition, "RobotBallPos", lambda: [3.0, 4.0])
    monkeypatch.setattr(candidate.UnitedPosition, "BallState", lambda _ball: "free")
    monkeypatch.setattr(candidate.UnitedPosition, "FieldBallPos", lambda: [5.0, 6.0])
    monkeypatch.setattr(candidate.helper, "attack_direction", lambda: -1.0)
    monkeypatch.setattr(candidate.helper, "stop", lambda: None)

    def stop_after_three_sleeps(_duration):
        sleeps.append(True)
        if len(sleeps) == 3:
            raise KeyboardInterrupt

    monkeypatch.setattr(candidate.time, "sleep", stop_after_three_sleeps)

    with pytest.raises(KeyboardInterrupt):
        candidate.main()

    lines = capsys.readouterr().out.splitlines()
    assert [json.loads(line) for line in lines] == [
        {
            "robot_id": 3,
            "role": "attacker",
            "state": "approach_shoulder",
            "field_robot_pose": [1.0, 2.0, 90.0],
            "robot_ball_pos": [3.0, 4.0],
            "field_ball_pos": [5.0, 6.0],
            "attack_direction": -1.0,
            "monotonic_timestamp": 10.0,
        },
        {
            "robot_id": 3,
            "role": "attacker",
            "state": "free_ball_enter",
            "field_robot_pose": [1.0, 2.0, 90.0],
            "robot_ball_pos": [3.0, 4.0],
            "field_ball_pos": [5.0, 6.0],
            "attack_direction": -1.0,
            "monotonic_timestamp": 10.2,
        },
    ]


@pytest.mark.parametrize(
    "role,robot,expected_state,expected_target",
    [
        ("attacker", "1", "search_lane", [-44.0, 36.0, 0.0]),
        ("defender", "2", "search_lane", [24.0, 36.0, 0.0]),
    ],
)
def test_lost_ball_patrol_never_reads_field_ball_and_uses_role_separated_lane(
    monkeypatch, role, robot, expected_state, expected_target
):
    targets = []
    monkeypatch.setattr(candidate.UnitedPosition, "FieldRobotPose", lambda: [0.0, 0.0, 0.0])
    monkeypatch.setattr(candidate.UnitedPosition, "RestrictedArea", lambda: True)
    monkeypatch.setattr(candidate.UnitedPosition, "RobotBallPos", lambda: candidate.BALL_NOT_FOUND)
    monkeypatch.setattr(candidate.UnitedPosition, "BallState", lambda _ball: "lost")
    monkeypatch.setattr(candidate.UnitedPosition, "FieldBallPos", lambda: pytest.fail("lost patrol read field ball"))
    monkeypatch.setattr(candidate.UnitedPosition, "ProjectSafePose", lambda target: targets.append(target) or target)
    monkeypatch.setattr(candidate.chassis, "FieldMoveVector", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(candidate.peripheral, "Dribble", lambda _value: None)
    monkeypatch.setattr(candidate.time, "monotonic", lambda: 0.0)
    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", "1")
    monkeypatch.setenv("XEL_ROBOT_ID", robot)

    assert candidate.step(role) == expected_state
    assert targets[-1] == expected_target


def test_lost_ball_patrol_assigns_coordinated_roles_different_stages():
    attacker = candidate.search_patrol_target(1.0, candidate.lane_side(1), 0.0, "attacker")
    defender = candidate.search_patrol_target(1.0, candidate.lane_side(2), 0.0, "defender")

    assert attacker == [-44.0, 36.0, 0.0]
    assert defender == [24.0, 36.0, 0.0]
    assert abs(attacker[0] - defender[0]) >= 2.0 * candidate.DROP_SHOULDER_REACHED_CM


def test_trace_does_not_read_field_ball_when_ball_is_held(monkeypatch, capsys):
    states = iter(("carry_lane",))
    timestamps = iter((10.0,))
    monkeypatch.setenv("COORDINATED_LANES_TRACE", "1")
    monkeypatch.setattr(candidate, "step", lambda _role: next(states))
    monkeypatch.setattr(candidate.time, "monotonic", lambda: next(timestamps))
    monkeypatch.setattr(candidate.UnitedPosition, "FieldRobotPose", lambda: [1.0, 2.0, 0.0])
    monkeypatch.setattr(candidate.UnitedPosition, "RobotBallPos", lambda: candidate.BALL_HELD)
    monkeypatch.setattr(candidate.UnitedPosition, "BallState", lambda _ball: "held")
    monkeypatch.setattr(candidate.UnitedPosition, "FieldBallPos", lambda: pytest.fail("trace read field ball while held"))
    monkeypatch.setattr(candidate.helper, "attack_direction", lambda: 1.0)
    monkeypatch.setattr(candidate.helper, "stop", lambda: None)
    monkeypatch.setattr(candidate.time, "sleep", lambda _duration: (_ for _ in ()).throw(KeyboardInterrupt))

    with pytest.raises(KeyboardInterrupt):
        candidate.main()

    assert json.loads(capsys.readouterr().out)["field_ball_pos"] == candidate.BALL_NOT_FOUND


def test_contested_ball_uses_shoulder_without_generic_lock(monkeypatch):
    moves = _free_ball_patch(monkeypatch, field_ball=(20.0, 0.0), local_ball=(30.0, 0.0), enemies=((24.0, 0.0),))
    monkeypatch.setenv("XEL_ROBOT_ID", "1")
    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", "1")
    monkeypatch.setattr(candidate.lockball, "LockBallSlip", lambda: pytest.fail("contested ball used generic lock"))

    assert candidate.step("attacker") == "contested_shoulder"
    assert moves[-1] == (0.0, 0.34, 0.0)


@pytest.mark.parametrize("field_ball", [
    (8.0, -40.0),
    (0.005, 40.0),
    (0.0, -40.0),
    (-0.005, 40.0),
    (-8.0, -40.0),
])
def test_centre_ownership_deadband_always_selects_r1(field_ball):
    assert candidate.ball_owner_direction(field_ball) == 1.0
    assert candidate.free_ball_owner(field_ball, 1.0, 1)
    assert not candidate.free_ball_owner(field_ball, -1.0, 3)


def test_ordinary_ball_has_one_owner_outside_centre_deadband():
    assert candidate.free_ball_owner((20.0, 0.0), 1.0, 1)
    assert not candidate.free_ball_owner((20.0, 0.0), -1.0, 3)
    assert candidate.free_ball_owner((-20.0, 0.0), -1.0, 3)
    assert not candidate.free_ball_owner((-20.0, 0.0), 1.0, 1)


@pytest.mark.parametrize("field_ball,direction,owner,nonowner", [
    ((8.01, 0.0), 1.0, 1, 3),
    ((-8.01, 0.0), -1.0, 3, 1),
])
def test_ownership_switches_symmetrically_just_beyond_centre_deadband(field_ball, direction, owner, nonowner):
    assert candidate.ball_owner_direction(field_ball) == direction
    assert candidate.free_ball_owner(field_ball, direction, owner)
    assert not candidate.free_ball_owner(field_ball, -direction, nonowner)


def test_sub_deadband_x_jitter_does_not_change_r1_free_ball_state(monkeypatch):
    field_ball = [[0.005, 10.0]]
    _free_ball_patch(monkeypatch, field_ball=field_ball[0])
    monkeypatch.setattr(candidate.UnitedPosition, "FieldBallPos", lambda: field_ball[0])
    monkeypatch.setenv("XEL_ROBOT_ID", "1")
    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", "1")

    states = []
    for ball_x in (0.005, -0.005, 0.004, -0.004):
        field_ball[0][0] = ball_x
        states.append(candidate.step("attacker"))

    assert states == ["approach_shoulder"] * 4


def test_ordinary_owner_progresses_shoulder_entry_then_lock(monkeypatch):
    calls = []
    pose = [[0.0, 0.0, 0.0]]
    local_ball = [[30.0, 0.0]]
    moves = _free_ball_patch(monkeypatch, pose=pose[0], local_ball=local_ball[0], field_ball=(30.0, 0.0))
    monkeypatch.setattr(candidate.UnitedPosition, "FieldRobotPose", lambda: pose[0])
    monkeypatch.setattr(candidate.UnitedPosition, "RobotBallPos", lambda: local_ball[0])
    monkeypatch.setenv("XEL_ROBOT_ID", "1")
    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", "1")
    monkeypatch.setattr(candidate.lockball, "LockBallSlip", lambda: calls.append("lock"))

    assert candidate.step("attacker") == "approach_shoulder"
    assert moves[-1] == (0.12, 0.34, 0.0)
    pose[0] = [10.0, 30.0, 0.0]
    assert candidate.step("attacker") == "free_ball_enter"
    assert moves[-1] == (0.14400000000000002, -0.24, 0.0)
    pose[0] = [22.0, 10.0, 0.0]
    local_ball[0] = [10.0, -8.0]
    assert candidate.step("attacker") == "free_ball_lock"
    assert calls == ["lock"]


@pytest.mark.parametrize(
    "local_ball,expected_state,expected_locks",
    [
        ((10.64, -14.58), "free_ball_lock", ["lock"]),
        ((11.3, -15.5), "free_ball_enter", []),
    ],
)
def test_goal_side_owner_locks_safe_18cm_entry_but_approaches_farther_ball(
    monkeypatch, local_ball, expected_state, expected_locks
):
    calls = []
    moves = _free_ball_patch(
        monkeypatch,
        pose=(-9.91, 15.08, 0.0),
        local_ball=local_ball,
        field_ball=(0.73, 0.5),
    )
    monkeypatch.setenv("XEL_ROBOT_ID", "1")
    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", "1")
    monkeypatch.setattr(candidate.lockball, "LockBallSlip", lambda: calls.append("lock"))
    candidate._free_approach_marker = (0.73, 0.5, 1.0)

    assert candidate.step("attacker") == expected_state
    assert calls == expected_locks
    assert (moves == []) is (expected_state == "free_ball_lock")


def test_ordinary_nonowner_yields_without_lock(monkeypatch):
    moves = _free_ball_patch(monkeypatch, field_ball=(30.0, 0.0), local_ball=(8.0, 0.0))
    monkeypatch.setenv("XEL_ROBOT_ID", "3")
    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", "-1")
    monkeypatch.setattr(candidate.lockball, "LockBallSlip", lambda: pytest.fail("nonowner used lock"))

    assert candidate.step("attacker") == "free_ball_yield"
    assert moves[-1] == (0.14400000000000002, -0.34, 0.0)


@pytest.mark.parametrize("field_ball,direction,robot", [((0.0, 0.0), 1.0, 3), ((0.0, 0.0), -1.0, 1)])
def test_ordinary_yield_stays_interior_and_differs_from_drop_yield(field_ball, direction, robot):
    ordinary = candidate.ordinary_yield_target(field_ball, direction, candidate.lane_side(robot))
    drop = candidate.drop_yield_target(field_ball, direction, candidate.lane_side(robot), robot)

    assert ordinary != drop
    assert abs(ordinary[0]) <= candidate.YIELD_INTERIOR_X_CM
    assert abs(ordinary[1]) <= candidate.YIELD_INTERIOR_Y_CM
    assert abs(ordinary[1]) == candidate.YIELD_INTERIOR_Y_CM


def test_defender_contested_cover_is_goal_side_lateral_and_outside_box(monkeypatch):
    moves = _free_ball_patch(monkeypatch, field_ball=(-35.0, 0.0), enemies=((-30.0, 0.0),))
    monkeypatch.setenv("XEL_ROBOT_ID", "2")
    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", "1")
    monkeypatch.setattr(candidate.lockball, "LockBallSlip", lambda: pytest.fail("defender must block, not ram"))

    assert candidate.step("defender") == "defender_contested_block"
    speed_x, speed_y, _yaw = moves[-1]
    assert speed_x < 0.0
    assert speed_y > 0.0
    assert abs(speed_x) <= candidate.MOVE_X_LIMIT
    assert abs(speed_y) <= candidate.MOVE_Y_LIMIT


def test_direction_symmetry_mirrors_shoulder_and_carry(monkeypatch):
    normal = candidate.attacker_shoulder([30.0, 5.0], 1.0, 1.0)
    swapped = candidate.attacker_shoulder([-30.0, 5.0], -1.0, 1.0)
    assert swapped[:2] == [-normal[0], normal[1]]

    moves = []
    monkeypatch.setattr(candidate.UnitedPosition, "FieldRobotPose", lambda: [0.0, 0.0, 193.0])
    monkeypatch.setattr(candidate.UnitedPosition, "RestrictedArea", lambda: True)
    monkeypatch.setattr(candidate.UnitedPosition, "RobotBallPos", lambda: candidate.BALL_HELD)
    monkeypatch.setattr(candidate.UnitedPosition, "BallState", lambda _ball: "held")
    monkeypatch.setattr(candidate.peripheral, "Dribble", lambda _value: None)
    monkeypatch.setattr(candidate.chassis, "FieldMoveVector", lambda *args, **_kwargs: moves.append(args))
    monkeypatch.setattr(candidate.time, "monotonic", lambda: 0.31)
    monkeypatch.setenv("XEL_ROBOT_ID", "1")
    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", "1")
    candidate._held_since = 0.0
    assert candidate.step("attacker") == "carry_lane"
    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", "-1")
    assert candidate.step("attacker") == "carry_lane"
    assert moves[-1] == (-moves[-2][0], moves[-2][1], 193.0)


def test_central_pre_boundary_possession_releases_shot_and_stops(monkeypatch):
    dribble_calls, shots, moves = _held_ball_patch(monkeypatch, pose=(60.0, 12.0, 0.0))
    stops = []
    monkeypatch.setattr(candidate.helper, "stop", lambda: stops.append(True))
    monkeypatch.setattr(candidate.time, "monotonic", lambda: 0.31)
    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", "1")
    candidate._held_since = 0.0

    assert candidate.step("attacker") == "finish_shot"
    assert dribble_calls == [False]
    assert shots == [True]
    assert stops == [True]
    assert moves == []


@pytest.mark.parametrize("direction,pose,goal_yaw", [
    (1.0, (60.0, 12.0, 339.0), 0.0),
    (-1.0, (-60.0, 12.0, 159.0), 180.0),
])
def test_misaligned_central_pre_boundary_possession_aligns_without_forward_carry(monkeypatch, direction, pose, goal_yaw):
    dribble_calls, shots, moves = _held_ball_patch(monkeypatch, pose=pose)
    monkeypatch.setattr(candidate.time, "monotonic", lambda: 0.31)
    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", str(int(direction)))
    candidate._held_since = 0.0

    assert candidate.step("attacker") == "finish_align_shot"
    assert dribble_calls == [0.85]
    assert shots == []
    assert moves == [(0.0, 0.0, goal_yaw)]
    assert moves[0][0] == 0.0


@pytest.mark.parametrize("direction,pose", [
    (1.0, (60.0, 12.0, 0.0)),
    (-1.0, (-60.0, 12.0, 180.0)),
])
def test_aligned_central_pre_boundary_possession_shoots_without_forward_carry(monkeypatch, direction, pose):
    dribble_calls, shots, moves = _held_ball_patch(monkeypatch, pose=pose)
    stops = []
    monkeypatch.setattr(candidate.helper, "stop", lambda: stops.append(True))
    monkeypatch.setattr(candidate.time, "monotonic", lambda: 0.31)
    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", str(int(direction)))
    candidate._held_since = 0.0

    assert candidate.step("attacker") == "finish_shot"
    assert dribble_calls == [False]
    assert shots == [True]
    assert stops == [True]
    assert moves == []


def test_wide_pre_boundary_recovery_moves_inward_without_forward_carry(monkeypatch):
    dribble_calls, shots, moves = _held_ball_patch(monkeypatch, pose=(60.0, 50.0, 0.0))
    monkeypatch.setattr(candidate.time, "monotonic", lambda: 0.31)
    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", "1")
    candidate._held_since = 0.0

    assert candidate.step("attacker") == "finish_recover_lane"
    assert dribble_calls == [0.85]
    assert shots == []
    assert moves == [(-0.12, -0.12, 0.0)]
    assert moves[0][0] <= 0.0


def test_end_zone_recovery_mirrors_attack_direction(monkeypatch):
    pose = [[60.0, 50.0, 193.0]]
    _dribble_calls, shots, moves = _held_ball_patch(monkeypatch, pose=pose[0])
    monkeypatch.setattr(candidate.UnitedPosition, "FieldRobotPose", lambda: pose[0])
    monkeypatch.setattr(candidate.time, "monotonic", lambda: 0.31)
    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", "1")
    candidate._held_since = 0.0
    assert candidate.step("attacker") == "finish_recover_lane"
    pose[0] = [-60.0, 50.0, 193.0]
    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", "-1")
    assert candidate.step("attacker") == "finish_recover_lane"

    assert shots == []
    assert moves[-1] == (-moves[-2][0], moves[-2][1], 193.0)


def test_newly_held_ball_secures_before_translation(monkeypatch):
    dribble_calls, _shots, moves = _held_ball_patch(monkeypatch)
    monkeypatch.setattr(candidate.time, "monotonic", lambda: 10.0)

    assert candidate.step("attacker") == "secure_possession"
    assert dribble_calls == [0.85]
    assert moves == [(0.0, 0.0, 0.0)]


def test_post_settle_carry_uses_conservative_speed_caps(monkeypatch):
    _dribble_calls, _shots, moves = _held_ball_patch(monkeypatch, pose=(0.0, 0.0, 0.0))
    monkeypatch.setattr(candidate.time, "monotonic", lambda: 10.31)
    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", "1")
    candidate._held_since = 10.0

    assert candidate.step("attacker") == "carry_lane"
    forward, lateral, _yaw = moves[-1]
    assert forward == candidate.CARRY_FORWARD_LIMIT
    assert abs(forward) <= candidate.CARRY_FORWARD_LIMIT
    assert abs(lateral) <= candidate.CARRY_LATERAL_LIMIT


def test_post_settle_defender_patrol_uses_its_phase_offset(monkeypatch):
    _dribble_calls, _shots, moves = _held_ball_patch(monkeypatch)
    monkeypatch.setattr(candidate.time, "monotonic", lambda: 10.31)
    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", "1")
    monkeypatch.setenv("XEL_ROBOT_ID", "2")
    candidate._held_since = 10.0

    assert candidate.step("defender") == "search_lane"
    assert moves[-1] == (-0.192, -candidate.MOVE_Y_LIMIT, 0.0)


def test_loss_and_reacquisition_restart_possession_settle(monkeypatch):
    state = ["held"]
    now = [10.0]
    _dribble_calls, _shots, moves = _held_ball_patch(monkeypatch)
    monkeypatch.setattr(candidate.UnitedPosition, "RobotBallPos", lambda: candidate.BALL_HELD if state[0] == "held" else candidate.BALL_NOT_FOUND)
    monkeypatch.setattr(candidate.UnitedPosition, "BallState", lambda _ball: state[0])
    monkeypatch.setattr(candidate.UnitedPosition, "FieldBallPos", lambda: candidate.BALL_NOT_FOUND)
    monkeypatch.setattr(candidate.UnitedPosition, "ProjectSafePose", lambda target: target)
    monkeypatch.setattr(candidate.time, "monotonic", lambda: now[0])

    assert candidate.step("attacker") == "secure_possession"
    state[0] = "not_found"
    now[0] = 10.1
    candidate.step("attacker")
    assert candidate._held_since is None

    state[0] = "held"
    now[0] = 10.2
    assert candidate.step("attacker") == "secure_possession"
    assert candidate._held_since == 10.2
    assert moves[-1] == (0.0, 0.0, 0.0)


def test_midfield_held_ball_keeps_generic_lane_carry(monkeypatch):
    _dribble_calls, shots, moves = _held_ball_patch(monkeypatch, pose=(58.0, 0.0, 0.0))
    monkeypatch.setattr(candidate.time, "monotonic", lambda: 0.31)
    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", "1")
    candidate._held_since = 0.0

    assert candidate.step("attacker") == "carry_lane"
    assert shots == []
    assert moves == [(0.18, 0.12, 0.0)]


@pytest.mark.parametrize("field_ball,direction,owner,nonowner", [
    ((67.0, 50.0), 1, 1, 3),
    ((-67.0, -50.0), -1, 3, 1),
])
def test_safe_drop_ball_has_one_mirrored_odd_attacker_owner(field_ball, direction, owner, nonowner):
    assert candidate.drop_ball_owner(field_ball, direction, owner)
    assert not candidate.drop_ball_owner(field_ball, direction, owner + 1)
    assert not candidate.drop_ball_owner(field_ball, -direction, nonowner)
    assert not candidate.drop_ball_owner(field_ball, -direction, nonowner + 1)


def test_safe_drop_owner_does_not_lock_before_close_entry(monkeypatch):
    calls = []
    moves = _free_ball_patch(
        monkeypatch,
        pose=(43.0, 60.0, 0.0),
        local_ball=(28.0, -10.0),
        field_ball=(67.0, 50.0),
        enemies=((66.0, 49.0),),
    )
    monkeypatch.setenv("XEL_ROBOT_ID", "1")
    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", "1")
    monkeypatch.setattr(candidate.lockball, "LockBallSlip", lambda: calls.append("lock"))

    assert candidate.step("attacker") == "drop_ball_enter"
    assert moves[-1] == (0.192, 0.0, 0.0)
    assert calls == []


def test_safe_drop_owner_locks_only_close_after_angled_entry(monkeypatch):
    calls = []
    pose = [[47.0, 62.0, 0.0]]
    local_ball = [[28.0, -10.0]]
    _free_ball_patch(monkeypatch, pose=pose[0], local_ball=local_ball[0], field_ball=(67.0, 50.0), enemies=((66.0, 49.0),))
    monkeypatch.setattr(candidate.UnitedPosition, "FieldRobotPose", lambda: pose[0])
    monkeypatch.setattr(candidate.UnitedPosition, "RobotBallPos", lambda: local_ball[0])
    monkeypatch.setenv("XEL_ROBOT_ID", "1")
    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", "1")
    monkeypatch.setattr(candidate.lockball, "LockBallSlip", lambda: calls.append("lock"))

    assert candidate.step("attacker") == "drop_ball_enter"
    pose[0] = [59.0, 58.0, 0.0]
    local_ball[0] = [9.0, -8.0]
    assert candidate.step("attacker") == "drop_ball_lock"
    assert calls == ["lock"]


def test_safe_drop_lock_releases_before_any_forward_corner_carry(monkeypatch):
    dribble_calls, shots, moves = _held_ball_patch(monkeypatch, pose=(68.0, 49.0, 0.0))
    stops = []
    monkeypatch.setattr(candidate.helper, "stop", lambda: stops.append(True))
    monkeypatch.setattr(candidate.time, "monotonic", lambda: 10.31)
    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", "1")
    candidate._corner_release_pending = True
    candidate._held_since = 10.0

    assert candidate.step("attacker") == "drop_ball_clearance"
    assert dribble_calls == [False]
    assert shots == [True]
    assert stops == [True]
    assert moves == []
    assert not candidate._corner_release_pending


def test_restricted_area_recovery_uses_safe_position_motion_without_dribbling(monkeypatch):
    dribble_calls = []
    targets = []
    monkeypatch.setattr(candidate.UnitedPosition, "FieldRobotPose", lambda: [68.0, 49.0, 193.0])
    monkeypatch.setattr(candidate.UnitedPosition, "RestrictedArea", lambda: False)
    monkeypatch.setattr(candidate.UnitedPosition, "ProjectSafePose", lambda _target: [59.0, 37.0, 193.0])
    monkeypatch.setattr(candidate.peripheral, "Dribble", lambda value: dribble_calls.append(value))
    monkeypatch.setattr(candidate.UnitedPosition, "Pos2Pos", lambda target, gain: targets.append((target, gain)))
    monkeypatch.setattr(candidate.chassis, "FieldMoveVector", lambda *args, **_kwargs: pytest.fail("restricted recovery used raw field vector"))

    assert candidate.step("attacker") == "restricted_recover"
    assert dribble_calls == [False]
    assert targets == [([59.0, 37.0, 193.0], 1.0)]


def test_close_goal_side_owner_locks_despite_nearby_opponent(monkeypatch):
    calls = []
    moves = _free_ball_patch(
        monkeypatch,
        pose=(-2.0, 0.0, 0.0),
        local_ball=(8.0, 0.0),
        field_ball=(0.0, 0.0),
        enemies=((1.0, 0.0),),
    )
    monkeypatch.setenv("XEL_ROBOT_ID", "1")
    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", "1")
    monkeypatch.setattr(candidate.lockball, "LockBallSlip", lambda: calls.append("lock"))

    assert candidate.step("attacker") == "free_ball_lock"
    assert calls == ["lock"]
    assert moves == []


def test_safe_drop_nonowner_yields_with_goal_side_lateral_separation(monkeypatch):
    moves = _free_ball_patch(monkeypatch, pose=(46.0, 17.0, 0.0), field_ball=(67.0, 50.0))
    monkeypatch.setenv("XEL_ROBOT_ID", "3")
    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", "-1")
    monkeypatch.setattr(candidate.lockball, "LockBallSlip", lambda: pytest.fail("nonowner used lock"))

    assert candidate.step("attacker") == "drop_ball_yield"
    assert moves[-1] == (0.07200000000000001, -0.34, 0.0)


def test_safe_drop_defender_blocks_opponent_owned_corner_ball(monkeypatch):
    moves = _free_ball_patch(monkeypatch, pose=(46.0, 17.0, 0.0), field_ball=(67.0, 50.0))
    targets = []
    monkeypatch.setattr(candidate.UnitedPosition, "ProjectSafePose", lambda target: targets.append(list(target)) or target)
    monkeypatch.setenv("XEL_ROBOT_ID", "2")
    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", "1")
    monkeypatch.setattr(candidate.lockball, "LockBallSlip", lambda: pytest.fail("defender must cover, not lock"))

    assert candidate.step("defender") == "drop_ball_defend"
    assert targets == [[19.0, 58.0, 0.0]]
    assert moves[-1] == (-0.324, candidate.MOVE_Y_LIMIT, 0.0)


def test_search_lane_patrol_mirrors_direction_stays_safe_and_changes_stages(monkeypatch):
    targets = []
    now = [0.0]
    monkeypatch.setattr(candidate.UnitedPosition, "FieldRobotPose", lambda: [0.0, 0.0, 193.0])
    monkeypatch.setattr(candidate.UnitedPosition, "RestrictedArea", lambda: True)
    monkeypatch.setattr(candidate.UnitedPosition, "RobotBallPos", lambda: candidate.BALL_NOT_FOUND)
    monkeypatch.setattr(candidate.UnitedPosition, "BallState", lambda _ball: "not_found")
    monkeypatch.setattr(candidate.UnitedPosition, "ProjectSafePose", lambda target: targets.append(list(target)) or target)
    monkeypatch.setattr(candidate.chassis, "FieldMoveVector", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(candidate.peripheral, "Dribble", lambda _value: None)
    monkeypatch.setattr(candidate.time, "monotonic", lambda: now[0])
    monkeypatch.setenv("XEL_ROBOT_ID", "1")

    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", "1")
    assert candidate.step("attacker") == "search_lane"
    now[0] = candidate.SEARCH_STAGE_SECONDS
    assert candidate.step("attacker") == "search_lane"
    positive_direction_targets = targets[:]

    now[0] = 0.0
    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", "-1")
    assert candidate.step("attacker") == "search_lane"
    now[0] = candidate.SEARCH_STAGE_SECONDS
    assert candidate.step("attacker") == "search_lane"
    negative_direction_targets = targets[2:]

    assert positive_direction_targets[0] != positive_direction_targets[1]
    assert [target[:2] for target in negative_direction_targets] == [
        [-target[0], target[1]] for target in positive_direction_targets
    ]
    for target in targets:
        assert abs(target[0]) <= 48.0
        assert abs(target[1]) <= 48.0
    assert [target[2] for target in positive_direction_targets] == [0.0, 0.0]
    assert [target[2] for target in negative_direction_targets] == [180.0, 180.0]


def test_move_caps_large_target_error(monkeypatch):
    moves = []
    monkeypatch.setattr(candidate.UnitedPosition, "ProjectSafePose", lambda target: target)
    monkeypatch.setattr(candidate.UnitedPosition, "FieldRobotPose", lambda: [0.0, 0.0, 0.0])
    monkeypatch.setattr(candidate.chassis, "FieldMoveVector", lambda *args, **kwargs: moves.append((args, kwargs)))

    candidate._move([100.0, -100.0, 180.0])

    assert moves == [((candidate.MOVE_X_LIMIT, -candidate.MOVE_Y_LIMIT, 0.0), {"KpW": 0.7})]


def test_move_slows_and_stops_near_target(monkeypatch):
    moves = []
    pose = [[0.0, 0.0, 0.0]]
    monkeypatch.setattr(candidate.UnitedPosition, "ProjectSafePose", lambda target: target)
    monkeypatch.setattr(candidate.UnitedPosition, "FieldRobotPose", lambda: pose[0])
    monkeypatch.setattr(candidate.chassis, "FieldMoveVector", lambda *args, **_kwargs: moves.append(args))

    candidate._move([10.0, -5.0, 0.0])
    pose[0] = [9.0, -1.0, 0.0]
    candidate._move([10.0, -5.0, 0.0])

    assert moves == [(0.12, -0.06, 0.0), (0.0, -0.048, 0.0)]


def test_move_velocity_signs_match_target_displacement(monkeypatch):
    moves = []
    monkeypatch.setattr(candidate.UnitedPosition, "ProjectSafePose", lambda target: target)
    monkeypatch.setattr(candidate.UnitedPosition, "FieldRobotPose", lambda: [10.0, -10.0, 0.0])
    monkeypatch.setattr(candidate.chassis, "FieldMoveVector", lambda *args, **_kwargs: moves.append(args))

    candidate._move([-10.0, 10.0, 180.0])

    assert moves == [(-0.24, 0.24, 0.0)]


def test_nonzero_pose_heading_is_preserved_for_move_carry_and_recovery(monkeypatch):
    moves = []
    pose = [[0.0, 0.0, 193.0]]
    monkeypatch.setattr(candidate.UnitedPosition, "ProjectSafePose", lambda target: target)
    monkeypatch.setattr(candidate.UnitedPosition, "FieldRobotPose", lambda: pose[0])
    monkeypatch.setattr(candidate.chassis, "FieldMoveVector", lambda *args, **_kwargs: moves.append(args))

    candidate._move([100.0, -100.0, 0.0])

    monkeypatch.setattr(candidate.UnitedPosition, "RestrictedArea", lambda: True)
    monkeypatch.setattr(candidate.UnitedPosition, "RobotBallPos", lambda: candidate.BALL_HELD)
    monkeypatch.setattr(candidate.UnitedPosition, "BallState", lambda _ball: "held")
    monkeypatch.setattr(candidate.peripheral, "Dribble", lambda _value: None)
    monkeypatch.setattr(candidate.time, "monotonic", lambda: 0.31)
    monkeypatch.setenv("LEAGUE_ATTACK_DIRECTION", "1")
    candidate._held_since = 0.0
    assert candidate.step("attacker") == "carry_lane"
    pose[0] = [80.0, 50.0, 193.0]
    assert candidate.step("attacker") == "finish_recover_lane"

    assert [move[2] for move in moves] == [193.0, 193.0, 193.0]


@pytest.mark.parametrize("field_ball,owner_direction", [((67.0, 50.0), 1.0), ((-67.0, -50.0), -1.0)])
def test_drop_targets_keep_all_four_robots_separated(field_ball, owner_direction):
    owner_robot = 1 if owner_direction > 0.0 else 3
    targets = [
        candidate.drop_entry_target(field_ball, owner_direction, candidate.lane_side(owner_robot)),
        *(candidate.drop_yield_target(field_ball, owner_direction, candidate.lane_side(robot), robot) for robot in range(1, 5) if robot != owner_robot),
    ]

    for index, target in enumerate(targets):
        for other in targets[index + 1:]:
            assert ((target[0] - other[0]) ** 2 + (target[1] - other[1]) ** 2) ** 0.5 >= 28.0
