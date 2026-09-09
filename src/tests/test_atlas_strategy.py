"""Atlas 决策与控制边界回归；替换运行时对象，不向真实底盘发命令。"""

from types import SimpleNamespace
from unittest.mock import Mock
import math

import pytest

import atlas
from LunaPre.positions import Positions
from LunaPre.strategy_tools import BALL_HELD, BALL_NOT_FOUND, POSE_NOT_FOUND


OPEN_WINDOW = {
    "can_shoot_now": True, "window_open": True, "reason": "ok",
    "goal_key": "blue_goal", "target_yaw_deg": 0.0,
}


@pytest.mark.parametrize("score", [atlas.HuckScore, atlas.UplaneScore])
def test_scores_are_bounded_and_periodic(score):
    for x in (-130, -60, 0, 60, 130):
        for y in (-100, -60, 0, 60, 100):
            for yaw in range(-180, 361, 15):
                result = score((x, y, yaw))
                assert 0 <= result <= 1
                assert result == pytest.approx(score((x, y, yaw + 360)))


def test_direct_alignment_is_continuous_across_zero_and_decreases_with_error():
    assert atlas.HuckScore((0, 0, 359)) == pytest.approx(atlas.HuckScore((0, 0, 1)))
    scores = [atlas.HuckScore((0, 0, yaw)) for yaw in (0, 30, 90, 179, 180)]
    assert all(a > b for a, b in zip(scores, scores[1:]))


@pytest.mark.parametrize("score", [atlas.HuckScore, atlas.UplaneScore])
def test_attack_progress_does_not_reward_own_half(score):
    assert score((60, 0, 0)) > score((-60, 0, 0))


@pytest.mark.parametrize("score", [atlas.HuckScore, atlas.UplaneScore])
def test_mirrored_lanes_have_equal_scores(score):
    for yaw in (0, 45, 90, 179, 270):
        assert score((20, 60, yaw)) == pytest.approx(score((20, -60, -yaw)))


def test_lane_score_has_no_jump_at_old_fifty_cm_gate():
    assert abs(atlas.UplaneScore((0, 50.001, 90)) - atlas.UplaneScore((0, 49.999, 90))) < 0.001


def test_both_implemented_strategies_can_win():
    for pose, expected in [((0, 0, 0), "huck"), ((0, 60, 90), "uplane")]:
        scores = {"huck": atlas.HuckScore(pose), "uplane": atlas.UplaneScore(pose)}
        assert atlas.select_shot(scores) == expected


def test_selection_holds_small_lead_but_releases_large_lead():
    assert atlas.select_shot({"huck": 0.5, "uplane": 0.5}) == "huck"
    assert atlas.select_shot({"huck": 0.54, "uplane": 0.5}, "uplane") == "uplane"
    assert atlas.select_shot({"huck": 0.7, "uplane": 0.5}, "uplane") == "huck"


@pytest.fixture
def runtime(monkeypatch):
    geometry = Positions()
    geometry.RobotChassisPos = Mock(return_value=[])
    position = SimpleNamespace(
        RobotBallPos=Mock(return_value=BALL_HELD),
        FieldRobotPose=Mock(return_value=[0.0, 0.0, 80.0]),
        ShootWindow=Mock(return_value={"can_shoot_now": False, "target_yaw_deg": 100.0}),
        Pos2Pos=Mock(),
        PointInShape=Positions().PointInShape,
        _shoot_blocker_windows=geometry._shoot_blocker_windows,
    )
    chassis = SimpleNamespace(GetYaw=Mock(return_value=80.0), stop=Mock(), FieldMoveVector=Mock())
    peripheral = SimpleNamespace(Dribble=Mock(), ShootBall=Mock())
    lockball = SimpleNamespace(LockBallSlip=Mock(return_value=True))
    turn = Mock(return_value=True)
    monkeypatch.setattr(atlas, "UnitedPosition", position)
    monkeypatch.setattr(atlas, "chassis", chassis)
    monkeypatch.setattr(atlas, "peripheral", peripheral)
    monkeypatch.setattr(atlas, "lockball", lockball)
    monkeypatch.setattr(atlas, "TurnTo_with_ball", turn)
    monkeypatch.setattr(atlas, "Circle", Mock())
    monkeypatch.setattr(atlas, "_last_shot", None)
    monkeypatch.setattr(atlas, "_shoot_ready_since", None)
    monkeypatch.setattr(atlas, "_last_kick_at", -float("inf"))
    monkeypatch.setattr(atlas, "_last_shot_log_at", -float("inf"))
    monkeypatch.setattr(atlas, "_uplane_phase", None)
    monkeypatch.setattr(atlas, "_uplane_side", 1)
    monkeypatch.setattr(atlas, "_uplane_deadline", 0.0)
    monkeypatch.setattr(atlas, "_uplane_retry_after", -float("inf"))
    monkeypatch.setattr(atlas, "_last_possession_at", -float("inf"))
    clock = [0.0]
    monkeypatch.setattr(atlas, "time", SimpleNamespace(monotonic=lambda: clock[0]))
    return SimpleNamespace(position=position, chassis=chassis, peripheral=peripheral, lockball=lockball, turn=turn, clock=clock, geometry=geometry)


def test_lost_ball_goes_home_and_resets_selection(runtime):
    atlas._last_shot = "uplane"
    runtime.position.RobotBallPos.return_value = BALL_NOT_FOUND
    atlas.logic_step()
    runtime.peripheral.Dribble.assert_called_once_with(False)
    runtime.position.Pos2Pos.assert_called_once_with(atlas.Settings.CheckPoints.Home)
    runtime.lockball.LockBallSlip.assert_not_called()
    assert atlas._last_shot is None


def test_not_held_only_runs_capture(runtime):
    runtime.lockball.LockBallSlip.return_value = False
    atlas.logic_step()
    runtime.position.ShootWindow.assert_not_called()
    runtime.turn.assert_not_called()
    runtime.peripheral.ShootBall.assert_not_called()


@pytest.mark.parametrize("pose", [POSE_NOT_FOUND, [0, 0, float("nan")]])
def test_missing_pose_stops_without_scoring_or_shooting(runtime, pose):
    runtime.position.FieldRobotPose.return_value = pose
    atlas.logic_step()
    runtime.chassis.stop.assert_called_once()
    runtime.position.ShootWindow.assert_not_called()
    runtime.peripheral.ShootBall.assert_not_called()


def test_open_shot_overrides_lane_score_and_previous_selection(runtime):
    runtime.position.FieldRobotPose.return_value = [0, 0, 0]
    runtime.chassis.GetYaw.return_value = 0
    runtime.position.ShootWindow.return_value = OPEN_WINDOW
    atlas._last_shot = "uplane"
    atlas.logic_step()
    runtime.peripheral.ShootBall.assert_not_called()
    runtime.clock[0] = atlas.SHOT_CONFIRM_S + 0.01
    atlas.logic_step()
    runtime.peripheral.ShootBall.assert_called_once()
    runtime.turn.assert_not_called()
    runtime.chassis.FieldMoveVector.assert_not_called()
    assert atlas._last_shot is None


def test_window_absolute_yaw_is_forwarded_without_negation(runtime):
    atlas.logic_step()
    runtime.position.ShootWindow.assert_called_once_with(current_yaw_deg=80.0, use_estimated_goal=False)
    runtime.turn.assert_called_once_with(100.0)
    runtime.peripheral.ShootBall.assert_not_called()


def test_aligned_but_blocked_stops_previous_motion(runtime):
    runtime.position.FieldRobotPose.return_value = [0, 0, 0]
    runtime.position.ShootWindow.return_value = {"can_shoot_now": False, "target_yaw_deg": None}
    atlas.logic_step()
    runtime.chassis.stop.assert_called_once()
    runtime.peripheral.ShootBall.assert_not_called()


def test_incomplete_turn_is_not_cancelled_by_stop(runtime):
    runtime.turn.return_value = False
    atlas.logic_step()
    runtime.turn.assert_called_once_with(100.0)
    runtime.chassis.stop.assert_not_called()


@pytest.mark.parametrize("missing", [False, True])
def test_blocked_or_failed_window_only_aims_never_shoots(runtime, missing):
    runtime.position.FieldRobotPose.return_value = [0, 0, 0]
    if missing:
        runtime.position.ShootWindow.side_effect = RuntimeError("unavailable")
    else:
        runtime.position.ShootWindow.return_value = {"can_shoot_now": False, "target_yaw_deg": None}
    atlas.logic_step()
    runtime.turn.assert_called_once_with(0.0)
    runtime.peripheral.ShootBall.assert_not_called()


def test_uplane_region_does_not_leak_between_halves(runtime):
    atlas.UplaneStep([85, -60, 270], {"target_yaw_deg": 20})
    atlas._reset_uplane("shot")
    atlas.UplaneStep([85, 60, 90], {"target_yaw_deg": 340})
    assert [call.args for call in runtime.turn.call_args_list] == [(20.0,), (340.0,)]
    runtime.chassis.FieldMoveVector.assert_not_called()


@pytest.mark.parametrize("y, sign", [(-20, -1), (20, 1), (-95, 1), (95, -1)])
def test_lane_translation_uses_correct_side_and_returns_from_edge(runtime, y, sign):
    atlas.UplaneStep([0, y, 0], {})
    vx, vy, facing = runtime.chassis.FieldMoveVector.call_args.args
    assert vx == 0
    assert vy * sign > 0
    assert abs(vy) <= atlas.CARRY_SPEED_MPS
    assert facing == (270 if y < 0 else 90)


def test_overshot_lane_moves_back_toward_region(runtime):
    atlas.UplaneStep([110, 60, 90], {})
    assert runtime.chassis.FieldMoveVector.call_args.args[0] < 0


def test_lane_waits_for_carry_turn(runtime):
    runtime.turn.return_value = False
    atlas.UplaneStep([0, 60, 0], {})
    runtime.chassis.FieldMoveVector.assert_not_called()


def test_live_boundary_trace_cannot_cancel_aim(runtime):
    # 将原实跑 x=50 边界轨迹平移到当前入区线；第一次入区后必须始终瞄准。
    offset = atlas.UPLANE_REGION[0][0] - 50.0
    for x, y in [(52.242, -55.636), (49.712, -62.784), (50.009, -50.277),
                 (49.995, -49.982), (50.408, -51.446), (49.603, -55.264)]:
        atlas.UplaneStep([x + offset, y, 310], {"target_yaw_deg": 35})
        assert atlas._uplane_phase == "aim"
    assert all(call.args == (35.0,) for call in runtime.turn.call_args_list)
    runtime.chassis.FieldMoveVector.assert_not_called()


def test_lane_advance_does_not_go_back_when_lateral_position_drifts(runtime):
    atlas.UplaneStep([0, -20, 270], {})
    assert atlas._uplane_phase == "lane"
    atlas.UplaneStep([0, -58, 270], {})
    assert atlas._uplane_phase == "advance"
    atlas.UplaneStep([20, -49, 270], {})
    assert atlas._uplane_phase == "advance"
    vx, vy, facing = runtime.chassis.FieldMoveVector.call_args.args
    assert vx > 0 and vy < 0 and facing == 270
    atlas.UplaneStep([74.9, -60, 270], {})
    assert atlas._uplane_phase == "advance"
    atlas.UplaneStep([75.1, -60, 270], {})
    assert atlas._uplane_phase == "aim"


def test_lane_side_is_fixed_through_midline_noise(runtime):
    atlas.UplaneStep([0, -0.1, 270], {})
    atlas.UplaneStep([0, 0.1, 270], {})
    assert atlas._uplane_side == -1
    assert runtime.chassis.FieldMoveVector.call_args.args[1] < 0
    runtime.turn.assert_called_with(270.0)


def test_active_uplane_is_not_preempted_by_new_score(runtime, monkeypatch):
    atlas.UplaneStep([0, -60, 270], {})
    monkeypatch.setattr(atlas, "HuckScore", lambda *args: 1.0)
    monkeypatch.setattr(atlas, "UplaneScore", lambda *args: 0.0)
    atlas.logic_step()
    assert atlas._last_shot == "uplane"


@pytest.mark.parametrize("missing", ["ball", "possession"])
def test_short_observation_loss_preserves_aim_but_long_loss_resets(runtime, missing):
    atlas.UplaneStep([80, -60, 270], {})
    atlas._last_possession_at = 0.0
    if missing == "ball":
        runtime.position.RobotBallPos.return_value = BALL_NOT_FOUND
    else:
        runtime.lockball.LockBallSlip.return_value = False
    runtime.clock[0] = 0.1
    atlas.logic_step()
    assert atlas._uplane_phase == "aim"
    runtime.position.Pos2Pos.assert_not_called()
    runtime.clock[0] = atlas.UPLANE_LOST_GRACE_S + 0.01
    atlas.logic_step()
    assert atlas._uplane_phase is None


def test_timeout_exits_uplane_without_immediately_reentering(runtime, monkeypatch):
    atlas.UplaneStep([0, -60, 270], {})
    runtime.clock[0] = atlas.UPLANE_TIMEOUT_S + 0.01
    atlas.UplaneStep([0, -60, 270], {})
    assert atlas._uplane_phase is None
    assert atlas._last_shot == "huck"
    monkeypatch.setattr(atlas, "HuckScore", lambda *args: 0.0)
    monkeypatch.setattr(atlas, "UplaneScore", lambda *args: 1.0)
    atlas.logic_step()
    assert atlas._last_shot == "huck"
    assert atlas._uplane_phase is None


@pytest.mark.parametrize("yaw", [90, 270, 180, 45, 315])
def test_wall_or_own_goal_heading_cannot_use_direct_trigger(runtime, yaw):
    runtime.position.FieldRobotPose.return_value = [0, 0, yaw]
    runtime.chassis.GetYaw.return_value = yaw
    runtime.position.ShootWindow.return_value = OPEN_WINDOW
    for now in (0, 0.1, 1.0):
        runtime.clock[0] = now
        atlas.logic_step()
    runtime.peripheral.ShootBall.assert_not_called()


@pytest.mark.parametrize("yaw", [-1, 0, 1, 359, 360, 361])
def test_direct_geometry_wraps_heading(yaw):
    assert atlas.can_direct_shoot([0, 0, yaw], yaw, OPEN_WINDOW)


def test_shot_must_intersect_inside_posts_from_actual_lateral_position():
    assert not atlas.can_direct_shoot([0, 60, 0], 0, OPEN_WINDOW)
    assert atlas.can_direct_shoot([0, 60, -25], -25, {**OPEN_WINDOW, "target_yaw_deg": -25})


@pytest.mark.parametrize("patch", [
    {"reason": "estimated_ok", "goal_key": "estimated_opponent_goal"},
    {"reason": "blocked"}, {"window_open": False}, {"goal_key": None},
])
def test_untrusted_window_never_grants_shoot_permission(patch):
    assert not atlas.can_direct_shoot([0, 0, 0], 0, {**OPEN_WINDOW, **patch})


def test_real_missing_goal_fallback_reproduces_old_misfire_but_atlas_rejects_it(runtime):
    position = object.__new__(Positions)
    position.RobotGoalsPos = lambda: {"blue_goal": None, "yellow_goal": None}
    position.RobotChassisPos = lambda: []
    position._get_goal_bboxes = lambda: [None, None, None, None]
    old_window = position.ShootWindow(current_yaw_deg=90)
    assert old_window["reason"] == "estimated_ok"
    assert old_window["can_shoot_now"] is True
    # 即使朝前也不允许用假门射击；Atlas 必须将禁用回退参数传给真实接口。
    assert not atlas.can_direct_shoot([0, 0, 0], 0, old_window)
    runtime.position.FieldRobotPose.return_value = [0, 0, 90]
    runtime.chassis.GetYaw.return_value = 90
    runtime.position.ShootWindow.side_effect = position.ShootWindow
    for now in (0, 0.1, 1.0):
        runtime.clock[0] = now
        atlas.logic_step()
    runtime.peripheral.ShootBall.assert_not_called()


def test_missing_sim_goal_detections_use_pose_window_and_really_call_kicker(runtime):
    position = object.__new__(Positions)
    position.RobotGoalsPos = lambda: {"blue_goal": None, "yellow_goal": None}
    position.RobotChassisPos = lambda: []
    position._get_goal_bboxes = lambda: [None, None, None, None]
    runtime.position.ShootWindow.side_effect = position.ShootWindow
    runtime.position.FieldRobotPose.return_value = [-4.72, -3.25, 10.0]
    runtime.chassis.GetYaw.return_value = 10.0
    atlas.logic_step()
    runtime.peripheral.ShootBall.assert_not_called()
    # 有几何窗口但尚未瞄到通道中间，继续转向后才允许射门。
    runtime.position.FieldRobotPose.return_value = [-4.72, -3.25, 1.34]
    runtime.chassis.GetYaw.return_value = 1.34
    for now in (0, 0.1):
        runtime.clock[0] = now
        atlas.logic_step()
    runtime.peripheral.ShootBall.assert_called_once()


@pytest.mark.parametrize("side", [-1, 1])
def test_front_opening_alone_cannot_authorize_far_side_frame_hit(side):
    yaw = -side * 67.5
    pose = [75, side * 60, yaw]
    front_hit = pose[1] + (103.7 - 75) * math.tan(math.radians(yaw))
    assert abs(front_hit) < 25  # 原门线检查会放行。
    assert not atlas.can_direct_shoot(pose, yaw, {**OPEN_WINDOW, "target_yaw_deg": yaw})


@pytest.mark.parametrize("side", [-1, 1])
def test_sideline_window_center_clears_both_front_and_back_posts(runtime, side):
    runtime.position.ShootWindow.return_value = {"reason": "no_goal_visible"}
    pose = [75, side * 60, 0]
    window = atlas.shooting_window(pose, 0)
    target = window["target_yaw_deg"]
    aimed = atlas.shooting_window([75, side * 60, target], target)
    assert atlas.can_direct_shoot(pose, target, aimed)
    for plane_x in (103.7, 111.15):
        assert abs(pose[1] + (plane_x - 75) * math.tan(math.radians(target))) < 25


def test_entering_window_near_post_keeps_aiming_instead_of_firing(runtime):
    runtime.position.ShootWindow.return_value = {"reason": "no_goal_visible"}
    pose = [75, 60, -53.0]
    window = atlas.shooting_window(pose, pose[2])
    assert window["can_shoot_now"]  # 球线虽擦进通道，朝向仍未对准中间。
    assert not atlas.can_direct_shoot(pose, pose[2], window)
    runtime.chassis.GetYaw.return_value = pose[2]
    atlas._turn_to_shot(pose, window)
    atlas.Circle.assert_called_once()
    runtime.chassis.stop.assert_not_called()


@pytest.mark.parametrize("yaw", [90, 180, 270])
def test_geometry_fallback_never_invents_front_goal(runtime, yaw):
    runtime.position.ShootWindow.return_value = {"reason": "no_goal_visible"}
    window = atlas.shooting_window([0, 0, yaw], yaw)
    assert not atlas.can_direct_shoot([0, 0, yaw], yaw, window)


def test_geometry_fallback_subtracts_known_blocker_and_aims_at_open_gap(runtime):
    runtime.position.ShootWindow.return_value = {"reason": "no_goal_visible"}
    runtime.geometry.RobotChassisPos.return_value = [[85, 0, 0.9]]
    window = atlas.shooting_window([0, 0, 0], 0)
    assert window["reason"] == "geometry_need_turn"
    assert len(window["open_windows_deg"]) == 2
    assert window["target_yaw_deg"] != 0
    assert not atlas.can_direct_shoot([0, 0, 0], 0, window)


def test_vision_width_does_not_expand_physical_goal_mouth():
    # 当前门线上 y≈30cm，虽然落在旧 90cm 视觉宽度里，已超出真实门柱。
    assert not atlas.can_direct_shoot([0, 0, 16.3], 16.3, OPEN_WINDOW)


def test_carry_turn_fifteen_degree_success_does_not_stall_outside_window(runtime):
    runtime.chassis.GetYaw.return_value = 10
    atlas._turn_to_shot([0, 0, 10], {"reason": "geometry_need_turn", "target_yaw_deg": 0})
    atlas.Circle.assert_called_once_with(0.6, atlas.SHOT_TURN_MIN_GAIN, -1)
    runtime.chassis.stop.assert_not_called()


@pytest.mark.parametrize("yaw,target,sign", [(333, 325, 1), (27, 35, -1), (359, 2, -1)])
def test_final_carry_turn_keeps_enough_output_in_both_directions(runtime, yaw, target, sign):
    runtime.chassis.GetYaw.return_value = yaw
    atlas._turn_to_shot([41, 67, yaw], {"reason": "geometry_need_turn", "target_yaw_deg": target})
    _, gain, direction = atlas.Circle.call_args.args
    assert gain * sign > 0
    assert abs(gain) >= atlas.SHOT_TURN_MIN_GAIN
    assert direction == -1
    runtime.chassis.stop.assert_not_called()


def test_geometry_fallback_does_not_override_visible_blocked_window(runtime):
    blocked = {"reason": "blocked", "can_shoot_now": False}
    runtime.position.ShootWindow.return_value = blocked
    assert atlas.shooting_window([0, 0, 0], 0) is blocked


@pytest.mark.parametrize("interruption", ["window", "ball", "possession", "pose"])
def test_confirmation_restarts_after_invalid_tick(runtime, interruption):
    runtime.position.FieldRobotPose.return_value = [0, 0, 0]
    runtime.chassis.GetYaw.return_value = 0
    runtime.position.ShootWindow.return_value = OPEN_WINDOW
    atlas.logic_step()
    runtime.clock[0] = 0.05
    if interruption == "window":
        runtime.position.ShootWindow.return_value = {}
    elif interruption == "ball":
        runtime.position.RobotBallPos.return_value = BALL_NOT_FOUND
    elif interruption == "possession":
        runtime.lockball.LockBallSlip.return_value = False
    else:
        runtime.position.FieldRobotPose.return_value = POSE_NOT_FOUND
    atlas.logic_step()
    runtime.position.ShootWindow.return_value = OPEN_WINDOW
    runtime.position.RobotBallPos.return_value = BALL_HELD
    runtime.position.FieldRobotPose.return_value = [0, 0, 0]
    runtime.lockball.LockBallSlip.return_value = True
    runtime.clock[0] = 0.1
    atlas.logic_step()
    runtime.peripheral.ShootBall.assert_not_called()
    runtime.clock[0] = 0.1 + atlas.SHOT_CONFIRM_S + 0.01
    atlas.logic_step()
    runtime.peripheral.ShootBall.assert_called_once()


def test_stale_held_state_cannot_repeat_kick_during_cooldown(runtime):
    runtime.position.FieldRobotPose.return_value = [0, 0, 0]
    runtime.chassis.GetYaw.return_value = 0
    runtime.position.ShootWindow.return_value = OPEN_WINDOW
    for now in (0, 0.1, 0.2, 0.3, 0.8):
        runtime.clock[0] = now
        atlas.logic_step()
    runtime.peripheral.ShootBall.assert_called_once()
    runtime.clock[0] = 0.1 + atlas.SHOT_COOLDOWN_S + 0.01
    atlas.logic_step()
    assert runtime.peripheral.ShootBall.call_count == 2
