import queue

from vision_utils.TunaIntelligence import TunaIntelligence


def test_tuna_intelligence_exposes_control_panel_goal_compat_methods():
    inst = TunaIntelligence.__new__(TunaIntelligence)
    inst.GoalQueue = queue.Queue(maxsize=1)
    inst.GoalQueue.put([[0, 1, 100, 200, 50, 40, 0.92]])
    inst.BallQueue = queue.Queue(maxsize=1)
    inst.BallQueue.put([[0, 120, 210, 20, 20, 0.88]])
    inst.ChassisQueue = queue.Queue(maxsize=1)
    inst.ChassisQueue.put([[0, 140, 220, 30, 30, 0.91]])
    inst.BallBBoxQueue = queue.Queue(maxsize=1)
    inst.BallBBoxQueue.put([[0, 10, 20, 30, 40, 0.86]])
    inst.ChassisBBoxQueue = queue.Queue(maxsize=1)
    inst.ChassisBBoxQueue.put([[0, 50, 60, 70, 80, 0.84]])

    def pixel2cm(x, y, cam_index):
        return float(x), float(y)

    inst.Pixel2CM = pixel2cm
    inst._to_robot_frame = lambda plane_x_cm, plane_y_cm, cam_index: (plane_x_cm, plane_y_cm)

    assert inst.get_goals_pos() == {"blue_goal": [100.0, 200.0, 0.92], "yellow_goal": None}
    assert inst.GetGoalsPos() == {"blue_goal": [100.0, 200.0, 0.92], "yellow_goal": None}
    assert inst.GetGoalBBoxes() == [(0, 1, 75, 160, 125, 200, 0.92)]
    assert inst.get_ball_pos() == [120.0, 210.0]
    assert inst.GetBallBBoxes() == [(0, 10, 20, 30, 40, 0.86)]


def test_tuna_ball_queue_output_keeps_small_far_ball_dimensions():
    out = TunaIntelligence._ball_queue_output_from_xyxy(2, (100, 200, 120, 220), 0.7)

    assert out == [2, 110, 140, 20, 20, 0.7]
