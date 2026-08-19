from vision_utils.Vision import UnitedVision


def _vision_merger(radius_cm=14.0, max_chassis=3):
    vision = UnitedVision.__new__(UnitedVision)
    vision.ChassisMergeDistanceCM = radius_cm
    vision.MaxVisibleChassis = max_chassis
    return vision


def test_chassis_fusion_merges_only_the_same_chassis_seen_by_two_cameras():
    vision = _vision_merger()

    merged = vision._merge_chassis_observations([
        (168.0, 4.0, 110.0, 80.0, 0.58, 0),
        (171.0, 1.0, 100.0, 85.0, 0.55, 2),
        (45.0, -25.0, 90.0, 70.0, 0.73, 1),
        (-55.0, 30.0, 95.0, 72.0, 0.68, 3),
    ])

    assert len(merged) == 3
    shared = next(chassis for chassis in merged if chassis[0] > 150)
    assert 169.0 <= shared[0] <= 170.0
    assert 2.0 <= shared[1] <= 3.0
    assert shared[4] == 0.58
    assert any(abs(chassis[0] - 45.0) < 1e-6 and abs(chassis[1] + 25.0) < 1e-6 for chassis in merged)
    assert any(abs(chassis[0] + 55.0) < 1e-6 and abs(chassis[1] - 30.0) < 1e-6 for chassis in merged)


def test_chassis_fusion_preserves_three_nearby_but_distinct_chassis():
    vision = _vision_merger(radius_cm=12.0)

    merged = vision._merge_chassis_observations([
        (0.0, 0.0, 100.0, 80.0, 0.70, 0),
        (3.0, 2.0, 95.0, 82.0, 0.60, 2),
        (22.0, 0.0, 90.0, 70.0, 0.80, 1),
        (44.0, 0.0, 88.0, 72.0, 0.75, 3),
    ])

    assert len(merged) == 3
    assert any(0.0 < chassis[0] < 3.0 for chassis in merged)
    assert any(abs(chassis[0] - 22.0) < 1e-6 for chassis in merged)
    assert any(abs(chassis[0] - 44.0) < 1e-6 for chassis in merged)
