import json
import os
import sys
import types

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from mlp.contracts import DecisionPacketV1, FaceMode, IntentId, ShootIntent, observation_from_simulator
from mlp.skill_executor import SkillExecutor
from teacher.badcase import TeacherBadcaseDetector
from teacher.rule_teacher import RuleTeacher, TeacherConfig
from teacher.scenarios import default_teacher_scenarios
from teacher.trace import TeacherTraceRecorder


def _obs(
    *,
    self_x=0.0,
    self_y=0.0,
    self_yaw=0.0,
    ball_x=30.0,
    ball_y=0.0,
    has_ball=False,
):
    return observation_from_simulator(
        self_x=self_x,
        self_y=self_y,
        self_yaw=self_yaw,
        ball_x=ball_x,
        ball_y=ball_y,
        has_ball=has_ball,
        wall_dist=[1000.0, 1000.0, 1000.0, 1000.0],
    )


def test_teacher_recovers_from_boundary():
    teacher = RuleTeacher(TeacherConfig())
    decision = teacher.decide(_obs(self_x=0.0, self_y=68.0, ball_x=-55.0, ball_y=-45.0))

    assert decision.state == "avoid_wall"
    assert decision.packet.intent_id == int(IntentId.AVOID_WALL)
    assert abs(decision.packet.target_y) < 60.0
    assert "pose_outside_safe_region" in decision.reasons
    assert decision.safety["target_legal"] is True


def test_teacher_approaches_visible_ball():
    teacher = RuleTeacher(TeacherConfig())
    decision = teacher.decide(_obs(self_x=-55.0, self_y=0.0, ball_x=35.0, ball_y=0.0))

    assert decision.state == "approach_ball"
    assert decision.packet.intent_id == int(IntentId.APPROACH_BALL)
    assert decision.safety["path_legal"] is True


def test_teacher_carries_around_goal_box_when_not_shooting():
    teacher = RuleTeacher(TeacherConfig(deep_shoot_x_cm=999.0))
    decision = teacher.decide(_obs(self_x=10.0, self_y=0.0, has_ball=True))

    assert decision.state == "carry_to_lane"
    assert decision.packet.intent_id == int(IntentId.CARRY_TO_POSE)
    assert decision.packet.target_x <= 60.0
    assert abs(decision.packet.target_y) >= 55.0
    assert decision.safety["target_legal"] is True
    assert decision.safety["path_legal"] is True


def test_trace_recorder_writes_jsonl(tmp_path):
    teacher = RuleTeacher(TeacherConfig())
    obs = _obs(ball_x=40.0, ball_y=5.0)
    decision = teacher.decide(obs)
    path = tmp_path / "teacher_trace.jsonl"

    recorder = TeacherTraceRecorder(path)
    row = recorder.record(obs=obs, decision=decision, scenario="unit", runtime={"t_s": 0.1})
    loaded = TeacherTraceRecorder.load_jsonl(path)

    assert loaded == [row]
    assert loaded[0]["decision"]["state"] == "approach_ball"


def test_badcase_detector_catches_illegal_and_loop():
    rows = []
    for idx in range(20):
        rows.append(
            {
                "runtime": {"t_s": idx * 0.1},
                "obs": {"self_x": 0.0, "self_y": 0.0, "ball_visible": False},
                "decision": {
                    "state": "avoid_wall" if idx % 2 else "carry_to_lane",
                    "packet": {"intent_id": int(IntentId.CARRY_TO_POSE)},
                    "safety": {"target_legal": idx != 3, "path_legal": True},
                },
            }
        )

    issues = TeacherBadcaseDetector(loop_window=18, loop_min_switches=8).analyze(rows)
    kinds = {issue.kind for issue in issues}

    assert "illegal_target" in kinds
    assert "state_loop" in kinds


def test_default_scenarios_are_serializable():
    payload = [scenario.to_dict() for scenario in default_teacher_scenarios()]
    roundtrip = json.loads(json.dumps(payload))

    assert len(roundtrip) >= 3
    assert all(item["name"] for item in roundtrip)


def test_skill_executor_avoid_wall_moves_to_safe_target(monkeypatch):
    calls = []

    class FakeUnitedPosition:
        @staticmethod
        def FieldRobotPos():
            return [0.0, 68.0, 12.0]

        @staticmethod
        def Pos2Pos(target, speed_gain=1.0):
            calls.append((target, speed_gain))

    class FakeChassis:
        @staticmethod
        def stop():
            calls.append(("stop",))

    class FakePeripheral:
        @staticmethod
        def Dribble(value):
            calls.append(("dribble", value))

        @staticmethod
        def ShootBall():
            calls.append(("shoot",))

    class FakeLockBall:
        @staticmethod
        def LockBallSlip():
            calls.append(("lock",))

    luna = types.ModuleType("LunaPre")
    luna.UnitedPosition = FakeUnitedPosition
    luna.chassis = FakeChassis
    luna.peripheral = FakePeripheral

    basic = types.ModuleType("BasicFuc")
    basic.lockball = FakeLockBall

    monkeypatch.setitem(sys.modules, "LunaPre", luna)
    monkeypatch.setitem(sys.modules, "BasicFuc", basic)

    packet = DecisionPacketV1(
        intent_id=int(IntentId.AVOID_WALL),
        target_x=0.0,
        target_y=54.0,
        target_yaw=0.0,
        face_mode=int(FaceMode.FACE_KEEP),
        move_gain=9.0,
    )
    result = SkillExecutor().execute(packet)

    assert result.executed is True
    assert result.status == "avoid_wall_pos2pos"
    assert calls[-1] == ([0.0, 54.0, 12.0], 3.0)

    calls.clear()
    shoot_packet = DecisionPacketV1(
        intent_id=int(IntentId.SHOOT),
        shoot_intent=int(ShootIntent.TRY_SHOOT),
    )
    result = SkillExecutor().execute(shoot_packet)

    assert result.executed is True
    assert result.status == "shoot"
    assert calls.count(("shoot",)) == 1
