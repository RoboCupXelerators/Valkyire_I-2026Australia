import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import BasicFuc as b
BALL_HELD = [0xDDD, 0xDDD]
def test_slipsideshot_drives_sideline_tick(monkeypatch):
    calls = []
    monkeypatch.setattr(b.UnitedPosition, "RobotBallPos", lambda: BALL_HELD); monkeypatch.setattr(b.UnitedPosition, "FieldBallPos", lambda: BALL_HELD); monkeypatch.setattr(b.UnitedPosition, "FieldRobotPose", lambda: [30, 56, 0])
    monkeypatch.setattr(b.peripheral, "Dribble", lambda value: calls.append(("dribble", value))); monkeypatch.setattr(b.chassis, "FieldMoveVector", lambda *args, **kwargs: calls.append(("move", args, kwargs)))
    assert b.Slipsideshot(attack_dir=1) is False
    assert calls[0] == ("dribble", 1.0)
    assert calls[1][0] == "move"
    assert abs(calls[1][1][0]) <= 0.25
    assert abs(calls[1][1][1]) <= 0.13
