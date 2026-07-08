"""Short-horizon ball landing prediction tests."""

from __future__ import annotations

import os
import sys

os.environ.setdefault("ROBOCUPX_BITS", "SIM")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import LunaPre.positions as positions_module  # noqa: E402
from LunaPre.positions import BallLandingPredictor, Positions  # noqa: E402
from LunaPre.strategy_tools import BALL_NOT_FOUND  # noqa: E402


def test_ball_landing_predictor_projects_short_horizon():
    predictor = BallLandingPredictor(velocity_alpha=1.0)
    predictor.observe([0.0, 0.0], timestamp_s=10.0)
    predictor.observe([20.0, 10.0], timestamp_s=10.2)

    result = predictor.predict(horizon_s=0.4, now_s=10.2, min_speed_cm_s=1.0)

    assert result["valid"] is True
    assert result["reason"] == "ok"
    assert result["velocity_cm_s"] == [100.0, 50.0]
    assert result["field_pos"] == [60.0, 30.0]


def test_ball_landing_predictor_rejects_stale_history():
    predictor = BallLandingPredictor(velocity_alpha=1.0)
    predictor.observe([0.0, 0.0], timestamp_s=1.0)
    predictor.observe([10.0, 0.0], timestamp_s=1.1)

    result = predictor.predict(now_s=2.0, max_age_s=0.3)

    assert result["valid"] is False
    assert result["reason"] == "stale"
    assert result["field_pos"] == BALL_NOT_FOUND


def test_ball_landing_predictor_clamps_to_field_bounds():
    predictor = BallLandingPredictor(velocity_alpha=1.0)
    predictor.observe([120.0, 0.0], timestamp_s=1.0)
    predictor.observe([130.0, 0.0], timestamp_s=1.1)

    result = predictor.predict(
        horizon_s=0.5,
        now_s=1.1,
        min_speed_cm_s=1.0,
        field_long_cm=260.0,
        field_short_cm=200.0,
    )

    assert result["valid"] is True
    assert result["field_pos"] == [130.0, 0.0]
    assert result["clamped"] is True


def test_positions_predict_ball_landing_uses_field_observations(monkeypatch):
    positions = Positions()
    samples = iter([[0.0, 0.0], [10.0, 0.0]])
    times = iter([10.0, 10.0, 10.1, 10.1])

    monkeypatch.setattr(positions, "FieldBallPos", lambda: next(samples))
    monkeypatch.setattr(positions_module.time, "monotonic", lambda: next(times))

    first = positions.PredictBallLanding(horizon_s=0.5, min_speed_cm_s=1.0)
    second = positions.PredictBallLanding(horizon_s=0.5, min_speed_cm_s=1.0)

    assert first["reason"] == "warming_up"
    assert second["valid"] is True
    assert second["reason"] == "ok"
    assert second["field_pos"] == [60.0, 0.0]
