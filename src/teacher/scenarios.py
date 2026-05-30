"""Reusable teacher scenarios for offline and Unity regression runs."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class TeacherScenario:
    name: str
    robot: list[float]
    ball: list[float]
    duration_s: float = 4.0
    has_ball: bool = False
    enemies: list[list[float]] = field(default_factory=list)
    expected_states: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "TeacherScenario":
        return cls(
            name=str(data.get("name", "scenario")),
            robot=[float(v) for v in data.get("robot", [0.0, 0.0, 0.0])[:3]],
            ball=[float(v) for v in data.get("ball", [0.0, 0.0])[:2]],
            duration_s=float(data.get("duration_s", 4.0)),
            has_ball=bool(data.get("has_ball", False)),
            enemies=[[float(v) for v in enemy[:2]] for enemy in data.get("enemies", [])],
            expected_states=[str(v) for v in data.get("expected_states", [])],
        )


def default_teacher_scenarios() -> list[TeacherScenario]:
    """Small scenario set that exposes common teacher failures quickly."""
    return [
        TeacherScenario(
            name="recover_from_left_wall",
            robot=[0.0, 68.0, 0.0],
            ball=[-55.0, -45.0],
            duration_s=4.0,
            expected_states=["avoid_wall"],
        ),
        TeacherScenario(
            name="approach_center_ball",
            robot=[-55.0, 0.0, 0.0],
            ball=[-20.0, 0.0],
            duration_s=4.0,
            expected_states=["approach_ball"],
        ),
        TeacherScenario(
            name="approach_side_ball",
            robot=[-60.0, -20.0, 0.0],
            ball=[-38.0, 35.0],
            duration_s=4.0,
            expected_states=["approach_ball"],
        ),
        TeacherScenario(
            name="carry_around_goal_box",
            robot=[10.0, 0.0, 0.0],
            ball=[14.0, 0.0],
            duration_s=3.0,
            has_ball=True,
            expected_states=["carry_to_lane"],
        ),
    ]


def load_teacher_scenarios(path: str | Path | None = None) -> list[TeacherScenario]:
    if path is None:
        return default_teacher_scenarios()
    src = Path(path)
    if not src.exists():
        return default_teacher_scenarios()
    data = json.loads(src.read_text(encoding="utf-8"))
    if isinstance(data, dict):
        data = data.get("scenarios", [])
    return [TeacherScenario.from_dict(item) for item in data]
