"""Badcase detection for teacher traces."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

from mlp.contracts import IntentId


@dataclass
class Badcase:
    kind: str
    severity: str
    message: str
    index: int
    details: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        return {
            "kind": self.kind,
            "severity": self.severity,
            "message": self.message,
            "index": self.index,
            "details": self.details,
        }


class TeacherBadcaseDetector:
    def __init__(
        self,
        *,
        stuck_window_s: float = 3.0,
        stuck_min_move_cm: float = 4.0,
        loop_window: int = 18,
        loop_min_switches: int = 8,
    ):
        self.stuck_window_s = float(stuck_window_s)
        self.stuck_min_move_cm = float(stuck_min_move_cm)
        self.loop_window = int(loop_window)
        self.loop_min_switches = int(loop_min_switches)

    def analyze(self, rows: list[dict[str, Any]]) -> list[Badcase]:
        issues: list[Badcase] = []
        issues.extend(self._illegal_target(rows))
        issues.extend(self._search_visible_ball(rows))
        issues.extend(self._state_loop(rows))
        issues.extend(self._stuck(rows))
        return issues

    def summarize(self, rows: list[dict[str, Any]], issues: list[Badcase]) -> dict[str, Any]:
        states: dict[str, int] = {}
        for row in rows:
            state = str(row.get("decision", {}).get("state", ""))
            states[state] = states.get(state, 0) + 1
        return {
            "frames": len(rows),
            "states": states,
            "badcase_count": len(issues),
            "badcases": [issue.to_dict() for issue in issues],
        }

    def _illegal_target(self, rows: list[dict[str, Any]]) -> list[Badcase]:
        issues: list[Badcase] = []
        for idx, row in enumerate(rows):
            decision = row.get("decision", {})
            safety = decision.get("safety", {})
            packet = decision.get("packet", {})
            if safety.get("target_legal", True) is False:
                issues.append(
                    Badcase(
                        "illegal_target",
                        "high",
                        "teacher produced a target inside a restricted/boundary region",
                        idx,
                        {"packet": packet, "safety": safety},
                    )
                )
            if safety.get("path_legal", True) is False:
                issues.append(
                    Badcase(
                        "illegal_path",
                        "medium",
                        "teacher target is legal but the straight path crosses a restricted region",
                        idx,
                        {"packet": packet, "safety": safety},
                    )
                )
        return issues

    def _search_visible_ball(self, rows: list[dict[str, Any]]) -> list[Badcase]:
        issues: list[Badcase] = []
        for idx, row in enumerate(rows):
            obs = row.get("obs", {})
            packet = row.get("decision", {}).get("packet", {})
            if bool(obs.get("ball_visible", False)) and int(packet.get("intent_id", -1)) == int(IntentId.SEARCH_BALL):
                issues.append(
                    Badcase(
                        "search_with_visible_ball",
                        "medium",
                        "teacher searched even though the ball was visible",
                        idx,
                        {"ball": [obs.get("ball_rel_x"), obs.get("ball_rel_y")], "packet": packet},
                    )
                )
        return issues

    def _state_loop(self, rows: list[dict[str, Any]]) -> list[Badcase]:
        if len(rows) < self.loop_window:
            return []
        recent = rows[-self.loop_window :]
        states = [str(row.get("decision", {}).get("state", "")) for row in recent]
        switches = sum(1 for a, b in zip(states, states[1:]) if a != b)
        unique = set(states)
        if switches >= self.loop_min_switches and len(unique) <= 3:
            return [
                Badcase(
                    "state_loop",
                    "high",
                    "teacher appears to oscillate between a small number of states",
                    len(rows) - 1,
                    {"states": states, "switches": switches},
                )
            ]
        return []

    def _stuck(self, rows: list[dict[str, Any]]) -> list[Badcase]:
        if len(rows) < 2:
            return []
        end_t = float(rows[-1].get("runtime", {}).get("t_s", rows[-1].get("t_wall", 0.0)))
        start_idx = None
        for idx in range(len(rows) - 1, -1, -1):
            t = float(rows[idx].get("runtime", {}).get("t_s", rows[idx].get("t_wall", 0.0)))
            if end_t - t >= self.stuck_window_s:
                start_idx = idx
                break
        if start_idx is None:
            return []

        start = rows[start_idx].get("obs", {})
        end = rows[-1].get("obs", {})
        dx = float(end.get("self_x", 0.0)) - float(start.get("self_x", 0.0))
        dy = float(end.get("self_y", 0.0)) - float(start.get("self_y", 0.0))
        moved = math.hypot(dx, dy)
        moving_frames = 0
        for row in rows[start_idx:]:
            intent = int(row.get("decision", {}).get("packet", {}).get("intent_id", -1))
            if intent in (int(IntentId.AVOID_WALL), int(IntentId.CARRY_TO_POSE), int(IntentId.APPROACH_BALL)):
                moving_frames += 1
        if moved < self.stuck_min_move_cm and moving_frames >= max(3, (len(rows) - start_idx) // 2):
            return [
                Badcase(
                    "stuck",
                    "high",
                    "teacher requested motion but pose barely changed",
                    len(rows) - 1,
                    {"moved_cm": moved, "window_s": self.stuck_window_s, "moving_frames": moving_frames},
                )
            ]
        return []
