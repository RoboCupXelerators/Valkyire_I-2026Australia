"""Trace recorder for teacher decisions."""

from __future__ import annotations

from collections import deque
import json
from pathlib import Path
import time
from typing import Any, Iterable

from mlp.contracts import DeployObservationV1

from .rule_teacher import TeacherDecision


class TeacherTraceRecorder:
    def __init__(self, path: str | Path | None = None, *, buffer_size: int = 600):
        self.path = Path(path) if path is not None else None
        self.buffer: deque[dict[str, Any]] = deque(maxlen=max(1, int(buffer_size)))
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)

    def record(
        self,
        *,
        obs: DeployObservationV1,
        decision: TeacherDecision,
        scenario: str = "",
        runtime: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        row = {
            "t_wall": time.time(),
            "scenario": scenario,
            "obs": obs.to_dict(),
            "decision": decision.to_dict(),
            "runtime": runtime or {},
        }
        self.buffer.append(row)
        if self.path is not None:
            with self.path.open("a", encoding="utf-8") as fh:
                fh.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n")
        return row

    def recent(self) -> list[dict[str, Any]]:
        return list(self.buffer)

    def write_recent(self, path: str | Path, rows: Iterable[dict[str, Any]] | None = None) -> Path:
        out = Path(path)
        out.parent.mkdir(parents=True, exist_ok=True)
        source = list(rows) if rows is not None else self.recent()
        with out.open("w", encoding="utf-8") as fh:
            for row in source:
                fh.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n")
        return out

    @staticmethod
    def load_jsonl(path: str | Path) -> list[dict[str, Any]]:
        rows: list[dict[str, Any]] = []
        with Path(path).open("r", encoding="utf-8") as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                rows.append(json.loads(line))
        return rows
