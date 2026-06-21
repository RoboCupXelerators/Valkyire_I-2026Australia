"""Optional recording wrapper for LunaPre Positions."""

from __future__ import annotations

import csv
import time
from pathlib import Path

from ReasonData import logger, Settings

from .positions import Positions
from . import runtime

cfg = runtime.cfg


class RecordingPositions(Positions):
    """Positions with Pos2Pos recording kept outside the core class."""

    def __init__(self):
        super().__init__()
        self._pos2pos_header_written: set[str] = set()

    def Pos2Pos(self, AimPos: list, speed_gain: int | None = None) -> bool:
        before_pose = self._safe_pose()
        start_t = time.time()
        reached = super().Pos2Pos(AimPos, speed_gain)
        after_pose = self._safe_pose()
        self._record_pos2pos_call(
            aim_pos=AimPos,
            before_pose=before_pose,
            after_pose=after_pose,
            speed_gain=speed_gain,
            reached=bool(reached),
            duration_s=time.time() - start_t,
        )
        return reached

    def _safe_pose(self):
        try:
            return self.FieldRobotPose()
        except Exception:
            return [0xFFF, 0xFFF, 0xFFF]

    def _record_pos2pos_call(
        self,
        *,
        aim_pos: list,
        before_pose: list,
        after_pose: list,
        speed_gain: int | float | None,
        reached: bool,
        duration_s: float,
    ) -> None:
        if not self._pos2pos_cfg.get("record_enabled", False):
            return

        try:
            aim_x, aim_y, aim_z = self._normalize_aim(aim_pos)
            loc_x, loc_y, loc_z = self._normalize_pose(before_pose)
            after_x, after_y, after_z = self._normalize_pose(after_pose)
            err_x = aim_x - loc_x
            err_y = aim_y - loc_y
            err_z = self._angle_error_deg(loc_z, aim_z)
            record_dir = self._record_dir()
            record_dir.mkdir(parents=True, exist_ok=True)
            session_id = getattr(cfg.Debug, "RecordSessionId", None) or time.strftime("%Y%m%d_%H%M%S", time.localtime())
            csv_path = record_dir / f"pos2pos_{session_id}.csv"
            fieldnames = [
                "tick", "robot_id", "mode", "reached",
                "aim_x", "aim_y", "aim_z",
                "loc_x", "loc_y", "loc_z",
                "err_x", "err_y", "err_z",
                "cmd_x", "cmd_y", "speed_gain",
                "pid_kp", "pid_ki", "pid_kd", "pid_limit",
                "after_x", "after_y", "after_z", "duration_s",
            ]
            need_header = str(csv_path) not in self._pos2pos_header_written or not csv_path.exists()
            with csv_path.open("a", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                if need_header:
                    writer.writeheader()
                    self._pos2pos_header_written.add(str(csv_path))
                writer.writerow({
                    "tick": round(time.time(), 6),
                    "robot_id": int(getattr(cfg.RoboInfo, "number", 1)),
                    "mode": "pos2pos_call",
                    "reached": int(reached),
                    "aim_x": round(aim_x, 4),
                    "aim_y": round(aim_y, 4),
                    "aim_z": round(aim_z, 4),
                    "loc_x": round(loc_x, 4),
                    "loc_y": round(loc_y, 4),
                    "loc_z": round(loc_z, 4),
                    "err_x": round(err_x, 4),
                    "err_y": round(err_y, 4),
                    "err_z": round(err_z, 4),
                    "cmd_x": 0.0,
                    "cmd_y": 0.0,
                    "speed_gain": round(float(speed_gain if speed_gain is not None else 1.0), 4),
                    "pid_kp": self._pos2pos_cfg["kp"],
                    "pid_ki": self._pos2pos_cfg["ki"],
                    "pid_kd": self._pos2pos_cfg["kd"],
                    "pid_limit": self._pos2pos_cfg["output_limit"],
                    "after_x": round(after_x, 4),
                    "after_y": round(after_y, 4),
                    "after_z": round(after_z, 4),
                    "duration_s": round(float(duration_s), 6),
                })
        except Exception as e:
            if Settings.Debug.FullLog:
                logger.debug(f"Pos2Pos record failed: {e}")

    def _record_dir(self) -> Path:
        record_dir = Path(self._pos2pos_cfg["record_dir"]).expanduser()
        if not record_dir.is_absolute():
            repo_root = Path(__file__).resolve().parents[2]
            record_dir = repo_root / record_dir
        return record_dir

    @staticmethod
    def _normalize_aim(aim_pos: list) -> tuple[float, float, float]:
        return float(aim_pos[0]), float(aim_pos[1]), float(aim_pos[2]) % 360.0

    @staticmethod
    def _normalize_pose(pose: list) -> tuple[float, float, float]:
        if not pose or len(pose) < 3 or list(pose[:3]) == [0xFFF, 0xFFF, 0xFFF]:
            return float(0xFFF), float(0xFFF), float(0xFFF)
        return float(pose[0]), float(pose[1]), float(pose[2])
