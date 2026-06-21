"""Optional recording wrapper for chassis.Car."""

from __future__ import annotations

from ReasonData import logger

from .chassis import Car
from .motion_recorder import get_motion_recorder


class RecordingCar(Car):
    """Car with motion recording attached as an external side-channel."""

    def __init__(self, *args, recorder=None, **kwargs):
        super().__init__(*args, **kwargs)
        self._recorder = recorder or get_motion_recorder()
        self.add_motor_output_hook(self._record_motor_output)
        self._check_recording_config()

    def _check_recording_config(self):
        """检查配置是否启用记录"""
        try:
            if getattr(self.cfg.Debug, "RecordMotion", False):
                session_id = getattr(self.cfg.Debug, "RecordSessionId", None)
                if not self._recorder.is_recording:
                    self._recorder.start_recording(session_id)
                    self.logger.info("[RecordingCar] Motion recording enabled from config")
        except Exception as e:
            self.logger.debug(f"[RecordingCar] Recording config check failed: {e}")

    def _record_motor_output(self, speed, car):
        self._recorder.record_output(
            motor1=speed[0],
            motor2=speed[1],
            motor3=speed[2],
            motor4=speed[3],
            vx=getattr(car, "_last_vx", 0),
            vy=getattr(car, "_last_vy", 0),
            wz=getattr(car, "_last_wz", 0),
            robot_id=getattr(self.cfg.RoboInfo, "number", 1),
        )
