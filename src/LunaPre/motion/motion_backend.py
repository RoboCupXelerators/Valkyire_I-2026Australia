import math
import time


class MotionBackend:
    """Wheel allocation, filtering, and output limiting for chassis commands."""

    def __init__(self, cfg):
        self.cfg = cfg
        self._last_cmd_t = time.monotonic()
        self._filtered_vx = 0.0
        self._filtered_vy = 0.0
        self._filtered_wz = 0.0
        self._last_motor_t = time.monotonic()
        self._last_motor_output = [0.0, 0.0, 0.0, 0.0]

    def _motion_bool(self, name, default):
        return bool(getattr(self.cfg.Motion, name, default))

    def _motion_float(self, name, default):
        try:
            return float(getattr(self.cfg.Motion, name, default))
        except Exception:
            return float(default)

    def _chassis_float(self, name, default):
        try:
            return float(getattr(self.cfg.Chassis, name, default))
        except Exception:
            return float(default)

    def _motion_str(self, name, default):
        try:
            return str(getattr(self.cfg.Motion, name, default))
        except Exception:
            return str(default)

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))

    @staticmethod
    def angle_error_deg(current, target):
        return (float(target) - float(current) + 180.0) % 360.0 - 180.0

    @staticmethod
    def max_abs(values):
        return max((abs(v) for v in values), default=0.0)

    @staticmethod
    def slew_value(current, target, max_delta):
        if target > current + max_delta:
            return current + max_delta
        if target < current - max_delta:
            return current - max_delta
        return target

    def reset_command_filter(self):
        self._filtered_vx = 0.0
        self._filtered_vy = 0.0
        self._filtered_wz = 0.0
        self._last_cmd_t = time.monotonic()

    def reset_motor_output(self):
        self._last_motor_output = [0.0, 0.0, 0.0, 0.0]
        self._last_motor_t = time.monotonic()

    def clamp_motor_vector(self, values, *, bypass_speed_limit: bool = False):
        """Apply the configured wheel-RPM cap unless a scoped debug caller opts out."""
        if bypass_speed_limit:
            return [float(v) for v in values]
        max_speed = abs(self._chassis_float("MaxWheelRPM", 500))
        if max_speed <= 0:
            return [0.0, 0.0, 0.0, 0.0]

        out = [float(v) for v in values]
        if self._motion_bool("NormalizeWheelOutput", True):
            peak = max(abs(v) for v in out) if out else 0.0
            if peak > max_speed:
                scale = max_speed / peak
                out = [v * scale for v in out]
        else:
            out = [max(-max_speed, min(max_speed, v)) for v in out]
        return out

    def apply_wheel_slew_limit(self, values, immediate=False):
        if immediate or not self._motion_bool("EnableWheelSlewLimit", True):
            self._last_motor_output = [float(v) for v in values]
            self._last_motor_t = time.monotonic()
            return values

        now = time.monotonic()
        dt = max(0.0, min(self._motion_float("TractionDtMax", 0.08), now - self._last_motor_t))
        if dt <= 1e-4:
            dt = 0.02
        max_delta = max(0.0, self._motion_float("TractionWheelSlewRate", 1800.0)) * dt
        limited = [
            self.slew_value(cur, target, max_delta)
            for cur, target in zip(self._last_motor_output, values)
        ]
        self._last_motor_output = [float(v) for v in limited]
        self._last_motor_t = now
        return limited

    def filter_chassis_command(self, speed_x, speed_y, speed_z):
        if not self._motion_bool("EnableTractionControl", True):
            self._filtered_vx = float(speed_x)
            self._filtered_vy = float(speed_y)
            self._filtered_wz = float(speed_z)
            self._last_cmd_t = time.monotonic()
            return float(speed_x), float(speed_y), float(speed_z)

        now = time.monotonic()
        dt = max(0.0, min(self._motion_float("TractionDtMax", 0.08), now - self._last_cmd_t))
        if dt <= 1e-4:
            dt = 0.02
        self._last_cmd_t = now

        target_x = float(speed_x)
        target_y = float(speed_y)
        dx = target_x - self._filtered_vx
        dy = target_y - self._filtered_vy
        delta_norm = math.hypot(dx, dy)
        max_planar_delta = max(0.0, self._motion_float("TractionPlanarAccelLimit", 3.0)) * dt
        if delta_norm > max_planar_delta > 0.0:
            scale = max_planar_delta / delta_norm
            dx *= scale
            dy *= scale
        self._filtered_vx += dx
        self._filtered_vy += dy

        max_w_delta = max(0.0, self._motion_float("TractionAngularAccelLimit", 450.0)) * dt
        self._filtered_wz = self.slew_value(self._filtered_wz, float(speed_z), max_w_delta)
        return self._filtered_vx, self._filtered_vy, self._filtered_wz

    def turn_translation_scale(self, speed_z):
        if not self._motion_bool("EnableTurnTranslationDerate", True):
            return 1.0

        threshold = abs(self._motion_float("TurnTranslationScaleWz", 120.0))
        if threshold <= 1e-6:
            return 1.0

        min_scale = self.clamp(self._motion_float("TurnTranslationMinScale", 0.6), 0.0, 1.0)
        load = self.clamp(abs(float(speed_z)) / threshold, 0.0, 1.0)
        return 1.0 - load * (1.0 - min_scale)

    def max_scale_to_fit(self, base, scaled, max_speed):
        """Return the largest 0..1 scale for scaled so base + scaled*alpha fits."""
        upper = 1.0
        for base_v, scaled_v in zip(base, scaled):
            base_v = float(base_v)
            scaled_v = float(scaled_v)
            if abs(scaled_v) <= 1e-9:
                if abs(base_v) > max_speed:
                    return 0.0
                continue
            if scaled_v > 0:
                upper = min(upper, (max_speed - base_v) / scaled_v)
            else:
                upper = min(upper, (-max_speed - base_v) / scaled_v)
        return self.clamp(upper, 0.0, 1.0)

    def allocate_wheel_mix(
        self, planar, turn, speed_z, *, bypass_speed_limit: bool = False
    ):
        if bypass_speed_limit:
            # A caller has explicitly limited this to a guarded debug motion.
            # Preserve its mix rather than scaling against competition RPM cap.
            return [float(p) + float(t) for p, t in zip(planar, turn)]
        if not self._motion_bool("EnableCoupledWheelAllocator", True):
            return [p + t for p, t in zip(planar, turn)]

        max_speed = abs(self._chassis_float("MaxWheelRPM", 500))
        if max_speed <= 0:
            return [0.0, 0.0, 0.0, 0.0]

        planar_scale = self.turn_translation_scale(speed_z)
        planar = [float(v) * planar_scale for v in planar]
        turn = [float(v) for v in turn]
        combined = [p + t for p, t in zip(planar, turn)]
        peak = self.max_abs(combined)
        if peak <= max_speed:
            return combined

        mode = self._motion_str("WheelAllocatorSaturationMode", "preserve_twist").strip().lower()
        if mode in {"turn_first", "rotation_first"}:
            turn_peak = self.max_abs(turn)
            if turn_peak > max_speed:
                scale = max_speed / turn_peak
                turn = [v * scale for v in turn]
            planar_fit = self.max_scale_to_fit(turn, planar, max_speed)
            return [turn_v + planar_v * planar_fit for turn_v, planar_v in zip(turn, planar)]

        if mode in {"translation_first", "planar_first"}:
            planar_peak = self.max_abs(planar)
            if planar_peak > max_speed:
                scale = max_speed / planar_peak
                planar = [v * scale for v in planar]
            turn_fit = self.max_scale_to_fit(planar, turn, max_speed)
            return [planar_v + turn_v * turn_fit for planar_v, turn_v in zip(planar, turn)]

        scale = max_speed / peak
        return [v * scale for v in combined]
