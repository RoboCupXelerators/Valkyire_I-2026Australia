"""Conservative current-and-vision dribbler control.

The lower board exposes the dribbler current as a slow telemetry channel.  It
is useful only when it is interpreted alongside the camera-derived ball state:
a higher current alone may be a blocked roller, and a held-looking ball alone
can be a short visual occlusion.  This module keeps that policy independent of
the serial and vision implementations so it can be unit-tested on a desktop.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time


VISION_LOST = "lost"
VISION_APPROACHING = "approaching"
VISION_NEAR = "near"
VISION_HELD = "held"
_KNOWN_VISION_STATES = frozenset(
    {VISION_LOST, VISION_APPROACHING, VISION_NEAR, VISION_HELD}
)

# Source-owned robot1 capture profile.  The prior 0.90 ceiling is too strong
# for close manual lockball work: it can push a ball sideways before the roller
# has centred it.  Keep the baseline at 0.50 and use a small *lateral-only*
# assist while the ball is still visually free.
ADAPTIVE_DRIBBLE_CAPTURE_BASE_POWER = 0.50
ADAPTIVE_DRIBBLE_CAPTURE_MAX_POWER = 0.65
ADAPTIVE_DRIBBLE_HOLD_POWER = 0.55
ADAPTIVE_DRIBBLE_RELIEF_POWER = 0.45
ADAPTIVE_DRIBBLE_CAPTURE_STEP = 0.05
ADAPTIVE_DRIBBLE_LATERAL_BOOST_MAX = 0.10
ADAPTIVE_DRIBBLE_LATERAL_FULL_ERROR_CM = 12.0


@dataclass(frozen=True)
class DribbleAdaptiveDecision:
    """One evaluated dribbler command, retained for telemetry/debugging."""

    power: float
    mode: str
    vision_state: str | None
    current_a: float | None


class DribbleAdaptiveController:
    """Use a validated visual state to interpret the dribbler-current signal.

    ``loaded_current_min_a`` is deliberately a *load* threshold rather than a
    possession detector.  Vision decides whether the ball is approaching or
    held; current merely confirms that the roller has enough traction.  With
    unavailable or non-finite current telemetry, the caller's original command
    is preserved instead of making a vision-only power change.
    """

    def __init__(
        self,
        *,
        enabled: bool = False,
        search_power: float = 0.20,
        capture_power_start: float = ADAPTIVE_DRIBBLE_CAPTURE_BASE_POWER,
        capture_power: float = ADAPTIVE_DRIBBLE_CAPTURE_MAX_POWER,
        hold_power: float = ADAPTIVE_DRIBBLE_HOLD_POWER,
        relief_power: float = ADAPTIVE_DRIBBLE_RELIEF_POWER,
        loaded_current_min_a: float = 1.65,
        excess_current_a: float = 2.45,
        capture_power_step: float = ADAPTIVE_DRIBBLE_CAPTURE_STEP,
        capture_power_step_interval_s: float = 1.10,
        lateral_boost_max: float = ADAPTIVE_DRIBBLE_LATERAL_BOOST_MAX,
        lateral_boost_full_error_cm: float = ADAPTIVE_DRIBBLE_LATERAL_FULL_ERROR_CM,
        min_update_interval_s: float = 0.25,
    ):
        self.enabled = bool(enabled)
        self.search_power = self._power(search_power)
        self.capture_power_start = self._power(capture_power_start)
        self.capture_power = self._power(capture_power)
        self.capture_power_start = min(self.capture_power_start, self.capture_power)
        self.hold_power = self._power(hold_power)
        self.relief_power = self._power(relief_power)
        self.loaded_current_min_a = self._positive(loaded_current_min_a, 1.65)
        self.excess_current_a = max(
            self.loaded_current_min_a, self._positive(excess_current_a, 2.45)
        )
        self.capture_power_step = max(0.0, self._power(capture_power_step))
        self.capture_power_step_interval_s = self._positive(
            capture_power_step_interval_s, 1.10
        )
        self.lateral_boost_max = self._power(lateral_boost_max)
        self.lateral_boost_full_error_cm = self._positive(
            lateral_boost_full_error_cm, ADAPTIVE_DRIBBLE_LATERAL_FULL_ERROR_CM
        )
        self.min_update_interval_s = max(0.0, float(min_update_interval_s))
        self._last_decision: DribbleAdaptiveDecision | None = None
        self._last_update_s = 0.0
        self._capture_started_s: float | None = None

    @staticmethod
    def _power(value) -> float:
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            return 0.0
        if not math.isfinite(numeric):
            return 0.0
        return max(0.0, min(1.0, numeric))

    @staticmethod
    def _positive(value, default: float) -> float:
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            return default
        return numeric if math.isfinite(numeric) and numeric > 0.0 else default

    @staticmethod
    def _current(value) -> float | None:
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            return None
        return numeric if math.isfinite(numeric) and numeric >= 0.0 else None

    @staticmethod
    def _vision_state(value) -> str | None:
        state = str(value).strip().lower() if value is not None else None
        return state if state in _KNOWN_VISION_STATES else None

    def reset(self) -> None:
        """Forget a previous hold/relief decision after output is switched off."""

        self._last_decision = None
        self._last_update_s = 0.0
        self._capture_started_s = None

    @property
    def last_decision(self) -> DribbleAdaptiveDecision | None:
        return self._last_decision

    def _reset_capture_ramp(self) -> None:
        self._capture_started_s = None

    def _capture_ramp_power(self, now_s: float) -> float:
        """Start softly, then raise roller power at bounded discrete steps."""

        if self._capture_started_s is None:
            self._capture_started_s = now_s
        elapsed_s = max(0.0, now_s - self._capture_started_s)
        steps = int(elapsed_s / self.capture_power_step_interval_s)
        return min(
            self.capture_power,
            self.capture_power_start + steps * self.capture_power_step,
        )

    def _lateral_boost(self, lateral_error_cm) -> float:
        """Return a small centred-capture assist from signed camera Y error."""

        try:
            error_cm = abs(float(lateral_error_cm))
        except (TypeError, ValueError):
            return 0.0
        if not math.isfinite(error_cm) or self.lateral_boost_max <= 0.0:
            return 0.0
        return self.lateral_boost_max * min(
            1.0, error_cm / self.lateral_boost_full_error_cm
        )

    def decide(
        self,
        requested_power,
        *,
        vision_state=None,
        current_a=None,
        lateral_error_cm=None,
        now_s: float | None = None,
    ) -> DribbleAdaptiveDecision:
        """Return a safe power target without commanding hardware directly."""

        requested = self._power(requested_power)
        state = self._vision_state(vision_state)
        current = self._current(current_a)
        now = time.monotonic() if now_s is None else float(now_s)

        if requested <= 0.0:
            decision = DribbleAdaptiveDecision(0.0, "off", state, current)
        elif not self.enabled:
            decision = DribbleAdaptiveDecision(requested, "disabled", state, current)
        elif state not in _KNOWN_VISION_STATES:
            decision = DribbleAdaptiveDecision(requested, "vision_unavailable", state, current)
        elif current is None:
            # Telemetry may be absent during startup/reconnect.  Never raise
            # roller power based on vision alone in that state.
            decision = DribbleAdaptiveDecision(requested, "current_unavailable", state, None)
        elif state == VISION_LOST:
            self._reset_capture_ramp()
            decision = DribbleAdaptiveDecision(
                min(requested, self.search_power), "search", state, current
            )
        elif state == VISION_APPROACHING:
            # Far from the capture point, preserve the existing low-power
            # command.  Raising roller power here tends to kick the ball away
            # before the docking velocity limiter can settle it.
            self._reset_capture_ramp()
            decision = DribbleAdaptiveDecision(
                requested, "approach_low", state, current
            )
        elif state == VISION_NEAR:
            # A centred ball starts at the gentle 0.50 base.  A sideways ball
            # needs a little more roller authority to pull itself into the
            # throat, but never exceeds the low 0.65 capture ceiling.
            ramp_power = min(
                self.capture_power,
                self._capture_ramp_power(now) + self._lateral_boost(lateral_error_cm),
            )
            if current >= self.excess_current_a:
                decision = DribbleAdaptiveDecision(
                    self.relief_power, "excess_load_relief", state, current
                )
            else:
                decision = DribbleAdaptiveDecision(
                    ramp_power, "near_ramping", state, current
                )
        elif current >= self.excess_current_a:
            # This is a relief command, not a claim that the motor has stalled:
            # an actual stall limit must be established in a separate guarded
            # mechanical test before adding a hard cutoff.
            decision = DribbleAdaptiveDecision(
                self.relief_power, "excess_load_relief", state, current
            )
        elif current < self.loaded_current_min_a:
            decision = DribbleAdaptiveDecision(
                self._capture_ramp_power(now), "held_ramping", state, current
            )
        else:
            decision = DribbleAdaptiveDecision(
                min(self.hold_power, self._capture_ramp_power(now)),
                "held",
                state,
                current,
            )

        previous = self._last_decision
        if (
            previous is not None
            and now - self._last_update_s < self.min_update_interval_s
            and decision.power != previous.power
            and decision.power > previous.power
            and decision.mode not in {"near_ramping", "held_ramping"}
        ):
            # Avoid increasing output on a transient state change.  Capture and
            # low-load recovery are the two explicit, vision-gated exceptions.
            decision = DribbleAdaptiveDecision(
                previous.power, "rate_limited", state, current
            )

        if previous is None or decision.power != previous.power:
            self._last_update_s = now
        self._last_decision = decision
        return decision
