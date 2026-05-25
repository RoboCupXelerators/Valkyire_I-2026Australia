"""Offline checks for chassis moving-while-turning control.

Run from Software:
    python src/tests/chassis_turning/offline_chassis_allocator_check.py
"""

from __future__ import annotations

import sys
from pathlib import Path

SRC_DIR = Path(__file__).resolve().parents[2]
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from chassis import Car  # noqa: E402
from ReasonData.config import Settings  # noqa: E402


class MotorCapture:
    def __init__(self) -> None:
        self.calls: list[tuple[int, int, int, int]] = []

    def __call__(self, a: int, b: int, c: int, d: int) -> None:
        self.calls.append((a, b, c, d))

    @property
    def last(self) -> tuple[int, int, int, int]:
        if not self.calls:
            raise AssertionError("motor was never called")
        return self.calls[-1]


def _set_motion_defaults() -> None:
    Settings.Motion.MaxSpeed = 500
    Settings.Motion.NormalizeWheelOutput = True
    Settings.Motion.EnableTractionControl = False
    Settings.Motion.EnableWheelSlewLimit = False
    Settings.Motion.EnableCoupledWheelAllocator = True
    Settings.Motion.WheelAllocatorSaturationMode = "preserve_twist"
    Settings.Motion.EnableTurnTranslationDerate = True
    Settings.Motion.TurnTranslationScaleWz = 120.0
    Settings.Motion.TurnTranslationMinScale = 0.6
    Settings.Debug.Beta = True


def _max_abs(values) -> int:
    return max(abs(v) for v in values)


def make_car(yaw_getter=lambda: 0.0) -> tuple[Car, MotorCapture]:
    capture = MotorCapture()
    car = Car(capture, GetYaw=yaw_getter, permission=lambda: True)
    return car, capture


def check_yaw_wrap_pid_uses_shortest_path() -> None:
    _set_motion_defaults()
    car, capture = make_car(lambda: -179.0)

    car.FieldMoveVector(0, 0, 179.0)
    out = capture.last
    car.stop()

    assert _max_abs(out) < 25, (
        "yaw PID should command a tiny shortest-path turn at -179 -> 179; "
        f"got {out}"
    )
    assert car._angle_error_deg(-179.0, 179.0) == -2.0


def check_allocator_derates_translation_while_turning() -> None:
    _set_motion_defaults()
    car, _ = make_car()
    Settings.Motion.TurnTranslationScaleWz = 100.0
    Settings.Motion.TurnTranslationMinScale = 0.5

    planar = [300.0, 300.0, 300.0, 300.0]
    turn = [-100.0, -100.0, 100.0, 100.0]
    allocated = car._allocate_wheel_mix(planar, turn, speed_z=100.0)
    car.stop()

    assert allocated == [50.0, 50.0, 250.0, 250.0], allocated


def check_allocator_respects_max_speed() -> None:
    _set_motion_defaults()
    car, capture = make_car()

    car.RobotMoveVector(260, 220, 180)
    out = capture.last
    car.stop()

    assert _max_abs(out) <= int(Settings.Motion.MaxSpeed), out


def check_allocator_modes() -> None:
    _set_motion_defaults()
    car, _ = make_car()
    Settings.Motion.EnableTurnTranslationDerate = False
    planar = [650.0, 650.0, 650.0, 650.0]
    turn = [-250.0, -250.0, 250.0, 250.0]

    Settings.Motion.WheelAllocatorSaturationMode = "preserve_twist"
    preserve = car._allocate_wheel_mix(planar, turn, speed_z=250.0)
    assert max(abs(v) for v in preserve) <= 500.0, preserve

    Settings.Motion.WheelAllocatorSaturationMode = "translation_first"
    translation_first = car._allocate_wheel_mix(planar, turn, speed_z=250.0)
    assert max(abs(v) for v in translation_first) <= 500.0, translation_first
    assert translation_first[0] > preserve[0], (translation_first, preserve)

    Settings.Motion.WheelAllocatorSaturationMode = "turn_first"
    turn_first = car._allocate_wheel_mix(planar, turn, speed_z=250.0)
    assert max(abs(v) for v in turn_first) <= 500.0, turn_first
    assert turn_first[2] - turn_first[0] >= translation_first[2] - translation_first[0], (
        turn_first,
        translation_first,
    )
    car.stop()


def main() -> int:
    checks = [
        check_yaw_wrap_pid_uses_shortest_path,
        check_allocator_derates_translation_while_turning,
        check_allocator_respects_max_speed,
        check_allocator_modes,
    ]
    for check in checks:
        check()
        print(f"PASS {check.__name__}")
    print("All chassis allocator offline checks passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
