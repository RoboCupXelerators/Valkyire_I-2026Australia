import importlib
import sys


sys.path.insert(0, "src")
teleop = importlib.import_module("teleop")


class FakeBits:
    def __init__(self): self.calls = []
    def Arm(self): self.calls.append(("arm",))
    def SetChassisTwist(self, *twist): self.calls.append(("twist", *twist))
    def stop(self): self.calls.append(("stop",))
    def shutdown(self): self.calls.append(("shutdown",))


class Keys:
    def __init__(self, values): self.values = iter(values)
    def read(self, timeout):
        del timeout
        return next(self.values)


def test_teleop_mapping_arm_stop_and_shutdown():
    bits = FakeBits()
    teleop.run_teleop(bits, Keys(["r", "w", " ", "a", "\x1b"]), clock=lambda: 1.0)
    assert bits.calls == [
        ("arm",), ("twist", 0.02, 0.0, 0.0), ("stop",),
        ("twist", 0.0, 0.02, 0.0), ("shutdown",),
    ]


def test_deadman_timeout_stops_and_exception_still_shutdown():
    bits = FakeBits()
    times = iter([0.0, 0.2, 0.21])
    teleop.run_teleop(bits, Keys(["w", None, "\x1b"]), clock=lambda: next(times))
    assert ("stop",) in bits.calls and bits.calls[-1] == ("shutdown",)

    class BrokenKeys:
        def read(self, timeout): raise RuntimeError("input failed")
    bits = FakeBits()
    try:
        teleop.run_teleop(bits, BrokenKeys())
    except RuntimeError:
        pass
    assert bits.calls == [("shutdown",)]
