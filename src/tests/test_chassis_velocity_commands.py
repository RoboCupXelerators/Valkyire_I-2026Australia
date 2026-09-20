"""Command semantics for the public chassis velocity APIs."""

from types import SimpleNamespace

from LunaPre.motion.chassis import Car


def _car():
    outputs = []
    car = Car(lambda *values: outputs.append(values), lambda: 0.0)
    cfg = SimpleNamespace(
        Ports=SimpleNamespace(
            LeftFront=1,
            LeftBack=2,
            RightFront=3,
            RightBack=4,
        ),
        Chassis=SimpleNamespace(
            WheelRadiusM=0.03,
            RobotRadiusM=0.10,
            MaxWheelRPM=1000,
        ),
        Motion=SimpleNamespace(
            NormalizeWheelOutput=True,
            EnableCoupledWheelAllocator=True,
            EnableTurnTranslationDerate=False,
            EnableWheelSlewLimit=True,
        ),
        Debug=SimpleNamespace(FullLog=False),
    )
    car.cfg = cfg
    car._motion_backend.cfg = cfg
    return car, outputs


def test_robot_move_vector_is_an_immediate_velocity_command_and_zero_brakes():
    car, outputs = _car()
    backend = car._motion_backend

    # Simulate a prior traction-controlled command.  It must neither limit the
    # next requested angular speed nor keep the motors moving after zero.
    backend._filtered_wz = 90.0
    backend._last_motor_output = [80.0, 80.0, -80.0, -80.0]

    assert car.RobotMoveVector(0.0, 0.0, 360.0) is True
    assert outputs[-1] == (-200, -200, 200, 200)
    assert backend._filtered_wz == 0.0
    assert backend._last_motor_output == [-200.0, -200.0, 200.0, 200.0]

    assert car.RobotMoveVector(0.0, 0.0, 0.0) is True
    assert outputs[-1] == (0, 0, 0, 0)
    assert backend._last_motor_output == [0.0, 0.0, 0.0, 0.0]


def test_robot_move_twist_zero_uses_the_same_immediate_brake_path():
    car, outputs = _car()
    car._motion_backend._last_motor_output = [80.0, 80.0, -80.0, -80.0]

    assert car.RobotMoveTwist(0.0, 0.0, 0.0) is True
    assert outputs[-1] == (0, 0, 0, 0)
