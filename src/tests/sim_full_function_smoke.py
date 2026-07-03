"""LunaPre 与 BasicFuc 的一次性模拟器 smoke 测试套件。

这个脚本有意不写成 pytest 用例。它用于连接正在运行的 Unity 模拟器，
同时用 fake 对象覆盖历史 BasicFuc 辅助函数接口，避免阻塞式策略函数
在夜间测试中把机器人一直卡住。

示例:
    PYTHONPATH=src python src/tests/sim_full_function_smoke.py --robot-id 1
"""

from __future__ import annotations

import argparse
import contextlib
import json
import os
import sys
import time
import traceback
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any, Callable

sys.dont_write_bytecode = True

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

INVALID_POSE = [0xFFF, 0xFFF, 0xFFF]
BALL_NOT_FOUND = [0xFFF, 0xFFF]
BALL_HELD = [0xDDD, 0xDDD]


@dataclass
class CaseResult:
    """单个测试用例的执行结果。"""

    name: str
    ok: bool
    duration_s: float
    details: Any = None
    error: str | None = None
    traceback: str | None = None


class SmokeRunner:
    """顺序执行 smoke 用例并收集结构化报告。"""

    def __init__(self):
        """初始化空结果列表。"""
        self.results: list[CaseResult] = []

    def run(self, name: str, func: Callable[[], Any]) -> None:
        """执行一个用例，捕获异常并记录耗时。"""
        start = time.time()
        try:
            details = func()
            self.results.append(CaseResult(name=name, ok=True, duration_s=time.time() - start, details=details))
            print(f"[PASS] {name}")
        except Exception as exc:
            self.results.append(
                CaseResult(
                    name=name,
                    ok=False,
                    duration_s=time.time() - start,
                    error=str(exc),
                    traceback=traceback.format_exc(),
                )
            )
            print(f"[FAIL] {name}: {exc}")

    @property
    def ok(self) -> bool:
        """所有用例通过时返回 ``True``。"""
        return all(item.ok for item in self.results)

    def report(self) -> dict[str, Any]:
        """生成可写入 JSON 的汇总报告。"""
        return {
            "ok": self.ok,
            "passed": sum(1 for item in self.results if item.ok),
            "failed": sum(1 for item in self.results if not item.ok),
            "results": [asdict(item) for item in self.results],
        }


@contextlib.contextmanager
def patched(target, **attrs):
    """临时替换对象属性，退出上下文后恢复。"""
    old_values = {}
    missing = object()
    for name, value in attrs.items():
        old_values[name] = getattr(target, name, missing)
        setattr(target, name, value)
    try:
        yield
    finally:
        for name, old_value in old_values.items():
            if old_value is missing:
                delattr(target, name)
            else:
                setattr(target, name, old_value)


class FakeChassis:
    """用于 BasicFuc 离线测试的底盘假对象。"""

    def __init__(self, yaw: float = 0.0):
        """初始化假底盘航向和调用记录。"""
        self.yaw = yaw
        self.calls: list[list[Any]] = []
        self.speedCache = [10, 10, 10, 10]

    def GetYaw(self):
        """返回当前假航向。"""
        return self.yaw

    def SetKp(self, value):
        """记录 SetKp 调用。"""
        self.calls.append(["SetKp", value])

    def Compass(self):
        """返回当前假指南针读数。"""
        return self.yaw

    def SetMotor(self, values, *args, **kwargs):
        """记录四轮电机直接控制调用。"""
        self.calls.append(["SetMotor", list(values), args, kwargs])

    def RobotMoveVector(self, *args):
        """记录机器人参考系速度向量调用。"""
        self.calls.append(["RobotMoveVector", args])

    def RobotMoveAngle(self, *args):
        """记录机器人参考系角度移动调用。"""
        self.calls.append(["RobotMoveAngle", args])

    def FieldMoveAngle(self, *args):
        """记录场地参考系角度移动调用。"""
        self.calls.append(["FieldMoveAngle", args])

    def FieldMoveVector(self, *args, **kwargs):
        """记录场地参考系速度向量调用。"""
        self.calls.append(["FieldMoveVector", args, kwargs])

    def RobotXMove(self, *args):
        """记录机器人 X 方向移动调用。"""
        self.calls.append(["RobotXMove", args])

    def RobotYMove(self, *args):
        """记录机器人 Y 方向移动调用。"""
        self.calls.append(["RobotYMove", args])

    def FieldTurn(self, *args):
        """记录场地航向转向调用。"""
        self.calls.append(["FieldTurn", args])

    def RobotTurn(self, *args):
        """记录机器人参考系转向调用。"""
        self.calls.append(["RobotTurn", args])

    def TurnTo(self, *args):
        """记录转向到目标航向调用。"""
        self.calls.append(["TurnTo", args])

    def stop(self):
        """记录停止底盘调用。"""
        self.calls.append(["stop"])


class FakePeripheral:
    """用于 BasicFuc 离线测试的外设假对象。"""

    def __init__(self):
        """初始化外设调用记录和吸球状态。"""
        self.calls: list[list[Any]] = []
        self.DribbleStatus = False

    def Dribble(self, value):
        """记录吸球器输出。"""
        self.DribbleStatus = value
        self.calls.append(["Dribble", value])

    def ShootBall(self):
        """记录射门调用。"""
        self.calls.append(["ShootBall"])


class FakeUnitedPosition:
    """用于 BasicFuc 离线测试的定位假对象。"""

    def __init__(
        self,
        *,
        robot_ball: list[float] | None = None,
        field_ball: list[float] | None = None,
        robot_pose: list[float] | None = None,
        robot_ball_angle: float = 0.0,
        robot_ball_distance: float = 100.0,
    ):
        """初始化假定位数据。"""
        self.robot_ball = list(robot_ball if robot_ball is not None else [30, 0])
        self.field_ball = list(field_ball if field_ball is not None else [30, 0])
        self.robot_pose = list(robot_pose if robot_pose is not None else [0, 0, 0])
        self.robot_ball_angle = robot_ball_angle
        self.robot_ball_distance = robot_ball_distance
        self.calls: list[list[Any]] = []
        self.vision_data = {}

    def RobotBallPos(self):
        """返回机器人参考系下的球位置。"""
        return list(self.robot_ball)

    def FieldBallPos(self):
        """返回场地参考系下的球位置。"""
        return list(self.field_ball)

    def FieldRobotPos(self):
        """返回机器人场地位姿。"""
        return list(self.robot_pose)

    def FieldRobotPose(self):
        """返回机器人场地位姿。"""
        return list(self.robot_pose)

    def FieldRobotPosition(self):
        """返回机器人场地位置，不包含航向。"""
        return list(self.robot_pose[:2])

    def RobotBallAngle(self):
        """返回机器人参考系下的球角度。"""
        return self.robot_ball_angle

    def RobotBallDistance(self):
        """返回机器人到球的距离。"""
        return self.robot_ball_distance

    def FieldChassisPos(self):
        """返回假敌方底盘位置列表。"""
        return [[10, -10]]

    def Local2Angle(self, target):
        """记录 Local2Angle 调用并返回默认角度。"""
        self.calls.append(["Local2Angle", target])
        return 0

    def Pos2Angle(self, source, target):
        """按两点计算场地角度并记录调用。"""
        self.calls.append(["Pos2Angle", source, target])
        sx, sy = source[:2]
        tx, ty = target[:2]
        import math

        return int(math.degrees(math.atan2(ty - sy, tx - sx)))

    def Pos2Pos(self, target, speed_gain=None):
        """记录 Pos2Pos 调用并模拟已到达。"""
        self.calls.append(["Pos2Pos", target, speed_gain])
        return True

    def Move2Path(self, path, wait_ms, **kwargs):
        """记录路径移动调用并模拟成功。"""
        self.calls.append(["Move2Path", path, wait_ms, kwargs])
        return True

    def Stop(self):
        """记录定位系统停止调用。"""
        self.calls.append(["Stop"])


def configure_sim_env(robot_id: int, force_sim: bool) -> None:
    """配置一次性 smoke 测试使用的模拟器环境变量。"""
    if force_sim:
        os.environ["ROBOCUPX_BITS"] = "SIM"
    else:
        os.environ.setdefault("ROBOCUPX_BITS", "SIM")
    os.environ.setdefault("XEL_UNITY_IP", "127.0.0.1")
    os.environ.setdefault("XEL_ROBOT_ID", str(robot_id))
    os.environ.setdefault("XEL_CMD_PORT", str(20001 + (max(robot_id, 1) - 1) * 10))
    os.environ.setdefault("XEL_STATUS_PORT", str(20002 + (max(robot_id, 1) - 1) * 10))


def import_stack():
    """导入 LunaPre、BasicFuc 和通信层测试对象。"""
    import LunaPre
    from LunaPre import Bits, UnitedPosition, chassis, compass, key, lidar, peripheral
    import BasicFuc
    from LunaPre.communication import Communication

    return {
        "LunaPre": LunaPre,
        "BasicFuc": BasicFuc,
        "Communication": Communication,
        "Bits": Bits,
        "UnitedPosition": UnitedPosition,
        "chassis": chassis,
        "compass": compass,
        "key": key,
        "lidar": lidar,
        "peripheral": peripheral,
    }


def _actual(obj):
    """解析 LunaPre 惰性代理，普通对象则原样返回。"""
    resolver = getattr(obj, "_resolve", None)
    if callable(resolver):
        return resolver()
    return obj


def _pose(UnitedPosition) -> list[float]:
    """读取模拟器场地位姿，失败时返回无效位姿哨兵值。"""
    try:
        pose = UnitedPosition.FieldRobotPose()
    except Exception:
        return list(INVALID_POSE)
    if not isinstance(pose, list) or len(pose) < 3 or pose == INVALID_POSE:
        return list(INVALID_POSE)
    return [float(pose[0]), float(pose[1]), float(pose[2])]


def wait_for_sim(Bits, UnitedPosition, timeout_s: float) -> dict[str, Any]:
    """等待模拟器状态包和有效位姿。"""
    start_rx = int(getattr(Bits, "_rx_count", 0))
    deadline = time.time() + timeout_s
    last_pose = list(INVALID_POSE)
    last_rx = start_rx
    while time.time() < deadline:
        last_pose = _pose(UnitedPosition)
        last_rx = int(getattr(Bits, "_rx_count", 0))
        if last_pose != INVALID_POSE and last_rx > start_rx:
            return {"connected": True, "pose": last_pose, "rx_count": last_rx}
        time.sleep(0.2)
    return {"connected": False, "pose": last_pose, "rx_count": last_rx, "start_rx": start_rx}


def short_motion(chassis, UnitedPosition, cmd: tuple[float, float, float], hold_s: float = 0.3) -> dict[str, Any]:
    """短时间下发一个运动命令，并记录起止位姿。"""
    start = _pose(UnitedPosition)
    t0 = time.time()
    while time.time() - t0 < hold_s:
        chassis.RobotMoveVector(*cmd)
        time.sleep(0.02)
    chassis.stop()
    time.sleep(0.12)
    end = _pose(UnitedPosition)
    return {"cmd": cmd, "start": start, "end": end}


def add_cases(runner: SmokeRunner, stack: dict[str, Any], *, sim_timeout_s: float, skip_sim_motion: bool) -> None:
    """向 runner 注册并执行所有 smoke 用例。"""
    BasicFuc = stack["BasicFuc"]
    Communication = stack["Communication"]
    Bits = stack["Bits"]
    UnitedPosition = stack["UnitedPosition"]
    chassis = stack["chassis"]
    compass = stack["compass"]
    key = stack["key"]
    lidar = stack["lidar"]
    peripheral = stack["peripheral"]

    sim_state: dict[str, Any] = {"connected": False}

    def luna_facade_exports():
        """检查 LunaPre 顶层门面导出的核心对象是否存在。"""
        names = ["Bits", "lidar", "chassis", "compass", "peripheral", "key", "UnitedPosition"]
        for name in names:
            assert stack[name] is not None, name
        assert type(UnitedPosition).__name__ == "RecordingPositions"
        return {
            "bits_type": type(_actual(Bits)).__name__,
            "chassis_type": type(_actual(chassis)).__name__,
            "position_type": type(UnitedPosition).__name__,
        }

    def simulator_connection():
        """检查能否从 Unity 收到有效状态帧。"""
        sim_state.update(wait_for_sim(Bits, UnitedPosition, sim_timeout_s))
        assert sim_state["connected"], sim_state
        return sim_state

    def positions_api_smoke():
        """扫一遍定位接口的主要读取和几何辅助函数。"""
        checks = {
            "pose": UnitedPosition.FieldRobotPose(),
            "position": UnitedPosition.FieldRobotPosition(),
            "direct_distance": UnitedPosition.DirectDistance(),
            "robot_ball": UnitedPosition.RobotBallPos(),
            "ball_state": UnitedPosition.BallState(),
            "field_ball": UnitedPosition.FieldBallPos(),
            "robot_ball_angle": UnitedPosition.RobotBallAngle(),
            "field_ball_angle": UnitedPosition.FieldBallAngle(),
            "robot_ball_distance": UnitedPosition.RobotBallDistance(),
            "robot_chassis": UnitedPosition.RobotChassisPos(),
            "field_chassis": UnitedPosition.FieldChassisPos(),
            "robot_chassis_angle": UnitedPosition.RobotChassisAngle(),
            "robot_goals": UnitedPosition.RobotGoalsPos(),
            "field_goals": UnitedPosition.FieldGoalsPos(),
            "local2angle": UnitedPosition.Local2Angle([0, 0]),
            "pos2angle": UnitedPosition.Pos2Angle([0, 0], [1, 1]),
            "safe_projection": UnitedPosition.ProjectSafePose([999, 999, 0]),
            "restricted_predicate": UnitedPosition.IsPoseInRestrictedArea([999, 999, 0]),
            "referee": UnitedPosition.Referee(),
        }
        assert isinstance(checks["pose"], list)
        assert isinstance(checks["direct_distance"], list)
        return checks

    def chassis_runtime_smoke():
        """扫一遍底盘运行时的常用运动接口。"""
        chassis.SetKp(3)
        yaw = chassis.Compass()
        chassis.RobotMoveVector(10, 0, 0)
        time.sleep(0.05)
        chassis.RobotMoveAngle(0, 10)
        time.sleep(0.05)
        chassis.FieldMoveVector(0, 0, yaw)
        time.sleep(0.05)
        chassis.FieldMoveAngle(yaw, yaw, 8)
        time.sleep(0.05)
        chassis.RobotTurn(8)
        time.sleep(0.05)
        chassis.FieldTurn(yaw)
        time.sleep(0.05)
        chassis.stop()
        return {"yaw": yaw, "last_vx": getattr(chassis, "_last_vx", None), "last_wz": getattr(chassis, "_last_wz", None)}

    def simulator_motion_smoke():
        """向模拟器下发短运动命令，确认机器人会响应。"""
        if skip_sim_motion:
            return {"skipped": True}
        if not sim_state.get("connected"):
            sim_state.update(wait_for_sim(Bits, UnitedPosition, sim_timeout_s))
        assert sim_state["connected"], sim_state
        return {
            "x": short_motion(chassis, UnitedPosition, (12.0, 0.0, 0.0)),
            "y": short_motion(chassis, UnitedPosition, (0.0, 12.0, 0.0)),
            "w": short_motion(chassis, UnitedPosition, (0.0, 0.0, 10.0)),
        }

    def peripheral_key_smoke():
        """检查外设吸球、射门和按键接口不会崩溃。"""
        peripheral.Dribble(0.2)
        time.sleep(0.1)
        peripheral.Dribble(False)
        peripheral.ShootBall()
        pressed = key.Press() if hasattr(key, "Press") else None
        return {"dribble_status": getattr(peripheral, "DribbleStatus", None), "key_pressed": pressed}

    def communication_protocol_smoke():
        """用假传输层测试 Communication 的打包、发送和解析。"""
        class FakeTransport:
            """Communication 测试使用的内存传输层。"""

            def __init__(self):
                """初始化发送缓存和关闭标记。"""
                self.sent = []
                self.closed = False

            def send(self, payload):
                """记录发送字节。"""
                self.sent.append(bytes(payload))

            def recv(self, _length):
                """返回包含最近发送帧的假接收缓冲。"""
                return b"xx" + self.sent[-1] + b"yy"

            def close(self):
                """标记传输层已关闭。"""
                self.closed = True

        comm = Communication.__new__(Communication)
        comm.transimission = FakeTransport()
        comm.BallFlag = [0, 0]
        comm.PeerPosition = [0, 0]
        comm.FullLog = False
        comm.RobotID = 1
        comm.Role = Communication.ROLE_OFFENCE_YELLOW
        comm.Status = Communication.STATUS_NO_BALL
        comm.LastFrame = None
        comm.LastPeerFrame = None

        frame = comm.PackSelfStatus(status=Communication.STATUS_CHASING, position=[12, -9], reserve=7)
        parsed = comm.ParseFrame(frame)
        assert parsed["signed_xy"] == [12, -9]
        assert comm.Send(frame) is True
        received = comm.Receive()
        assert received["status"] == Communication.STATUS_CHASING
        assert comm.SendStatus(position=[-3, 4]) is True
        assert comm.SendBallFlag([1, 2]) is True
        assert comm.SetBallFlag([300, -1]) == [255, 0]
        assert comm.SetPeerPosition([5, -6]) == [5, -6]
        assert comm.SendMessage(frame) is True
        assert comm.RecvMessage() is not None
        comm.Close()
        assert comm.transimission.closed is True
        return {"frame": list(frame), "parsed": parsed, "sent_count": len(comm.transimission.sent)}

    def basicfuc_pure_smoke():
        """测试 BasicFuc 中不依赖真实运行时的纯函数和小工具。"""
        assert BasicFuc.RoundThresholdJudger(358, 360, 0, 5) is True
        assert BasicFuc.RoundThresholdJudger(20, 360, 0, 5) is False
        assert BasicFuc.FindNearestAngle([0, 90, 180], 100) == 90
        assert BasicFuc.FindNearestAngle([0, 180], 359) == 0
        assert BasicFuc.FindNearstAngle([0, 90, 180], 100) == 90
        assert BasicFuc.LinearMap(5, (0, 10), (0, 100)) == 50
        assert BasicFuc.linear_map(5, (0, 10), (0, 100)) == 50
        try:
            BasicFuc.LinearMap(5, (1, 1), (0, 100))
        except ValueError:
            pass
        else:
            raise AssertionError("LinearMap should reject zero-width input range")
        assert BasicFuc._AngleErrorDeg(350, 10) == 20
        assert BasicFuc._Clamp(7, 0, 3) == 3
        lock = BasicFuc.LockSeries()
        lock._write_lockball_pid_log = lambda *args, **kwargs: None
        assert lock._robot_ball_angle(1, 1) == 45
        assert lock._clamp(9, 0, 5) == 5
        assert 0 <= lock._alignment_speed_x_scale(99) <= 1
        assert isinstance(lock._capture_control(), tuple)
        return {"lock_class": type(lock).__name__}

    def basicfuc_lockseries_smoke():
        """用 fake 对象测试 LockSeries 的主要锁球分支。"""
        fake_chassis = FakeChassis()
        fake_peripheral = FakePeripheral()
        lock = BasicFuc.LockSeries()
        lock._write_lockball_pid_log = lambda *args, **kwargs: None

        with patched(BasicFuc, UnitedPosition=FakeUnitedPosition(robot_ball=BALL_NOT_FOUND), chassis=fake_chassis, peripheral=fake_peripheral):
            assert lock.LockBallSlip() is False

        with patched(BasicFuc, UnitedPosition=FakeUnitedPosition(robot_ball=BALL_HELD), chassis=fake_chassis, peripheral=fake_peripheral):
            assert lock.LockBallSlip() is True

        with patched(BasicFuc, UnitedPosition=FakeUnitedPosition(robot_ball=[30, 4]), chassis=fake_chassis, peripheral=fake_peripheral):
            assert lock.LockBallSlip() is False
            assert lock.LockBallDash() is False
            lock.Lockballangle()
            lock.Lockballmove()
            score = lock.BallScore()

        assert any(call[0] in {"RobotMoveVector", "RobotMoveAngle", "RobotTurn", "stop"} for call in fake_chassis.calls)
        return {"calls": fake_chassis.calls[:12], "score": score}

    def basicfuc_strategy_helper_smoke():
        """用 fake 对象测试 BasicFuc 历史策略辅助函数。"""
        fake_chassis = FakeChassis()
        fake_peripheral = FakePeripheral()

        with patched(
            BasicFuc,
            UnitedPosition=FakeUnitedPosition(robot_ball=BALL_NOT_FOUND),
            chassis=fake_chassis,
            peripheral=fake_peripheral,
        ):
            assert BasicFuc.NormalShoot() is False
            assert fake_peripheral.calls[-1] == ["Dribble", False]

        with patched(BasicFuc, UnitedPosition=FakeUnitedPosition(robot_ball=BALL_HELD)):
            assert BasicFuc.NormalShoot() is True

        class FakeLockBall:
            """可配置返回值的假锁球器。"""

            def __init__(self, result="locked"):
                """初始化调用计数和返回值。"""
                self.calls = 0
                self.result = result

            def LockBallSlip(self):
                """记录调用并返回预设结果。"""
                self.calls += 1
                return self.result

        fake_lock = FakeLockBall()
        with patched(BasicFuc, UnitedPosition=FakeUnitedPosition(robot_ball=[20, 2]), lockball=fake_lock, peripheral=fake_peripheral):
            assert BasicFuc.NormalShoot() == "locked"
            assert fake_lock.calls == 1

        with patched(
            BasicFuc,
            UnitedPosition=FakeUnitedPosition(robot_ball=[30, 0]),
            chassis=fake_chassis,
            peripheral=fake_peripheral,
            compass=lambda: 90,
        ):
            assert BasicFuc.CircleAround(90, max_ticks=3) is True

        turn_lock = FakeLockBall(True)
        turn_chassis = FakeChassis(yaw=0)
        with patched(
            BasicFuc,
            UnitedPosition=FakeUnitedPosition(robot_ball=[8, 0]),
            chassis=turn_chassis,
            lockball=turn_lock,
        ):
            assert BasicFuc.TurnTo_with_ball(0) is True
            assert turn_lock.calls == 1

        turn_lock = FakeLockBall(True)
        turn_chassis = FakeChassis(yaw=0)
        with patched(
            BasicFuc,
            UnitedPosition=FakeUnitedPosition(robot_ball=[8, 0]),
            chassis=turn_chassis,
            lockball=turn_lock,
        ):
            assert BasicFuc.TurnTo_with_ball(90) is False
            assert turn_lock.calls == 1
            assert any(call[0] in ("SetMotor", "FieldMoveVector") for call in turn_chassis.calls)

        with patched(BasicFuc, UnitedPosition=FakeUnitedPosition(), chassis=fake_chassis):
            assert BasicFuc.GoBack() is True

        with patched(
            BasicFuc,
            UnitedPosition=FakeUnitedPosition(robot_ball=BALL_HELD, field_ball=BALL_HELD, robot_pose=[80, 40, 0]),
            chassis=fake_chassis,
            peripheral=fake_peripheral,
        ):
            assert BasicFuc.Slipsideshot() in (True, False)
            assert BasicFuc.Slipsideshot(attack_dir=1) in (True, False)

        with patched(
            BasicFuc,
            UnitedPosition=FakeUnitedPosition(field_ball=BALL_HELD, robot_pose=[85, 65, 0]),
            chassis=fake_chassis,
            peripheral=fake_peripheral,
            compass=lambda: 0,
        ):
            assert BasicFuc.OhMyBackStep() is False

        with patched(BasicFuc, OhMyBackStep=lambda HomePos=None: "alias-ok"):
            assert BasicFuc.OHMYBACK() == "alias-ok"

        return {"chassis_calls": fake_chassis.calls[:20], "peripheral_calls": fake_peripheral.calls[:20]}

    runner.run("luna_facade_exports", luna_facade_exports)
    runner.run("simulator_connection", simulator_connection)
    runner.run("positions_api_smoke", positions_api_smoke)
    runner.run("chassis_runtime_smoke", chassis_runtime_smoke)
    runner.run("simulator_motion_smoke", simulator_motion_smoke)
    runner.run("peripheral_key_smoke", peripheral_key_smoke)
    runner.run("communication_protocol_smoke", communication_protocol_smoke)
    runner.run("basicfuc_pure_smoke", basicfuc_pure_smoke)
    runner.run("basicfuc_lockseries_smoke", basicfuc_lockseries_smoke)
    runner.run("basicfuc_strategy_helper_smoke", basicfuc_strategy_helper_smoke)


def parse_args() -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-id", type=int, default=1, help="要测试的 Unity 机器人 id，默认 1")
    parser.add_argument("--sim-timeout", type=float, default=8.0, help="等待 Unity 状态包的秒数")
    parser.add_argument("--skip-sim-motion", action="store_true", help="只测试导入/API/fake 策略辅助函数，不跑模拟器运动")
    parser.add_argument("--no-force-sim", action="store_true", help="不强制设置 ROBOCUPX_BITS=SIM")
    parser.add_argument(
        "--output-dir",
        default="Records/function_smoke",
        help="JSON 报告目录；相对路径按仓库根目录解析，绝对路径保持不变",
    )
    return parser.parse_args()


def write_report(report: dict[str, Any], output_dir: str) -> Path:
    """把 smoke 测试报告写入指定目录并返回文件路径。"""
    out_dir = Path(output_dir).expanduser()
    if not out_dir.is_absolute():
        out_dir = REPO_ROOT / out_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"function_smoke_{time.strftime('%Y%m%d_%H%M%S')}.json"
    path.write_text(json.dumps(report, indent=2, ensure_ascii=True, default=str), encoding="utf-8")
    return path


def main() -> int:
    """执行完整 smoke 测试并返回进程退出码。"""
    args = parse_args()
    configure_sim_env(args.robot_id, force_sim=not args.no_force_sim)

    runner = SmokeRunner()
    stack = import_stack()
    try:
        add_cases(runner, stack, sim_timeout_s=args.sim_timeout, skip_sim_motion=args.skip_sim_motion)
    finally:
        try:
            stack["chassis"].stop()
        except Exception:
            pass
        try:
            stack["UnitedPosition"].Stop()
        except Exception:
            pass

    report = runner.report()
    report["env"] = {
        "ROBOCUPX_BITS": os.environ.get("ROBOCUPX_BITS"),
        "XEL_ROBOT_ID": os.environ.get("XEL_ROBOT_ID"),
        "XEL_CMD_PORT": os.environ.get("XEL_CMD_PORT"),
        "XEL_STATUS_PORT": os.environ.get("XEL_STATUS_PORT"),
    }
    report_path = write_report(report, args.output_dir)
    print(f"[REPORT] {report_path}")
    print(f"[SUMMARY] passed={report['passed']} failed={report['failed']} ok={report['ok']}")
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
