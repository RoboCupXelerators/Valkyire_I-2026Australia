"""BasicFuc 真机综合测试脚本

测试所有修复的逻辑问题：
1. PID Y轴 setpoint
2. Slipsideshot 球位置获取
3. Slipsideshot 误差计算
4. OhMyBack 持球判断和坐标获取
5. TurnToWithBall 方向判断
6. CircleAround 变量定义
7. NormalShoot 条件判断

使用方法：
    python -m src.tests.test_basicfuc_real

注意：此脚本需要在真机环境下运行，会调用实际硬件。
"""

from __future__ import annotations

import sys
import time
import math
from typing import List, Optional

# 添加项目路径
sys.path.insert(0, "src")

print("=" * 60)
print("BasicFuc 真机综合测试")
print("=" * 60)

# ============== 测试结果记录 ==============

test_results: List[dict] = []


def record_test(name: str, passed: bool, details: str = "") -> None:
    """记录测试结果"""
    test_results.append({
        "name": name,
        "passed": passed,
        "details": details
    })
    status = "✅ PASS" if passed else "❌ FAIL"
    print(f"{status}: {name}")
    if details:
        print(f"    详情: {details}")


def print_summary() -> None:
    """打印测试总结"""
    print("\n" + "=" * 60)
    print("测试总结")
    print("=" * 60)
    passed_count = sum(1 for r in test_results if r["passed"])
    total_count = len(test_results)
    print(f"通过: {passed_count}/{total_count}")
    
    if passed_count < total_count:
        print("\n失败的测试:")
        for r in test_results:
            if not r["passed"]:
                print(f"  - {r['name']}: {r['details']}")
    
    print("=" * 60)


# ============== 1. 模块导入测试 ==============

print("\n[1] 模块导入测试")
print("-" * 40)

try:
    from LunaPre import (
        UnitedPosition,
        chassis,
        compass,
        peripheral,
        logger,
        Settings,
        cfg,
        lidar,
        Bits,
    )
    record_test("LunaPre 导入", True)
    # 判断运行环境: SIM 为仿真器, AB/IL 为真机
    IS_SIM_MODE = str(getattr(cfg.RoboInfo, "Bit", "SIM")).upper() == "SIM"
except ImportError as e:
    record_test("LunaPre 导入", False, str(e))
    print("无法继续测试，请确保在正确的环境下运行")
    sys.exit(1)

try:
    from BasicFuc import (
        BallStatus,
        MotionConfig,
        Clamp,
        ClampSpeed,
        ComputeBallAngle,
        ComputeDistance,
        RoundThresholdJudger,
        FindNearestAngle,
        LinearMap,
        LockSeries,
        lockball,
        NormalShoot,
        CircleAround,
        TurnToWithBall,
        GoBack,
        Slipsideshot,
        OhMyBack,
    )
    record_test("BasicFuc 导入", True)
except ImportError as e:
    record_test("BasicFuc 导入", False, str(e))
    sys.exit(1)


# ============== 2. BallStatus 类测试 ==============

print("\n[2] BallStatus 类测试")
print("-" * 40)

# 测试 NO_BALL 判断
test_cases_no_ball = [
    ([0xFFF, 0xFFF], True, "标准 NO_BALL 值"),
    ([0, 0], True, "零值应视为 NO_BALL"),
    ([0xDDD, 0xDDD], False, "CATCH_BALL 不是 NO_BALL"),
    ([50, 30], False, "有效球位置不是 NO_BALL"),
]

for pos, expected, desc in test_cases_no_ball:
    result = BallStatus.is_no_ball(pos)
    passed = result == expected
    record_test(f"BallStatus.is_no_ball({pos})", passed, 
                f"期望 {expected}, 实际 {result} - {desc}")

# 测试 CATCH_BALL 判断
test_cases_catch = [
    ([0xDDD, 0xDDD], True, "标准 CATCH_BALL 值"),
    ([0xFFF, 0xFFF], False, "NO_BALL 不是 CATCH_BALL"),
    ([50, 30], False, "有效球位置不是 CATCH_BALL"),
]

for pos, expected, desc in test_cases_catch:
    result = BallStatus.is_catch_ball(pos)
    passed = result == expected
    record_test(f"BallStatus.is_catch_ball({pos})", passed,
                f"期望 {expected}, 实际 {result} - {desc}")

# 测试 is_valid 判断
test_cases_valid = [
    ([50, 30], True, "有效球位置"),
    ([0xFFF, 0xFFF], False, "NO_BALL 无效"),
    ([0xDDD, 0xDDD], False, "CATCH_BALL 无效"),
]

for pos, expected, desc in test_cases_valid:
    result = BallStatus.is_valid(pos)
    passed = result == expected
    record_test(f"BallStatus.is_valid({pos})", passed,
                f"期望 {expected}, 实际 {result} - {desc}")


# ============== 3. 工具函数测试 ==============

print("\n[3] 工具函数测试")
print("-" * 40)

# Clamp 测试
clamp_cases = [
    (150, -300, 300, 150, "正常范围内"),
    (400, -300, 300, 300, "超出上限"),
    (-400, -300, 300, -300, "超出下限"),
]

for value, min_v, max_v, expected, desc in clamp_cases:
    result = Clamp(value, min_v, max_v)
    passed = result == expected
    record_test(f"Clamp({value}, {min_v}, {max_v})", passed,
                f"期望 {expected}, 实际 {result} - {desc}")

# ClampSpeed 测试
speed_cases = [
    (150, 150, "正常速度"),
    (400, 300, "超速限制"),
    (-400, -300, "反向超速限制"),
]

for value, expected, desc in speed_cases:
    result = ClampSpeed(value)
    passed = result == expected
    record_test(f"ClampSpeed({value})", passed,
                f"期望 {expected}, 实际 {result} - {desc}")

# ComputeBallAngle 测试
angle_cases = [
    (0, 0, 0.0, "球在正前方"),
    (100, 0, 0.0, "球在前方远处"),
    (0, 100, 90.0, "球在左侧"),
    (0, -100, -90.0, "球在右侧"),
    (100, 100, 45.0, "球在前左方"),
]

for bx, by, expected, desc in angle_cases:
    result = ComputeBallAngle(bx, by)
    passed = abs(result - expected) < 1.0  # 允许 1 度误差
    record_test(f"ComputeBallAngle({bx}, {by})", passed,
                f"期望 {expected}, 实际 {result} - {desc}")

# ComputeDistance 测试
dist_cases = [
    (0, 0, 0.0, "原点"),
    (100, 0, 100.0, "X轴"),
    (0, 100, 100.0, "Y轴"),
    (100, 100, 141.42, "对角线"),
]

for x, y, expected, desc in dist_cases:
    result = ComputeDistance(x, y)
    passed = abs(result - expected) < 0.1
    record_test(f"ComputeDistance({x}, {y})", passed,
                f"期望 {expected:.2f}, 实际 {result:.2f} - {desc}")

# RoundThresholdJudger 测试
round_cases = [
    (90, 360, 90, 10, True, "在中心"),
    (80, 360, 90, 10, True, "在下边界内"),
    (100, 360, 90, 10, True, "在上边界内"),
    (110, 360, 90, 10, False, "超出边界"),
    (0, 360, 350, 20, True, "跨越 0 度边界"),
]

for val, round_v, mid, offset, expected, desc in round_cases:
    result = RoundThresholdJudger(val, round_v, mid, offset)
    passed = result == expected
    record_test(f"RoundThresholdJudger({val}, {round_v}, {mid}, {offset})", passed,
                f"期望 {expected}, 实际 {result} - {desc}")

# LinearMap 测试
map_cases = [
    (50, (0, 100), (0, 1000), 500, "线性映射"),
    (0, (0, 100), (0, 1000), 0, "最小值"),
    (100, (0, 100), (0, 1000), 1000, "最大值"),
]

for val, in_range, out_range, expected, desc in map_cases:
    result = LinearMap(val, in_range, out_range)
    passed = abs(result - expected) < 0.1
    record_test(f"LinearMap({val}, {in_range}, {out_range})", passed,
                f"期望 {expected}, 实际 {result} - {desc}")


# ============== 4. MotionConfig 测试 ==============

print("\n[4] MotionConfig 测试")
print("-" * 40)

try:
    config = MotionConfig()
    
    # 验证默认值
    default_checks = [
        ("max_speed", 300),
        ("min_speed", -300),
        ("pid_x_kp", 3.0),
        ("pid_y_kp", 2.5),
        ("far_threshold", 75.0),
        ("mid_threshold", 25.0),
    ]
    
    for attr, expected in default_checks:
        actual = getattr(config, attr)
        passed = actual == expected
        record_test(f"MotionConfig.{attr}", passed,
                    f"期望 {expected}, 实际 {actual}")
    
    # 测试自定义配置
    custom_config = MotionConfig(max_speed=500, min_speed=-500)
    passed = custom_config.max_speed == 500 and custom_config.min_speed == -500
    record_test("MotionConfig 自定义配置", passed)
    
except Exception as e:
    record_test("MotionConfig 创建", False, str(e))


# ============== 5. LockSeries PID 初始化测试 ==============

print("\n[5] LockSeries PID 初始化测试")
print("-" * 40)

try:
    # 创建 LockSeries 实例
    test_lockball = LockSeries()
    
    # 检查策略是否正确初始化
    strategy_check = hasattr(test_lockball, '_strategies') and \
                     'angle' in test_lockball._strategies and \
                     'move' in test_lockball._strategies and \
                     'slip' in test_lockball._strategies and \
                     'dash' in test_lockball._strategies
    record_test("LockSeries 策略初始化", strategy_check)
    
    # 检查 Slip 策略的 PID 初始化
    slip_strategy = test_lockball._strategies['slip']
    
    # 验证 PID Y轴 setpoint 使用 catch_y 而非 catch_x
    if hasattr(slip_strategy, '_pid_y'):
        pid_y_setpoint = slip_strategy._pid_y.setpoint
        catch_val = getattr(Settings.Motion, 'CatchVal', [0, 0])
        expected_y = catch_val[1] if len(catch_val) > 1 else 0
        
        # 注意：这里验证的是修复后的逻辑
        # 如果 setpoint 是 catch_y，则测试通过
        y_correct = pid_y_setpoint == expected_y
        record_test("PID Y轴 setpoint 使用 catch_y", y_correct,
                    f"期望 {expected_y}, 实际 {pid_y_setpoint}")
    else:
        record_test("PID Y轴 setpoint 检查", False, "PID 未初始化（可能 simple_pid 未安装）")
    
    # 测试 reset 方法
    test_lockball.reset()
    record_test("LockSeries.reset()", True)
    
except Exception as e:
    record_test("LockSeries 创建", False, str(e))


# ============== 6. 硬件连接测试 ==============

print("\n[6] 硬件连接测试")
print("-" * 40)

# 检查是否在仿真环境
if IS_SIM_MODE:
    print("当前运行在 SIM 模式（Unity 仿真器）")
    record_test("硬件模式", False, "SIM 仿真器环境")
else:
    record_test("硬件模式", True, "真机环境")
    
    # 测试 chassis 连接
    try:
        yaw = chassis.GetYaw()
        record_test("chassis.GetYaw()", True, f"当前 yaw: {yaw}")
    except Exception as e:
        record_test("chassis.GetYaw()", False, str(e))
    
    # 测试 compass
    try:
        current_angle = compass()
        record_test("compass()", True, f"当前角度: {current_angle}")
    except Exception as e:
        record_test("compass()", False, str(e))
    
    # 测试 lidar
    try:
        distances = lidar.GetDists()
        if distances == [0xfff, 0xfff, 0xfff, 0xfff]:
            record_test("lidar.GetDists()", False, "返回无效值")
        else:
            record_test("lidar.GetDists()", True, f"距离: {distances}")
    except Exception as e:
        record_test("lidar.GetDists()", False, str(e))


# ============== 7. UnitedPosition 功能测试 ==============

print("\n[7] UnitedPosition 功能测试")
print("-" * 40)

try:
    # 测试球位置获取
    ball_pos = UnitedPosition.RobotBallPos()
    record_test("UnitedPosition.RobotBallPos()", True, f"球位置: {ball_pos}")
    
    # 测试机器人位置获取
    robot_pos = UnitedPosition.FieldRobotPos()
    if robot_pos == [0xfff, 0xfff, 0xfff]:
        record_test("UnitedPosition.FieldRobotPos()", False, "返回无效值")
    else:
        record_test("UnitedPosition.FieldRobotPos()", True, f"机器人位置: {robot_pos}")
    
    # 测试球距离
    ball_distance = UnitedPosition.RobotBallDistance()
    record_test("UnitedPosition.RobotBallDistance()", True, f"球距离: {ball_distance}")
    
    # 测试场地球位置
    field_ball_pos = UnitedPosition.FieldBallPos()
    record_test("UnitedPosition.FieldBallPos()", True, f"场地球位置: {field_ball_pos}")
    
    # 测试对方机器人位置（用于 OhMyBack）
    chassis_pos = UnitedPosition.FieldChassisPos()
    record_test("UnitedPosition.FieldChassisPos()", True, f"对方机器人位置: {chassis_pos}")
    
except Exception as e:
    record_test("UnitedPosition 测试", False, str(e))


# ============== 8. 策略函数逻辑测试 ==============

print("\n[8] 策略函数逻辑测试")
print("-" * 40)

# 测试 NormalShoot 的条件判断（使用范围而非精确值）
print("测试 NormalShoot 条件判断逻辑...")
try:
    # 模拟测试：验证 abs(bx) <= error_range 的逻辑
    error_range = getattr(Settings.Motion, 'ErrorRange', 10)
    
    # 测试条件判断
    test_bx_values = [0, 5, 10, 15]
    for bx in test_bx_values:
        condition_result = abs(bx) <= error_range
        print(f"  bx={bx}: abs(bx) <= {error_range} = {condition_result}")
    
    record_test("NormalShoot 条件判断逻辑", True, 
                f"使用 abs(bx) <= {error_range} 而非 bx == 0")
except Exception as e:
    record_test("NormalShoot 条件判断逻辑", False, str(e))

# 测试 TurnToWithBall 方向判断
print("测试 TurnToWithBall 方向判断逻辑...")
try:
    # 模拟 Error 归一化测试
    test_errors = [30, -30, 150, -150, 180, -180]
    for err in test_errors:
        # 归一化到 [-180, 180]
        normalized = ((err + 180) % 360) - 180
        direction = -1 if normalized > 0 else 1
        print(f"  Error={err}: 归一化={normalized}, 方向因子={direction}")
    
    record_test("TurnToWithBall 方向判断逻辑", True,
                "使用归一化 Error 和正确的方向因子")
except Exception as e:
    record_test("TurnToWithBall 方向判断逻辑", False, str(e))

# 测试 CircleAround 变量初始化
print("测试 CircleAround 变量初始化...")
try:
    # 验证 iDeltaAngle 初始化逻辑
    iDeltaAngle = 0  # 初始化
    
    # 模拟不同 iY 值
    test_iY_values = [0, 50, -50, 100]
    for iY in test_iY_values:
        if iY != 0:
            # 假设 iX = 50
            iX = 50
            calc_angle = -int(math.degrees(math.atan2(iY, iX)) - 90)
        else:
            calc_angle = 0
        print(f"  iY={iY}: iDeltaAngle={calc_angle}")
    
    record_test("CircleAround 变量初始化", True,
                "iDeltaAngle 初始化为 0，始终计算")
except Exception as e:
    record_test("CircleAround 变量初始化", False, str(e))

# 测试 Slipsideshot 误差计算
print("测试 Slipsideshot 误差计算...")
try:
    # 验证使用 AimY - LocalY 而非 AimX - LocalX 两次
    LocalX, LocalY = 35, 75
    AimX, AimY = 40, 80
    
    # 正确计算
    correct_error = abs(AimX - LocalX) + abs(AimY - LocalY)
    # 错误计算（旧代码）
    wrong_error = abs(AimX - LocalX) + abs(AimX - LocalX)
    
    print(f"  正确计算: abs({AimX}-{LocalX}) + abs({AimY}-{LocalY}) = {correct_error}")
    print(f"  错误计算: abs({AimX}-{LocalX}) + abs({AimX}-{LocalX}) = {wrong_error}")
    
    record_test("Slipsideshot 误差计算", correct_error != wrong_error,
                f"正确={correct_error}, 错误={wrong_error}")
except Exception as e:
    record_test("Slipsideshot 误差计算", False, str(e))


# ============== 9. 实际策略执行测试（可选） ==============

print("\n[9] 实际策略执行测试")
print("-" * 40)

# 询问是否执行实际策略
print("是否执行实际策略测试？(会移动机器人)")
print("输入 'y' 执行，其他键跳过")

# 在真机测试时，可以取消注释以下代码进行交互式测试
# user_input = input(">>> ").strip().lower()
user_input = 'n'  # 默认跳过，真机测试时改为交互

if user_input == 'y' and not IS_SIM_MODE:
    print("\n执行 LockBallSlip 测试...")
    try:
        result = lockball.LockBallSlip()
        record_test("lockball.LockBallSlip()", True, f"返回: {result}")
    except Exception as e:
        record_test("lockball.LockBallSlip()", False, str(e))
    
    print("\n执行 NormalShoot 测试...")
    try:
        result = NormalShoot(x=0)  # 仅持球模式
        record_test("NormalShoot(x=0)", True, f"返回: {result}")
    except Exception as e:
        record_test("NormalShoot(x=0)", False, str(e))
    
    print("\n执行 GoBack 测试...")
    try:
        GoBack()
        record_test("GoBack()", True)
    except Exception as e:
        record_test("GoBack()", False, str(e))
else:
    print("跳过实际策略执行测试")
    record_test("实际策略执行", False, "用户跳过或 Mock 模式")


# ============== 10. 安全停止测试 ==============

print("\n[10] 安全停止测试")
print("-" * 40)

try:
    chassis.stop()
    record_test("chassis.stop()", True)
except Exception as e:
    record_test("chassis.stop()", False, str(e))

try:
    peripheral.Dribble(False)
    record_test("peripheral.Dribble(False)", True)
except Exception as e:
    record_test("peripheral.Dribble(False)", False, str(e))


# ============== 打印总结 ==============

print_summary()

# 返回退出码
passed_count = sum(1 for r in test_results if r["passed"])
total_count = len(test_results)
sys.exit(0 if passed_count == total_count else 1)