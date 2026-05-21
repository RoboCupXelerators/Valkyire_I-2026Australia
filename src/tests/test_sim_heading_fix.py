"""
模拟器航向角控制系统验证脚本

目的：
- 测试修改后的 angularSpeedScale (0.5) 是否能正确控制航向角速度
- 对标真机采集数据中的 w 轴响应
- 检测是否存在其他隐藏问题

使用方法：
    python -m src.tests.test_sim_heading_fix
    
环境变量：
    XEL_UNITY_IP=127.0.0.1           # Unity 模拟器地址
    TEST_WZ_CMD_LEVELS=0.1,0.2,0.5   # 测试的 wz 指令幅值 (单位: rad/s)
    TEST_HOLD_TIME=1.0                # 保持指令时间 (秒)
    TEST_COAST_TIME=0.5               # 指令释放后的滑行观测时间
    TEST_REPEAT=3                     # 每个幅值重复测试次数
"""

from __future__ import annotations

import csv
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

sys.path.insert(0, "src")

try:
    from LunaPre import UnitedPosition, chassis, logger, Settings
except Exception as e:
    print(f"Warning: Cannot import LunaPre: {e}")
    logger = None


@dataclass
class HeadingTestResult:
    """单次航向角测试结果"""
    cmd_level: float          # 指令幅值 (rad/s)
    hold_time: float          # 指令保持时间
    
    # 初始状态
    start_yaw_deg: float
    start_time: float
    
    # 保持阶段
    hold_samples: list[tuple[float, float]]  # (yaw_deg, elapsed_s)
    
    # 滑行阶段
    coast_samples: list[tuple[float, float]]  # (yaw_deg, elapsed_s)
    
    # 分析结果
    peak_rate_deg_s: Optional[float] = None
    avg_rate_deg_s: Optional[float] = None
    coast_start_rate_deg_s: Optional[float] = None
    expected_rate_deg_s: Optional[float] = None  # 理论值 = cmd * 57.2958 (rad/s -> deg/s)


def _env_float(name: str, default: float) -> float:
    """从环境变量读取浮点数"""
    try:
        return float(os.environ.get(name, default))
    except Exception:
        return float(default)


def _env_csv_floats(name: str, default: list[float]) -> list[float]:
    """从环境变量读取逗号分隔的浮点数列表"""
    raw = str(os.environ.get(name, "")).strip()
    if not raw:
        return list(default)
    out: list[float] = []
    for part in raw.split(","):
        try:
            out.append(float(part.strip()))
        except Exception:
            pass
    return out or list(default)


def _angle_diff_deg(current: float, start: float) -> float:
    """计算最短角差（处理 0/360 边界）"""
    diff = float(current) - float(start)
    # 规范化到 [-180, 180]
    while diff > 180.0:
        diff -= 360.0
    while diff < -180.0:
        diff += 360.0
    return diff


def get_current_pose() -> Optional[list[float]]:
    """获取当前位姿 (x_cm, y_cm, yaw_deg)"""
    try:
        if logger:
            pose = UnitedPosition.FieldRobotPos()
        else:
            # 如果没有 LunaPre，从 XeleratorSimBits 获取
            from src.utils.XeleratorSimBits import XeleratorSimBits
            bits = XeleratorSimBits()
            x, y, yaw = bits.GetPose()
            pose = [x * 100, y * 100, yaw]  # 转为 cm
        
        if isinstance(pose, list) and len(pose) >= 3:
            return [float(pose[0]), float(pose[1]), float(pose[2])]
    except Exception as e:
        if logger:
            logger.error(f"Failed to get pose: {e}")
        else:
            print(f"Failed to get pose: {e}")
    return None


def set_twist(vx: float, vy: float, wz: float) -> bool:
    """设置机器人速度命令"""
    try:
        if logger:
            chassis.SetTwist(vx, vy, wz)
        else:
            from src.utils.XeleratorSimBits import XeleratorSimBits
            bits = XeleratorSimBits()
            bits.SetTwist(vx, vy, wz)
        return True
    except Exception as e:
        if logger:
            logger.error(f"Failed to set twist: {e}")
        else:
            print(f"Failed to set twist: {e}")
        return False


def test_heading_response(cmd_wz: float, hold_time: float = 1.0, coast_time: float = 0.5, 
                         sample_hz: float = 20.0) -> Optional[HeadingTestResult]:
    """
    测试单个航向角指令的响应
    
    Args:
        cmd_wz: 角速度指令 (rad/s)
        hold_time: 保持指令的时间 (秒)
        coast_time: 指令释放后观测滑行的时间 (秒)
        sample_hz: 采样频率 (Hz)
    
    Returns:
        测试结果或 None（失败）
    """
    sample_interval = 1.0 / sample_hz
    
    # 等待位姿稳定
    time.sleep(0.2)
    pose = get_current_pose()
    if pose is None:
        print(f"❌ Failed to get initial pose for wz={cmd_wz}")
        return None
    
    start_yaw_deg = pose[2]
    start_time = time.time()
    
    print(f"\n🧪 Testing wz={cmd_wz:+.3f} rad/s (expected {cmd_wz*57.2958:+.1f} deg/s)")
    print(f"   Initial yaw: {start_yaw_deg:.1f}°")
    
    result = HeadingTestResult(
        cmd_level=cmd_wz,
        hold_time=hold_time,
        start_yaw_deg=start_yaw_deg,
        start_time=start_time,
        hold_samples=[],
        coast_samples=[]
    )
    result.expected_rate_deg_s = cmd_wz * 57.2958
    
    # Phase 1: 保持指令阶段
    print("   [Holding command...]")
    set_twist(0.0, 0.0, cmd_wz)
    hold_start = time.time()
    
    while time.time() - hold_start < hold_time:
        pose = get_current_pose()
        if pose is not None:
            elapsed = time.time() - hold_start
            yaw = pose[2]
            result.hold_samples.append((yaw, elapsed))
            time.sleep(sample_interval)
        else:
            print(f"   ⚠️  Lost pose during hold phase")
            break
    
    # Phase 2: 滑行阶段
    print("   [Coasting...]")
    set_twist(0.0, 0.0, 0.0)
    coast_start = time.time()
    coast_start_yaw = get_current_pose()[2] if get_current_pose() else start_yaw_deg
    
    while time.time() - coast_start < coast_time:
        pose = get_current_pose()
        if pose is not None:
            elapsed = time.time() - coast_start
            yaw = pose[2]
            result.coast_samples.append((yaw, elapsed))
            time.sleep(sample_interval)
        else:
            print(f"   ⚠️  Lost pose during coast phase")
            break
    
    # 停止任何运动
    set_twist(0.0, 0.0, 0.0)
    
    # 分析结果
    if len(result.hold_samples) >= 2:
        yaw_diffs = []
        time_diffs = []
        for i in range(1, len(result.hold_samples)):
            y_curr, t_curr = result.hold_samples[i]
            y_prev, t_prev = result.hold_samples[i-1]
            
            dyaw = _angle_diff_deg(y_curr, y_prev)
            dt = t_curr - t_prev
            
            if dt > 0:
                rate = dyaw / dt
                yaw_diffs.append(dyaw)
                time_diffs.append(dt)
                
                if abs(rate) > (abs(result.peak_rate_deg_s or 0)):
                    result.peak_rate_deg_s = rate
        
        if yaw_diffs:
            total_dyaw = sum(yaw_diffs)
            total_dt = sum(time_diffs)
            result.avg_rate_deg_s = total_dyaw / total_dt if total_dt > 0 else None
    
    # 分析滑行阶段
    if len(result.coast_samples) >= 2:
        y_curr, t_curr = result.coast_samples[1]
        y_prev, t_prev = result.coast_samples[0]
        dyaw = _angle_diff_deg(y_curr, y_prev)
        dt = t_curr - t_prev
        if dt > 0:
            result.coast_start_rate_deg_s = dyaw / dt
    
    return result


def analyze_results(results: list[HeadingTestResult]) -> dict:
    """分析测试结果并生成报告"""
    analysis = {
        "total_tests": len(results),
        "tests": []
    }
    
    print("\n" + "="*80)
    print("📊 TEST RESULTS ANALYSIS")
    print("="*80)
    
    issues = []
    
    for result in results:
        expected = result.expected_rate_deg_s or 0.0
        actual_avg = result.avg_rate_deg_s or 0.0
        actual_peak = result.peak_rate_deg_s or 0.0
        coast_rate = result.coast_start_rate_deg_s or 0.0
        
        ratio = actual_avg / expected if expected != 0 else float('inf')
        
        test_info = {
            "cmd_wz_rad_s": result.cmd_level,
            "expected_rate_deg_s": expected,
            "actual_avg_rate_deg_s": actual_avg,
            "actual_peak_rate_deg_s": actual_peak,
            "coast_start_rate_deg_s": coast_rate,
            "ratio_actual_vs_expected": ratio,
            "hold_samples_count": len(result.hold_samples),
            "coast_samples_count": len(result.coast_samples)
        }
        analysis["tests"].append(test_info)
        
        print(f"\n📌 Test: cmd_wz = {result.cmd_level:+.3f} rad/s ({result.cmd_level*57.2958:+.1f} deg/s)")
        print(f"   Expected rate:     {expected:+.1f} deg/s")
        print(f"   Actual avg rate:   {actual_avg:+.1f} deg/s  (ratio: {ratio:.2f}x)")
        print(f"   Actual peak rate:  {actual_peak:+.1f} deg/s")
        print(f"   Coast start rate:  {coast_rate:+.1f} deg/s")
        print(f"   Samples: {len(result.hold_samples)} hold, {len(result.coast_samples)} coast")
        
        # 检查问题
        if abs(ratio - 1.0) > 0.2:
            issues.append(f"❌ Large discrepancy at wz={result.cmd_level}: ratio={ratio:.2f}x")
        
        if actual_avg * expected < 0:
            issues.append(f"❌ Sign reversal at wz={result.cmd_level}: expected {expected:+.1f}, got {actual_avg:+.1f}")
        
        if coast_rate == 0.0:
            issues.append(f"⚠️  No coast-phase deceleration at wz={result.cmd_level}")
    
    print("\n" + "="*80)
    if issues:
        print("⚠️  ISSUES DETECTED:")
        for issue in issues:
            print(f"   {issue}")
    else:
        print("✅ No major issues detected!")
    print("="*80)
    
    return analysis


def main():
    """主测试流程"""
    print("""
╔════════════════════════════════════════════════════════════════════════════╗
║         SIMULATOR HEADING ANGLE CONTROL SYSTEM TEST                         ║
║                                                                              ║
║  This script validates the heading angle (yaw/wz) response after the       ║
║  angularSpeedScale fix (1.47 → 0.5).                                       ║
╚════════════════════════════════════════════════════════════════════════════╝
    """)
    
    # 读取配置
    wz_levels = _env_csv_floats("TEST_WZ_CMD_LEVELS", [0.1, 0.2, 0.5, 1.0])
    hold_time = _env_float("TEST_HOLD_TIME", 1.0)
    coast_time = _env_float("TEST_COAST_TIME", 0.5)
    repeat_count = int(_env_float("TEST_REPEAT", 3))
    sample_hz = _env_float("TEST_SAMPLE_HZ", 20.0)
    
    print(f"⚙️  Configuration:")
    print(f"   wz command levels: {wz_levels}")
    print(f"   Hold time: {hold_time}s, Coast time: {coast_time}s")
    print(f"   Repeats per level: {repeat_count}")
    print(f"   Sample rate: {sample_hz} Hz")
    
    # 等待初始化
    print(f"\n⏳ Waiting for simulator connection...")
    time.sleep(2.0)
    
    pose = get_current_pose()
    if pose is None:
        print("❌ Cannot connect to simulator! Check XEL_UNITY_IP and connectivity.")
        return 1
    print(f"✅ Connected! Initial pose: {pose}")
    
    all_results = []
    
    # 测试每个幅值
    for wz_cmd in wz_levels:
        for repeat in range(repeat_count):
            print(f"\n🔄 Repeat {repeat+1}/{repeat_count} for wz={wz_cmd}")
            result = test_heading_response(
                wz_cmd,
                hold_time=hold_time,
                coast_time=coast_time,
                sample_hz=sample_hz
            )
            if result:
                all_results.append(result)
            else:
                print(f"⚠️  Test failed, skipping")
            time.sleep(0.5)  # 测试间隔
    
    # 分析结果
    analysis = analyze_results(all_results)
    
    # 保存结果
    output_dir = Path("Records/heading_tests")
    output_dir.mkdir(parents=True, exist_ok=True)
    
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    json_path = output_dir / f"heading_test_{timestamp}.json"
    
    with open(json_path, "w") as f:
        json.dump(analysis, f, indent=2)
    print(f"\n📄 Results saved to: {json_path}")
    
    # 生成简单报告
    csv_path = output_dir / f"heading_test_{timestamp}.csv"
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "cmd_wz_rad_s", "expected_rate_deg_s", "actual_avg_rate_deg_s",
                "actual_peak_rate_deg_s", "coast_start_rate_deg_s",
                "ratio_actual_vs_expected", "hold_samples_count", "coast_samples_count"
            ]
        )
        writer.writeheader()
        writer.writerows(analysis["tests"])
    print(f"📊 CSV report saved to: {csv_path}")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
