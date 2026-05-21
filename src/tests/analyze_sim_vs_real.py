"""
模拟器 vs 真机对标分析

对比真机采集数据和模拟器测试结果，验证参数修复是否有效

使用方法：
    python -m src.tests.analyze_sim_vs_real \
        --real Records/friction/friction_real_sampling_20260519_190045.csv \
        --sim Records/heading_tests/heading_test_YYYYMMDD_HHMMSS.json
"""

from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path
from typing import Optional
from dataclasses import dataclass
from collections import defaultdict

sys.path.insert(0, "src")


@dataclass
class RealDataPoint:
    """真机采集单点数据"""
    axis: str              # x, y, w
    level: float           # 指令幅值
    phase: str             # forward, backward, forward_coast, backward_coast
    cmd: float             # 指令值
    yaw_deg: float         # 航向角
    est_wz_deg_s: float    # 估计角速度
    phase_dyaw_deg: float  # 阶段内角度变化


@dataclass
class SimulatorDataPoint:
    """模拟器测试单点数据"""
    cmd_wz_rad_s: float
    expected_rate_deg_s: float
    actual_avg_rate_deg_s: float
    ratio: float


def load_real_data(csv_path: Path) -> list[RealDataPoint]:
    """从真机采集 CSV 加载数据"""
    points = []
    
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            axis = row.get("axis", "").strip()
            level = float(row.get("level", 0))
            phase = row.get("phase", "").strip()
            cmd_wz = float(row.get("cmd_wz", 0))
            yaw_deg = float(row.get("yaw_deg", 0))
            est_wz_deg_s = float(row.get("est_wz_deg_s", 0))
            phase_dyaw_deg = float(row.get("phase_dyaw_deg", 0))
            
            if axis == "w":  # 只看航向轴
                points.append(RealDataPoint(
                    axis=axis,
                    level=level,
                    phase=phase,
                    cmd=cmd_wz,
                    yaw_deg=yaw_deg,
                    est_wz_deg_s=est_wz_deg_s,
                    phase_dyaw_deg=phase_dyaw_deg
                ))
    
    return points


def load_sim_data(json_path: Path) -> list[SimulatorDataPoint]:
    """从模拟器 JSON 加载数据"""
    with open(json_path) as f:
        data = json.load(f)
    
    points = []
    for test in data.get("tests", []):
        points.append(SimulatorDataPoint(
            cmd_wz_rad_s=test["cmd_wz_rad_s"],
            expected_rate_deg_s=test["expected_rate_deg_s"],
            actual_avg_rate_deg_s=test["actual_avg_rate_deg_s"],
            ratio=test["ratio_actual_vs_expected"]
        ))
    
    return points


def analyze_real_data(points: list[RealDataPoint]) -> dict:
    """分析真机数据"""
    # 按指令幅值和方向分组
    by_level_and_sign = defaultdict(list)
    
    for point in points:
        # 只看 "动作" 阶段（不看 coast）
        if "coast" not in point.phase:
            sign = "+" if point.cmd > 0 else "-"
            key = (abs(point.level), sign)
            by_level_and_sign[key].append(point)
    
    analysis = {}
    for (level, sign), group in sorted(by_level_and_sign.items()):
        # 过滤有效数据（实际在动）
        valid = [p for p in group if abs(p.est_wz_deg_s) > 0.1]
        
        if valid:
            rates = [p.est_wz_deg_s for p in valid]
            avg_rate = sum(rates) / len(rates)
            
            analysis[f"level_{level}_{sign}"] = {
                "level": level,
                "sign": sign,
                "count": len(valid),
                "avg_rate_deg_s": avg_rate,
                "min_rate_deg_s": min(rates),
                "max_rate_deg_s": max(rates),
                "std_dev": (sum((r - avg_rate)**2 for r in rates) / len(rates))**0.5 if len(rates) > 1 else 0
            }
    
    return analysis


def compare_data(real_analysis: dict, sim_points: list[SimulatorDataPoint]) -> dict:
    """对比真机和模拟器数据"""
    print("""
╔════════════════════════════════════════════════════════════════════════════╗
║             SIMULATOR vs REAL ROBOT HEADING RESPONSE ANALYSIS              ║
╚════════════════════════════════════════════════════════════════════════════╝
    """)
    
    print("\n📊 REAL ROBOT DATA (from friction sampling):")
    print("-" * 80)
    for key, stats in sorted(real_analysis.items()):
        level = stats["level"]
        sign = stats["sign"]
        avg = stats["avg_rate_deg_s"]
        print(f"  Level {level:5.1f} {sign:1s}: avg={avg:7.1f} ± {stats['std_dev']:5.1f} deg/s  "
              f"(n={stats['count']:3d}, range={stats['min_rate_deg_s']:7.1f}..{stats['max_rate_deg_s']:7.1f})")
    
    print("\n🤖 SIMULATOR DATA (after fix):")
    print("-" * 80)
    for point in sorted(sim_points, key=lambda p: p.cmd_wz_rad_s):
        print(f"  cmd={point.cmd_wz_rad_s:+.3f} rad/s: expected={point.expected_rate_deg_s:7.1f}, "
              f"actual={point.actual_avg_rate_deg_s:7.1f} deg/s (ratio={point.ratio:.2f}x)")
    
    # 提取对应的真机数据用于对比
    print("\n🔍 CROSS-CHECK: Mapping real to simulator:")
    print("-" * 80)
    
    issues = []
    
    for sim_point in sim_points:
        # 将模拟器的 rad/s 转为 deg/s 指令幅值
        cmd_deg_s = sim_point.cmd_wz_rad_s * 57.2958
        sign = "+" if cmd_deg_s > 0 else "-"
        level = abs(cmd_deg_s)
        
        # 寻找最接近的真机数据
        matching_real = None
        for key, real_stats in real_analysis.items():
            if real_stats["sign"] == sign and abs(real_stats["level"] - level) < 5.0:
                matching_real = real_stats
                break
        
        if matching_real:
            ratio_real = matching_real["avg_rate_deg_s"] / level if level != 0 else 0
            ratio_sim = sim_point.actual_avg_rate_deg_s / cmd_deg_s if cmd_deg_s != 0 else 0
            
            print(f"  Sim cmd {cmd_deg_s:+6.1f} deg/s:")
            print(f"    - Real response: {matching_real['avg_rate_deg_s']:7.1f} deg/s (ratio={ratio_real:.2f})")
            print(f"    - Sim response:  {sim_point.actual_avg_rate_deg_s:7.1f} deg/s (ratio={ratio_sim:.2f})")
            print(f"    - Match: {abs(ratio_sim - ratio_real) < 0.2}")
            
            if abs(ratio_sim - ratio_real) > 0.3:
                issues.append(f"❌ Large response mismatch at {cmd_deg_s:+.1f} deg/s: "
                            f"sim={ratio_sim:.2f} vs real={ratio_real:.2f}")
    
    print("\n" + "="*80)
    if issues:
        print("⚠️  IDENTIFIED ISSUES:")
        for issue in issues:
            print(f"   {issue}")
    else:
        print("✅ Simulator response matches real robot!")
    print("="*80)
    
    return {
        "real_analysis": real_analysis,
        "sim_points": [
            {
                "cmd_wz_rad_s": p.cmd_wz_rad_s,
                "expected_rate_deg_s": p.expected_rate_deg_s,
                "actual_avg_rate_deg_s": p.actual_avg_rate_deg_s,
                "ratio": p.ratio
            }
            for p in sim_points
        ],
        "issues": issues
    }


def main():
    """主分析流程"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Compare simulator and real robot heading response")
    parser.add_argument("--real", type=Path, 
                       default=Path("Records/friction/friction_real_sampling_20260519_190045.csv"),
                       help="Real robot friction sampling CSV")
    parser.add_argument("--sim", type=Path, help="Simulator heading test JSON")
    
    args = parser.parse_args()
    
    # 加载数据
    if not args.real.exists():
        print(f"❌ Real data file not found: {args.real}")
        return 1
    
    print(f"📖 Loading real data from: {args.real}")
    real_points = load_real_data(args.real)
    print(f"   Loaded {len(real_points)} data points")
    
    if args.sim:
        if not args.sim.exists():
            print(f"❌ Simulator data file not found: {args.sim}")
            return 1
        
        print(f"📖 Loading simulator data from: {args.sim}")
        sim_points = load_sim_data(args.sim)
        print(f"   Loaded {len(sim_points)} data points")
    else:
        sim_points = []
        print("ℹ️  No simulator data provided (use --sim to compare)")
    
    # 分析真机数据
    real_analysis = analyze_real_data(real_points)
    
    # 对比
    if sim_points:
        comparison = compare_data(real_analysis, sim_points)
    else:
        print("\n📊 REAL ROBOT SUMMARY:")
        for key, stats in sorted(real_analysis.items()):
            print(f"  {key}: {stats}")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
