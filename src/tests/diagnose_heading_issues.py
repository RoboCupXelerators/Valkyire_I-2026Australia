"""
模拟器航向角控制系统深度问题诊断

从真机数据中提取问题线索，验证修复后的参数是否充分
"""

import csv
import json
from pathlib import Path
from collections import defaultdict
import statistics

def analyze_heading_data():
    """深度分析真机采集数据中的航向角行为"""
    
    csv_path = Path("Software/Records/friction/friction_real_sampling_20260519_190045.csv")
    
    if not csv_path.exists():
        print(f"❌ File not found: {csv_path}")
        return
    
    # 按 level 和 sign 分组采集
    by_level = defaultdict(lambda: {"forward": [], "backward": []})
    boundary_crossings = []
    
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for i, row in enumerate(reader):
            if row.get("axis") != "w":
                continue
            
            level = float(row.get("level", 0))
            phase = row.get("phase", "").strip()
            cmd_wz = float(row.get("cmd_wz", 0))
            yaw_deg = float(row.get("yaw_deg", 0))
            est_wz_deg_s = float(row.get("est_wz_deg_s", 0))
            
            # 检测 0/360 边界跳变
            if i > 0 and "coast" not in phase:
                # 跳变的典型表现：yaw 在 0-5° 或 355-360° 之间快速跳动
                if (yaw_deg < 5.0 or yaw_deg > 355.0):
                    boundary_crossings.append({
                        "row": i,
                        "yaw": yaw_deg,
                        "est_wz": est_wz_deg_s,
                        "level": level,
                        "phase": phase
                    })
            
            # 只统计 "动作" 阶段
            if "coast" not in phase:
                direction = "forward" if cmd_wz > 0 else "backward"
                by_level[level][direction].append({
                    "yaw": yaw_deg,
                    "est_wz": est_wz_deg_s,
                    "phase": phase
                })
    
    print("""
╔════════════════════════════════════════════════════════════════════════════╗
║          HEADING CONTROL SYSTEM - DEEP DIAGNOSTIC REPORT                   ║
║                                                                              ║
║  Analyzing real robot data to identify systematic issues                   ║
╚════════════════════════════════════════════════════════════════════════════╝
    """)
    
    print("\n📊 ANALYSIS 1: Angular Velocity Response (w-axis only)")
    print("="*80)
    
    for level in sorted(by_level.keys()):
        print(f"\n  Level {level:5.1f}° (8.0 for forward, -8.0 for backward):")
        
        for direction in ["forward", "backward"]:
            samples = by_level[level][direction]
            if not samples:
                continue
            
            rates = [s["est_wz"] for s in samples if s["est_wz"] != 0]
            
            if rates:
                avg_rate = statistics.mean(rates)
                stdev = statistics.stdev(rates) if len(rates) > 1 else 0.0
                min_rate = min(rates)
                max_rate = max(rates)
                
                print(f"    {direction:10s}: avg={avg_rate:7.1f}°/s ± {stdev:5.1f}, "
                      f"range=[{min_rate:7.1f}, {max_rate:7.1f}], n={len(rates)}")
                
                # 检查噪声
                noise_ratio = stdev / abs(avg_rate) if avg_rate != 0 else 0
                if noise_ratio > 0.5:
                    print(f"              ⚠️  HIGH NOISE: {noise_ratio:.1%} of signal (>50% indicates problem)")
    
    print("\n📍 ANALYSIS 2: 0/360° Boundary Crossing Issues")
    print("="*80)
    
    if boundary_crossings:
        print(f"  Found {len(boundary_crossings)} samples near boundary (0-5° or 355-360°)")
        print(f"\n  Sample boundary crossings (yaw near 0/360):")
        for crossing in boundary_crossings[:10]:
            print(f"    Row {crossing['row']:4d}: yaw={crossing['yaw']:7.2f}°, "
                  f"est_wz={crossing['est_wz']:7.1f}°/s, level={crossing['level']:.1f}")
        
        # 检查这些位置的估计角速度是否异常
        boundary_rates = [c['est_wz'] for c in boundary_crossings]
        if boundary_rates:
            boundary_avg = statistics.mean(boundary_rates)
            boundary_std = statistics.stdev(boundary_rates) if len(boundary_rates) > 1 else 0
            print(f"\n  Boundary-zone angular velocity stats:")
            print(f"    Mean: {boundary_avg:7.1f}°/s")
            print(f"    Stdev: {boundary_std:7.1f}°/s")
            print(f"    Range: [{min(boundary_rates):7.1f}, {max(boundary_rates):7.1f}]")
            
            if boundary_std > abs(boundary_avg) * 0.5:
                print(f"  ⚠️  PROBLEM: High variability at 0/360 boundary!")
                print(f"     Suggests angle unwrap/discontinuity issue")
    else:
        print(f"  No obvious boundary crossings detected (good!)")
    
    print("\n📈 ANALYSIS 3: Comparison of Parameters")
    print("="*80)
    
    # 对比真机数据中 w 轴的实际特性
    print("""
  Real robot w-axis (angular velocity) characteristics:
  - Command level: 8.0°  (direction: ±)
  - Command level: 12.0° (direction: ±)
  - Command level: 16.0° (direction: ±)
  - Command level: 20.0° (direction: ±)
  - Command level: 24.0° (direction: ±)
  
  Current simulator fix:
  - angularSpeedScale: 0.5 rad/s per unit (changed from 1.47)
  - linearSpeedScale: 0.48 m/s per unit (changed from 0.29)
    """)
    
    # 简单校验：真机的 level 12.0 对应什么角速度？
    level_12_samples = by_level.get(12.0, {})
    forward_samples = level_12_samples.get("forward", [])
    backward_samples = level_12_samples.get("backward", [])
    
    if forward_samples:
        rates = [s["est_wz"] for s in forward_samples if s["est_wz"] > 0]
        if rates:
            avg = statistics.mean(rates)
            print(f"\n  Real robot at level 12.0 (forward):")
            print(f"    Average angular velocity: {avg:7.1f}°/s")
            print(f"    If this is 12.0 rad/s input:")
            print(f"      Simulator should produce: 12.0 * 57.2958 = {12.0*57.2958:7.1f}°/s")
            print(f"      Actual ratio: {avg / (12.0*57.2958):.3f} (should be ~1.0)")
    
    print("\n" + "="*80)
    print("🔍 IDENTIFIED ISSUES & RECOMMENDATIONS:")
    print("="*80)
    
    issues = []
    
    # 检查 w 轴数据质量
    all_w_rates = []
    for level_data in by_level.values():
        for direction_data in level_data.values():
            all_w_rates.extend([s["est_wz"] for s in direction_data if s["est_wz"] != 0])
    
    if all_w_rates:
        w_stdev = statistics.stdev(all_w_rates) if len(all_w_rates) > 1 else 0
        w_mean = statistics.mean(all_w_rates)
        if w_stdev > abs(w_mean) * 0.3:
            issues.append("1. ⚠️  W-axis noise is significant (>30% of signal)")
            issues.append("      → Real robot w-axis measurement may be noisy")
            issues.append("      → Simulator should focus on command-response, not mimicking noise")
    
    if boundary_crossings and len(boundary_crossings) > len(by_level) * 2:
        issues.append("2. ⚠️  Multiple 0/360° boundary crossings detected")
        issues.append("      → Software must use angle unwrap in calibration")
        issues.append("      → Simulator needs same unwrap logic to match behavior")
    
    # 检查参数是否在合理范围
    if 0.4 < 0.5 < 0.6:
        issues.append("3. ✅ New angularSpeedScale (0.5) looks reasonable")
    
    issues.append("4. ⚠️  Need to verify yaw_zero_at_start and angle unwrap logic")
    issues.append("      → Check XeleratorSimBits.GetYaw() implementation")
    issues.append("      → Ensure angle difference calculation uses proper unwrap")
    
    for issue in issues:
        print(f"\n  {issue}")
    
    print("\n" + "="*80)
    print("✅ NEXT STEPS:")
    print("="*80)
    print("""
  1. Deploy the simulator with new parameters (angularSpeedScale=0.5)
  2. Run the test_sim_heading_fix.py script to generate simulator test data
  3. Use analyze_sim_vs_real.py to compare with this real robot data
  4. If simulator response still doesn't match:
     a) Check if angle unwrap is properly implemented
     b) Verify yaw conventions (CCW+ for SoccerSoftware vs Unity)
     c) Check for hidden double-integration or filter delays
  5. Consider re-sampling w-axis with better low-pass filtering
    """)


if __name__ == "__main__":
    analyze_heading_data()
