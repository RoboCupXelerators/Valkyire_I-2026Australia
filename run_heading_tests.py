#!/usr/bin/env python
"""
模拟器航向角系统测试流程 - Windows/跨平台自动化脚本

用法:
    python run_heading_tests.py
    python run_heading_tests.py --skip-real-test
    python run_heading_tests.py --output-dir custom_output
"""

import sys
import os
import subprocess
import time
from pathlib import Path
from datetime import datetime


class Colors:
    """ANSI 颜色"""
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def print_header(text):
    """打印标题"""
    print(f"\n{Colors.BOLD}{Colors.BLUE}{'='*76}{Colors.ENDC}")
    print(f"{Colors.BOLD}{Colors.BLUE}  {text}{Colors.ENDC}")
    print(f"{Colors.BOLD}{Colors.BLUE}{'='*76}{Colors.ENDC}\n")


def print_step(num, text):
    """打印步骤标题"""
    print(f"\n{Colors.CYAN}STEP {num}: {text}{Colors.ENDC}")
    print(f"{Colors.CYAN}{'-'*76}{Colors.ENDC}")


def print_ok(text):
    """打印成功消息"""
    print(f"{Colors.GREEN}✅ {text}{Colors.ENDC}")


def print_error(text):
    """打印错误消息"""
    print(f"{Colors.RED}❌ {text}{Colors.ENDC}")


def print_warning(text):
    """打印警告消息"""
    print(f"{Colors.YELLOW}⚠️  {text}{Colors.ENDC}")


def check_file_exists(path, name):
    """检查文件是否存在"""
    if not Path(path).exists():
        print_error(f"{name} not found: {path}")
        return False
    print_ok(f"{name}: {path}")
    return True


def run_command(cmd, description="Running command"):
    """运行命令"""
    print(f"\n{description}...")
    print(f"Command: {cmd}\n")
    
    result = subprocess.run(cmd, shell=True)
    if result.returncode != 0:
        print_error(f"Command failed with exit code {result.returncode}")
        return False
    
    print_ok(f"{description} completed")
    return True


def main():
    """主程序"""
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Simulator heading angle control system - automated test suite"
    )
    parser.add_argument(
        "--output-dir",
        default="Records/heading_tests",
        help="Output directory for test results"
    )
    parser.add_argument(
        "--skip-real-test",
        action="store_true",
        help="Skip simulator test, only run analysis"
    )
    parser.add_argument(
        "--unity-ip",
        default=os.environ.get("XEL_UNITY_IP", "127.0.0.1"),
        help="Unity simulator IP address"
    )
    
    args = parser.parse_args()
    
    # 初始化
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    real_data_file = "Software/Records/friction/friction_real_sampling_20260519_190045.csv"
    
    print_header("SIMULATOR HEADING ANGLE CONTROL - AUTOMATED TEST SUITE")
    
    print(f"\n⚙️  Configuration:")
    print(f"   Unity IP:        {args.unity_ip}")
    print(f"   Output dir:      {output_dir}")
    print(f"   Real data:       {real_data_file}")
    print(f"   Timestamp:       {datetime.now().isoformat()}")
    
    # 前置检查
    print_step(0, "PRE-CHECK")
    
    all_ok = True
    all_ok &= check_file_exists(real_data_file, "Real robot data")
    all_ok &= check_file_exists("src/tests/test_sim_heading_fix.py", "Test script")
    all_ok &= check_file_exists("src/tests/analyze_sim_vs_real.py", "Analysis script")
    all_ok &= check_file_exists("src/tests/diagnose_heading_issues.py", "Diagnostic script")
    
    if not all_ok:
        print_error("Pre-check failed!")
        return 1
    
    # Step 1: 诊断分析
    print_step(1, "Running diagnostic analysis")
    
    diagnosis_log = output_dir / "diagnosis.log"
    cmd = f"python -m src.tests.diagnose_heading_issues > \"{diagnosis_log}\" 2>&1"
    if not run_command(cmd, "Diagnostic analysis"):
        print_warning("Diagnostic analysis failed, but continuing...")
    else:
        print_ok(f"Diagnosis log saved to: {diagnosis_log}")
    
    # Step 2: 模拟器测试（如果不跳过）
    if not args.skip_real_test:
        print_step(2, "Testing simulator heading response")
        print("This will take ~1-2 minutes depending on your test parameters")
        
        test_log = output_dir / "test.log"
        
        # 设置环境变量
        env_vars = f"XEL_UNITY_IP={args.unity_ip} " \
                  f"TEST_WZ_CMD_LEVELS=0.1,0.2,0.5,1.0 " \
                  f"TEST_HOLD_TIME=1.0 " \
                  f"TEST_COAST_TIME=0.5 " \
                  f"TEST_REPEAT=2 " \
                  f"TEST_SAMPLE_HZ=20"
        
        cmd = f"{env_vars} python -m src.tests.test_sim_heading_fix > \"{test_log}\" 2>&1"
        
        if not run_command(cmd, "Simulator test"):
            print_error("Simulator test failed!")
            print(f"Check logs at: {test_log}")
            return 1
        
        print_ok(f"Test log saved to: {test_log}")
        
        # 查找最新的 JSON 文件
        json_files = sorted(output_dir.glob("heading_test_*.json"), reverse=True)
        if not json_files:
            print_error("No test JSON file generated!")
            return 1
        
        latest_json = json_files[0]
        print_ok(f"Test data JSON: {latest_json}")
    else:
        # 查找现有的 JSON 文件
        json_files = sorted(output_dir.glob("heading_test_*.json"), reverse=True)
        if not json_files:
            print_error("No existing test data found and --skip-real-test specified!")
            return 1
        
        latest_json = json_files[0]
        print_ok(f"Using existing test data: {latest_json}")
    
    # Step 3: 对标分析
    print_step(3, "Comparing with real robot data")
    
    comparison_log = output_dir / "comparison.log"
    cmd = f"python -m src.tests.analyze_sim_vs_real " \
          f"--real \"{real_data_file}\" " \
          f"--sim \"{latest_json}\" > \"{comparison_log}\" 2>&1"
    
    if not run_command(cmd, "Analysis"):
        print_error("Analysis failed!")
        return 1
    
    print_ok(f"Comparison log saved to: {comparison_log}")
    
    # 最终总结
    print_header("TEST SUITE COMPLETED")
    
    print("\n📄 Output files:")
    print(f"   - {diagnosis_log}")
    print(f"   - {output_dir / 'test.log'}")
    print(f"   - {latest_json}")
    print(f"   - {latest_json.with_suffix('.csv')}")
    print(f"   - {comparison_log}")
    
    print("\n📊 Next steps:")
    print("   1. Review comparison.log to check if simulator matches real robot")
    print("   2. Look for ratio_actual_vs_expected values (should be close to 1.0)")
    print("   3. If ratio is not close to 1.0, check final_report.md for debugging tips")
    
    print(f"\n💾 All results saved to: {output_dir.absolute()}")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
