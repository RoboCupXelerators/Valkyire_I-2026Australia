#!/bin/bash
# 运行测试脚本
# 使用方法: ./run_tests.sh [test_file]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"

# 设置 PYTHONPATH
export PYTHONPATH="$PROJECT_ROOT/src:$PYTHONPATH"

if [ -z "$1" ]; then
    # 运行所有测试
    echo "Running all tests..."
    python -m pytest src/tests/ -v
else
    # 运行指定测试
    echo "Running test: $1"
    python "$1"
fi