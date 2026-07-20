#!/usr/bin/env bash
set -euo pipefail

# run_motor_mock.sh — Run motor control in mock mode (no hardware needed).
# Simulates the C board and DM2325 responses.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

cd "$REPO_DIR"

echo "=== Motor Control Mock Test ==="
echo ""

echo "[1/2] Running protocol unit tests..."
PYTHONPATH="$REPO_DIR/src:$REPO_DIR" python3 -m pytest tests/test_protocol.py tests/test_safety.py tests/test_mock_motor_control.py -v
echo ""

echo "[2/2] Running mock speed command..."
PYTHONPATH="$REPO_DIR/src:$REPO_DIR" python3 -m tools.motor_control.cli speed \
    --motor-id 1 --target 0.5 --duration 1.0 --mock --dry-run --verbose
echo ""

echo "=== Mock test complete ==="
