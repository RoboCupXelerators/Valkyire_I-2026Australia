#!/usr/bin/env bash
set -euo pipefail

# run_motor_probe.sh — Ping the real C board and query status.
# Does NOT send any motor control commands.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

cd "$REPO_DIR"

echo "=== C Board Probe (PING + Status) ==="
echo ""

PYTHONPATH="$REPO_DIR/src:$REPO_DIR" python3 -m tools.motor_control.cli status --verbose
