#!/usr/bin/env bash
set -euo pipefail

# probe_c_board.sh — Scan for the RoboMaster C board via USB.
# Read-only: does not send any commands to the C board or motors.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

echo "=== C Board USB Probe ==="
echo ""

# Show all USB devices
echo "[1/3] USB devices (lsusb):"
lsusb 2>/dev/null || echo "  lsusb not available"
echo ""

# Show serial device nodes
echo "[2/3] Serial device nodes:"
echo "  /dev/ttyACM*:"
ls -la /dev/ttyACM* 2>/dev/null || echo "    (none)"
echo "  /dev/ttyUSB*:"
ls -la /dev/ttyUSB* 2>/dev/null || echo "    (none)"
echo "  /dev/serial/by-id/:"
ls -la /dev/serial/by-id/* 2>/dev/null || echo "    (none)"
echo ""

# Show recent USB kernel log
echo "[3/3] Recent kernel USB events:"
if dmesg 2>/dev/null | grep -i "usb\|cdc\|tty" | tail -n 30; then
    :
else
    echo "  dmesg not accessible, trying journalctl..."
    journalctl -k --no-pager -n 100 2>/dev/null | grep -i "usb\|cdc\|tty" | tail -n 30 || echo "  (no USB events found)"
fi
echo ""

# Run Python probe
echo "=== Python device scan ==="
cd "$REPO_DIR"
PYTHONPATH="$REPO_DIR/src:$REPO_DIR" python3 -m tools.motor_control.cli probe
