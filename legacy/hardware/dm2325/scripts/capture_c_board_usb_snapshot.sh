#!/usr/bin/env bash
set -euo pipefail

echo "=== C Board USB Snapshot ==="
echo "Date: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
echo

echo "--- lsusb ---"
lsusb || true
echo

echo "--- lsusb -t ---"
lsusb -t || true
echo

echo "--- /dev/ttyACM* ---"
ls -l /dev/ttyACM* 2>/dev/null || echo "(none)"
echo

echo "--- /dev/ttyUSB* ---"
ls -l /dev/ttyUSB* 2>/dev/null || echo "(none)"
echo

echo "--- /dev/serial/by-id/ ---"
ls -l /dev/serial/by-id/ 2>/dev/null || echo "(none)"
echo

echo "--- Recent kernel USB log (last 60 lines) ---"
journalctl -k --since "-10 minutes" --no-pager 2>/dev/null | grep -iE "usb|cdc|tty|acm|dfu|stm32" | tail -60 || echo "(no matching kernel log)"
echo

echo "--- Current user groups ---"
groups || true
echo

echo "--- USB device permissions ---"
ls -l /dev/bus/usb/*/* 2>/dev/null | head -20 || echo "(none)"
