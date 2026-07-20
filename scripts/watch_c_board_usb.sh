#!/usr/bin/env bash
set -euo pipefail

echo '=== Initial USB devices ==='
lsusb || true
lsusb -t || true

echo
echo 'Now connect the powered RoboMaster C board with a known-good MicroUSB data cable.'
echo 'Watching kernel events. Press Ctrl+C to stop.'
echo

sudo udevadm monitor --kernel --udev --property
