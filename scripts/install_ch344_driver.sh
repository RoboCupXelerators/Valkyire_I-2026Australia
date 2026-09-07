#!/usr/bin/env bash
set -euo pipefail

# WCH's ch343ser_linux driver supports CH344 and builds against the running
# kernel. The vendor driver creates /dev/ttyCH343USB0..3 for a quad UART.
REPO_URL="${CH344_DRIVER_REPO:-https://github.com/WCHSoftGroup/ch343ser_linux.git}"

usage() {
  cat <<'USAGE'
Usage:
  ./install_ch344_driver.sh [--dry-run]

Builds and installs WCH's ch343ser_linux driver, which supports CH344,
against the currently running kernel.
USAGE
}

DRY_RUN=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ "$DRY_RUN" -eq 1 ]]; then
  usage
  echo "[dry-run] Would clone $REPO_URL"
  echo "[dry-run] Would build driver/ch343.ko for kernel $(uname -r)"
  echo "[dry-run] Would run privileged 'make install' and 'modprobe ch343'"
  exit 0
fi

WORK_DIR="$(mktemp -d -t ch343ser.XXXXXX)"
cleanup() {
  rm -rf "$WORK_DIR"
}
trap cleanup EXIT

run_privileged() {
  if [[ ${EUID} -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

echo "Fetching WCH CH344-capable driver from $REPO_URL"
git clone --depth 1 "$REPO_URL" "$WORK_DIR/ch343ser_linux"

DRIVER_DIR="$WORK_DIR/ch343ser_linux/driver"
if [[ ! -f "$DRIVER_DIR/Makefile" ]]; then
  echo "ERROR: WCH driver source does not contain driver/Makefile" >&2
  exit 1
fi

make -C "$DRIVER_DIR"
run_privileged make -C "$DRIVER_DIR" install

echo "Loading CH344-capable ch343 module"
run_privileged modprobe ch343

echo
echo "Driver installed. Reconnect the adapter and check:"
echo "  lsusb"
echo "  ls -l /dev/ttyCH343USB* /dev/ttyACM* 2>/dev/null"
echo "Configure the device to use its corresponding serial channel."
