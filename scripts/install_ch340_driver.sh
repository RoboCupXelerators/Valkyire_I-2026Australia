#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODULE_NAME="ch341"
MODULE_SRC="$SCRIPT_DIR/${MODULE_NAME}.ko"
MODULE_DEST="/lib/modules/$(uname -r)/extra/${MODULE_NAME}.ko"
MODULE_CONF="/etc/modules-load.d/${MODULE_NAME}.conf"

usage() {
  cat <<'USAGE'
Usage:
  ./install_ch340_driver.sh [--dry-run]

Installs the compiled CH340/CH341 USB serial kernel module for the current Jetson kernel.
The module file is expected at: ./ch341.ko
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

run_sudo() {
  if [[ $EUID -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

if [[ ! -f "$MODULE_SRC" ]]; then
  echo "ERROR: module file not found: $MODULE_SRC" >&2
  exit 1
fi

echo "Installing ${MODULE_NAME} USB serial driver for kernel $(uname -r)"

if [[ "$DRY_RUN" -eq 1 ]]; then
  echo "[dry-run] Would install $MODULE_SRC -> $MODULE_DEST"
  echo "[dry-run] Would run: depmod -a"
  echo "[dry-run] Would run: modprobe $MODULE_NAME"
  exit 0
fi

run_sudo mkdir -p "$(dirname "$MODULE_DEST")"
run_sudo install -m 0644 "$MODULE_SRC" "$MODULE_DEST"
run_sudo depmod -a
run_sudo sh -c "printf '%s\n' '$MODULE_NAME' > '$MODULE_CONF'"

if ! run_sudo modprobe "$MODULE_NAME"; then
  echo "modprobe failed, trying insmod directly..." >&2
  run_sudo insmod "$MODULE_DEST"
fi

echo "CH340/CH341 driver installed and loaded."
