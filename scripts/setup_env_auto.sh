#!/usr/bin/env bash
set -euo pipefail

# Auto bootstrap script for this repo.
#
# Goal:
# - Detect platform first
# - If Jetson (e.g. Jetson Nano/Orin family): setup python venv and (optionally) install CUDA/torch
#   and install Python deps from requirements_jetson.txt
# - If Raspberry Pi 5 + Ubuntu 24.04 LTS (Hailo runtime machine): setup hailort driver/runtime,
#   create venv and install Python deps from requirements_hailo.txt
#
# IMPORTANT:
# - This script is a NEW entry point. It does NOT delete/replace the original scripts:
#     - setup_venv.sh
#     - setup_venv_jetson_orin_nano.sh
#     - install_cuda.sh
# - Some steps (like switching apt sources.list) are intrusive; they are made optional where reasonable.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"
VENV_DIR="$PROJECT_DIR/venv"

REQ_JETSON="$PROJECT_DIR/requirements_jetson.txt"
REQ_HAILO="$PROJECT_DIR/requirements_hailo.txt"

USE_TSINGHUA_MIRROR_PI=1
USE_SYSTEM_SITE_PACKAGES=1
WITH_CUDA_ON_JETSON=0
INSTALL_HAILO_DEBS_ON_PI=1

usage() {
  cat <<'USAGE'
Usage:
  ./setup_env_auto.sh [options]

Options:
  --pi-no-mirror            Do NOT change /etc/apt/sources.list on Raspberry Pi path
  --no-system-site          Create venv WITHOUT --system-site-packages
  --jetson-with-cuda        On Jetson path: install JetPack/CUDA deps + install torch wheels (like install_cuda.sh)
  --pi-skip-hailo-debs      On Raspberry Pi path: skip installing hailort deb packages (Python deps still installed)
  -h, --help                Show this help

What it detects:
  - Jetson: /etc/nv_tegra_release exists OR /proc/device-tree/model contains "jetson" (case-insensitive)
  - Raspberry Pi 5: /proc/device-tree/model contains "Raspberry Pi 5" (case-insensitive)

Which requirements file is used:
  - Jetson -> requirements_jetson.txt
  - Raspberry Pi 5 (Hailo) -> requirements_hailo.txt

Notes:
  - You still can run the original scripts directly if you want:
      ./setup_venv.sh
      ./setup_venv_jetson_orin_nano.sh
      ./install_cuda.sh
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --pi-no-mirror) USE_TSINGHUA_MIRROR_PI=0; shift ;;
    --no-system-site) USE_SYSTEM_SITE_PACKAGES=0; shift ;;
    --jetson-with-cuda) WITH_CUDA_ON_JETSON=1; shift ;;
    --pi-skip-hailo-debs) INSTALL_HAILO_DEBS_ON_PI=0; shift ;;
    -h|--help) usage; exit 0 ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

read_model() {
  if [[ -r /proc/device-tree/model ]]; then
    tr -d '\0' </proc/device-tree/model
  else
    echo ""
  fi
}

is_jetson() {
  [[ -f /etc/nv_tegra_release ]] && return 0
  local model
  model="$(read_model)"
  echo "$model" | grep -qi "jetson"
}

is_rpi5() {
  local model
  model="$(read_model)"
  echo "$model" | grep -qi "raspberry pi 5"
}

print_sysinfo() {
  echo "[System info]"
  uname -a || true
  echo
  [[ -f /etc/os-release ]] && { echo "[os-release]"; cat /etc/os-release; echo; }
  echo "[arch] $(dpkg --print-architecture 2>/dev/null || echo unknown)"
  echo
  echo "[model] $(read_model)"
  echo
}

require_cmd() {
  local cmd="$1"
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "ERROR: required command not found: $cmd" >&2
    exit 1
  fi
}

switch_ubuntu_sources_to_tsinghua_24_04_ports() {
  # Intended for Raspberry Pi 5 Ubuntu 24.04 (arm64)
  if ! command -v lsb_release >/dev/null 2>&1; then
    sudo apt-get update
    sudo apt-get install -y lsb-release
  fi

  if ! lsb_release -d | grep -q "Ubuntu 24.04"; then
    echo "WARNING: This path is intended for Ubuntu 24.04 LTS on Raspberry Pi 5."
    echo "Current: $(lsb_release -d || true)"
    echo
  fi

  echo "Backing up /etc/apt/sources.list -> /etc/apt/sources.list.bak"
  sudo cp /etc/apt/sources.list /etc/apt/sources.list.bak

  cat >/tmp/sources.list.tsinghua <<'EOF'
# Ubuntu 24.04 LTS (Noble Numbat) - Tsinghua mirror (ARM64/Raspberry Pi)
# https://mirrors.tuna.tsinghua.edu.cn/help/ubuntu-ports/
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ noble main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ noble-updates main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ noble-backports main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ noble-security main restricted universe multiverse
EOF

  sudo mv /tmp/sources.list.tsinghua /etc/apt/sources.list
  sudo apt-get clean
  sudo apt-get update
}

ensure_apt_deps_common() {
  sudo apt-get update
  sudo apt-get install -y \
    build-essential pkg-config \
    libgl1 libglib2.0-0
}

ensure_python_venv_deps() {
  # On Ubuntu 24.04: python3 is 3.12; package name python3.12-venv exists
  # On Ubuntu 22.04: python3 is 3.10; package name python3-venv is enough
  sudo apt-get update
  if apt-cache show python3.12-venv >/dev/null 2>&1; then
    sudo apt-get install -y python3.12-venv python3-pip python3-dev
  else
    sudo apt-get install -y python3-venv python3-pip python3-dev
  fi
}

rebuild_venv() {
  require_cmd python3
  if [[ -d "$VENV_DIR" ]]; then
    echo "Backing up existing venv: $VENV_DIR -> ${VENV_DIR}.bak_YYYYmmdd_HHMMSS"
    mv "$VENV_DIR" "${VENV_DIR}.bak_$(date +%Y%m%d_%H%M%S)"
  fi

  local venv_args=()
  if [[ "$USE_SYSTEM_SITE_PACKAGES" -eq 1 ]]; then
    venv_args+=(--system-site-packages)
  fi

  echo "Creating venv at: $VENV_DIR"
  python3 -m venv "$VENV_DIR" "${venv_args[@]}"
  # shellcheck disable=SC1091
  source "$VENV_DIR/bin/activate"
  python -m pip install -U pip wheel setuptools
}

configure_pip_mirror_user() {
  local mirror_url="https://pypi.tuna.tsinghua.edu.cn/simple"
  echo "Configuring user-level pip mirror (~/.config/pip/pip.conf)"
  mkdir -p "$HOME/.config/pip"
  cat >"$HOME/.config/pip/pip.conf" <<EOF
[global]
index-url = ${mirror_url}
timeout = 120
trusted-host = pypi.tuna.tsinghua.edu.cn
EOF
}

install_requirements_from() {
  local req_file="$1"
  if [[ ! -f "$req_file" ]]; then
    echo "ERROR: requirements file not found: $req_file" >&2
    exit 1
  fi
  echo "Installing Python dependencies from: $req_file"
  python -m pip install -r "$req_file"
}

install_hailo_debs_pi() {
  # Assumes running on Raspberry Pi 5 (Ubuntu 24.04 arm64)
  if [[ "$INSTALL_HAILO_DEBS_ON_PI" -eq 0 ]]; then
    echo "Skipping Hailo deb install (--pi-skip-hailo-debs)."
    return 0
  fi

  echo "Installing Hailo runtime via local deb packages..."
  sudo apt-get update
  sudo apt-get install -y \
    linux-headers-$(uname -r) \
    linux-modules-extra-$(uname -r) \
    dkms

  if [[ ! -f "$PROJECT_DIR/hailort-pcie-driver_4.23.0_all.deb" || ! -f "$PROJECT_DIR/hailort_4.23.0_arm64.deb" ]]; then
    echo "ERROR: Hailo deb packages not found in project dir: $PROJECT_DIR" >&2
    echo "Expected files:" >&2
    echo "  - hailort-pcie-driver_4.23.0_all.deb" >&2
    echo "  - hailort_4.23.0_arm64.deb" >&2
    exit 1
  fi

  sudo dpkg -i "$PROJECT_DIR/hailort-pcie-driver_4.23.0_all.deb" || true
  sudo dpkg -i "$PROJECT_DIR/hailort_4.23.0_arm64.deb" || true
  sudo apt-get -f install -y
}

install_ch340_driver() {
  local installer="$SCRIPT_DIR/install_ch340_driver.sh"
  if [[ ! -x "$installer" ]]; then
    chmod +x "$installer"
  fi

  echo "Installing CH340/CH341 USB serial driver..."
  "$installer"
}

install_cuda_jetson() {
  # This matches install_cuda.sh but kept here so we can control sequence.
  echo "Installing JetPack (nvidia-jetpack) ..."
  sudo apt-get update
  sudo apt-get install -y nvidia-jetpack

  command -v nvcc || true
  ls -l /usr/local/cuda*/bin/nvcc 2>/dev/null || true

  # shellcheck disable=SC1091
  source "$VENV_DIR/bin/activate"

  # Clean old CPU-only torch if present
  python -m pip uninstall -y torch torchvision torchaudio || true

  echo "Installing torch/torchvision (JetPack6 + CUDA 12.6 wheels) ..."
  python -m pip install -U pip
  python -m pip install torch torchvision --index-url https://pypi.jetson-ai-lab.io/jp6/cu126

  # cuDSS repo keyring + package (CUDA 12.6)
  sudo apt-get update
  sudo apt-get install -y wget ca-certificates gnupg
  wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/arm64/cuda-keyring_1.1-1_all.deb
  sudo dpkg -i cuda-keyring_1.1-1_all.deb

  sudo apt-get update
  sudo apt-get install -y libcudss0-cuda-12
  sudo ldconfig

  echo "/usr/lib/aarch64-linux-gnu/libcudss/12" | sudo tee /etc/ld.so.conf.d/libcudss.conf >/dev/null
  sudo ldconfig
  ldconfig -p | grep -i cudss || true

  python -c "import torch; print(torch.__version__); print('torch.version.cuda =', torch.version.cuda); print('cuda available =', torch.cuda.is_available())" || true
}

setup_for_jetson() {
  echo "=== Detected platform: Jetson ==="
  echo "Using requirements: $REQ_JETSON"
  echo

  if [[ ! -f /etc/nv_tegra_release ]]; then
    echo "NOTE: /etc/nv_tegra_release not found; platform detected by device-tree model only."
    echo
  fi

  ensure_apt_deps_common
  ensure_python_venv_deps

  # Install the USB serial driver before other Jetson-specific setup.
  install_ch340_driver

  rebuild_venv
  configure_pip_mirror_user

  if [[ "$WITH_CUDA_ON_JETSON" -eq 1 ]]; then
    install_cuda_jetson
  else
    echo "Skipping CUDA install (use --jetson-with-cuda to install JetPack/CUDA + torch wheels)."
    echo
  fi

  # shellcheck disable=SC1091
  source "$VENV_DIR/bin/activate"
  install_requirements_from "$REQ_JETSON"
}

setup_for_rpi5_hailo() {
  echo "=== Detected platform: Raspberry Pi 5 (Hailo) ==="
  echo "Using requirements: $REQ_HAILO"
  echo

  # Hard guard requested by user: Raspberry Pi 5 + Ubuntu 24.04 LTS
  if command -v lsb_release >/dev/null 2>&1; then
    if ! lsb_release -d | grep -q "Ubuntu 24.04"; then
      echo "ERROR: Raspberry Pi path expects Ubuntu 24.04 LTS." >&2
      echo "Current: $(lsb_release -d || true)" >&2
      echo "If you are on another OS version, run the original scripts manually or adapt this script." >&2
      exit 1
    fi
  else
    echo "WARNING: lsb_release not found; cannot strictly verify Ubuntu 24.04." >&2
    echo "(Will continue; install 'lsb-release' to enable strict check.)" >&2
    echo
  fi

  if [[ "$USE_TSINGHUA_MIRROR_PI" -eq 1 ]]; then
    switch_ubuntu_sources_to_tsinghua_24_04_ports
  else
    echo "Keeping current apt sources.list (--pi-no-mirror)."
  fi

  ensure_apt_deps_common
  ensure_python_venv_deps
  install_hailo_debs_pi
  rebuild_venv
  configure_pip_mirror_user

  # shellcheck disable=SC1091
  source "$VENV_DIR/bin/activate"
  install_requirements_from "$REQ_HAILO"
}

main() {
  print_sysinfo

  # Ensure sudo will work early (to avoid half-done changes later).
  if ! sudo -n true 2>/dev/null; then
    echo "This script will run apt/dpkg steps and needs sudo. You may be prompted for password." 
    sudo true
  fi

  if is_rpi5; then
    setup_for_rpi5_hailo
  elif is_jetson; then
    setup_for_jetson
  else
    echo "ERROR: Unsupported device. Cannot detect Jetson or Raspberry Pi 5." >&2
    echo "Model: $(read_model)" >&2
    echo "If you are sure about your platform, please run the original scripts manually." >&2
    exit 1
  fi

  echo
  echo "Done."
  echo "Next time:"
  echo "  cd '$PROJECT_DIR' && source venv/bin/activate"
}

main "$@"
