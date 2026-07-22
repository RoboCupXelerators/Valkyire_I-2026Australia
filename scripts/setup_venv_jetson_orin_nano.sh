#!/usr/bin/env bash
set -euo pipefail

# Jetson Orin Nano (JetPack 6 / Ubuntu 22.04 / Python 3.10) venv bootstrap.
#
# Background:
# - This repo codebase uses Python 3.10+ syntax (e.g. match/case, PEP604 "A | B" types).
# - Your current machine is Jetson Orin Nano (L4T R36.x, Ubuntu 22.04, Python 3.10).
# - You stated: Orin Nano will NOT run Hailo parts, so we skip any local hailort wheel line
#   in requirements.txt by default to avoid install failure.

if [ -z "${BASH_VERSION:-}" ]; then
  echo "ERROR: run this script with bash: bash scripts/setup_venv_jetson_orin_nano.sh" >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
VENV_DIR="$PROJECT_DIR/venv"
REQ_FILE="$PROJECT_DIR/requirements.txt"

USE_TSINGHUA_MIRROR=0
USE_SYSTEM_SITE_PACKAGES=1
SKIP_HAILO=1

usage() {
  cat <<'USAGE'
Usage:
  bash scripts/setup_venv_jetson_orin_nano.sh [options]

Options:
  --mirror-tsinghua        Switch ONLY Ubuntu apt sources.list to Tsinghua mirror (backup first)
  --no-system-site         Create venv WITHOUT --system-site-packages
  --with-hailo             Do NOT filter hailort wheel line in requirements.txt (advanced)
  -h, --help               Show this help

Notes:
  - This script does NOT remove/modify NVIDIA Jetson (L4T) apt source lists under
    /etc/apt/sources.list.d/.
  - Default behavior is to skip local hailort wheel line in requirements.txt.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mirror-tsinghua) USE_TSINGHUA_MIRROR=1; shift ;;
    --no-system-site) USE_SYSTEM_SITE_PACKAGES=0; shift ;;
    --with-hailo) SKIP_HAILO=0; shift ;;
    -h|--help) usage; exit 0 ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

print_sysinfo() {
  echo "[System info]"
  uname -a || true
  echo

  if [[ -f /etc/os-release ]]; then
    echo "[os-release]"
    cat /etc/os-release
    echo
  fi

  echo "[arch] $(dpkg --print-architecture 2>/dev/null || echo unknown)"
  echo

  echo "[python]"
  command -v python3 >/dev/null 2>&1 && python3 --version || echo "python3 not found"
  command -v python  >/dev/null 2>&1 && python  --version || true
  echo

  echo "[jetson]"
  if [[ -f /etc/nv_tegra_release ]]; then
    cat /etc/nv_tegra_release
  else
    echo "No /etc/nv_tegra_release (may not be a Jetson)"
  fi
  if [[ -r /proc/device-tree/model ]]; then
    echo -n "Model: "
    tr -d '\0' </proc/device-tree/model
    echo
  fi
  echo
}

get_os_codename() {
  # shellcheck disable=SC1091
  source /etc/os-release
  echo "${VERSION_CODENAME:-}"
}

switch_ubuntu_sources_to_tsinghua() {
  local codename
  codename="$(get_os_codename)"
  if [[ -z "$codename" ]]; then
    echo "Cannot determine Ubuntu codename from /etc/os-release" >&2
    return 1
  fi

  local arch
  arch="$(dpkg --print-architecture)"
  local base_url
  if [[ "$arch" == "arm64" || "$arch" == "armhf" ]]; then
    base_url="https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/"
  else
    base_url="https://mirrors.tuna.tsinghua.edu.cn/ubuntu/"
  fi

  echo "Backing up /etc/apt/sources.list -> /etc/apt/sources.list.bak"
  sudo cp /etc/apt/sources.list /etc/apt/sources.list.bak

  cat >/tmp/sources.list.tsinghua <<EOF
# Ubuntu ${codename} - Tsinghua mirror
deb ${base_url} ${codename} main restricted universe multiverse
deb ${base_url} ${codename}-updates main restricted universe multiverse
deb ${base_url} ${codename}-backports main restricted universe multiverse
deb ${base_url} ${codename}-security main restricted universe multiverse
EOF

  echo "Writing new /etc/apt/sources.list (NVIDIA L4T lists under /etc/apt/sources.list.d are kept)"
  sudo mv /tmp/sources.list.tsinghua /etc/apt/sources.list

  sudo apt-get clean
  sudo apt-get update
}

ensure_apt_deps() {
  sudo apt-get update
  sudo apt-get install -y \
    python3-venv python3-pip python3-dev \
    build-essential pkg-config \
    libgl1 libglib2.0-0
}

rebuild_venv() {
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

configure_pip_mirror() {
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

install_requirements() {
  if [[ ! -f "$REQ_FILE" ]]; then
    echo "No requirements.txt found at: $REQ_FILE"
    return 0
  fi

  local tmp_req
  tmp_req="$(mktemp)"
  cp "$REQ_FILE" "$tmp_req"

  # You said Orin Nano does NOT use Hailo in runtime; skip local hailort wheel line.
  if [[ "$SKIP_HAILO" -eq 1 ]]; then
    grep -v -E '^\s*\./hailort-.*\.whl\s*$' "$tmp_req" >"${tmp_req}.filtered" || true
    mv "${tmp_req}.filtered" "$tmp_req"
  fi

  echo "Installing Python dependencies from: $REQ_FILE"
  echo "Effective requirements temp file: $tmp_req"
  python -m pip install -r "$tmp_req"
  rm -f "$tmp_req"
}

python_version_guard() {
  local py_major py_minor
  py_major="$(python3 -c 'import sys; print(sys.version_info.major)')"
  py_minor="$(python3 -c 'import sys; print(sys.version_info.minor)')"
  if [[ "$py_major" -lt 3 || ( "$py_major" -eq 3 && "$py_minor" -lt 10 ) ]]; then
    echo "ERROR: This repo needs Python >= 3.10, but current is: $(python3 --version)" >&2
    echo "On Jetson Orin Nano (Ubuntu 22.04) you should have Python 3.10 by default." >&2
    exit 1
  fi
}

main() {
  print_sysinfo
  python_version_guard

  # Soft check: remind if not Orin Nano
  if [[ -r /proc/device-tree/model ]]; then
    local model
    model="$(tr -d '\0' </proc/device-tree/model)"
    if ! echo "$model" | grep -qi "orin nano"; then
      echo "Warning: This script is intended for Jetson Orin Nano, but model is: $model"
      echo "Continuing anyway."
      echo
    fi
  fi

  if [[ "$USE_TSINGHUA_MIRROR" -eq 1 ]]; then
    switch_ubuntu_sources_to_tsinghua
  else
    echo "Keeping current apt sources (use --mirror-tsinghua to switch Ubuntu sources)."
    echo
  fi

  ensure_apt_deps
  rebuild_venv
  configure_pip_mirror
  install_requirements

  echo
  echo "Done."
  echo "Python: $(python --version)"
  echo "Venv:   $VIRTUAL_ENV"
  echo
  echo "Next time:"
  echo "  cd '$PROJECT_DIR' && source venv/bin/activate"
}

main "$@"
