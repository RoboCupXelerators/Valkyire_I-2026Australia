#!/usr/bin/env bash
# Wrapper to start main.py under project virtualenv
set -e

# Adjust these paths if your project is in a different location
PROJECT_DIR="/home/user/caesear"
VENV_ACTIVATE="$PROJECT_DIR/venv/bin/activate"
MAIN_PY="$PROJECT_DIR/src/main.py"
LOG_DIR="$PROJECT_DIR/log"

mkdir -p "$LOG_DIR"

# Activate virtualenv
if [ -f "$VENV_ACTIVATE" ]; then
    # shellcheck disable=SC1090
    . "$VENV_ACTIVATE"
else
    echo "Virtualenv activate not found: $VENV_ACTIVATE" >&2
    exit 2
fi

# Ensure unbuffered output so systemd/journald receives logs promptly
export PYTHONUNBUFFERED=1
export ROS_DOMAIN_ID=99

cd "$PROJECT_DIR/src"

exec python -u "$MAIN_PY" >> "$LOG_DIR/main.log" 2>> "$LOG_DIR/main.err"
