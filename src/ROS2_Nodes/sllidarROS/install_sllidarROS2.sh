#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PACKAGE_DIR="$SCRIPT_DIR/sllidar_ros2"
BUILD_DIR="$SCRIPT_DIR/build"
INSTALL_DIR="$SCRIPT_DIR/install"

if [[ ! -d "$PACKAGE_DIR/.git" ]]; then
    if [[ -e "$PACKAGE_DIR" ]]; then
        echo "Refusing to overwrite non-git directory: $PACKAGE_DIR" >&2
        exit 1
    fi
    git clone https://github.com/Slamtec/sllidar_ros2.git "$PACKAGE_DIR"
fi

ROS_DISTRO=${ROS_DISTRO:-}
if [[ -z "$ROS_DISTRO" ]]; then
    for setup in /opt/ros/*/setup.bash; do
        if [[ -f "$setup" ]]; then
            ROS_DISTRO=$(basename "$(dirname "$setup")")
            break
        fi
    done
fi
if [[ -z "$ROS_DISTRO" || ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
    echo "No ROS2 installation found under /opt/ros" >&2
    exit 1
fi

source "/opt/ros/$ROS_DISTRO/setup.bash"
set -u
if ! command -v colcon >/dev/null 2>&1; then
    echo "colcon is required; install it in the ROS2 environment first" >&2
    exit 1
fi

colcon build --symlink-install \
    --base-paths "$PACKAGE_DIR" \
    --build-base "$BUILD_DIR" \
    --install-base "$INSTALL_DIR"

echo "Built SLLidar ROS2 package: $INSTALL_DIR/setup.bash"
