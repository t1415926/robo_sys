#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
USE_RVIZ="${USE_RVIZ:-true}"
BUILD_MODE="${BUILD_MODE:-auto}"

SYSTEM_PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

log() {
  printf '[start_sim_nav] %s\n' "$*"
}

die() {
  printf '[start_sim_nav] ERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Usage:
  ./start_sim_nav.sh [options]

Options:
  --no-rviz        Do not start RViz2.
  --build          Always rebuild before launch.
  --no-build       Do not build, only source install/setup.bash and launch.
  -h, --help       Show this help.

Environment:
  ROS_DISTRO       ROS distro name. Default: humble
  USE_RVIZ         true or false. Default: true
  BUILD_MODE       auto, always, or never. Default: auto

Examples:
  ./start_sim_nav.sh
  ./start_sim_nav.sh --no-rviz
  BUILD_MODE=always ./start_sim_nav.sh
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-rviz)
      USE_RVIZ=false
      shift
      ;;
    --build)
      BUILD_MODE=always
      shift
      ;;
    --no-build)
      BUILD_MODE=never
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "Unknown option: $1"
      ;;
  esac
done

[[ -f "${ROS_SETUP}" ]] || die "Cannot find ${ROS_SETUP}. Is ROS2 ${ROS_DISTRO} installed?"

if [[ -n "${CONDA_PREFIX:-}" ]]; then
  log "Detected conda environment: ${CONDA_PREFIX}"
  log "Using system Python environment for ROS2 in this script."
fi

unset PYTHONHOME
unset PYTHONPATH
export PATH="${SYSTEM_PATH}"

# shellcheck source=/dev/null
source "${ROS_SETUP}"

cd "${PROJECT_DIR}"

if ! command -v colcon >/dev/null 2>&1; then
  die "colcon is not installed or not on PATH."
fi

check_runtime_package() {
  local pkg="$1"
  if ! ros2 pkg prefix "${pkg}" >/dev/null 2>&1; then
    die "Missing ROS package: ${pkg}. Please install the dependencies listed in README.md."
  fi
}

check_runtime_package gazebo_ros
check_runtime_package gazebo_ros2_control
check_runtime_package nav2_map_server
check_runtime_package nav2_planner
check_runtime_package nav2_controller
check_runtime_package nav2_bt_navigator
check_runtime_package xacro

need_build=false
case "${BUILD_MODE}" in
  auto)
    if [[ ! -f "${PROJECT_DIR}/install/setup.bash" ]]; then
      need_build=true
    fi
    ;;
  always)
    need_build=true
    ;;
  never)
    need_build=false
    ;;
  *)
    die "BUILD_MODE must be auto, always, or never."
    ;;
esac

if [[ "${need_build}" == "true" ]]; then
  log "Building workspace..."
  colcon build --symlink-install
else
  log "Skipping build. Set BUILD_MODE=always or pass --build to rebuild."
fi

[[ -f "${PROJECT_DIR}/install/setup.bash" ]] || die "Missing install/setup.bash. Run with --build first."

# shellcheck source=/dev/null
source "${PROJECT_DIR}/install/setup.bash"

log "Launching simulation navigation stack..."
log "USE_RVIZ=${USE_RVIZ}"
exec ros2 launch my_robot_bringup sim_bringup.launch.py use_rviz:="${USE_RVIZ}"
