#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
USE_RVIZ="${USE_RVIZ:-true}"
BUILD_MODE="${BUILD_MODE:-auto}"
AUTO_GOAL="${AUTO_GOAL:-false}"
GOAL_X="${GOAL_X:-2.2}"
GOAL_Y="${GOAL_Y:-1.8}"
GOAL_YAW="${GOAL_YAW:-0.0}"

SYSTEM_PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

log() {
  printf '[start_gazebo_omni_nav] %s\n' "$*"
}

die() {
  printf '[start_gazebo_omni_nav] ERROR: %s\n' "$*" >&2
  exit 1
}

source_ros_setup() {
  local setup_file="$1"

  set +u
  # shellcheck source=/dev/null
  source "${setup_file}"
  set -u
}

workspace_sources_changed() {
  [[ -f "${PROJECT_DIR}/install/setup.bash" ]] || return 0

  local changed
  changed="$(find "${PROJECT_DIR}/src" "${PROJECT_DIR}/start_gazebo_omni_nav.sh" \
    -type f -newer "${PROJECT_DIR}/install/setup.bash" -print -quit)"
  [[ -n "${changed}" ]]
}

usage() {
  cat <<'EOF'
Usage:
  ./start_gazebo_omni_nav.sh [options]

Options:
  --no-rviz        Do not start RViz2.
  --build          Always rebuild before launch.
  --no-build       Do not build, only source install/setup.bash and launch.
  --goal X Y [YAW] Automatically send a Nav2 goal in the 2D map frame.
  -h, --help       Show this help.

Examples:
  ./start_gazebo_omni_nav.sh --build
  ./start_gazebo_omni_nav.sh --goal 1.0 -1.0 0.0
  ./start_gazebo_omni_nav.sh --no-rviz
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
    --goal)
      [[ $# -ge 3 ]] || die "--goal requires X and Y, with optional YAW."
      AUTO_GOAL=true
      GOAL_X="$2"
      GOAL_Y="$3"
      if [[ $# -ge 4 && "$4" != --* ]]; then
        GOAL_YAW="$4"
        shift 4
      else
        shift 3
      fi
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

source_ros_setup "${ROS_SETUP}"

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

check_runtime_package gazebo_msgs
check_runtime_package gazebo_ros
check_runtime_package nav2_map_server
check_runtime_package nav2_planner
check_runtime_package nav2_controller
check_runtime_package nav2_bt_navigator
check_runtime_package nav2_msgs
check_runtime_package robot_state_publisher
check_runtime_package tf2_ros
check_runtime_package xacro

need_build=false
case "${BUILD_MODE}" in
  auto)
    if [[ ! -f "${PROJECT_DIR}/install/setup.bash" ]]; then
      need_build=true
    elif workspace_sources_changed; then
      log "Detected source changes newer than install/setup.bash."
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

source_ros_setup "${PROJECT_DIR}/install/setup.bash"

log "Launching Gazebo omni navigation stack..."
log "USE_RVIZ=${USE_RVIZ}"
log "AUTO_GOAL=${AUTO_GOAL} GOAL_X=${GOAL_X} GOAL_Y=${GOAL_Y} GOAL_YAW=${GOAL_YAW}"
exec ros2 launch my_robot_bringup gazebo_omni_bringup.launch.py \
  use_rviz:="${USE_RVIZ}" \
  auto_goal:="${AUTO_GOAL}" \
  goal_x:="${GOAL_X}" \
  goal_y:="${GOAL_Y}" \
  goal_yaw:="${GOAL_YAW}"
