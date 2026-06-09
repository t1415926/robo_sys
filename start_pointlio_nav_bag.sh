#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
BAG_PATH="${BAG_PATH:-/home/dtc/point_lio_ws/test_bag}"
USE_RVIZ="${USE_RVIZ:-true}"
BUILD_MODE="${BUILD_MODE:-auto}"
AUTO_GOAL="${AUTO_GOAL:-false}"
GOAL_X="${GOAL_X:-2.0}"
GOAL_Y="${GOAL_Y:-0.0}"
GOAL_YAW="${GOAL_YAW:-0.0}"

SYSTEM_PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

log() {
  printf '[start_pointlio_nav_bag] %s\n' "$*"
}

die() {
  printf '[start_pointlio_nav_bag] ERROR: %s\n' "$*" >&2
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
  changed="$(find "${PROJECT_DIR}/src" "${PROJECT_DIR}/start_pointlio_nav_bag.sh" \
    -type f -newer "${PROJECT_DIR}/install/setup.bash" -print -quit)"
  [[ -n "${changed}" ]]
}

usage() {
  cat <<'EOF'
Usage:
  ./start_pointlio_nav_bag.sh [options]

Options:
  --bag PATH       Bag directory. Default: /home/dtc/point_lio_ws/test_bag
  --no-rviz        Do not start RViz2.
  --build          Always rebuild before launch.
  --no-build       Do not build, only source install/setup.bash and launch.
  --goal X Y [YAW] Automatically send a Nav2 goal in the map frame.
  -h, --help       Show this help.

Examples:
  ./start_pointlio_nav_bag.sh --build
  ./start_pointlio_nav_bag.sh --build --goal 2.0 0.0 0.0
  ./start_pointlio_nav_bag.sh --bag /home/dtc/point_lio_ws/test_bag --no-rviz
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag)
      [[ $# -ge 2 ]] || die "--bag requires a path."
      BAG_PATH="$2"
      shift 2
      ;;
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
[[ -d "${BAG_PATH}" ]] || die "Cannot find bag directory: ${BAG_PATH}"

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

check_runtime_package nav2_map_server
check_runtime_package nav2_planner
check_runtime_package nav2_controller
check_runtime_package nav2_bt_navigator
check_runtime_package tf2_ros

if ! python3 -c 'import sensor_msgs_py.point_cloud2' >/dev/null 2>&1; then
  die "Missing Python module: sensor_msgs_py.point_cloud2. Please install ros-${ROS_DISTRO}-sensor-msgs-py."
fi

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
  colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble
else
  log "Skipping build. Set BUILD_MODE=always or pass --build to rebuild."
fi

[[ -f "${PROJECT_DIR}/install/setup.bash" ]] || die "Missing install/setup.bash. Run with --build first."

source_ros_setup "${PROJECT_DIR}/install/setup.bash"

check_runtime_package point_lio
check_runtime_package livox_ros_driver2

log "Launching PointLIO bag navigation stack..."
log "BAG_PATH=${BAG_PATH}"
log "USE_RVIZ=${USE_RVIZ}"
log "AUTO_GOAL=${AUTO_GOAL} GOAL_X=${GOAL_X} GOAL_Y=${GOAL_Y} GOAL_YAW=${GOAL_YAW}"
exec ros2 launch my_robot_bringup pointlio_nav_bag.launch.py \
  bag_path:="${BAG_PATH}" \
  use_rviz:="${USE_RVIZ}" \
  auto_goal:="${AUTO_GOAL}" \
  goal_x:="${GOAL_X}" \
  goal_y:="${GOAL_Y}" \
  goal_yaw:="${GOAL_YAW}"
