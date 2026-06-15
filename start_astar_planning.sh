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
  printf '[start_astar_planning] %s\n' "$*"
}

die() {
  printf '[start_astar_planning] ERROR: %s\n' "$*" >&2
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
  changed="$(find "${PROJECT_DIR}/src" "${PROJECT_DIR}/start_astar_planning.sh" \
    -type f -newer "${PROJECT_DIR}/install/setup.bash" -print -quit)"
  [[ -n "${changed}" ]]
}

usage() {
  cat <<'EOF'
Usage:
  ./start_astar_planning.sh [options]

Options:
  --no-rviz        Do not start RViz2.
  --build          Always rebuild before launch.
  --no-build       Do not build, only source install/setup.bash and launch.
  --goal X Y [YAW] Publish one /goal_pose for A* planning.
  -h, --help       Show this help.

Examples:
  ./start_astar_planning.sh --build
  ./start_astar_planning.sh --goal 2.2 1.8 0.0
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

unset PYTHONHOME
unset PYTHONPATH
export PATH="${SYSTEM_PATH}"

source_ros_setup "${ROS_SETUP}"
cd "${PROJECT_DIR}"

if ! command -v colcon >/dev/null 2>&1; then
  die "colcon is not installed or not on PATH."
fi

need_build=false
case "${BUILD_MODE}" in
  auto)
    if [[ ! -f "${PROJECT_DIR}/install/setup.bash" ]] || workspace_sources_changed; then
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

log "Launching A* planning demo without Nav2 planner/controller..."
exec ros2 launch my_robot_bringup astar_sim_bringup.launch.py \
  use_rviz:="${USE_RVIZ}" \
  auto_goal:="${AUTO_GOAL}" \
  goal_x:="${GOAL_X}" \
  goal_y:="${GOAL_Y}" \
  goal_yaw:="${GOAL_YAW}"
