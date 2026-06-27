#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
BUILD_MODE="${BUILD_MODE:-auto}"
USE_RVIZ="${USE_RVIZ:-true}"
AUTO_GOAL="${AUTO_GOAL:-false}"
GOAL_X="${GOAL_X:-2.0}"
GOAL_Y="${GOAL_Y:-0.0}"
GOAL_YAW="${GOAL_YAW:-0.0}"
MAP_FILE="${MAP_FILE:-${PROJECT_DIR}/src/my_robot_navigation/maps/real_site_map.yaml}"
PRIOR_MAP_PCD="${PRIOR_MAP_PCD:-${PROJECT_DIR}/src/pointlio/PCD/2026-06-25_19-00-45_scans.pcd}"
POINTLIO_PARAM_FILE="${POINTLIO_PARAM_FILE:-${PROJECT_DIR}/src/my_robot_bringup/config/pointlio_mid360_nav.yaml}"
POSE_GRAPH_PARAM_FILE="${POSE_GRAPH_PARAM_FILE:-${PROJECT_DIR}/src/pointlio/config/pose_graph_mid360s.yaml}"
NAV2_PARAM_FILE="${NAV2_PARAM_FILE:-${PROJECT_DIR}/src/my_robot_navigation/config/nav2_pointcloud_params.yaml}"
SYSTEM_PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

DRIVER_PID=""
POINTLIO_PID=""
BASE_LINK_TF_PID=""
MAP_TO_BASE_TF_PID=""
POSE_GRAPH_PID=""
NAV2_PID=""

log() {
  printf '[start_real_site_planning] %s\n' "$*"
}

die() {
  printf '[start_real_site_planning] ERROR: %s\n' "$*" >&2
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
  changed="$(find "${PROJECT_DIR}/src" "${PROJECT_DIR}/start_real_site_planning.sh" \
    -type f -newer "${PROJECT_DIR}/install/setup.bash" -print -quit)"
  [[ -n "${changed}" ]]
}

check_runtime_package() {
  local pkg="$1"
  if ! ros2 pkg prefix "${pkg}" >/dev/null 2>&1; then
    die "Missing ROS package: ${pkg}."
  fi
}

usage() {
  cat <<'EOF'
Usage:
  ./start_real_site_planning.sh [options]

Options:
  --map PATH        2D map yaml for map_server. Default: src/my_robot_navigation/maps/real_site_map.yaml
  --pcd PATH        Prior point cloud map for Point-LIO. Default: /home/nvidia/robo_sys/src/pointlio/PCD/2026-06-25_19-00-45_scans.pcd
  --pointlio PATH   Point-LIO params yaml. Default: src/my_robot_bringup/config/pointlio_mid360_nav.yaml
  --nav2 PATH       Nav2 params yaml. Default: src/my_robot_navigation/config/nav2_pointcloud_params.yaml
  --pose-graph PATH Pose graph backend params yaml. Default: src/pointlio/config/pose_graph_mid360s.yaml
  --no-rviz         Do not start RViz.
  --build           Always rebuild before launch.
  --no-build        Do not build, only source install/setup.bash and launch.
  --goal X Y [YAW]  Automatically send one Nav2 goal in the map frame.
  -h, --help        Show this help.

Examples:
  ./start_real_site_planning.sh
  ./start_real_site_planning.sh --goal 2.0 0.0 0.0
  ./start_real_site_planning.sh --map /path/to/real_site_map.yaml --pcd /path/to/2026-06-25_19-00-45_scans.pcd
EOF
}

shutdown_children() {
  log "Stopping planning stack..."
  for pid_var in NAV2_PID BASE_LINK_TF_PID MAP_TO_BASE_TF_PID POSE_GRAPH_PID POINTLIO_PID DRIVER_PID; do
    pid="${!pid_var}"
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -INT "${pid}" 2>/dev/null || true
    fi
  done
}

finalize() {
  local exit_code=$?
  trap - EXIT INT TERM

  for pid_var in NAV2_PID BASE_LINK_TF_PID MAP_TO_BASE_TF_PID POSE_GRAPH_PID POINTLIO_PID DRIVER_PID; do
    pid="${!pid_var}"
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      wait "${pid}" 2>/dev/null || true
    fi
  done

  exit "${exit_code}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --map)
      [[ $# -ge 2 ]] || die "--map requires a path."
      MAP_FILE="$2"
      shift 2
      ;;
    --pcd)
      [[ $# -ge 2 ]] || die "--pcd requires a path."
      PRIOR_MAP_PCD="$2"
      shift 2
      ;;
    --pointlio)
      [[ $# -ge 2 ]] || die "--pointlio requires a yaml path."
      POINTLIO_PARAM_FILE="$2"
      shift 2
      ;;
    --nav2)
      [[ $# -ge 2 ]] || die "--nav2 requires a yaml path."
      NAV2_PARAM_FILE="$2"
      shift 2
      ;;
    --pose-graph)
      [[ $# -ge 2 ]] || die "--pose-graph requires a yaml path."
      POSE_GRAPH_PARAM_FILE="$2"
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
[[ -f "${MAP_FILE}" ]] || die "Map yaml does not exist: ${MAP_FILE}"
[[ -f "${PRIOR_MAP_PCD}" ]] || die "Prior map PCD does not exist: ${PRIOR_MAP_PCD}"
[[ -f "${POINTLIO_PARAM_FILE}" ]] || die "Point-LIO params file does not exist: ${POINTLIO_PARAM_FILE}"
[[ -f "${POSE_GRAPH_PARAM_FILE}" ]] || die "Pose graph params file does not exist: ${POSE_GRAPH_PARAM_FILE}"
[[ -f "${NAV2_PARAM_FILE}" ]] || die "Nav2 params file does not exist: ${NAV2_PARAM_FILE}"

unset PYTHONHOME
unset PYTHONPATH
export PATH="${SYSTEM_PATH}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:/usr/local/lib"

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
  colcon build --packages-select livox_ros_driver2 point_lio my_robot_navigation my_robot_bringup
else
  log "Skipping build. Set BUILD_MODE=always or pass --build to rebuild."
fi

[[ -f "${PROJECT_DIR}/install/setup.bash" ]] || die "Missing install/setup.bash. Run with --build first."
source_ros_setup "${PROJECT_DIR}/install/setup.bash"

check_runtime_package livox_ros_driver2
check_runtime_package point_lio
check_runtime_package tf2_ros
check_runtime_package nav2_map_server
check_runtime_package nav2_planner
check_runtime_package nav2_controller
check_runtime_package nav2_bt_navigator

trap shutdown_children INT TERM
trap finalize EXIT

log "Launching Livox MID360s driver..."
ros2 launch livox_ros_driver2 msg_MID360s_launch.py &
DRIVER_PID=$!

sleep 3
if ! kill -0 "${DRIVER_PID}" 2>/dev/null; then
  die "Livox driver exited early."
fi

log "Launching Point-LIO prior-map localization..."
log "PRIOR_MAP_PCD=${PRIOR_MAP_PCD}"
ros2 run point_lio pointlio_mapping \
  --ros-args \
  --params-file "${POINTLIO_PARAM_FILE}" \
  -p use_sim_time:=false \
  -p common.use_prior_map:=false \
  -p pcd_save.pcd_save_en:=false &
POINTLIO_PID=$!

sleep 2
if ! kill -0 "${POINTLIO_PID}" 2>/dev/null; then
  die "Point-LIO exited early."
fi

log "Launching pose graph backend..."
ros2 run point_lio pose_graph_backend \
  --ros-args \
  --params-file "${POSE_GRAPH_PARAM_FILE}" \
  -p use_sim_time:=false \
  -p prior.map_path:="${PRIOR_MAP_PCD}" &
POSE_GRAPH_PID=$!

sleep 2
if ! kill -0 "${POSE_GRAPH_PID}" 2>/dev/null; then
  die "Pose graph backend exited early."
fi

log "Publishing static TF aft_mapped -> base_footprint..."
ros2 run tf2_ros static_transform_publisher --frame-id aft_mapped --child-frame-id base_footprint &
MAP_TO_BASE_TF_PID=$!

log "Publishing static TF base_footprint -> base_link..."
ros2 run tf2_ros static_transform_publisher --frame-id base_footprint --child-frame-id base_link &
BASE_LINK_TF_PID=$!

log "Launching Nav2 planning stack..."
log "MAP_FILE=${MAP_FILE}"
log "NAV2_PARAM_FILE=${NAV2_PARAM_FILE}"
ros2 launch my_robot_navigation navigation.launch.py \
  use_sim_time:=false \
  map:="${MAP_FILE}" \
  params_file:="${NAV2_PARAM_FILE}" \
  use_map_server:=true \
  use_rviz:="${USE_RVIZ}" \
  auto_goal:="${AUTO_GOAL}" \
  goal_x:="${GOAL_X}" \
  goal_y:="${GOAL_Y}" \
  goal_yaw:="${GOAL_YAW}" &
NAV2_PID=$!

wait "${NAV2_PID}"
