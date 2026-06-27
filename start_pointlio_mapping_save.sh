#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
BUILD_MODE="${BUILD_MODE:-auto}"
USE_RVIZ="${USE_RVIZ:-true}"
POINTLIO_PARAM_FILE="${POINTLIO_PARAM_FILE:-${PROJECT_DIR}/src/my_robot_bringup/config/pointlio_mid360_nav.yaml}"
POINTLIO_SAVE_FILE="${PROJECT_DIR}/src/pointlio/PCD/scans.pcd"
TIMESTAMP="$(date +%F_%H-%M-%S)"
MAP_OUTPUT="${MAP_OUTPUT:-${PROJECT_DIR}/src/pointlio/PCD/${TIMESTAMP}_scans.pcd}"
PCD_SAVE_INTERVAL="${PCD_SAVE_INTERVAL:--1}"
RVIZ_CONFIG_FILE="${RVIZ_CONFIG_FILE:-${PROJECT_DIR}/src/pointlio/rviz_cfg/loam_livox.rviz}"
SYSTEM_PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

DRIVER_PID=""
POINTLIO_PID=""
RVIZ_PID=""
SHUTDOWN_REQUESTED=false

log() {
  printf '[start_pointlio_mapping_save] %s\n' "$*"
}

die() {
  printf '[start_pointlio_mapping_save] ERROR: %s\n' "$*" >&2
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
  changed="$(find "${PROJECT_DIR}/src" "${PROJECT_DIR}/start_pointlio_mapping_save.sh" \
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
  ./start_pointlio_mapping_save.sh [options]

Options:
  --output PATH      Final saved PCD path. Default: src/pointlio/PCD/<timestamp>_scans.pcd
  --params PATH      Point-LIO parameter yaml. Default: src/my_robot_bringup/config/pointlio_mid360_nav.yaml
  --interval N       Point-LIO pcd_save.interval. Default: -1 (save one full map on exit)
  --no-rviz          Do not start RViz monitoring.
  --rviz-config PATH RViz config file. Default: src/pointlio/rviz_cfg/loam_livox.rviz
  --build            Always rebuild before launch.
  --no-build         Do not build, only source install/setup.bash and launch.
  -h, --help         Show this help.

Examples:
  ./start_pointlio_mapping_save.sh
  ./start_pointlio_mapping_save.sh --build
  ./start_pointlio_mapping_save.sh --no-rviz
  ./start_pointlio_mapping_save.sh --output /home/nvidia/ssd/maps/site_a.pcd
EOF
}

shutdown_children() {
  SHUTDOWN_REQUESTED=true
  log "Stopping Point-LIO and Livox driver..."
  if [[ -n "${POINTLIO_PID}" ]] && kill -0 "${POINTLIO_PID}" 2>/dev/null; then
    kill -INT "${POINTLIO_PID}" 2>/dev/null || true
  fi
  if [[ -n "${DRIVER_PID}" ]] && kill -0 "${DRIVER_PID}" 2>/dev/null; then
    kill -INT "${DRIVER_PID}" 2>/dev/null || true
  fi
  if [[ -n "${RVIZ_PID}" ]] && kill -0 "${RVIZ_PID}" 2>/dev/null; then
    kill -INT "${RVIZ_PID}" 2>/dev/null || true
  fi
}

finalize() {
  local exit_code=$?
  trap - EXIT INT TERM

  if [[ -n "${POINTLIO_PID}" ]]; then
    wait "${POINTLIO_PID}" 2>/dev/null || true
  fi

  if [[ -n "${DRIVER_PID}" ]] && kill -0 "${DRIVER_PID}" 2>/dev/null; then
    kill -INT "${DRIVER_PID}" 2>/dev/null || true
    wait "${DRIVER_PID}" 2>/dev/null || true
  fi

  if [[ -n "${RVIZ_PID}" ]] && kill -0 "${RVIZ_PID}" 2>/dev/null; then
    kill -INT "${RVIZ_PID}" 2>/dev/null || true
    wait "${RVIZ_PID}" 2>/dev/null || true
  fi

  if [[ -f "${POINTLIO_SAVE_FILE}" ]]; then
    mkdir -p "$(dirname "${MAP_OUTPUT}")"
    cp -f "${POINTLIO_SAVE_FILE}" "${MAP_OUTPUT}"
    log "Saved point cloud map to: ${MAP_OUTPUT}"
    log "Point-LIO raw save file remains at: ${POINTLIO_SAVE_FILE}"
  else
    log "No PCD map found at ${POINTLIO_SAVE_FILE}. The run may have ended before Point-LIO flushed the map."
  fi

  exit "${exit_code}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output)
      [[ $# -ge 2 ]] || die "--output requires a path."
      MAP_OUTPUT="$2"
      shift 2
      ;;
    --params)
      [[ $# -ge 2 ]] || die "--params requires a yaml path."
      POINTLIO_PARAM_FILE="$2"
      shift 2
      ;;
    --interval)
      [[ $# -ge 2 ]] || die "--interval requires an integer."
      PCD_SAVE_INTERVAL="$2"
      shift 2
      ;;
    --no-rviz)
      USE_RVIZ=false
      shift
      ;;
    --rviz-config)
      [[ $# -ge 2 ]] || die "--rviz-config requires a path."
      RVIZ_CONFIG_FILE="$2"
      shift 2
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
[[ -f "${POINTLIO_PARAM_FILE}" ]] || die "Cannot find Point-LIO params file: ${POINTLIO_PARAM_FILE}"
if [[ "${USE_RVIZ}" == "true" ]]; then
  [[ -f "${RVIZ_CONFIG_FILE}" ]] || die "Cannot find RViz config file: ${RVIZ_CONFIG_FILE}"
fi

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
  colcon build --packages-select livox_ros_driver2 point_lio my_robot_bringup
else
  log "Skipping build. Set BUILD_MODE=always or pass --build to rebuild."
fi

[[ -f "${PROJECT_DIR}/install/setup.bash" ]] || die "Missing install/setup.bash. Run with --build first."
source_ros_setup "${PROJECT_DIR}/install/setup.bash"

check_runtime_package livox_ros_driver2
check_runtime_package point_lio
if [[ "${USE_RVIZ}" == "true" ]]; then
  check_runtime_package rviz2
fi

mkdir -p "${PROJECT_DIR}/src/pointlio/PCD"
rm -f "${POINTLIO_SAVE_FILE}"

trap shutdown_children INT TERM
trap finalize EXIT

log "Launching Livox MID360s driver..."
ros2 launch livox_ros_driver2 msg_MID360s_launch.py &
DRIVER_PID=$!

sleep 3

if ! kill -0 "${DRIVER_PID}" 2>/dev/null; then
  die "Livox driver exited early."
fi

log "Launching Point-LIO mapping with PCD save enabled..."
log "POINTLIO_PARAM_FILE=${POINTLIO_PARAM_FILE}"
log "MAP_OUTPUT=${MAP_OUTPUT}"
log "PCD_SAVE_INTERVAL=${PCD_SAVE_INTERVAL}"
ros2 run point_lio pointlio_mapping \
  --ros-args \
  --params-file "${POINTLIO_PARAM_FILE}" \
  -p use_sim_time:=false \
  -p pcd_save.pcd_save_en:=true \
  -p pcd_save.interval:="${PCD_SAVE_INTERVAL}" &
POINTLIO_PID=$!

if [[ "${USE_RVIZ}" == "true" ]]; then
  sleep 2
  log "Launching RViz monitoring..."
  log "RVIZ_CONFIG_FILE=${RVIZ_CONFIG_FILE}"
  rviz2 -d "${RVIZ_CONFIG_FILE}" &
  RVIZ_PID=$!
fi

wait "${POINTLIO_PID}"
