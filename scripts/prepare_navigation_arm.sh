#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/common.sh"
source_debug

ACTION="${1:-start}"
LOG_DIR="$TRASH_ROBOT_LOG_DIR/arm"
mkdir -p "$LOG_DIR"

NAV_ARM_START_ENABLED="${TRASH_NAV_ARM_START_ENABLED:-1}"
NAV_ARM_INIT_ENABLED="${TRASH_NAV_ARM_INIT_ENABLED:-1}"
NAV_ARM_INIT_X="${TRASH_NAV_ARM_INIT_X:-118.1718722}"
NAV_ARM_INIT_Y="${TRASH_NAV_ARM_INIT_Y:--0.725102626}"
NAV_ARM_INIT_Z="${TRASH_NAV_ARM_INIT_Z:-32.39033932}"
NAV_ARM_INIT_UNIT="${TRASH_NAV_ARM_INIT_UNIT:-mm}"
NAV_ARM_MOVE_TIMEOUT="${TRASH_NAV_ARM_MOVE_TIMEOUT:-25}"
NAV_ARM_SERVICE_WAIT_SEC="${TRASH_NAV_ARM_SERVICE_WAIT_SEC:-45}"

service_ready() {
  local service="$1"
  ros2 service list 2>/dev/null | grep -qx "$service"
}

wait_for_service() {
  local service="$1"
  local timeout_sec="${2:-20}"
  local waited=0
  while [ "$waited" -lt "$timeout_sec" ]; do
    if service_ready "$service"; then
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
  done
  echo "ERROR $service service offline after ${timeout_sec}s" >&2
  return 1
}

arm_services_ready() {
  service_ready /move_point_cmd && service_ready /get_pose_cmd
}

arm_coord_for_service() {
  local value="$1"
  case "$NAV_ARM_INIT_UNIT" in
    mm|millimeter|millimeters)
      awk -v value="$value" 'BEGIN { printf "%.9f", value / 1000.0 }'
      ;;
    m|meter|meters)
      awk -v value="$value" 'BEGIN { printf "%.9f", value }'
      ;;
    *)
      echo "ERROR unsupported TRASH_NAV_ARM_INIT_UNIT=$NAV_ARM_INIT_UNIT (use mm or m)" >&2
      return 1
      ;;
  esac
}

ensure_arm_services() {
  if [ "$NAV_ARM_START_ENABLED" != "1" ]; then
    echo "navigation arm service startup disabled by TRASH_NAV_ARM_START_ENABLED=0"
    return 0
  fi

  if arm_services_ready; then
    echo "arm services already online"
    return 0
  fi

  echo "arm services are offline; starting arm control before navigation"
  if ! TRASH_STACK_INTERNAL=1 "$SCRIPT_DIR/start_arm.sh" start; then
    echo "WARN arm start reported failure; MoveIt may still be booting, waiting for services"
  fi
  wait_for_service /move_point_cmd "$NAV_ARM_SERVICE_WAIT_SEC"
  wait_for_service /get_pose_cmd "$NAV_ARM_SERVICE_WAIT_SEC"
}

move_navigation_initial_pose() {
  if [ "$NAV_ARM_INIT_ENABLED" != "1" ]; then
    echo "navigation arm initial pose disabled by TRASH_NAV_ARM_INIT_ENABLED=0"
    return 0
  fi

  if ! arm_services_ready; then
    echo "ERROR arm services are not ready; cannot set navigation initial pose" >&2
    return 1
  fi

  local cmd_x cmd_y cmd_z
  cmd_x="$(arm_coord_for_service "$NAV_ARM_INIT_X")"
  cmd_y="$(arm_coord_for_service "$NAV_ARM_INIT_Y")"
  cmd_z="$(arm_coord_for_service "$NAV_ARM_INIT_Z")"

  acquire_motion_lock NAV_ARM_INIT
  local result=0
  timeout "${NAV_ARM_MOVE_TIMEOUT}s" ros2 service call /move_point_cmd roarm_moveit/srv/MovePointCmd \
    "{x: ${cmd_x}, y: ${cmd_y}, z: ${cmd_z}}" \
    > "$LOG_DIR/navigation_initial_pose.log" 2>&1 || result=$?
  release_motion_lock NAV_ARM_INIT

  if [ "$result" -ne 0 ]; then
    echo "ERROR failed to set navigation arm initial pose; see $LOG_DIR/navigation_initial_pose.log" >&2
    tail -n 80 "$LOG_DIR/navigation_initial_pose.log" 2>/dev/null || true
    return "$result"
  fi
  if ! grep -Eiq 'success[:=][[:space:]]*true|success=True' "$LOG_DIR/navigation_initial_pose.log"; then
    echo "ERROR /move_point_cmd did not report success; see $LOG_DIR/navigation_initial_pose.log" >&2
    tail -n 80 "$LOG_DIR/navigation_initial_pose.log" 2>/dev/null || true
    return 1
  fi

  echo "arm navigation initial pose set source_${NAV_ARM_INIT_UNIT}=(${NAV_ARM_INIT_X},${NAV_ARM_INIT_Y},${NAV_ARM_INIT_Z}) service_m=(${cmd_x},${cmd_y},${cmd_z})"
  return 0
}

status() {
  local cmd_x cmd_y cmd_z
  cmd_x="$(arm_coord_for_service "$NAV_ARM_INIT_X" 2>/dev/null || echo invalid)"
  cmd_y="$(arm_coord_for_service "$NAV_ARM_INIT_Y" 2>/dev/null || echo invalid)"
  cmd_z="$(arm_coord_for_service "$NAV_ARM_INIT_Z" 2>/dev/null || echo invalid)"
  echo "navigation arm init: enabled=$NAV_ARM_INIT_ENABLED unit=$NAV_ARM_INIT_UNIT source=($NAV_ARM_INIT_X,$NAV_ARM_INIT_Y,$NAV_ARM_INIT_Z) service_m=($cmd_x,$cmd_y,$cmd_z)"
  echo -n "/move_point_cmd: "
  service_ready /move_point_cmd && echo ONLINE || echo OFFLINE
  echo -n "/get_pose_cmd: "
  service_ready /get_pose_cmd && echo ONLINE || echo OFFLINE
}

case "$ACTION" in
  start)
    ensure_arm_services
    move_navigation_initial_pose
    ;;
  status)
    status
    ;;
  *)
    echo "usage: $0 start|status" >&2
    exit 2
    ;;
esac
