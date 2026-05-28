#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/common.sh"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/lib/stack.sh"
if [ "${TRASH_STACK_INTERNAL:-}" != "1" ]; then
  source_debug
  stack_dispatch navigation "$@"
  exit $?
fi
source_debug

ACTION="${1:-start}"
MAP_ARG="${2:-}"
LOG_DIR="$TRASH_ROBOT_LOG_DIR/navigation"
mkdir -p "$LOG_DIR"

resolve_map() {
  local map_file="$1"
  stack_resolve_map "$map_file"
}

topic_publishers() {
  local topic="$1"
  local count
  count="$(timeout 3s ros2 topic info "$topic" 2>/dev/null | awk '/Publisher count:/ {print $3; found=1} END {if (!found) print 0}')"
  if [ "${count:-0}" = "0" ]; then
    count="$(timeout 6s ros2 topic info "$topic" --verbose 2>/dev/null | awk '/Publisher count:/ {print $3; found=1; exit} END {if (!found) print 0}')"
  fi
  printf '%s\n' "${count:-0}"
}

wait_topic_publishers() {
  local topic="$1"
  local timeout_sec="${2:-20}"
  local waited=0
  while [ "$waited" -lt "$timeout_sec" ]; do
    if [ "$(topic_publishers "$topic")" -gt 0 ]; then
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
  done
  echo "ERROR $topic offline after ${timeout_sec}s" >&2
  return 1
}

action_online() {
  local action="$1"
  ros2 action list 2>/dev/null | grep -qx "$action"
}

node_online() {
  local node="$1"
  ros2 node list 2>/dev/null | grep -qx "$node"
}

wait_action_online() {
  local action="$1"
  local timeout_sec="${2:-20}"
  local waited=0
  while [ "$waited" -lt "$timeout_sec" ]; do
    if action_online "$action"; then
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
  done
  echo "ERROR $action action offline after ${timeout_sec}s" >&2
  return 1
}

wait_node_online() {
  local node="$1"
  local timeout_sec="${2:-20}"
  local waited=0
  while [ "$waited" -lt "$timeout_sec" ]; do
    if node_online "$node"; then
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
  done
  echo "ERROR $node node offline after ${timeout_sec}s" >&2
  return 1
}

navigation_map_ready() {
  [ "$(topic_publishers /map)" -gt 0 ] &&
    node_online /map_server &&
    node_online /amcl
}

navigation_goal_ready() {
  navigation_map_ready &&
    node_online /bt_navigator &&
    action_online /navigate_to_pose
}

ensure_navigation_arm() {
  if [ "${TRASH_NAV_PREPARE_ARM:-1}" != "1" ]; then
    echo "navigation arm preparation disabled by TRASH_NAV_PREPARE_ARM=0"
    return 0
  fi
  "$SCRIPT_DIR/prepare_navigation_arm.sh" start
}

ensure_base_stack() {
  if [ "$(topic_publishers /odom)" -gt 0 ] && [ "$(topic_publishers /scan)" -gt 0 ]; then
    echo "base hardware topics already online"
    return 0
  fi

  echo "base hardware topics are offline; starting base before navigation"
  TRASH_STACK_INTERNAL=1 "$SCRIPT_DIR/start_base.sh" start
  wait_topic_publishers /odom 25
  wait_topic_publishers /scan 25
}

camera_depth_ready() {
  [ "$(topic_publishers /camera/camera/depth/image_rect_raw)" -gt 0 ] &&
    [ "$(topic_publishers /camera/camera/depth/camera_info)" -gt 0 ]
}

ensure_depth_camera() {
  if camera_depth_ready; then
    return 0
  fi

  echo "camera depth topics are offline; starting D435i before navigation"
  TRASH_STACK_INTERNAL=1 "$SCRIPT_DIR/start_camera.sh" start

  local waited=0
  while [ "$waited" -lt 20 ]; do
    if camera_depth_ready; then
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
  done

  echo "ERROR D435i depth topics are still offline; refusing to start navigation with fake depth avoidance" >&2
  return 1
}

status() {
  echo "mode_lock: $(current_mode)"
  status_pid camera || true
  status_pid nav_depth_to_scan || true
  status_pid nav2 || true
  status_pid mission_supervisor || true
  for topic in /map /amcl_pose /odom /scan /camera/camera/depth/image_rect_raw /camera/camera/depth/camera_info /scan_depth /tf; do
    echo "$topic publishers=$(topic_publishers "$topic")"
  done
  for node in /map_server /amcl /planner_server /controller_server /bt_navigator; do
    echo -n "$node node: "
    node_online "$node" && echo ONLINE || echo OFFLINE
  done
  echo -n "navigate_to_pose action: "
  ros2 action list 2>/dev/null | grep -qx /navigate_to_pose && echo ONLINE || echo OFFLINE
}

start_navigation() {
  local map_file
  map_file="$(resolve_map "$1")"
  if [ ! -f "$map_file" ]; then
    echo "ERROR map not found: $map_file" >&2
    exit 1
  fi

  if ! ensure_navigation_arm; then
    echo "ERROR navigation arm preparation failed" >&2
    exit 1
  fi

  if navigation_goal_ready; then
    echo "navigation already running"
    echo "map: $map_file"
    echo "requires initial pose: ./scripts/init_pose.sh <x> <y> <yaw_deg>"
    echo "logs: $LOG_DIR"
    exit 0
  fi

  export TRASH_MODE_OWNER="start_navigation.sh"
  acquire_mode NAVIGATION
  echo "$map_file" > "$TRASH_ROBOT_RUNTIME/current_map.txt"

  if ! ensure_base_stack; then
    release_mode NAVIGATION
    exit 1
  fi
  if ! ensure_depth_camera; then
    release_mode NAVIGATION
    exit 1
  fi
  # Navigation and online SLAM must not publish /map at the same time.
  # A stale slam_toolbox publisher makes AMCL rebuild its map repeatedly and
  # Nav2 will often abort with zero-length plans.
  stop_mapping >/dev/null 2>&1 || true
  stop_nav >/dev/null 2>&1 || true
  acquire_mode NAVIGATION

  start_detached nav_depth_to_scan "$LOG_DIR/depth_to_scan.log" \
    ros2 launch trash_robot_bringup depth_to_scan.launch.py
  wait_topic_publishers /scan_depth 15 || echo "WARN /scan_depth offline; continuing with lidar /scan only"

  start_detached nav2 "$LOG_DIR/nav2.log" \
    ros2 launch trash_robot_bringup nav2_real_depth.launch.py map:="$map_file"
  if ! wait_topic_publishers /map 45; then
    release_mode NAVIGATION
    echo "ERROR map_server did not publish /map; see $LOG_DIR/nav2.log" >&2
    exit 1
  fi
  if ! wait_node_online /map_server 20 || ! wait_node_online /amcl 20; then
    release_mode NAVIGATION
    echo "ERROR map_server/amcl did not appear; see $LOG_DIR/nav2.log" >&2
    exit 1
  fi
  if ! wait_action_online /navigate_to_pose 45; then
    release_mode NAVIGATION
    echo "ERROR Nav2 action server is offline; see $LOG_DIR/nav2.log" >&2
    exit 1
  fi

  if [ "${TRASH_START_MISSION_SUPERVISOR:-1}" = "1" ]; then
    if pgrep -f "trash_robot_mission.*mission_supervisor" >/dev/null 2>&1; then
      echo "mission_supervisor already running"
    else
      start_detached mission_supervisor "$LOG_DIR/mission_supervisor.log" \
        ros2 launch trash_robot_mission mission_supervisor.launch.py \
        route_file:="$TRASH_ROBOT_ROOT/config/mission/patrol_routes.yaml" \
        sort_config_file:="$TRASH_ROBOT_ROOT/config/grasp/trash_sort_params.yaml" \
        auto_start:=false
    fi
  fi

  echo "navigation started"
  echo "map: $map_file"
  echo "requires initial pose: ./scripts/init_pose.sh <x> <y> <yaw_deg>"
  echo "logs: $LOG_DIR"
}

case "$ACTION" in
  start)
    start_navigation "$MAP_ARG"
    ;;
  stop)
    stop_nav >/dev/null 2>&1 || true
    echo "navigation stopped"
    ;;
  restart)
    stop_nav >/dev/null 2>&1 || true
    start_navigation "$MAP_ARG"
    ;;
  status)
    status
    ;;
  *)
    echo "usage: $0 start [map.yaml]|stop|restart [map.yaml]|status" >&2
    exit 2
    ;;
esac
