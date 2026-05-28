#!/usr/bin/env bash
set -e

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
source_debug

ACTION="${1:-start}"
LOG_DIR="$TRASH_ROBOT_LOG_DIR/mapping"
mkdir -p "$LOG_DIR"

topic_publishers() {
  local topic="$1"
  timeout 3s ros2 topic info "$topic" 2>/dev/null | awk '/Publisher count:/ {print $3; found=1} END {if (!found) print 0}'
}

mapping_running() {
  status_pid mapping >/dev/null 2>&1 && return 0
  local pid=""
  [ -f "$TRASH_ROBOT_RUNTIME/mapping.pid" ] && pid="$(cat "$TRASH_ROBOT_RUNTIME/mapping.pid" 2>/dev/null || true)"
  [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null
}

start_mapping() {
  if mapping_running; then
    echo "mapping already running"
    status_mapping
    return 0
  fi
  stop_nav >/dev/null 2>&1 || true
  export TRASH_MODE_OWNER="start_mapping.sh"
  acquire_mode NAVIGATION
  set_runtime_mode "NAVIGATION" "start_mapping.sh" "slam_toolbox mapping"
  sleep 2

  PIDS=()
  cleanup() {
    local code=$?
    trap - EXIT INT TERM
    kill "${PIDS[@]}" 2>/dev/null || true
    rm -f "$TRASH_ROBOT_RUNTIME/mapping.pid"
    release_mode NAVIGATION
    exit "$code"
  }
  trap cleanup EXIT INT TERM

  write_pid mapping "$$"
  echo "$$" > "$TRASH_ROBOT_RUNTIME/mapping.pid"
  ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=false \
    slam_params_file:="$TRASH_ROBOT_ROOT/config/navigation/slam_toolbox.yaml" \
    > "$LOG_DIR/slam_toolbox.log" 2>&1 &
  PIDS+=("$!")
  echo "mapping started"
  echo "requires base/lidar to be running; Manager starts base before mapping"
  echo "logs: $LOG_DIR"
  while true; do sleep 2; done
}

status_mapping() {
  echo "mode_lock: $(current_mode)"
  status_pid mapping || true
  if [ -f "$TRASH_ROBOT_RUNTIME/mapping.pid" ]; then
    local pid
    pid="$(cat "$TRASH_ROBOT_RUNTIME/mapping.pid" 2>/dev/null || true)"
    echo "runtime/mapping.pid: ${pid:-empty}"
    [ -n "$pid" ] && ps -fp "$pid" 2>/dev/null || true
  fi
  echo "/map publishers=$(topic_publishers /map)"
  echo -n "slam_toolbox node: "
  ros2 node list 2>/dev/null | grep -qx /slam_toolbox && echo ONLINE || echo OFFLINE
}

case "$ACTION" in
  start)
    start_mapping
    ;;
  stop)
    stop_mapping >/dev/null 2>&1 || true
    echo "mapping stopped"
    ;;
  restart)
    stop_mapping >/dev/null 2>&1 || true
    start_mapping
    ;;
  status)
    status_mapping
    ;;
  *)
    echo "usage: $0 start|stop|restart|status" >&2
    exit 2
    ;;
esac
