#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/common.sh"
if [ "${TRASH_STACK_INTERNAL:-}" != "1" ]; then
  # shellcheck disable=SC1091
  source "$SCRIPT_DIR/lib/stack.sh"
  source_debug
  stack_dispatch base "$@"
  exit $?
fi
source_debug

ACTION="${1:-start}"
LOG_DIR="$TRASH_ROBOT_LOG_DIR/base"
mkdir -p "$LOG_DIR"

topic_publishers() {
  local topic="$1"
  timeout 3s ros2 topic info "$topic" 2>/dev/null | awk '/Publisher count:/ {print $3; found=1} END {if (!found) print 0}'
}

status() {
  status_pid base || true
  for topic in /cmd_vel /odom /scan /tf; do
    echo "$topic publishers=$(topic_publishers "$topic")"
  done
}

start_base() {
  if status_pid base >/dev/null 2>&1; then
    if [ "$(topic_publishers /odom)" -gt 0 ] && [ "$(topic_publishers /scan)" -gt 0 ]; then
      echo "base already running"
      status
      return 0
    fi
    echo "WARN: stale base pid without /odom or /scan; restarting" >&2
    stop_base >/dev/null 2>&1 || true
    sleep 2
  fi
  mkdir -p "$LOG_DIR"
  start_detached base "$LOG_DIR/base_lidar_tf.log" \
    ros2 launch trash_robot_bringup robot_bringup.launch.py
  echo "base/lidar/tf started"
  echo "logs: $LOG_DIR/base_lidar_tf.log"
}

case "$ACTION" in
  start)
    start_base
    ;;
  stop)
    stop_base >/dev/null 2>&1 || true
    echo "base stopped"
    ;;
  restart)
    stop_base >/dev/null 2>&1 || true
    start_base
    ;;
  status)
    status
    ;;
  *)
    echo "usage: $0 start|stop|restart|status" >&2
    exit 2
    ;;
esac
