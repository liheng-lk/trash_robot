#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
source_runtime

usage() {
  cat <<'USAGE'
Usage:
  ./scripts/start_estop.sh trigger [reason]
  ./scripts/start_estop.sh reset
  ./scripts/start_estop.sh status

Aliases:
  start -> trigger
  stop  -> reset
USAGE
}

publish_zero_velocity_burst() {
  local topic
  local msg="{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
  if ! command -v ros2 >/dev/null 2>&1; then
    echo "WARN: ros2 not found; ESTOP lock written but zero velocity was not published" >&2
    return 0
  fi
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    for topic in /cmd_vel /trash_robot_v3/base/cmd_vel; do
      timeout 2s ros2 topic pub --once "$topic" geometry_msgs/msg/Twist "$msg" >/dev/null 2>&1 || true
    done
    sleep 0.05
  done
}

cmd="${1:-status}"
case "$cmd" in
  start|trigger)
    reason="${2:-manual}"
    trigger_estop "$reason"
    acquire_mode ESTOP
    publish_zero_velocity_burst
    echo "ESTOP triggered: $reason"
    ;;
  stop|reset)
    reset_estop
    release_mode ESTOP
    echo "ESTOP reset"
    ;;
  status)
    echo "mode=$(current_mode)"
    motion_status
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
