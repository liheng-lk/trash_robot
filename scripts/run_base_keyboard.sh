#!/usr/bin/env bash
set -e

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
source_debug

ACTION="${1:-start}"

topic_publishers() {
  local topic="$1"
  timeout 3s ros2 topic info "$topic" 2>/dev/null | awk '/Publisher count:/ {print $3; found=1} END {if (!found) print 0}'
}

topic_subscribers() {
  local topic="$1"
  timeout 3s ros2 topic info "$topic" 2>/dev/null | awk '/Subscription count:/ {print $3; found=1} END {if (!found) print 0}'
}

ensure_base_ready() {
  local odom_pubs scan_pubs cmd_subs
  odom_pubs="$(topic_publishers /odom)"
  scan_pubs="$(topic_publishers /scan)"
  cmd_subs="$(topic_subscribers /cmd_vel)"
  if [ "${odom_pubs:-0}" -lt 1 ] || [ "${scan_pubs:-0}" -lt 1 ] || [ "${cmd_subs:-0}" -lt 1 ]; then
    echo "ERROR: 底盘未就绪：/odom pubs=$odom_pubs /scan pubs=$scan_pubs /cmd_vel subs=$cmd_subs" >&2
    echo "请先执行：./scripts/start_base.sh start" >&2
    return 1
  fi
}

ensure_safe_mode() {
  local mode
  mode="$(current_mode)"
  case "$mode" in
    IDLE)
      return 0
      ;;
    *)
      echo "ERROR: 当前模式锁为 $mode，禁止底盘手动遥控" >&2
      echo "请先停止导航/抓取/标定，并确认 mode_lock=IDLE。" >&2
      return 1
      ;;
  esac
}

case "$ACTION" in
  start|run)
    ensure_safe_mode
    ensure_base_ready
    exec python3 "$TRASH_ROBOT_ROOT/scripts/run_base_keyboard.py"
    ;;
  probe|status)
    echo "mode_lock=$(current_mode)"
    echo "/cmd_vel subscribers=$(topic_subscribers /cmd_vel)"
    echo "/odom publishers=$(topic_publishers /odom)"
    echo "/scan publishers=$(topic_publishers /scan)"
    python3 "$TRASH_ROBOT_ROOT/scripts/run_base_keyboard.py" --probe
    ;;
  *)
    echo "usage: $0 start|probe|status" >&2
    exit 2
    ;;
esac
