#!/usr/bin/env bash
# Runtime chain check — PASS/WARN/FAIL with timeouts. Read-only, no hardware motion.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT/scripts/common.sh"
# shellcheck disable=SC1091
source "$ROOT/scripts/lib/stack.sh"
source_debug

TIMEOUT="${TRASH_CHECK_TIMEOUT:-4}"
FAIL=0
WARN=0

check_topic_pub() {
  local topic="$1"
  local required="${2:-1}"
  local count
  count="$(stack_topic_publishers "$topic")"
  if [ "${count:-0}" -gt 0 ]; then
    echo "PASS $topic publishers=$count"
    return 0
  fi
  if [ "$required" = "1" ]; then
    echo "FAIL $topic no publisher"
    FAIL=$((FAIL + 1))
  else
    echo "WARN $topic no publisher"
    WARN=$((WARN + 1))
  fi
  return 1
}

echo "===== check_robot_runtime $(date -Iseconds) ====="
echo "mode=$(current_mode) ros_domain=${ROS_DOMAIN_ID:-?}"

check_topic_pub /odom 0
check_topic_pub /scan 0
check_topic_pub /camera/camera/color/image_raw 0
check_topic_pub /camera/camera/aligned_depth_to_color/image_raw 0
check_topic_pub /camera/camera/color/camera_info 0
check_topic_pub /trash_grasp_plan 0
check_topic_pub /trash_target_camera_point 0
check_topic_pub /trash_target_point_camera 0
check_topic_pub /trash_target_point_arm 0

if [ -f "$ROOT/config/grasp/handeye_point.yaml" ]; then
  echo "PASS handeye_point.yaml exists"
else
  echo "FAIL handeye_point.yaml missing"
  FAIL=$((FAIL + 1))
fi

if timeout "$TIMEOUT" ros2 service list 2>/dev/null | grep -qx /trash_manager/start_base; then
  echo "PASS manager services"
else
  echo "WARN manager services not listed"
  WARN=$((WARN + 1))
fi

echo "===== summary: FAIL=$FAIL WARN=$WARN ====="
if [ "$FAIL" -gt 0 ]; then
  exit 1
fi
exit 0
