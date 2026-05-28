#!/usr/bin/env bash
set +e

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"

if command -v flock >/dev/null 2>&1; then
  exec 9>"$TRASH_ROBOT_RUNTIME/stop_all.lock"
  flock -n 9 || exit 0
fi

stop_grasp >/dev/null 2>&1 || true
stop_video >/dev/null 2>&1 || true
stop_mapping >/dev/null 2>&1 || true
stop_nav >/dev/null 2>&1 || true
stop_base >/dev/null 2>&1 || true
stop_arm >/dev/null 2>&1 || true
stop_handeye >/dev/null 2>&1 || true
stop_camera >/dev/null 2>&1 || true
stop_pid video_rtp >/dev/null 2>&1 || true

rm -f "$TRASH_ROBOT_RUNTIME/video_rtp.pid"
rm -f "$TRASH_MODE_LOCK_FILE"
if ! estop_active; then
  rm -f "$TRASH_MOTION_LOCK_FILE"
fi
set_runtime_mode "IDLE" "stop_all.sh" "manager-owned components stopped"

echo "trash_robot_v3 stopped"
