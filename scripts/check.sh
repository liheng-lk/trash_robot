#!/usr/bin/env bash
set +e
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
source_debug
FULL="${1:-}"

reclaim_runtime() {
  local apply="${1:-0}"
  echo "===== reclaim runtime ${apply} ====="
  local now
  now="$(date +%s)"
  local removed=0
  if [ "$apply" = "1" ]; then
    find /dev/shm -maxdepth 1 -user "$(id -un)" \( -name 'fastrtps_*' -o -name 'sem.fastrtps_*' -o -name 'fastdds_*' -o -name 'sem.fastdds_*' \) -print -delete 2>/dev/null | sed 's/^/removed shm /'
    find "$TRASH_ROBOT_RUNTIME/logs" -type f -mtime +14 -print -delete 2>/dev/null | sed 's/^/removed log /'
    find "$TRASH_ROBOT_RUNTIME/test_reports" -type f -mtime +30 -print -delete 2>/dev/null | sed 's/^/removed report /'
    find "$HOME/.ros/log" -type f -mtime +7 -print -delete 2>/dev/null | sed 's/^/removed roslog /'
  else
    find /dev/shm -maxdepth 1 -user "$(id -un)" \( -name 'fastrtps_*' -o -name 'sem.fastrtps_*' -o -name 'fastdds_*' -o -name 'sem.fastdds_*' \) -print 2>/dev/null | sed 's/^/would remove shm /'
    find "$TRASH_ROBOT_RUNTIME/logs" -type f -mtime +14 -print 2>/dev/null | sed 's/^/would remove log /'
    find "$TRASH_ROBOT_RUNTIME/test_reports" -type f -mtime +30 -print 2>/dev/null | sed 's/^/would remove report /'
    find "$HOME/.ros/log" -type f -mtime +7 -print 2>/dev/null | sed 's/^/would remove roslog /'
  fi
  echo "reclaim scan done at $now"
}

if [ "$FULL" = "--reclaim" ] || [ "$FULL" = "--reclaim-dry-run" ]; then
  reclaim_runtime 0
  exit 0
elif [ "$FULL" = "--reclaim-apply" ]; then
  reclaim_runtime 1
  exit 0
fi

echo "===== workspace ====="
echo "$TRASH_ROBOT_ROOT"
echo "ROS_DISTRO=${ROS_DISTRO:-unknown}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unknown}"

echo "===== resource guard ====="
root_used="$(df -P / | awk 'NR==2 {gsub(/%/,"",$5); print $5}')"
echo "root_used_percent=${root_used:-unknown}"
if [ -n "${root_used:-}" ] && [ "$root_used" -ge 95 ]; then
  echo "BLOCK root filesystem >=95%; run ./scripts/check.sh --reclaim-apply and clean build/logs before full robot run"
elif [ -n "${root_used:-}" ] && [ "$root_used" -ge 90 ]; then
  echo "WARN root filesystem >=90%; avoid heavy tests until cleaned"
fi
if ps -eo pcpu,args | awk '$1+0 > 10 && $0 ~ /update-manager/ {found=1} END {exit found?0:1}'; then
  echo "WARN update-manager cpu >10%; stop it before robot tests"
fi

echo "===== packages ====="
ros2 pkg list | grep -E "trash_robot|roarm|base_driver|sllidar" || true

echo "===== required files ====="
for f in \
  config/hardware/camera_realsense.yaml \
  src/trash_robot_bringup/launch/camera_realsense.launch.py \
  config/grasp/handeye_point.yaml \
  config/grasp/trash_sort_params.yaml \
  config/mission/patrol_routes.yaml \
  scripts/start_camera.sh \
  scripts/start_base.sh \
  scripts/start_arm.sh \
  scripts/start_navigation.sh \
  scripts/start_grasp.sh; do
  if [ -e "$TRASH_ROBOT_ROOT/$f" ]; then
    echo "OK   $f"
  else
    echo "MISS $f"
  fi
done

echo "===== nodes ====="
ros2 node list || true

echo "===== important topics ====="
ros2 topic list | grep -E "trash_|camera|cmd_vel|scan|map|amcl|tf|odom" || true

echo "===== camera status ====="
"$TRASH_ROBOT_ROOT/scripts/start_camera.sh" status || true

echo "===== services ====="
ros2 service list | grep -E "get_pose_cmd|move_point_cmd|trash_grasp|trash_mission|trash_manager|trash_safety" || true

echo "===== actions ====="
ros2 action list | grep -E "navigate_to_pose|trash_grasp_sort" || true

echo "===== process duplicates ====="
while read -r name pattern; do
  [ -z "$name" ] && continue
  count="$(pgrep -f "$pattern" 2>/dev/null | wc -l | tr -d ' ')"
  echo "$name count=$count"
done <<'EOF'
camera realsense2_camera_node
video light_mjpeg_streamer
grasp roarm_sort_grasper
mission mission_supervisor
manager trash_robot_manager.*robot_manager|/lib/trash_robot_manager/robot_manager
EOF

echo "===== system status ====="
timeout 3s ros2 topic echo /trash_system_status --once || true
timeout 3s ros2 topic echo /trash_resource_status --once || true

echo "===== VLM health ====="
if [ -f "$TRASH_ROBOT_ROOT/config/perception/vlm_trash_classifier.yaml" ]; then
  echo "OK   config/perception/vlm_trash_classifier.yaml"
else
  echo "MISS config/perception/vlm_trash_classifier.yaml"
fi
if [ -n "${DASHSCOPE_API_KEY:-}" ]; then
  echo "OK   DASHSCOPE_API_KEY configured"
else
  echo "WAIT DASHSCOPE_API_KEY not exported; VLM will fail closed"
fi

echo "===== quick target check ====="
timeout 4s ros2 topic echo /trash_target_point_camera --once || true
timeout 4s ros2 topic echo /trash_target_point_arm --once || true

if [ "$FULL" = "--full" ]; then
  echo "===== qos / hz smoke ====="
  for topic in /camera/camera/color/image_raw /camera/camera/aligned_depth_to_color/image_raw /scan /cmd_vel /trash_target_point_arm; do
    echo "--- $topic"
    ros2 topic info "$topic" --verbose 2>/dev/null | sed -n '1,80p' || true
  done

  echo "===== resource snapshot ====="
  df -h "$TRASH_ROBOT_ROOT" || true
  free -h || true
  ps -eo pid,ppid,pcpu,pmem,args --sort=-pcpu | head -30 || true
fi
