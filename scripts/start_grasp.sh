#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/common.sh"
if [ "${TRASH_STACK_INTERNAL:-}" != "1" ]; then
  # shellcheck disable=SC1091
  source "$SCRIPT_DIR/lib/stack.sh"
  source_debug
  case "${1:-dry}" in
    dry|live) stack_dispatch grasp start "$@" ;;
    stop|status|restart) stack_dispatch grasp "$1" "${2:-}" ;;
    *) stack_dispatch grasp "$@" ;;
  esac
  exit $?
fi
source_debug

ACTION="${1:-dry}"
MODE="${2:-}"
VLM_PROVIDER="${3:-${TRASH_VLM_PROVIDER:-}}"

LOG_ROOT="$TRASH_ROBOT_LOG_DIR/grasp_vlm"
HANDEYE_FILE="$TRASH_ROBOT_ROOT/config/grasp/handeye_point.yaml"

usage() {
  echo "usage: $0 dry|live|stop|status|restart [dry|live] [provider]" >&2
}

reject_removed_backend() {
  case " ${*:-} " in
    *" bpu "*|*" coco "*|*" mono2d "*|*" trash4 "*)
      echo "ERROR: 当前阶段只启用 VLM API；BPU/COCO 本地模型后续接入" >&2
      exit 2
      ;;
  esac
}

topic_has_publisher() {
  local topic="$1"
  local count
  count="$(timeout 8s ros2 topic info "$topic" 2>/dev/null | awk '/Publisher count:/ {print $3; found=1} END {if (!found) print 0}')"
  [ "${count:-0}" -gt 0 ]
}

wait_for_grasp_camera_topics() {
  local deadline="${1:-35}"
  local waited=0
  while [ "$waited" -lt "$deadline" ]; do
    if topic_has_publisher /camera/camera/color/image_raw \
      && topic_has_publisher /camera/camera/aligned_depth_to_color/image_raw \
      && topic_has_publisher /camera/camera/color/camera_info; then
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
  done
  return 1
}

video_port_ready() {
  local port="${1:-8092}"
  python3 - "$port" <<'PY' >/dev/null 2>&1
import socket
import sys

try:
    sock = socket.create_connection(("127.0.0.1", int(sys.argv[1])), timeout=0.5)
    sock.close()
except OSError:
    sys.exit(1)
PY
}

video_health_ready() {
  local port="${1:-8092}"
  python3 - "$port" <<'PY' >/dev/null 2>&1
import json
import sys
import urllib.request

try:
    with urllib.request.urlopen(f"http://127.0.0.1:{int(sys.argv[1])}/health", timeout=0.8) as resp:
        data = json.loads(resp.read().decode("utf-8"))
except Exception:
    sys.exit(1)

sys.exit(0 if data.get("state") == "has_frame" else 1)
PY
}

wait_for_video_stream() {
  local port="${1:-8092}"
  local deadline="${2:-20}"
  local waited=0
  while [ "$waited" -lt "$deadline" ]; do
    if video_health_ready "$port"; then
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
  done
  return 1
}

ensure_video_for_grasp() {
  local mode="$1"
  if start_video_stream; then
    return 0
  fi

  if [ "$mode" = "live" ]; then
    return 1
  fi
  return 0
}

ensure_camera_for_grasp() {
  if wait_for_grasp_camera_topics 12; then
    return 0
  fi
  echo "WARN: camera topics not ready; starting camera before grasp stack" >&2
  TRASH_STACK_INTERNAL=1 bash "$TRASH_ROBOT_ROOT/scripts/start_camera.sh" start || true
  if wait_for_grasp_camera_topics 40; then
    return 0
  fi
  return 1
}

service_ready() {
  local service="$1"
  ros2 service list 2>/dev/null | grep -qx "$service"
}

config_active_provider() {
  local cfg="$TRASH_ROBOT_ROOT/config/perception/vlm_trash_classifier.yaml"
  if [ ! -f "$cfg" ]; then
    echo ""
    return 0
  fi
  awk -F: '/^active_provider:/ {gsub(/[ "]/, "", $2); print $2; exit}' "$cfg"
}

effective_provider() {
  if [ -n "$VLM_PROVIDER" ]; then
    echo "$VLM_PROVIDER"
    return 0
  fi
  local from_cfg
  from_cfg="$(config_active_provider)"
  if [ -n "$from_cfg" ]; then
    echo "$from_cfg"
    return 0
  fi
  echo "dashscope"
}

start_local_hobot_dosod() {
  local log_dir="$1"
  local model_file="${TRASH_LOCAL_DOSOD_MODEL:-/opt/tros/humble/lib/hobot_dosod/config/dosod_mlp3x_l_rep-int8.bin}"
  local default_vocab="$TRASH_ROBOT_ROOT/config/perception/dosod_trash_vocabulary.json"
  if [ ! -f "$default_vocab" ]; then
    default_vocab="/opt/tros/humble/lib/hobot_dosod/config/offline_vocabulary.json"
  fi
  local vocab_file="${TRASH_LOCAL_DOSOD_VOCAB:-$default_vocab}"
  local score_threshold="${TRASH_LOCAL_DOSOD_SCORE_THRESHOLD:-0.30}"
  if [ ! -f "$model_file" ]; then
    echo "ERROR: local dosod model missing: $model_file" >&2
    return 1
  fi
  if [ ! -f "$vocab_file" ]; then
    echo "ERROR: local dosod vocabulary missing: $vocab_file" >&2
    return 1
  fi
  mkdir -p "$log_dir"
  stop_pid grasp_local_dosod >/dev/null 2>&1 || true
  start_detached grasp_local_dosod "$log_dir/local_hobot_dosod.log" \
    ros2 run hobot_dosod hobot_dosod --ros-args \
    -p feed_type:=1 \
    -p is_shared_mem_sub:=0 \
    -p ros_img_sub_topic_name:=/camera/camera/color/image_raw \
    -p model_file_name:="$model_file" \
    -p vocabulary_file_name:="$vocab_file" \
    -p score_threshold:="$score_threshold" \
    -p dump_render_img:=0 \
    -p dump_ai_result:=0
  sleep 2
  if ! topic_has_publisher /perception/detection/dosod; then
    echo "WARN: /perception/detection/dosod has no publisher yet; check $log_dir/local_hobot_dosod.log" >&2
  fi
  return 0
}

status() {
  echo "mode_lock: $(current_mode)"
  status_pid grasp_pipeline || true
  status_pid grasp_local_dosod || true
  echo -n "vlm node: "
  ros2 node list 2>/dev/null | grep -qx /vlm_trash_classifier && echo ONLINE || echo OFFLINE
  echo -n "depth locator: "
  if ros2 node list 2>/dev/null | grep -qx /pixel_depth_locator; then
    echo ONLINE
  elif topic_has_publisher /trash_target_point_camera && topic_has_publisher /trash_target_depth_status; then
    echo ONLINE_VLM_INTEGRATED
  else
    echo OFFLINE
  fi
  echo -n "handeye transformer: "
  ros2 node list 2>/dev/null | grep -qx /handeye_target_transformer && echo ONLINE || echo OFFLINE
  echo -n "grasp node: "
  ros2 node list 2>/dev/null | grep -qx /roarm_sort_grasper && echo ONLINE || echo OFFLINE
  echo -n "/trash_grasp_once: "
  if ros2 node list 2>/dev/null | grep -qx /roarm_sort_grasper && service_ready /trash_grasp_once; then
    echo ONLINE
  else
    echo OFFLINE
  fi
  echo -n "/move_point_cmd: "
  service_ready /move_point_cmd && echo ONLINE || echo OFFLINE
  echo -n "/get_pose_cmd: "
  service_ready /get_pose_cmd && echo ONLINE || echo OFFLINE
  echo -n "video 8092: "
  if video_health_ready "${TRASH_VIDEO_PORT:-8092}"; then
    echo ONLINE
  elif video_port_ready "${TRASH_VIDEO_PORT:-8092}"; then
    echo PORT_ONLY
  else
    echo OFFLINE
  fi
}

stop_grasp_stack() {
  stop_grasp >/dev/null 2>&1 || true
  echo "grasp stopped"
}

start_stack() {
  local mode="$1"
  reject_removed_backend "$@"
  if [ "$mode" != "dry" ] && [ "$mode" != "live" ]; then
    usage
    exit 2
  fi

  export TRASH_MODE_OWNER="start_grasp.sh"
  acquire_mode GRASP

  if [ ! -f "$HANDEYE_FILE" ]; then
    release_mode GRASP
    echo "ERROR: missing hand-eye file: $HANDEYE_FILE" >&2
    exit 1
  fi

  if ! ensure_camera_for_grasp; then
    release_mode GRASP
    echo "ERROR: camera topics not ready after wait (color/aligned_depth/camera_info)" >&2
    exit 1
  fi

  if ! ensure_video_for_grasp "$mode"; then
    release_mode GRASP
    echo "ERROR: video stream is required for live grasp monitoring" >&2
    exit 1
  fi

  local dry_run="true"
  local log_dir="$LOG_ROOT/dry"
  local provider_name
  provider_name="$(effective_provider)"
  local enable_local_candidate="${TRASH_ENABLE_LOCAL_CANDIDATE:-0}"
  if [ "$mode" = "live" ]; then
    dry_run="false"
    log_dir="$LOG_ROOT/live"
    if ! service_ready /move_point_cmd || ! service_ready /get_pose_cmd; then
      release_mode GRASP
      echo "ERROR: live grasp requires /move_point_cmd and /get_pose_cmd; start ./scripts/start_arm.sh start first" >&2
      exit 1
    fi
  fi

  mkdir -p "$log_dir"
  stop_pid grasp_pipeline >/dev/null 2>&1 || true
  if [ "$provider_name" = "local_hobot" ] || [ "$enable_local_candidate" = "1" ]; then
    if ! start_local_hobot_dosod "$log_dir"; then
      release_mode GRASP
      exit 1
    fi
  else
    stop_pid grasp_local_dosod >/dev/null 2>&1 || true
  fi

  local launch_args=(
    dry_run:="$dry_run"
    auto_grasp:=false
    auto_execute:=false
    camera_point_source:=vlm
    use_legacy_camera_point:=false
    vlm_config_file:="$TRASH_ROBOT_ROOT/config/perception/vlm_trash_classifier.yaml"
    sort_config_file:="$TRASH_ROBOT_ROOT/config/grasp/trash_sort_params.yaml"
    handeye_file:="$HANDEYE_FILE"
    pixel_image_width:=640
    pixel_image_height:=480
  )
  if [ -n "$VLM_PROVIDER" ]; then
    launch_args+=(vlm_provider:="$VLM_PROVIDER")
  fi

  nohup ros2 launch trash_robot_bringup perception_grasp.launch.py "${launch_args[@]}" \
    > "$log_dir/grasp_pipeline.log" 2>&1 &
  write_pid grasp_pipeline "$!"

  echo "grasp stack started: mode=$mode backend=vlm provider=$provider_name"
  echo "local_candidate=${enable_local_candidate} topic=/trash_local_candidate"
  echo "dry_run=$dry_run"
  echo "logs: $log_dir"
  echo "manual grasp: ros2 service call /trash_grasp_once std_srvs/srv/Trigger '{}'"
}

case "$ACTION" in
  dry|live)
    reject_removed_backend "$@"
    start_stack "$ACTION"
    ;;
  stop)
    stop_grasp_stack
    ;;
  status)
    status
    ;;
  restart)
    reject_removed_backend "$@"
    [ -n "$MODE" ] || MODE="dry"
    stop_grasp_stack >/dev/null 2>&1 || true
    start_stack "$MODE"
    ;;
  *)
    reject_removed_backend "$@"
    usage
    exit 2
    ;;
esac
