#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT/scripts/common.sh"
source_debug

ACTION="${1:-status}"
MODEL_DIR="${TRASH_LOCAL_VLM_MODEL_DIR:-$TRASH_ROBOT_ROOT/models}"
VIT_MODEL="${TRASH_LOCAL_VLM_VIT_MODEL:-vit_model_int16_v2.bin}"
LLM_MODEL="${TRASH_LOCAL_VLM_LLM_MODEL:-Qwen2.5-0.5B-Instruct-Q4_0.gguf}"
IMAGE_TOPIC="${TRASH_LOCAL_VLM_IMAGE_TOPIC:-/camera/camera/color/image_raw}"
PROMPT_TOPIC="${TRASH_LOCAL_VLM_PROMPT_TOPIC:-/trash_local_vlm_prompt}"
RAW_TOPIC="${TRASH_LOCAL_VLM_RAW_TOPIC:-/trash_local_vlm_raw}"
TEXT_TOPIC="${TRASH_LOCAL_VLM_TEXT_TOPIC:-/trash_local_vlm_text}"
LOG_DIR="$TRASH_ROBOT_LOG_DIR/local_vlm"
PID_NAME="local_llamacpp_vlm"
DEFAULT_PROMPT='请识别画面中的垃圾。重点判断物体是什么、是否是垃圾、属于哪类垃圾。请尽量输出JSON：{"has_target":true,"object":"物体名","category":"GARBAGE_RECYCLE/GARBAGE_OTHER/GARBAGE_HAZARD/GARBAGE_KITCHEN","bbox":[x1,y1,x2,y2],"confidence":0.0,"description":"简短描述"}'

usage() {
  cat <<USAGE
usage: $0 status|configure-ion|once|start|stop|restart|prompt|echo [prompt]

本地 VLM 测试，不控制机械臂，不调用云 API。
模型目录: $MODEL_DIR
输出: $RAW_TOPIC / $TEXT_TOPIC
提示词: $PROMPT_TOPIC
USAGE
}

model_path() {
  printf '%s/%s' "$MODEL_DIR" "$1"
}

require_models() {
  [ -f "$(model_path "$VIT_MODEL")" ] || { echo "ERROR: missing VIT model: $(model_path "$VIT_MODEL")" >&2; exit 1; }
  [ -f "$(model_path "$LLM_MODEL")" ] || { echo "ERROR: missing LLM model: $(model_path "$LLM_MODEL")" >&2; exit 1; }
}

heap_mb() {
  local heap="$1"
  sudo -n awk '/total size/ {printf "%d", $5 / 1048576}' "/sys/kernel/debug/ion/heaps/$heap" 2>/dev/null || echo 0
}

ion_status_line() {
  local cma reserved carveout total
  cma="$(heap_mb ion_cma)"
  reserved="$(heap_mb cma_reserved)"
  carveout="$(heap_mb carveout)"
  total=$((cma + reserved + carveout))
  echo "ION=${cma}MB+${reserved}MB+${carveout}MB total=${total}MB"
}

ion_total_mb() {
  local cma reserved carveout
  cma="$(heap_mb ion_cma)"
  reserved="$(heap_mb cma_reserved)"
  carveout="$(heap_mb carveout)"
  echo $((cma + reserved + carveout))
}

check_ion_ready() {
  local total
  total="$(ion_total_mb)"
  if [ "$total" -lt 1500 ]; then
    echo "ERROR: local VLM needs ION about 1.6GB; current $(ion_status_line)" >&2
    echo "Run: $0 configure-ion，然后重启 RDK 后再测试。" >&2
    return 1
  fi
  return 0
}

configure_ion() {
  echo "current: $(ion_status_line)"
  echo "set ION to official VLM profile: 320MB+640MB+640MB"
  sudo -n cp /boot/config.txt "/boot/config.txt.before_local_vlm_$(date +%Y%m%d_%H%M%S)"
  sudo -n srpi-config nonint do_config_ion_memory_x5 '320MB+640MB+640MB'
  echo "configured /boot/config.txt; reboot is required before it takes effect."
  sudo -n cat /boot/config.txt
}

ensure_camera_topic() {
  local count
  count="$((timeout 4s ros2 topic info "$IMAGE_TOPIC" 2>/dev/null || true) | awk '/Publisher count:/ {print $3; found=1} END {if (!found) print 0}')"
  if [ "${count:-0}" -gt 0 ]; then
    return 0
  fi
  echo "WARN: camera topic not ready, starting camera stack..." >&2
  TRASH_STACK_INTERNAL=1 bash "$TRASH_ROBOT_ROOT/scripts/start_camera.sh" start || true
  sleep 3
}

run_once() {
  require_models
  check_ion_ready
  local prompt image timeout_sec
  prompt="${*:2}"
  [ -n "$prompt" ] || prompt="$DEFAULT_PROMPT"
  image="${TRASH_LOCAL_VLM_TEST_IMAGE:-config/image2.jpg}"
  timeout_sec="${TRASH_LOCAL_VLM_TIMEOUT_SEC:-120}"
  cd "$MODEL_DIR"
  timeout "$timeout_sec" ros2 run hobot_llamacpp hobot_llamacpp --ros-args \
    -p feed_type:=0 \
    -p image:="$image" \
    -p image_type:=0 \
    -p user_prompt:="$prompt" \
    -p ai_msg_pub_topic_name:="$RAW_TOPIC" \
    -p text_msg_pub_topic_name:="$TEXT_TOPIC" \
    -p model_file_name:="$VIT_MODEL" \
    -p llm_model_name:="$LLM_MODEL"
}

start_live() {
  require_models
  check_ion_ready
  ensure_camera_topic
  mkdir -p "$LOG_DIR"
  stop_pid "$PID_NAME" >/dev/null 2>&1 || true
  start_detached "$PID_NAME" "$LOG_DIR/local_llamacpp_vlm.log" bash -lc "
    set -e
    cd '$MODEL_DIR'
    source /opt/ros/humble/setup.bash
    source /opt/tros/humble/setup.bash
    [ -f '$TRASH_ROBOT_ROOT/install/setup.bash' ] && source '$TRASH_ROBOT_ROOT/install/setup.bash'
    export ROS_DOMAIN_ID='${ROS_DOMAIN_ID:-1}'
    export ROS_LOCALHOST_ONLY='${ROS_LOCALHOST_ONLY:-0}'
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI='${CYCLONEDDS_URI:-file://$TRASH_ROBOT_ROOT/config/dds/cyclonedds_unicast.xml}'
    ros2 run hobot_llamacpp hobot_llamacpp --ros-args --log-level warn \
      -p feed_type:=1 \
      -p is_shared_mem_sub:=0 \
      -p model_type:=0 \
      -p ros_img_sub_topic_name:='$IMAGE_TOPIC' \
      -p ros_string_sub_topic_name:='$PROMPT_TOPIC' \
      -p ai_msg_pub_topic_name:='$RAW_TOPIC' \
      -p text_msg_pub_topic_name:='$TEXT_TOPIC' \
      -p model_file_name:='$VIT_MODEL' \
      -p llm_model_name:='$LLM_MODEL'
  "
  echo "local VLM started"
  echo "log: $LOG_DIR/local_llamacpp_vlm.log"
  echo "prompt: $0 prompt"
  echo "output: ros2 topic echo $RAW_TOPIC"
}

publish_prompt() {
  local prompt
  prompt="${*:2}"
  [ -n "$prompt" ] || prompt="$DEFAULT_PROMPT"
  python3 - "$PROMPT_TOPIC" "$prompt" <<'PY'
import subprocess
import sys

topic = sys.argv[1]
prompt = sys.argv[2]
msg = "{data: " + repr(prompt) + "}"
subprocess.run(["ros2", "topic", "pub", "--once", topic, "std_msgs/msg/String", msg], check=True)
PY
}

show_status() {
  echo "package: $(ros2 pkg prefix hobot_llamacpp 2>/dev/null || echo MISSING)"
  echo "models:"
  ls -lh "$(model_path "$VIT_MODEL")" "$(model_path "$LLM_MODEL")" 2>/dev/null || true
  echo "$(ion_status_line)"
  status_pid "$PID_NAME" || true
  echo "topics:"
  ros2 topic list 2>/dev/null | grep -E "${RAW_TOPIC}|${TEXT_TOPIC}|${PROMPT_TOPIC}|${IMAGE_TOPIC}" || true
}

case "$ACTION" in
  status) show_status ;;
  configure-ion) configure_ion ;;
  once) run_once "$@" ;;
  start) start_live ;;
  stop) stop_pid "$PID_NAME" >/dev/null 2>&1 || true; echo "local VLM stopped" ;;
  restart) stop_pid "$PID_NAME" >/dev/null 2>&1 || true; start_live ;;
  prompt) publish_prompt "$@" ;;
  echo) ros2 topic echo "$RAW_TOPIC" ;;
  *) usage; exit 2 ;;
esac
