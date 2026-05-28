#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/common.sh"
source_debug
if [ "${TRASH_YOLO_LOCAL_DDS:-1}" = "1" ]; then
  export CYCLONEDDS_URI="file://$TRASH_ROBOT_ROOT/config/dds/cyclonedds_local.xml"
fi

ACTION="${1:-start}"
LOG_DIR="$TRASH_ROBOT_LOG_DIR/yolo_detector"
YOLO_WORK_DIR="$TRASH_ROBOT_RUNTIME/yolo_detector_work"
DNN_TOPIC="${TRASH_YOLO_DNN_TOPIC:-/hobot_dnn_detection}"
IMAGE_TOPIC="${TRASH_YOLO_IMAGE_TOPIC:-/camera/camera/color/image_raw}"
DEPTH_TOPIC="${TRASH_YOLO_DEPTH_TOPIC:-/camera/camera/aligned_depth_to_color/image_raw}"
CAMERA_INFO_TOPIC="${TRASH_YOLO_CAMERA_INFO_TOPIC:-/camera/camera/color/camera_info}"
YOLO_PROFILE="${TRASH_YOLO_PROFILE:-coco}"
if [ "$YOLO_PROFILE" = "paper_ball" ]; then
  DEFAULT_CANDIDATE_RATE="${TRASH_YOLO_CANDIDATE_RATE:-4.0}"
  DEFAULT_PROCESS_RATE="${TRASH_YOLO_PROCESS_RATE:-4.0}"
else
  DEFAULT_CANDIDATE_RATE="${TRASH_YOLO_CANDIDATE_RATE:-10.0}"
  DEFAULT_PROCESS_RATE="${TRASH_YOLO_PROCESS_RATE:-0.0}"
fi
YOLO_DNN_LOG_LEVEL="${TRASH_YOLO_DNN_LOG_LEVEL:-error}"
YOLO_CANDIDATE_LOG_LEVEL="${TRASH_YOLO_CANDIDATE_LOG_LEVEL:-warn}"

resolve_config_file() {
  if [ -n "${TRASH_YOLO_CONFIG_FILE:-}" ]; then
    echo "$TRASH_YOLO_CONFIG_FILE"
    return
  fi

  case "$YOLO_PROFILE" in
    coco)
      echo "/opt/tros/humble/lib/dnn_node_example/config/yolov8workconfig.json"
      ;;
    paper_ball)
      echo "$TRASH_ROBOT_ROOT/models/paper_ball_yolov8n/yolov8_paper_ball_workconfig.json"
      ;;
    *)
      echo "ERROR: unknown TRASH_YOLO_PROFILE=$YOLO_PROFILE, expected coco or paper_ball" >&2
      exit 2
      ;;
  esac
}

CONFIG_FILE="$(resolve_config_file)"

usage() {
  echo "usage: $0 start|stop|restart|status" >&2
  echo "env: TRASH_YOLO_PROFILE=coco|paper_ball or TRASH_YOLO_CONFIG_FILE=/path/to/workconfig.json" >&2
}

config_model_file() {
  python3 - "$CONFIG_FILE" <<'PY'
import json
import sys
from pathlib import Path

config_path = Path(sys.argv[1])
with config_path.open("r", encoding="utf-8") as f:
    config = json.load(f)
print(config.get("model_file", ""))
PY
}

validate_yolo_config() {
  if [ ! -f "$CONFIG_FILE" ]; then
    echo "ERROR: YOLO config missing: $CONFIG_FILE" >&2
    return 1
  fi

  local model_file
  model_file="$(config_model_file)"
  if [ -z "$model_file" ] || [ ! -f "$model_file" ]; then
    echo "ERROR: YOLO model missing: ${model_file:-<empty>}" >&2
    echo "profile=$YOLO_PROFILE config=$CONFIG_FILE" >&2
    if [ "$YOLO_PROFILE" = "paper_ball" ]; then
      echo "hint: convert models/paper_ball_yolov8n/paper_ball_yolov8n_bpu6_op11.onnx to X5 NV12 .bin first" >&2
    fi
    return 1
  fi
}

topic_has_publisher() {
  local topic="$1"
  local count
  count="$(timeout 5s ros2 topic info "$topic" 2>/dev/null | awk '/Publisher count:/ {print $3; found=1} END {if (!found) print 0}')"
  [ "${count:-0}" -gt 0 ]
}

wait_for_camera_topics() {
  local deadline="${1:-25}"
  local waited=0
  while [ "$waited" -lt "$deadline" ]; do
    if topic_has_publisher "$IMAGE_TOPIC" \
      && topic_has_publisher "$DEPTH_TOPIC" \
      && topic_has_publisher "$CAMERA_INFO_TOPIC"; then
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
  done
  return 1
}

start_camera_if_needed() {
  if wait_for_camera_topics 5; then
    return 0
  fi
  echo "WARN: camera topics not ready; starting camera before YOLO detector" >&2
  TRASH_STACK_INTERNAL=1 bash "$TRASH_ROBOT_ROOT/scripts/start_camera.sh" start || true
  wait_for_camera_topics 35
}

start_yolo() {
  validate_yolo_config || exit 1
  local model_file
  model_file="$(config_model_file)"
  if ! start_camera_if_needed; then
    echo "ERROR: camera topics not ready for YOLO detector" >&2
    exit 1
  fi

  mkdir -p "$LOG_DIR"
  mkdir -p "$YOLO_WORK_DIR"
  : > "$LOG_DIR/yolo_dnn.log"
  : > "$LOG_DIR/yolo_trash_candidate.log"
  rm -rf "$YOLO_WORK_DIR/config"
  cp -a /opt/tros/humble/lib/dnn_node_example/config "$YOLO_WORK_DIR/config"
  stop_pid yolo_candidate >/dev/null 2>&1 || true
  stop_pid yolo_dnn >/dev/null 2>&1 || true

  start_detached yolo_dnn "$LOG_DIR/yolo_dnn.log" \
    bash -lc "cd '$YOLO_WORK_DIR' && exec ros2 run dnn_node_example example --ros-args \
      -p config_file:="$CONFIG_FILE" \
      -p feed_type:=1 \
      -p is_shared_mem_sub:=0 \
      -p ros_img_topic_name:="$IMAGE_TOPIC" \
      -p msg_pub_topic_name:="$DNN_TOPIC" \
      -p dump_render_img:=0 \
      --log-level "$YOLO_DNN_LOG_LEVEL""

  sleep 2

  start_detached yolo_candidate "$LOG_DIR/yolo_trash_candidate.log" \
    ros2 run trash_robot_vision yolo_trash_candidate --ros-args \
      -p detection_topic:="$DNN_TOPIC" \
      -p depth_topic:="$DEPTH_TOPIC" \
      -p camera_info_topic:="$CAMERA_INFO_TOPIC" \
      -p candidate_topic:=/trash_yolo_candidate \
      -p debug_camera_point_topic:=/trash_yolo_target_camera_point \
      -p publish_rate_hz:="$DEFAULT_CANDIDATE_RATE" \
      -p max_process_rate_hz:="$DEFAULT_PROCESS_RATE" \
      -p min_confidence:="${TRASH_YOLO_MIN_CONFIDENCE:-0.08}" \
      -p roi_coordinate_mode:="${TRASH_YOLO_ROI_COORDINATE_MODE:-letterbox}" \
      -p model_input_width:="${TRASH_YOLO_MODEL_INPUT_WIDTH:-640}" \
      -p model_input_height:="${TRASH_YOLO_MODEL_INPUT_HEIGHT:-640}" \
      -p reject_edge_margin_norm:="${TRASH_YOLO_REJECT_EDGE_MARGIN_NORM:-0.04}" \
      -p min_bbox_area_norm:="${TRASH_YOLO_MIN_BBOX_AREA_NORM:-0.002}" \
      -p max_depth_m:="${TRASH_YOLO_MAX_DEPTH_M:-2.80}" \
      -p require_depth:="${TRASH_YOLO_REQUIRE_DEPTH:-true}" \
      --log-level "$YOLO_CANDIDATE_LOG_LEVEL"

  echo "YOLO detector started"
  echo "profile=$YOLO_PROFILE"
  echo "config=$CONFIG_FILE"
  echo "model=$model_file"
  echo "dnn_topic=$DNN_TOPIC"
  echo "candidate_topic=/trash_yolo_candidate"
  echo "camera_point_topic=/trash_yolo_target_camera_point"
  echo "candidate_rate_hz=$DEFAULT_CANDIDATE_RATE"
  echo "process_rate_hz=$DEFAULT_PROCESS_RATE"
  echo "logs=$LOG_DIR"
}

stop_yolo() {
  stop_pid yolo_candidate >/dev/null 2>&1 || true
  stop_pid yolo_dnn >/dev/null 2>&1 || true
  echo "YOLO detector stopped"
}

status_yolo() {
  echo "profile=$YOLO_PROFILE"
  echo "config=$CONFIG_FILE"
  if [ -f "$CONFIG_FILE" ]; then
    local model_file
    model_file="$(config_model_file)"
    echo "model=$model_file"
    if [ -n "$model_file" ] && [ -f "$model_file" ]; then
      echo "model_status=OK"
    else
      echo "model_status=MISSING"
    fi
  else
    echo "model=<config missing>"
    echo "model_status=MISSING"
  fi
  status_pid yolo_dnn || true
  local yolo_dnn_pid_file yolo_dnn_pid
  yolo_dnn_pid_file="$(pid_file_for yolo_dnn)"
  if [ -f "$yolo_dnn_pid_file" ]; then
    yolo_dnn_pid="$(cat "$yolo_dnn_pid_file" 2>/dev/null || true)"
    if [ -n "$yolo_dnn_pid" ] && kill -0 "$yolo_dnn_pid" 2>/dev/null; then
      echo -n "yolo_dnn_cmd: "
      ps -o args= -p "$yolo_dnn_pid" 2>/dev/null || true
    fi
  fi
  status_pid yolo_candidate || true
  echo -n "$DNN_TOPIC publishers: "
  timeout 4s ros2 topic info "$DNN_TOPIC" 2>/dev/null | awk '/Publisher count:/ {print $3; found=1} END {if (!found) print 0}'
  echo -n "/trash_yolo_candidate publishers: "
  timeout 4s ros2 topic info /trash_yolo_candidate 2>/dev/null | awk '/Publisher count:/ {print $3; found=1} END {if (!found) print 0}'
  echo -n "/trash_yolo_target_camera_point publishers: "
  timeout 4s ros2 topic info /trash_yolo_target_camera_point 2>/dev/null | awk '/Publisher count:/ {print $3; found=1} END {if (!found) print 0}'
}

case "$ACTION" in
  start)
    start_yolo
    ;;
  stop)
    stop_yolo
    ;;
  restart)
    validate_yolo_config || exit 1
    stop_yolo
    start_yolo
    ;;
  status)
    status_yolo
    ;;
  *)
    usage
    exit 2
    ;;
esac
