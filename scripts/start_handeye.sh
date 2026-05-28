#!/usr/bin/env bash
set -e

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
source_debug

ACTION="${1:-start}"
LOG_DIR="$TRASH_ROBOT_LOG_DIR/handeye"
OUTPUT_FILE="$TRASH_ROBOT_ROOT/config/grasp/handeye_point.new.yaml"
DEFAULT_HOST_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
DEFAULT_HOST_IP="${DEFAULT_HOST_IP:-127.0.0.1}"
HANDEYE_HOST="${TRASH_HANDEYE_HOST:-0.0.0.0}"
HANDEYE_VIDEO_MODE="${TRASH_HANDEYE_VIDEO_MODE:-webrtc}"
HANDEYE_WEBRTC_URL="${TRASH_HANDEYE_WEBRTC_URL:-http://$DEFAULT_HOST_IP:8889/handeye/}"
HANDEYE_RTSP_URL="${TRASH_HANDEYE_RTSP_URL:-rtsp://$DEFAULT_HOST_IP:8554/handeye}"
HANDEYE_INTERNAL_STREAM="${TRASH_HANDEYE_ENABLE_INTERNAL_STREAM:-false}"
HANDEYE_STREAM_PERIOD="${TRASH_HANDEYE_STREAM_PERIOD:-0.5}"
HANDEYE_JPEG_QUALITY="${TRASH_HANDEYE_JPEG_QUALITY:-38}"
HANDEYE_STREAM_MAX_WIDTH="${TRASH_HANDEYE_STREAM_MAX_WIDTH:-480}"
HANDEYE_WEBRTC_WIDTH="${TRASH_HANDEYE_WEBRTC_WIDTH:-${TRASH_HANDEYE_VIDEO_WIDTH:-640}}"
HANDEYE_WEBRTC_HEIGHT="${TRASH_HANDEYE_WEBRTC_HEIGHT:-${TRASH_HANDEYE_VIDEO_HEIGHT:-360}}"
HANDEYE_SHOW_DETECTION_OVERLAY="${TRASH_HANDEYE_SHOW_DETECTION_OVERLAY:-false}"
mkdir -p "$LOG_DIR"

status() {
  echo "mode_lock: $(current_mode)"
  status_pid camera || true
  status_pid handeye_web || true
  echo "output_file: $OUTPUT_FILE"
  if command -v curl >/dev/null 2>&1 && curl -fsS --max-time 1 http://127.0.0.1:8093/status >/tmp/trash_handeye_status.json 2>/dev/null; then
    python3 - <<'PY' 2>/dev/null || cat /tmp/trash_handeye_status.json
import json
from pathlib import Path
data = json.loads(Path("/tmp/trash_handeye_status.json").read_text())
print(f"video_mode: {data.get('video_mode')}")
print(f"webrtc_url: {data.get('webrtc_url')}")
print(f"rtsp_url: {data.get('rtsp_url')}")
print(f"internal_stream_enabled: {data.get('internal_stream_enabled')}")
print(f"image: {data.get('image')} camera_info: {data.get('camera_info')} target: {data.get('target')}")
PY
  else
    echo "video_mode: $HANDEYE_VIDEO_MODE"
    echo "webrtc_url: $HANDEYE_WEBRTC_URL"
    echo "rtsp_url: $HANDEYE_RTSP_URL"
  fi
}

start_handeye() {
  export TRASH_MODE_OWNER="start_handeye.sh"
  acquire_mode CALIBRATION
  if status_pid handeye_web >/dev/null 2>&1; then
    echo "handeye already running"
    status
    return 0
  fi

  "$TRASH_ROBOT_ROOT/scripts/start_camera.sh" handeye > "$LOG_DIR/camera.log" 2>&1

  nohup ros2 run trash_robot_grasp handeye_web_calibrator \
    --ros-args \
    -p host:="$HANDEYE_HOST" \
    -p target_type:=chessboard \
    -p output_file:="$OUTPUT_FILE" \
    -p detect_period_s:=1.0 \
    -p stream_period_s:="$HANDEYE_STREAM_PERIOD" \
    -p jpeg_quality:="$HANDEYE_JPEG_QUALITY" \
    -p stream_max_width:="$HANDEYE_STREAM_MAX_WIDTH" \
    -p use_chessboard_sb:=false \
    -p detect_scale:=0.25 \
    -p image_accept_period_s:=0.10 \
    -p get_pose_timeout_s:=8.0 \
    -p max_detection_age_s:=1.2 \
    -p video_mode:="$HANDEYE_VIDEO_MODE" \
    -p webrtc_url:="$HANDEYE_WEBRTC_URL" \
    -p rtsp_url:="$HANDEYE_RTSP_URL" \
    -p webrtc_stream_width:="$HANDEYE_WEBRTC_WIDTH" \
    -p webrtc_stream_height:="$HANDEYE_WEBRTC_HEIGHT" \
    -p show_detection_overlay:="$HANDEYE_SHOW_DETECTION_OVERLAY" \
    -p enable_internal_stream:="$HANDEYE_INTERNAL_STREAM" \
    > "$LOG_DIR/handeye_web.log" 2>&1 &
  write_pid handeye_web "$!"

  echo "Handeye WebUI: http://$DEFAULT_HOST_IP:8093"
  echo "Handeye WebRTC: $HANDEYE_WEBRTC_URL"
  echo "Handeye RTSP: $HANDEYE_RTSP_URL"
  echo "Output: $OUTPUT_FILE"
  echo "正式文件不会自动覆盖: $TRASH_ROBOT_ROOT/config/grasp/handeye_point.yaml"
  echo "Target: chessboard inner corners 4x4, square size 5mm"
}

stop_handeye_tool() {
  stop_handeye >/dev/null 2>&1 || true
  echo "handeye stopped"
}

case "$ACTION" in
  start)
    start_handeye
    ;;
  stop)
    stop_handeye_tool
    ;;
  restart)
    stop_handeye_tool >/dev/null 2>&1 || true
    start_handeye
    ;;
  status)
    status
    ;;
  *)
    echo "usage: $0 start|stop|restart|status" >&2
    exit 2
    ;;
esac
