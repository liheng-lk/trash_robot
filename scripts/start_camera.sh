#!/usr/bin/env bash
set -e
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
source_debug

MODE="${1:-start}"  # start | restart | handeye | stop | status | rtp | srt | stop-rtp | rtp-status
LOG_DIR="$TRASH_ROBOT_LOG_DIR/camera"
mkdir -p "$LOG_DIR"
RTP_PID_FILE="$TRASH_ROBOT_RUNTIME/video_rtp.pid"
CAMERA_MODE_FILE="$TRASH_ROBOT_RUNTIME/camera_mode.txt"
REQUESTED_PROFILE="full"

camera_process_running() {
  ps -eo pid=,args= | awk '
    /realsense2_camera_node|camera_realsense[.]launch[.]py/ &&
    $0 !~ /awk|grep|start_camera[.]sh|bash -c|bash -lc|ssh / { found = 1 }
    END { exit found ? 0 : 1 }
  '
}

topic_publishers() {
  local topic="$1"
  local info=""
  info="$(timeout 3s ros2 topic info "$topic" 2>/dev/null || true)"
  printf '%s\n' "$info" | awk '/Publisher count:/ {print $3; found=1} END {if (!found) print 0}'
}

topic_subscribers() {
  local topic="$1"
  local info=""
  info="$(timeout 3s ros2 topic info "$topic" 2>/dev/null || true)"
  printf '%s\n' "$info" | awk '/Subscription count:/ {print $3; found=1} END {if (!found) print 0}'
}

camera_required_topics() {
  local profile="${1:-full}"
  printf '%s\n' \
    /camera/camera/color/image_raw \
    /camera/camera/color/camera_info
  if [ "$profile" != "handeye" ]; then
    printf '%s\n' /camera/camera/aligned_depth_to_color/image_raw
  fi
}

camera_has_required_publishers() {
  local profile="${1:-full}"
  local topic=""
  while IFS= read -r topic; do
    [ "$(topic_publishers "$topic")" -gt 0 ] || return 1
  done < <(camera_required_topics "$profile")
  return 0
}

wait_for_camera_ready() {
  local profile="${1:-full}"
  local deadline="${2:-25}"
  local waited=0
  while [ "$waited" -lt "$deadline" ]; do
    if camera_has_required_publishers "$profile"; then
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
  done
  return 1
}

enable_pointcloud_for_navigation() {
  [ "$REQUESTED_PROFILE" = "full" ] || return 0
  if ! awk '/pointcloud_enable:/ {gsub(/[ "\047]/, "", $2); if ($2 == "true") found=1} END {exit found ? 0 : 1}' "$TRASH_ROBOT_ROOT/config/hardware/camera_realsense.yaml"; then
    return 0
  fi

  if ! ros2 param get /camera/camera pointcloud__neon_.enable >/dev/null 2>&1; then
    return 0
  fi

  ros2 param set /camera/camera pointcloud__neon_.enable true >/dev/null 2>&1 || true

  local waited=0
  while [ "$waited" -lt 10 ]; do
    if [ "$(topic_publishers /camera/camera/depth/color/points)" -gt 0 ]; then
      echo "camera pointcloud online"
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
  done

  echo "WARN camera pointcloud did not appear; depth image and /scan_depth remain online"
}

stop_video_rtp() {
  stop_pid video_rtp >/dev/null 2>&1 || true
  rm -f "$RTP_PID_FILE"
}

find_color_v4l2_device() {
  if command -v ffmpeg >/dev/null 2>&1; then
    for dev in /dev/video*; do
      [ -e "$dev" ] || continue
      if timeout 3s ffmpeg -hide_banner -f v4l2 -list_formats all -i "$dev" 2>&1 | grep -qE "yuyv422|YUYV"; then
        echo "$dev"
        return 0
      fi
    done
  fi
  echo "/dev/video4"
}

gst_h264_encoder_chain() {
  if ! command -v gst-inspect-1.0 >/dev/null 2>&1; then
    return 1
  fi
  if gst-inspect-1.0 hobot_h264enc >/dev/null 2>&1; then
    echo "videoconvert ! video/x-raw,format=NV12 ! hobot_h264enc"
  elif gst-inspect-1.0 avenc_h264_omx >/dev/null 2>&1; then
    echo "videoconvert ! video/x-raw,format=I420 ! avenc_h264_omx bitrate=1800000"
  elif gst-inspect-1.0 openh264enc >/dev/null 2>&1; then
    echo "videoconvert ! video/x-raw,format=I420 ! openh264enc bitrate=1800000 complexity=low"
  else
    return 1
  fi
}

case "$MODE" in
  stop)
    stop_camera >/dev/null 2>&1 || true
    echo "camera stopped"
    exit 0
    ;;
  restart)
    stop_camera >/dev/null 2>&1 || true
    sleep 2
    ;;
  handeye)
    stop_camera >/dev/null 2>&1 || true
    export TRASH_CAMERA_MODE=handeye
    REQUESTED_PROFILE="handeye"
    sleep 2
    MODE="start"
    ;;
  status)
    if camera_process_running; then
      echo "camera process: RUNNING"
    else
      echo "camera process: STOPPED"
    fi
    for topic in \
      /camera/camera/color/image_raw \
      /camera/camera/color/camera_info \
      /camera/camera/aligned_depth_to_color/image_raw \
      /camera/camera/depth/image_rect_raw \
      /camera/camera/depth/color/points; do
      pubs="$(topic_publishers "$topic")"
      subs="$(topic_subscribers "$topic")"
      echo "$topic publishers=${pubs:-0} subscribers=${subs:-0}"
    done
    exit 0
    ;;
  stop-rtp)
    stop_video_rtp >/dev/null 2>&1 || true
    echo "camera RTP video stopped"
    exit 0
    ;;
  rtp-status)
    if [ -f "$RTP_PID_FILE" ] && kill -0 "$(cat "$RTP_PID_FILE")" 2>/dev/null; then
      echo "camera RTP video: RUNNING pid=$(cat "$RTP_PID_FILE")"
    else
      echo "camera RTP video: STOPPED"
    fi
    echo "log: $LOG_DIR/video_rtp.log"
    exit 0
    ;;
  rtp)
    TARGET_IP="${2:-}"
    PORT="${3:-5000}"
    DEVICE="${4:-$(find_color_v4l2_device)}"
    if [ -z "$TARGET_IP" ]; then
      echo "usage: ./scripts/start_camera.sh rtp <target_ip> [port] [video_device]"
      exit 2
    fi
    stop_video_rtp >/dev/null 2>&1 || true

    # Direct V4L2 access is exclusive on the D435i color stream. Stop the ROS
    # camera first so low-latency RTP does not fight realsense2_camera.
    stop_camera >/dev/null 2>&1 || true
    sleep 1

    GST_ENCODER="$(gst_h264_encoder_chain || true)"
    if command -v gst-launch-1.0 >/dev/null 2>&1 && [ -n "$GST_ENCODER" ]; then
      nohup bash -c "exec -a trash_video_rtp gst-launch-1.0 -e v4l2src device='$DEVICE' io-mode=mmap ! 'video/x-raw,format=YUY2,width=640,height=480,framerate=30/1' ! $GST_ENCODER ! h264parse config-interval=1 ! rtph264pay config-interval=1 pt=96 ! udpsink host='$TARGET_IP' port='$PORT' sync=false async=false" \
        > "$LOG_DIR/video_rtp.log" 2>&1 &
    elif command -v ffmpeg >/dev/null 2>&1 && ffmpeg -hide_banner -encoders 2>/dev/null | grep -q "h264_v4l2m2m"; then
      nohup bash -c "exec -a trash_video_rtp ffmpeg -hide_banner -loglevel warning -fflags nobuffer -flags low_delay -f v4l2 -input_format yuyv422 -video_size 640x480 -framerate 30 -i '$DEVICE' -an -c:v h264_v4l2m2m -b:v 1800k -g 30 -f rtp 'rtp://$TARGET_IP:$PORT?pkt_size=1200'" \
        > "$LOG_DIR/video_rtp.log" 2>&1 &
    else
      echo "no hardware H264 RTP backend found: need gst-launch-1.0+hobot_h264enc or ffmpeg+h264_v4l2m2m"
      exit 2
    fi
    echo "$!" > "$RTP_PID_FILE"
    write_pid video_rtp "$!"
    echo "camera RTP video started"
    echo "device: $DEVICE"
    echo "target: udp://$TARGET_IP:$PORT"
    echo "log: $LOG_DIR/video_rtp.log"
    echo "receiver example:"
    echo "  ffplay -fflags nobuffer -flags low_delay -framedrop rtp://0.0.0.0:$PORT"
    exit 0
    ;;
  srt)
    TARGET_IP="${2:-}"
    PORT="${3:-8888}"
    DEVICE="${4:-$(find_color_v4l2_device)}"
    if [ -z "$TARGET_IP" ]; then
      echo "usage: ./scripts/start_camera.sh srt <target_ip> [port] [video_device]"
      exit 2
    fi
    if ! command -v gst-launch-1.0 >/dev/null 2>&1 || ! gst-inspect-1.0 srtsink >/dev/null 2>&1; then
      echo "GStreamer SRT is unavailable: need gst-launch-1.0 and srtsink"
      exit 2
    fi
    GST_ENCODER="$(gst_h264_encoder_chain || true)"
    if [ -z "$GST_ENCODER" ]; then
      echo "no GStreamer H264 encoder found"
      exit 2
    fi
    stop_video_rtp >/dev/null 2>&1 || true
    stop_camera >/dev/null 2>&1 || true
    sleep 1
    nohup bash -c "exec -a trash_video_rtp gst-launch-1.0 -e v4l2src device='$DEVICE' io-mode=mmap ! 'video/x-raw,format=YUY2,width=640,height=480,framerate=30/1' ! $GST_ENCODER ! h264parse config-interval=1 ! mpegtsmux ! srtsink uri='srt://$TARGET_IP:$PORT?mode=caller&latency=80'" \
      > "$LOG_DIR/video_rtp.log" 2>&1 &
    echo "$!" > "$RTP_PID_FILE"
    write_pid video_rtp "$!"
    echo "camera SRT video started"
    echo "device: $DEVICE"
    echo "target: srt://$TARGET_IP:$PORT"
    echo "log: $LOG_DIR/video_rtp.log"
    echo "receiver example:"
    echo "  ffplay -fflags nobuffer -flags low_delay 'srt://0.0.0.0:$PORT?mode=listener&latency=80'"
    exit 0
    ;;
  start)
    ;;
  *)
    echo "usage: ./scripts/start_camera.sh [start|restart|handeye|stop|status|rtp|srt|stop-rtp|rtp-status]"
    exit 2
    ;;
esac

if [ "${TRASH_CAMERA_MODE:-}" = "handeye" ]; then
  REQUESTED_PROFILE="handeye"
fi

if camera_process_running; then
  CURRENT_PROFILE="$(cat "$CAMERA_MODE_FILE" 2>/dev/null || echo full)"
  if [ "$CURRENT_PROFILE" != "$REQUESTED_PROFILE" ]; then
    echo "camera running in $CURRENT_PROFILE mode; restarting for $REQUESTED_PROFILE"
    stop_camera >/dev/null 2>&1 || true
    sleep 2
  elif ! camera_has_required_publishers "$REQUESTED_PROFILE"; then
    echo "camera process exists but required ROS publishers are missing; restarting stale camera"
    stop_camera >/dev/null 2>&1 || true
    sleep 2
  else
    echo "$REQUESTED_PROFILE" > "$CAMERA_MODE_FILE"
    enable_pointcloud_for_navigation
    echo "camera already running"
    echo "logs: $LOG_DIR/realsense.log"
    exit 0
  fi
fi

if ! ros2 pkg prefix realsense2_camera >/dev/null 2>&1; then
  echo "realsense2_camera package not found"
  exit 2
fi

echo "$REQUESTED_PROFILE" > "$CAMERA_MODE_FILE"

if [ "${TRASH_CAMERA_FOREGROUND:-0}" = "1" ]; then
  echo "camera starting in foreground profile=$REQUESTED_PROFILE"
  exec ros2 launch trash_robot_bringup camera_realsense.launch.py
fi

nohup ros2 launch trash_robot_bringup camera_realsense.launch.py \
  > "$LOG_DIR/realsense.log" 2>&1 &
write_pid camera "$!"

if wait_for_camera_ready "$REQUESTED_PROFILE" 25; then
  echo "camera topics online"
else
  echo "camera start failed: required ROS publishers did not appear within 25s"
  echo "last log lines:"
  tail -n 80 "$LOG_DIR/realsense.log" 2>/dev/null || true
  exit 1
fi
enable_pointcloud_for_navigation

echo "camera started"
echo "profile: $REQUESTED_PROFILE"
echo "logs: $LOG_DIR/realsense.log"
echo "topics:"
echo "  /camera/camera/color/image_raw"
echo "  /camera/camera/color/camera_info"
echo "  /camera/camera/aligned_depth_to_color/image_raw"
echo "  /camera/camera/depth/image_rect_raw"
echo "  /camera/camera/depth/color/points"
