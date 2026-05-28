#!/usr/bin/env bash

TRASH_ROBOT_ROOT="${TRASH_ROBOT_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
TRASH_ROBOT_RUNTIME="$TRASH_ROBOT_ROOT/runtime"
TRASH_ROBOT_LOG_DIR="$TRASH_ROBOT_RUNTIME/logs"
TRASH_GENERATED_CONFIG="$TRASH_ROBOT_RUNTIME/generated_config"
TRASH_MANAGER_MODE_FILE="$TRASH_ROBOT_RUNTIME/manager_mode.yaml"
export TRASH_ROBOT_ROOT TRASH_ROBOT_RUNTIME TRASH_ROBOT_LOG_DIR TRASH_GENERATED_CONFIG TRASH_MANAGER_MODE_FILE

mkdir -p "$TRASH_ROBOT_RUNTIME" "$TRASH_ROBOT_LOG_DIR" "$TRASH_GENERATED_CONFIG"

# shellcheck disable=SC1091
source "$TRASH_ROBOT_ROOT/scripts/lib/mode_lock.sh"
# shellcheck disable=SC1091
source "$TRASH_ROBOT_ROOT/scripts/lib/motion_lock.sh"

configure_cyclonedds() {
  export ROS_DOMAIN_ID="${TRASH_ROS_DOMAIN_ID:-1}"
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI="file://$TRASH_ROBOT_ROOT/config/dds/cyclonedds_unicast.xml"
  unset FASTRTPS_DEFAULT_PROFILES_FILE
  unset RMW_FASTRTPS_USE_QOS_FROM_XML
  unset ROS_DISABLE_LOANED_MESSAGES
}

set_runtime_mode() {
  local mode="$1"
  local owner="${2:-script}"
  local reason="${3:-manual}"
  mkdir -p "$TRASH_ROBOT_RUNTIME"
  {
    printf 'mode: "%s"\n' "$mode"
    printf 'owner: "%s"\n' "$owner"
    printf 'reason: "%s"\n' "$reason"
    printf 'stamp: %s\n' "$(date +%s)"
  } > "$TRASH_MANAGER_MODE_FILE"
}

current_runtime_mode() {
  if [ ! -f "$TRASH_MANAGER_MODE_FILE" ]; then
    echo "IDLE"
    return 0
  fi
  awk -F: '/^mode:/ {gsub(/[ "]/, "", $2); print $2; exit}' "$TRASH_MANAGER_MODE_FILE"
}

clear_runtime_mode_if() {
  local current
  current="$(current_runtime_mode)"
  for mode in "$@"; do
    if [ "$current" = "$mode" ]; then
      set_runtime_mode "IDLE" "script" "clear $current"
      return 0
    fi
  done
}

stop_pause() {
  sleep "${TRASH_STOP_SLEEP:-0.4}"
}

start_detached() {
  local name="$1"
  local logfile="$2"
  shift 2
  mkdir -p "$(dirname "$logfile")"
  if command -v setsid >/dev/null 2>&1; then
    setsid nohup "$@" > "$logfile" 2>&1 < /dev/null &
  else
    nohup "$@" > "$logfile" 2>&1 < /dev/null &
  fi
  write_pid "$name" "$!"
}

load_local_secrets() {
  local secrets_file="$TRASH_ROBOT_RUNTIME/secrets/vlm.env"
  if [ -f "$secrets_file" ]; then
    set +u
    # shellcheck disable=SC1090
    source "$secrets_file"
    set +u
  fi
}

strip_path_entry() {
  local var_name="$1"
  local needle="$2"
  local value="${!var_name:-}"
  local out=""
  local old_ifs="$IFS"
  IFS=":"
  for part in $value; do
    if [ -n "$part" ] && [[ "$part" != *"$needle"* ]]; then
      if [ -z "$out" ]; then
        out="$part"
      else
        out="$out:$part"
      fi
    fi
  done
  IFS="$old_ifs"
  export "$var_name=$out"
}

source_runtime() {
  set +u
  strip_path_entry AMENT_PREFIX_PATH "/home/sunrise/sdk/roarm_ws_em0"
  strip_path_entry CMAKE_PREFIX_PATH "/home/sunrise/sdk/roarm_ws_em0"
  strip_path_entry COLCON_PREFIX_PATH "/home/sunrise/sdk/roarm_ws_em0"
  strip_path_entry LD_LIBRARY_PATH "/home/sunrise/sdk/roarm_ws_em0"
  strip_path_entry PYTHONPATH "/home/sunrise/sdk/roarm_ws_em0"

  [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
  export PYTHONDONTWRITEBYTECODE=1

  if [ -f /opt/tros/humble/setup.bash ]; then
    source /opt/tros/humble/setup.bash
  elif [ -f /opt/tros/setup.bash ]; then
    source /opt/tros/setup.bash
  fi

  [ -f "$TRASH_ROBOT_ROOT/install/setup.bash" ] && source "$TRASH_ROBOT_ROOT/install/setup.bash"
  configure_cyclonedds
  load_local_secrets
  case ":${PYTHONPATH:-}:" in
    *":/usr/lib/python3/dist-packages:"*) ;;
    *) export PYTHONPATH="/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}" ;;
  esac
  set +u
}

source_debug() {
  set +u
  strip_path_entry AMENT_PREFIX_PATH "/home/sunrise/sdk/roarm_ws_em0"
  strip_path_entry CMAKE_PREFIX_PATH "/home/sunrise/sdk/roarm_ws_em0"
  strip_path_entry COLCON_PREFIX_PATH "/home/sunrise/sdk/roarm_ws_em0"
  strip_path_entry LD_LIBRARY_PATH "/home/sunrise/sdk/roarm_ws_em0"
  strip_path_entry PYTHONPATH "/home/sunrise/sdk/roarm_ws_em0"
  [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
  if [ -f /opt/tros/humble/setup.bash ]; then
    source /opt/tros/humble/setup.bash
  elif [ -f /opt/tros/setup.bash ]; then
    source /opt/tros/setup.bash
  fi
  [ -f "$TRASH_ROBOT_ROOT/install/setup.bash" ] && source "$TRASH_ROBOT_ROOT/install/setup.bash"
  configure_cyclonedds
  load_local_secrets
  case ":${PYTHONPATH:-}:" in
    *":/usr/lib/python3/dist-packages:"*) ;;
    *) export PYTHONPATH="/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}" ;;
  esac
  export PYTHONDONTWRITEBYTECODE=1
  set +u
}

stop_nav() {
  set +e
  stop_pid navigation
  stop_pid nav_depth_to_scan
  stop_pid nav2
  stop_pid mission_supervisor
  clear_runtime_mode_if NAVIGATION
  release_mode NAVIGATION
  set -e
}

stop_pid_tree_exact() {
  local pid="$1"
  local child
  [ -n "$pid" ] || return 0
  kill -0 "$pid" 2>/dev/null || return 0
  for child in $(pgrep -P "$pid" 2>/dev/null || true); do
    stop_pid_tree_exact "$child"
  done
  kill -TERM "$pid" 2>/dev/null || true
  sleep "${TRASH_STOP_SLEEP:-0.4}"
  kill -0 "$pid" 2>/dev/null && kill -KILL "$pid" 2>/dev/null || true
}

stop_mapping() {
  set +e
  stop_pid mapping
  local legacy_pid=""
  if [ -f "$TRASH_ROBOT_RUNTIME/mapping.pid" ]; then
    legacy_pid="$(cat "$TRASH_ROBOT_RUNTIME/mapping.pid" 2>/dev/null || true)"
  fi
  if [ -n "$legacy_pid" ] && kill -0 "$legacy_pid" 2>/dev/null; then
    local args
    args="$(ps -o args= -p "$legacy_pid" 2>/dev/null || true)"
    if [[ "$args" == *"start_mapping.sh"* ]]; then
      stop_pid_tree_exact "$legacy_pid"
    fi
  fi
  rm -f "$TRASH_ROBOT_RUNTIME/mapping.pid"
  clear_runtime_mode_if NAVIGATION
  release_mode NAVIGATION
  set -e
}

stop_base() {
  set +e
  stop_pid base
  set -e
}

stop_handeye() {
  set +e
  stop_pid handeye_web
  local mode_file="$TRASH_ROBOT_RUNTIME/camera_mode.txt"
  if [ "$(cat "$mode_file" 2>/dev/null || true)" = "handeye" ]; then
    stop_camera >/dev/null 2>&1 || true
  fi
  clear_runtime_mode_if CALIBRATION
  release_mode CALIBRATION
  set -e
}

stop_camera() {
  set +e
  stop_pid camera
  rm -f "$TRASH_ROBOT_RUNTIME/camera_mode.txt"
  set -e
}

stop_arm() {
  set +e
  stop_pid arm_keyboard_control
  stop_pid arm_keyboard_servo
  stop_pid arm_keyboard_getpose
  stop_pid arm_keyboard_movepoint
  stop_pid arm_keyboard_gripper
  stop_pid arm_keyboard_moveit
  stop_pid arm_keyboard_driver
  stop_pid arm_getpose
  stop_pid arm_movepoint
  stop_pid arm_gripper
  stop_pid arm_moveit
  stop_pid arm_driver
  clear_runtime_mode_if ARM_MANUAL
  release_mode ARM_MANUAL
  set -e
}

stop_grasp() {
  set +e
  stop_pid grasp_pipeline
  stop_pid grasp_vlm
  stop_pid grasp_local_dosod
  # Clean up untracked grasp perception nodes left by manual ros2 run/launch sessions.
  pkill -TERM -f 'ros2 run trash_robot_vision vlm_trash_classifier' 2>/dev/null || true
  pkill -TERM -f '/home/sunrise/trash_robot_v3/install/trash_robot_vision/lib/trash_robot_vision/vlm_trash_classifier' 2>/dev/null || true
  pkill -TERM -f '/home/sunrise/trash_robot_v3/install/trash_robot_grasp/lib/trash_robot_grasp/handeye_target_transformer' 2>/dev/null || true
  pkill -TERM -f '/home/sunrise/trash_robot_v3/install/trash_robot_grasp/lib/trash_robot_grasp/roarm_sort_grasper' 2>/dev/null || true
  clear_runtime_mode_if GRASP
  release_mode GRASP
  set -e
}

stop_video() {
  set +e
  stop_pid video
  rm -f "$TRASH_ROBOT_RUNTIME/video_stream.pid" "$TRASH_ROBOT_RUNTIME/video_rtp.pid"
  release_mode VIDEO
  set -e
}
