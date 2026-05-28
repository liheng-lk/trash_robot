#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/common.sh"
if [ "${TRASH_STACK_INTERNAL:-}" != "1" ]; then
  # shellcheck disable=SC1091
  source "$SCRIPT_DIR/lib/stack.sh"
  source_debug
  stack_dispatch arm "$@"
  exit $?
fi
source_debug

ACTION="${1:-start}"
LOG_DIR="$TRASH_ROBOT_LOG_DIR/arm"
ROARM_SERIAL_PORT="${ROARM_SERIAL_PORT:-/dev/roarm}"
ROARM_SERIAL_WAIT_SEC="${ROARM_SERIAL_WAIT_SEC:-15}"
ROARM_MOVEIT_BOOT_WAIT_SEC="${ROARM_MOVEIT_BOOT_WAIT_SEC:-10}"
ROARM_SERVICE_WAIT_SEC="${ROARM_SERVICE_WAIT_SEC:-35}"
ROARM_MOVE_VELOCITY_SCALE="${ROARM_MOVE_VELOCITY_SCALE:-0.32}"
ROARM_MOVE_ACCELERATION_SCALE="${ROARM_MOVE_ACCELERATION_SCALE:-0.32}"
ROARM_MOVE_PLANNING_TIME="${ROARM_MOVE_PLANNING_TIME:-0.6}"
mkdir -p "$LOG_DIR"

wait_for_path() {
  local path="$1"
  local timeout="$2"
  local start
  start="$(date +%s)"
  while [ ! -e "$path" ]; do
    if [ $(( $(date +%s) - start )) -ge "$timeout" ]; then
      return 1
    fi
    sleep 0.5
  done
}

wait_for_service() {
  local service="$1"
  local timeout="$2"
  local start
  start="$(date +%s)"
  while true; do
    ros2 service list 2>/dev/null | grep -qx "$service" && return 0
    if [ $(( $(date +%s) - start )) -ge "$timeout" ]; then
      return 1
    fi
    sleep 0.5
  done
}

pid_running() {
  local name="$1" file pid
  file="$(pid_file_for "$name")"
  [ -f "$file" ] || return 1
  pid="$(cat "$file" 2>/dev/null || true)"
  [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null
}

get_pose_responsive() {
  timeout 5s ros2 service call /get_pose_cmd roarm_moveit/srv/GetPoseCmd "{}" >/tmp/trash_robot_v3_get_pose_probe.log 2>&1
}

arm_processes_ready() {
  pid_running arm_driver &&
    pid_running arm_moveit &&
    pid_running arm_getpose &&
    pid_running arm_movepoint &&
    pid_running arm_gripper
}

status() {
  echo "serial: $ROARM_SERIAL_PORT exists=$([ -e "$ROARM_SERIAL_PORT" ] && echo yes || echo no)"
  status_pid arm_driver || true
  status_pid arm_moveit || true
  status_pid arm_getpose || true
  status_pid arm_movepoint || true
  status_pid arm_gripper || true
  echo -n "/get_pose_cmd: "
  if get_pose_responsive; then
    echo RESPONSIVE
  elif ros2 service list 2>/dev/null | grep -qx /get_pose_cmd; then
    echo LISTED_BUT_NOT_RESPONDING
  else
    echo OFFLINE
  fi
  echo -n "/move_point_cmd: "
  if pid_running arm_movepoint && ros2 service list 2>/dev/null | grep -qx /move_point_cmd; then
    echo ONLINE
  elif ros2 service list 2>/dev/null | grep -qx /move_point_cmd; then
    echo LISTED_BUT_PROCESS_STOPPED
  else
    echo OFFLINE
  fi
}

start_arm() {
  if [ "$(current_mode)" = "CALIBRATION" ]; then
    echo "ERROR: 当前 CALIBRATION 模式占用维护流程，禁止启动机械臂运行服务" >&2
    exit 1
  fi
  if [ "$(current_mode)" = "ARM_MANUAL" ]; then
    echo "ERROR: 当前 ARM_MANUAL 模式占用机械臂，禁止启动机械臂运行服务" >&2
    exit 1
  fi
  if arm_processes_ready && get_pose_responsive && ros2 service list 2>/dev/null | grep -qx /move_point_cmd; then
    echo "arm services already ready"
    status
    return 0
  fi
  rm -f "$LOG_DIR/start_arm_error.log"
  if ! wait_for_path "$ROARM_SERIAL_PORT" "$ROARM_SERIAL_WAIT_SEC"; then
    echo "ERROR roarm serial device not found: $ROARM_SERIAL_PORT" | tee "$LOG_DIR/start_arm_error.log"
    echo "hint: check USB power/cable and udev rule /dev/roarm -> ttyUSBx" | tee -a "$LOG_DIR/start_arm_error.log"
    exit 1
  fi

  stop_arm >/dev/null 2>&1 || true

  nohup ros2 run roarm_driver roarm_driver --ros-args \
    -p serial_port:="$ROARM_SERIAL_PORT" \
    -p joint_command_min_interval_s:=0.08 \
    -p joint_command_deadband_rad:=0.003 \
    > "$LOG_DIR/roarm_driver.log" 2>&1 &
  write_pid arm_driver "$!"
  sleep 2

  if command -v xvfb-run >/dev/null 2>&1; then
    nohup xvfb-run -a ros2 launch roarm_moveit_cmd command_control.launch.py use_rviz:=false \
      > "$LOG_DIR/roarm_moveit.log" 2>&1 &
  else
    nohup ros2 launch roarm_moveit_cmd command_control.launch.py use_rviz:=false \
      > "$LOG_DIR/roarm_moveit.log" 2>&1 &
  fi
  write_pid arm_moveit "$!"
  sleep "$ROARM_MOVEIT_BOOT_WAIT_SEC"

  nohup ros2 run roarm_moveit_cmd getposecmd > "$LOG_DIR/getposecmd.log" 2>&1 &
  write_pid arm_getpose "$!"
  nohup ros2 run roarm_moveit_cmd movepointcmd --ros-args \
    -p velocity_scale:="$ROARM_MOVE_VELOCITY_SCALE" \
    -p acceleration_scale:="$ROARM_MOVE_ACCELERATION_SCALE" \
    -p planning_time:="$ROARM_MOVE_PLANNING_TIME" \
    > "$LOG_DIR/movepointcmd.log" 2>&1 &
  write_pid arm_movepoint "$!"
  nohup ros2 run roarm_moveit_cmd setgrippercmd > "$LOG_DIR/setgrippercmd.log" 2>&1 &
  write_pid arm_gripper "$!"

  wait_for_service /move_point_cmd "$ROARM_SERVICE_WAIT_SEC" || {
    echo "ERROR /move_point_cmd service not ready after ${ROARM_SERVICE_WAIT_SEC}s" | tee "$LOG_DIR/start_arm_error.log"
    tail -n 60 "$LOG_DIR/movepointcmd.log" 2>/dev/null || true
    exit 1
  }
  wait_for_service /get_pose_cmd "$ROARM_SERVICE_WAIT_SEC" || {
    echo "ERROR /get_pose_cmd service not ready after ${ROARM_SERVICE_WAIT_SEC}s" | tee "$LOG_DIR/start_arm_error.log"
    tail -n 60 "$LOG_DIR/getposecmd.log" 2>/dev/null || true
    exit 1
  }
  get_pose_responsive || {
    echo "ERROR /get_pose_cmd service listed but not responding" | tee "$LOG_DIR/start_arm_error.log"
    cat /tmp/trash_robot_v3_get_pose_probe.log 2>/dev/null || true
    exit 1
  }

  echo "arm services started"
  echo "serial: $ROARM_SERIAL_PORT"
  echo "logs: $LOG_DIR"
}

case "$ACTION" in
  start)
    start_arm
    ;;
  stop)
    stop_arm >/dev/null 2>&1 || true
    echo "arm stopped"
    ;;
  restart)
    stop_arm >/dev/null 2>&1 || true
    start_arm
    ;;
  status)
    status
    ;;
  *)
    echo "usage: $0 start|stop|restart|status" >&2
    exit 2
    ;;
esac
