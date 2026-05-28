#!/usr/bin/env bash
set -e

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
source_debug

ACTION="${1:-start}"
LOG_DIR="$TRASH_ROBOT_LOG_DIR/arm_keyboard"
ROARM_SERIAL_PORT="${ROARM_SERIAL_PORT:-/dev/roarm}"
mkdir -p "$LOG_DIR"

wait_for_path() {
  local path="$1"
  local timeout="${2:-15}"
  local start
  start="$(date +%s)"
  while [ ! -e "$path" ]; do
    if [ $(( $(date +%s) - start )) -ge "$timeout" ]; then
      return 1
    fi
    sleep 0.5
  done
}

status() {
  echo "mode_lock: $(current_mode)"
  echo "serial: $ROARM_SERIAL_PORT exists=$([ -e "$ROARM_SERIAL_PORT" ] && echo yes || echo no)"
  status_pid arm_keyboard_driver || true
  status_pid arm_keyboard_moveit || true
  status_pid arm_keyboard_servo || true
  status_pid arm_keyboard_getpose || true
  status_pid arm_keyboard_movepoint || true
  status_pid arm_keyboard_gripper || true
}

start_keyboard_stack() {
  if status_pid arm_keyboard_driver >/dev/null 2>&1; then
    echo "arm keyboard stack already running"
    status
    return 0
  fi
  stop_arm >/dev/null 2>&1 || true
  export TRASH_MODE_OWNER="start_arm_keyboard.sh"
  acquire_mode ARM_MANUAL
  if ! wait_for_path "$ROARM_SERIAL_PORT" 15; then
    release_mode ARM_MANUAL
    echo "ERROR roarm serial device not found: $ROARM_SERIAL_PORT" | tee "$LOG_DIR/start_arm_keyboard_error.log"
    exit 1
  fi

  nohup ros2 run roarm_driver roarm_driver --ros-args \
    -p serial_port:="$ROARM_SERIAL_PORT" \
    -p joint_command_min_interval_s:=0.08 \
    -p joint_command_deadband_rad:=0.003 \
    > "$LOG_DIR/roarm_driver.log" 2>&1 &
  write_pid arm_keyboard_driver "$!"

  sleep 2
  if command -v xvfb-run >/dev/null 2>&1; then
    nohup xvfb-run -a ros2 launch roarm_moveit_cmd command_control.launch.py use_rviz:=false \
      > "$LOG_DIR/roarm_moveit.log" 2>&1 &
  else
    nohup ros2 launch roarm_moveit_cmd command_control.launch.py use_rviz:=false \
      > "$LOG_DIR/roarm_moveit.log" 2>&1 &
  fi
  write_pid arm_keyboard_moveit "$!"

  sleep 8
  nohup ros2 launch trash_robot_bringup roarm_servo_headless.launch.py \
    > "$LOG_DIR/servo.log" 2>&1 &
  write_pid arm_keyboard_servo "$!"

  sleep 3
  timeout 10s bash -c 'until ros2 service list | grep -qx /servo_node/start_servo; do sleep 0.5; done' \
    >/dev/null 2>&1 || true
  ros2 service call /servo_node/start_servo std_srvs/srv/Trigger "{}" \
    >> "$LOG_DIR/servo.log" 2>&1 || true

  nohup ros2 run roarm_moveit_cmd getposecmd > "$LOG_DIR/getposecmd.log" 2>&1 &
  write_pid arm_keyboard_getpose "$!"
  nohup ros2 run roarm_moveit_cmd movepointcmd \
    --ros-args \
    -p velocity_scale:="${ROARM_MOVE_VELOCITY_SCALE:-0.32}" \
    -p acceleration_scale:="${ROARM_MOVE_ACCELERATION_SCALE:-0.32}" \
    -p planning_time:="${ROARM_MOVE_PLANNING_TIME:-0.6}" \
    > "$LOG_DIR/movepointcmd.log" 2>&1 &
  write_pid arm_keyboard_movepoint "$!"
  nohup ros2 run roarm_moveit_cmd setgrippercmd > "$LOG_DIR/setgrippercmd.log" 2>&1 &
  write_pid arm_keyboard_gripper "$!"

  echo "arm keyboard stack started"
  echo "serial: $ROARM_SERIAL_PORT"
  echo "logs: $LOG_DIR"
  echo "Run keyboard control in an SSH terminal:"
  echo "  cd $TRASH_ROBOT_ROOT"
  echo "  ./scripts/run_arm_keyboard.sh"
}

case "$ACTION" in
  start)
    start_keyboard_stack
    ;;
  stop)
    stop_arm >/dev/null 2>&1 || true
    echo "arm keyboard stopped"
    ;;
  restart)
    stop_arm >/dev/null 2>&1 || true
    start_keyboard_stack
    ;;
  status)
    status
    ;;
  *)
    echo "usage: $0 start|stop|restart|status" >&2
    exit 2
    ;;
esac
