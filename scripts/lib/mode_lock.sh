#!/usr/bin/env bash

TRASH_MODE_LOCK_FILE="${TRASH_MODE_LOCK_FILE:-/tmp/trash_robot_v3_mode.lock}"
TRASH_PID_DIR="${TRASH_PID_DIR:-/tmp/trash_robot_v3_pids}"

ensure_runtime_dirs() {
  mkdir -p "$(dirname "$TRASH_MODE_LOCK_FILE")" "$TRASH_PID_DIR"
}

current_mode() {
  ensure_runtime_dirs
  if [ ! -f "$TRASH_MODE_LOCK_FILE" ]; then
    echo "IDLE"
    return 0
  fi
  awk -F: '/^mode:/ {gsub(/[ "]/, "", $2); print $2; found=1; exit} END {if (!found) print "IDLE"}' "$TRASH_MODE_LOCK_FILE"
}

deny_if_conflict() {
  local requested="$1"
  local current
  current="$(current_mode)"
  if [ "$current" = "ESTOP" ] && [ "$requested" != "ESTOP" ]; then
    echo "ERROR: 当前 ESTOP 锁定，禁止启动 $requested" >&2
    return 1
  fi
  case "$requested:$current" in
    GRASP:ARM_MANUAL)
      echo "ERROR: 当前 ARM_MANUAL 模式占用机械臂，禁止启动 GRASP live" >&2
      return 1
      ;;
    GRASP:CALIBRATION)
      echo "ERROR: 当前 CALIBRATION 模式占用相机，禁止启动 GRASP" >&2
      return 1
      ;;
    ARM_MANUAL:GRASP|ARM_MANUAL:CALIBRATION)
      echo "ERROR: 当前 $current 模式占用机械臂，禁止启动 ARM_MANUAL" >&2
      return 1
      ;;
    CALIBRATION:GRASP|CALIBRATION:NAVIGATION|CALIBRATION:VIDEO|CALIBRATION:ARM_MANUAL)
      echo "ERROR: 当前 $current 模式占用资源，禁止启动 CALIBRATION" >&2
      return 1
      ;;
    NAVIGATION:CALIBRATION)
      echo "ERROR: 当前 CALIBRATION 模式占用相机，禁止启动 NAVIGATION" >&2
      return 1
      ;;
    VIDEO:CALIBRATION)
      echo "ERROR: 当前 CALIBRATION 模式占用相机，禁止启动 VIDEO" >&2
      return 1
      ;;
  esac
  return 0
}

acquire_mode() {
  local mode="$1"
  ensure_runtime_dirs
  deny_if_conflict "$mode" || return 1
  {
    printf 'mode: "%s"\n' "$mode"
    printf 'owner: "%s"\n' "${TRASH_MODE_OWNER:-script}"
    printf 'stamp: %s\n' "$(date +%s)"
  } > "$TRASH_MODE_LOCK_FILE"
}

release_mode() {
  local mode="$1"
  ensure_runtime_dirs
  if [ "$(current_mode)" = "$mode" ]; then
    rm -f "$TRASH_MODE_LOCK_FILE"
  fi
}

pid_file_for() {
  local name="$1"
  ensure_runtime_dirs
  printf '%s/%s.pid\n' "$TRASH_PID_DIR" "$name"
}

write_pid() {
  local name="$1"
  local pid="$2"
  ensure_runtime_dirs
  if [ -z "$pid" ] || ! kill -0 "$pid" 2>/dev/null; then
    echo "ERROR: invalid pid for $name: $pid" >&2
    return 1
  fi
  echo "$pid" > "$(pid_file_for "$name")"
}

stop_pid() {
  local name="$1"
  local file pid pgid
  file="$(pid_file_for "$name")"
  if [ ! -f "$file" ]; then
    return 0
  fi
  pid="$(cat "$file" 2>/dev/null || true)"
  if [ -z "$pid" ] || ! kill -0 "$pid" 2>/dev/null; then
    rm -f "$file"
    return 0
  fi
  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
  if [ -n "$pgid" ]; then
    kill -TERM "-$pgid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  else
    kill -TERM "$pid" 2>/dev/null || true
  fi
  sleep "${TRASH_STOP_SLEEP:-0.8}"
  if kill -0 "$pid" 2>/dev/null; then
    if [ -n "$pgid" ]; then
      kill -KILL "-$pgid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
    else
      kill -KILL "$pid" 2>/dev/null || true
    fi
  fi
  rm -f "$file"
}

status_pid() {
  local name="$1"
  local file pid
  file="$(pid_file_for "$name")"
  if [ ! -f "$file" ]; then
    echo "$name: STOPPED"
    return 1
  fi
  pid="$(cat "$file" 2>/dev/null || true)"
  if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
    echo "$name: RUNNING pid=$pid"
    return 0
  fi
  rm -f "$file"
  echo "$name: STOPPED stale_pid=${pid:-none}"
  return 1
}
