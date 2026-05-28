#!/usr/bin/env bash

TRASH_MOTION_LOCK_FILE="${TRASH_MOTION_LOCK_FILE:-/tmp/trash_robot_v3_motion.lock}"
TRASH_ESTOP_LOCK_FILE="${TRASH_ESTOP_LOCK_FILE:-/tmp/trash_robot_v3_estop.lock}"

ensure_motion_dirs() {
  mkdir -p "$(dirname "$TRASH_MOTION_LOCK_FILE")" "$(dirname "$TRASH_ESTOP_LOCK_FILE")"
}

motion_lock_owner() {
  ensure_motion_dirs
  if [ ! -f "$TRASH_MOTION_LOCK_FILE" ]; then
    echo "IDLE"
    return 0
  fi
  awk -F: '/^owner:/ {gsub(/[ "]/, "", $2); print $2; found=1; exit} END {if (!found) print "UNKNOWN"}' "$TRASH_MOTION_LOCK_FILE"
}

estop_active() {
  ensure_motion_dirs
  [ -f "$TRASH_ESTOP_LOCK_FILE" ]
}

acquire_motion_lock() {
  local owner="$1"
  local current
  ensure_motion_dirs
  if estop_active; then
    echo "ERROR: ESTOP active; motion lock denied for $owner" >&2
    return 1
  fi
  current="$(motion_lock_owner)"
  if [ "$current" != "IDLE" ] && [ "$current" != "$owner" ]; then
    echo "ERROR: motion lock already held by $current; denied for $owner" >&2
    return 1
  fi
  {
    printf 'owner: "%s"\n' "$owner"
    printf 'stamp: %s\n' "$(date +%s)"
  } > "$TRASH_MOTION_LOCK_FILE"
}

release_motion_lock() {
  local owner="$1"
  ensure_motion_dirs
  if [ "$(motion_lock_owner)" = "$owner" ]; then
    rm -f "$TRASH_MOTION_LOCK_FILE"
  fi
}

trigger_estop() {
  local reason="${1:-manual}"
  ensure_motion_dirs
  {
    printf 'active: true\n'
    printf 'reason: "%s"\n' "$reason"
    printf 'stamp: %s\n' "$(date +%s)"
  } > "$TRASH_ESTOP_LOCK_FILE"
  {
    printf 'owner: "ESTOP"\n'
    printf 'stamp: %s\n' "$(date +%s)"
  } > "$TRASH_MOTION_LOCK_FILE"
}

reset_estop() {
  ensure_motion_dirs
  rm -f "$TRASH_ESTOP_LOCK_FILE"
  if [ "$(motion_lock_owner)" = "ESTOP" ]; then
    rm -f "$TRASH_MOTION_LOCK_FILE"
  fi
}

motion_status() {
  local estop="false"
  estop_active && estop="true"
  printf 'motion_owner=%s\n' "$(motion_lock_owner)"
  printf 'estop_active=%s\n' "$estop"
}
