#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/common.sh"
source_debug

ACTION="${1:-start}"
LOG_DIR="$TRASH_ROBOT_LOG_DIR/web"
mkdir -p "$LOG_DIR"

USER_SITE="$(python3 - <<'PY'
import site
print(site.getusersitepackages())
PY
)"
case ":${PYTHONPATH:-}:" in
  *":$USER_SITE:"*) ;;
  *) export PYTHONPATH="$USER_SITE${PYTHONPATH:+:$PYTHONPATH}" ;;
esac

export TRASH_WEB_HOST="${TRASH_WEB_HOST:-0.0.0.0}"
export TRASH_WEB_PORT="${TRASH_WEB_PORT:-8095}"
export TRASH_ROBOT_HOST="${TRASH_ROBOT_HOST:-192.168.1.121}"

missing="$(python3 - <<'PY'
missing = []
for name in ("fastapi", "uvicorn"):
    try:
        __import__(name)
    except Exception:
        missing.append(name)
print(" ".join(missing))
PY
)"
if [ -n "$missing" ]; then
  echo "Missing WebUI dependencies: $missing"
  echo "Install on robot: python3 -m pip install --user fastapi uvicorn"
  exit 2
fi

manager_process_alive() {
  python3 - "$TRASH_ROBOT_ROOT" <<'PY'
import sys
from pathlib import Path
root = sys.argv[1]
needle = f"{root}/install/trash_robot_manager/lib/trash_robot_manager/robot_manager"
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():
        continue
    try:
        cmdline = (proc / 'cmdline').read_bytes().replace(b'\0', b' ').decode('utf-8', errors='ignore')
    except OSError:
        continue
    if needle in cmdline:
        sys.exit(0)
sys.exit(1)
PY
}

manager_ready() {
  manager_process_alive || return 1
  timeout 4s ros2 node list 2>/dev/null | grep -qx /trash_robot_manager || return 1
  timeout 4s ros2 service list 2>/dev/null | grep -qx /trash_manager/start_base
}

start_manager_if_needed() {
  if manager_ready; then
    return 0
  fi
  if manager_process_alive; then
    for _ in $(seq 1 10); do
      manager_ready && return 0
      sleep 0.5
    done
    echo "WARN: robot_manager process exists but node/service is not visible; WebUI will start in degraded mode" >&2
    return 0
  fi
  start_detached manager "$LOG_DIR/robot_manager.log" \
    ros2 run trash_robot_manager robot_manager --ros-args -p project_root:="$TRASH_ROBOT_ROOT"
  for _ in $(seq 1 12); do
    manager_ready && return 0
    sleep 0.5
  done
  echo "WARN: robot_manager did not become ready; WebUI will still start in degraded mode" >&2
  return 0
}

mission_ready() {
  timeout 4s ros2 service list 2>/dev/null | grep -qx /trash_mission/start_patrol
}

start_mission_if_needed() {
  if pgrep -af "mission_supervisor|/lib/trash_robot_mission/mission_supervisor" >/dev/null 2>&1; then
    return 0
  fi
  start_detached mission_supervisor "$TRASH_ROBOT_LOG_DIR/navigation/mission_supervisor.log" \
    ros2 launch trash_robot_mission mission_supervisor.launch.py \
      route_file:="$TRASH_ROBOT_ROOT/config/mission/patrol_routes.yaml" \
      sort_config_file:="$TRASH_ROBOT_ROOT/config/grasp/trash_sort_params.yaml" \
      auto_start:=false
  for _ in $(seq 1 12); do
    mission_ready && return 0
    sleep 0.5
  done
  echo "WARN: mission_supervisor did not expose patrol service yet; WebUI will show patrol as offline" >&2
  return 0
}

wait_http() {
  local waited=0
  while [ "$waited" -lt 20 ]; do
    if python3 - "$TRASH_WEB_PORT" <<'PY' >/dev/null 2>&1
import socket
import sys
sock = socket.create_connection(("127.0.0.1", int(sys.argv[1])), timeout=0.5)
sock.close()
PY
    then
      return 0
    fi
    waited=$((waited + 1))
    sleep 1
  done
  return 1
}

start_web() {
  # WebUI can stay alive while the ROS manager has crashed or been stopped.
  # Always repair backend prerequisites before returning "already running".
  start_manager_if_needed
  start_mission_if_needed
  if status_pid web_console >/dev/null 2>&1; then
    echo "web_console already running"
    echo "WebUI: http://$TRASH_ROBOT_HOST:$TRASH_WEB_PORT"
    return 0
  fi
  start_detached web_console "$LOG_DIR/web_console.log" \
    ros2 run trash_robot_web web_console --ros-args \
      -p project_root:="$TRASH_ROBOT_ROOT" \
      -p host:="$TRASH_WEB_HOST" \
      -p port:="$TRASH_WEB_PORT" \
      -p video_url:="http://$TRASH_ROBOT_HOST:8092/stream.mjpg"
  if ! wait_http; then
    echo "ERROR: WebUI did not become ready on port $TRASH_WEB_PORT" >&2
    tail -n 80 "$LOG_DIR/web_console.log" 2>/dev/null || true
    exit 1
  fi
  echo "web_console started"
  echo "WebUI: http://$TRASH_ROBOT_HOST:$TRASH_WEB_PORT"
  echo "logs: $LOG_DIR/web_console.log"
}

case "$ACTION" in
  start)
    start_web
    ;;
  stop)
    stop_pid web_console >/dev/null 2>&1 || true
    echo "web_console stopped"
    ;;
  restart)
    stop_pid web_console >/dev/null 2>&1 || true
    sleep 1
    start_web
    ;;
  status)
    status_pid web_console || true
    if manager_ready; then
      echo "robot_manager: RUNNING"
    elif manager_process_alive; then
      echo "robot_manager: STARTING_OR_DEGRADED"
    else
      echo "robot_manager: STOPPED"
    fi
    if mission_ready; then
      echo "mission_supervisor: READY"
    else
      echo "mission_supervisor: NOT_READY"
    fi
    echo "WebUI: http://$TRASH_ROBOT_HOST:$TRASH_WEB_PORT"
    ;;
  *)
    echo "usage: $0 start|stop|restart|status" >&2
    exit 2
    ;;
esac
