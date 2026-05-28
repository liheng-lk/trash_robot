#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/common.sh"
source_debug

ACTION="${1:-start}"
LOG_DIR="$TRASH_ROBOT_LOG_DIR/rviz_web"
DISPLAY_ID="${TRASH_RVIZ_DISPLAY:-:88}"
SCREEN_GEOMETRY="${TRASH_RVIZ_SCREEN:-1400x900x24}"
VNC_PORT="${TRASH_RVIZ_VNC_PORT:-5901}"
WEB_PORT="${TRASH_RVIZ_WEB_PORT:-6080}"
HOST="${TRASH_ROBOT_HOST:-192.168.1.121}"
RVIZ_CONFIG="${TRASH_RVIZ_CONFIG:-$TRASH_ROBOT_ROOT/config/rviz/navigation_light.rviz}"
NOVNC_DIR="${TRASH_NOVNC_DIR:-$TRASH_ROBOT_RUNTIME/third_party/noVNC}"

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
export PATH="$HOME/.local/bin:$PATH"

wait_port() {
  local port="$1"
  local waited=0
  while [ "$waited" -lt 12 ]; do
    if python3 - "$port" <<'PY' >/dev/null 2>&1
import socket
import sys
sock = socket.create_connection(("127.0.0.1", int(sys.argv[1])), timeout=0.4)
sock.close()
PY
    then
      return 0
    fi
    waited=$((waited + 1))
    sleep 0.5
  done
  return 1
}

rviz_web_ready() {
  wait_port "$WEB_PORT"
}

ensure_deps() {
  if [ ! -f "$NOVNC_DIR/vnc.html" ] || ! python3 - <<'PY' >/dev/null 2>&1
import websockify
PY
  then
    "$SCRIPT_DIR/setup_rviz_web.sh"
  fi
}

start_marker_bridge() {
  if pgrep -af "map_to_marker.py" >/dev/null 2>&1; then
    return 0
  fi
  start_detached rviz_map_marker "$LOG_DIR/map_to_marker.log" \
    python3 "$SCRIPT_DIR/map_to_marker.py"
}

start_rviz_web() {
  if [ ! -f "$RVIZ_CONFIG" ]; then
    echo "ERROR RViz config not found: $RVIZ_CONFIG" >&2
    exit 1
  fi
  ensure_deps

  if ! pgrep -af "Xvfb $DISPLAY_ID" >/dev/null 2>&1; then
    start_detached rviz_xvfb "$LOG_DIR/xvfb.log" \
      Xvfb "$DISPLAY_ID" -screen 0 "$SCREEN_GEOMETRY" -ac +extension GLX +render -noreset
    sleep 1
  fi

  if ! wait_port "$VNC_PORT"; then
    start_detached rviz_x11vnc "$LOG_DIR/x11vnc.log" \
      x11vnc -display "$DISPLAY_ID" -rfbport "$VNC_PORT" -localhost -forever -shared -nopw -noxdamage -repeat
    sleep 1
  fi

  if ! rviz_web_ready; then
    start_detached rviz_websockify "$LOG_DIR/websockify.log" \
      websockify --web "$NOVNC_DIR" "$WEB_PORT" "127.0.0.1:$VNC_PORT"
    sleep 1
  fi

  start_marker_bridge

  if ! pgrep -af "rviz2.*$RVIZ_CONFIG" >/dev/null 2>&1; then
    export DISPLAY="$DISPLAY_ID"
    export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
    export QT_QPA_PLATFORM=xcb
    export QT_OPENGL=software
    export LIBGL_ALWAYS_SOFTWARE=1
    export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
    unset WAYLAND_DISPLAY
    start_detached rviz2_web "$LOG_DIR/rviz2.log" \
      rviz2 -d "$RVIZ_CONFIG"
  fi

  if ! rviz_web_ready; then
    echo "ERROR RViz Web did not become ready on port $WEB_PORT" >&2
    tail -n 80 "$LOG_DIR/websockify.log" 2>/dev/null || true
    exit 1
  fi

  echo "rviz_web started"
  echo "RViz Web: http://$HOST:$WEB_PORT/vnc.html?autoconnect=1&resize=remote&path=websockify"
  echo "rviz_config: $RVIZ_CONFIG"
  echo "logs: $LOG_DIR"
}

stop_rviz_web() {
  stop_pid rviz2_web >/dev/null 2>&1 || true
  stop_pid rviz_websockify >/dev/null 2>&1 || true
  stop_pid rviz_x11vnc >/dev/null 2>&1 || true
  stop_pid rviz_xvfb >/dev/null 2>&1 || true
  stop_pid rviz_map_marker >/dev/null 2>&1 || true
  pkill -f "rviz2.*$RVIZ_CONFIG" 2>/dev/null || true
  pkill -f "websockify.*$WEB_PORT.*127.0.0.1:$VNC_PORT" 2>/dev/null || true
  pkill -f "x11vnc.*-rfbport $VNC_PORT" 2>/dev/null || true
  pkill -f "Xvfb $DISPLAY_ID" 2>/dev/null || true
  pkill -f "map_to_marker.py" 2>/dev/null || true
  echo "rviz_web stopped"
}

status_rviz_web() {
  status_pid rviz_xvfb || true
  status_pid rviz_x11vnc || true
  status_pid rviz_websockify || true
  status_pid rviz2_web || true
  status_pid rviz_map_marker || true
  echo "RViz Web: http://$HOST:$WEB_PORT/vnc.html?autoconnect=1&resize=remote&path=websockify"
}

case "$ACTION" in
  start)
    start_rviz_web
    ;;
  stop)
    stop_rviz_web
    ;;
  restart)
    stop_rviz_web
    sleep 1
    start_rviz_web
    ;;
  status)
    status_rviz_web
    ;;
  *)
    echo "usage: $0 start|stop|restart|status" >&2
    exit 2
    ;;
esac
