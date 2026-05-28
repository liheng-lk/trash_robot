#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export TRASH_YOLO_PROFILE="${TRASH_YOLO_PROFILE:-paper_ball}"

usage() {
  echo "usage: $0 dry|live|restart [dry|live] [provider]" >&2
  echo "       $0 stop|status" >&2
}

case "${1:-}" in
  dry|live)
    "$SCRIPT_DIR/start_yolo_detector.sh" start
    export TRASH_ENABLE_LOCAL_CANDIDATE=0
    exec "$SCRIPT_DIR/start_grasp.sh" "$@"
    ;;
  restart)
    "$SCRIPT_DIR/start_yolo_detector.sh" restart
    export TRASH_ENABLE_LOCAL_CANDIDATE=0
    exec "$SCRIPT_DIR/start_grasp.sh" "$@"
    ;;
  stop)
    "$SCRIPT_DIR/start_yolo_detector.sh" stop || true
    exec "$SCRIPT_DIR/start_grasp.sh" "$@"
    ;;
  status)
    "$SCRIPT_DIR/start_yolo_detector.sh" status || true
    exec "$SCRIPT_DIR/start_grasp.sh" "$@"
    ;;
  *)
    usage
    exit 2
    ;;
esac
