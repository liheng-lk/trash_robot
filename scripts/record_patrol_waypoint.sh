#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck disable=SC1091
source "$SCRIPT_DIR/common.sh"
source_debug

exec python3 "$SCRIPT_DIR/record_patrol_waypoint.py" "$@"
