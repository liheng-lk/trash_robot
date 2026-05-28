#!/usr/bin/env bash
set -e
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
source_debug
NAME="${1:-map_$(date +%Y%m%d_%H%M%S)}"
TARGET="$TRASH_ROBOT_ROOT/maps/$NAME"
mkdir -p "$TRASH_ROBOT_ROOT/maps"
ros2 run nav2_map_server map_saver_cli -f "$TARGET"
echo "$TARGET.yaml" > "$TRASH_ROBOT_RUNTIME/current_map.txt"
echo "map saved: $TARGET.yaml"
