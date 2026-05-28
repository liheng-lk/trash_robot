#!/usr/bin/env bash
set -e

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"

# This command needs a real terminal. Keep it foreground and use the debug DDS
# profile so it talks to the headless servo stack without FastDDS SHM errors.
source_debug

if [ "$(current_mode)" != "ARM_MANUAL" ]; then
  echo "ERROR: 当前不是 ARM_MANUAL 模式。先执行："
  echo "  ./scripts/start_arm_keyboard.sh start"
  exit 1
fi

if ! ros2 node list | grep -qx /servo_node; then
  echo "ERROR: /servo_node is not running. Start it first:"
  echo "  ./scripts/start_arm_keyboard.sh start"
  exit 1
fi

ros2 service call /servo_node/start_servo std_srvs/srv/Trigger "{}" >/dev/null 2>&1 || true

exec ros2 run roarm_moveit_cmd keyboardcontrol
