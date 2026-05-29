#!/usr/bin/env bash
# Unified stack control for trash_robot_v3.
# Requires: source scripts/common.sh before this file.

STACK_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STACK_SCRIPT_DIR="$(cd "$STACK_LIB_DIR/.." && pwd)"

stack_usage() {
  cat <<'EOF'
Trash Robot V3 统一栈控制

用法:
  ./scripts/trash_stack.sh <组件> <start|stop|restart|status> [参数...]
  ./scripts/trash_stack.sh profile <名称> <start|stop>     # 组合启动
  ./scripts/trash_stack.sh status-all
  ./scripts/trash_stack.sh stop-all

组件:
  base | camera | arm | mapping | navigation | grasp | video | handeye

抓取:
  ./scripts/trash_stack.sh grasp start dry|live
  ./scripts/trash_stack.sh grasp stop|restart|status

导航:
  ./scripts/trash_stack.sh navigation start [map.yaml]

组合 profile (config/system/stack_contract.yaml):
  sensors | mobility | perception_dry | patrol

环境:
  TRASH_USE_MANAGER=1     优先走 Manager service
  TRASH_STACK_FOREGROUND=1  前台运行（调试用）

兼容: ./scripts/start_<组件>.sh 仍可用，内部调用本库。
EOF
}

stack_resolve_map() {
  local map_file="${1:-}"
  if [ -z "$map_file" ] && [ -f "$TRASH_ROBOT_RUNTIME/current_map.txt" ]; then
    map_file="$(tr -d '[:space:]' < "$TRASH_ROBOT_RUNTIME/current_map.txt")"
  fi
  if [ -z "$map_file" ] || [ ! -f "$map_file" ]; then
    if [ -f "$TRASH_ROBOT_ROOT/maps/344.yaml" ]; then
      map_file="$TRASH_ROBOT_ROOT/maps/344.yaml"
    else
      map_file="$(find "$TRASH_ROBOT_ROOT/maps" -maxdepth 1 -name '*.yaml' 2>/dev/null | head -n 1)"
    fi
  fi
  if [ -z "$map_file" ] || [ ! -f "$map_file" ]; then
    echo "ERROR: no map yaml found under $TRASH_ROBOT_ROOT/maps" >&2
    return 1
  fi
  printf '%s\n' "$map_file"
}

stack_topic_publishers() {
  local topic="$1"
  timeout 3s ros2 topic info "$topic" 2>/dev/null | awk '/Publisher count:/ {print $3; found=1} END {if (!found) print 0}'
}

stack_manager_ready() {
  ros2 service list 2>/dev/null | grep -qx /trash_manager/start_base
}

stack_manager_call() {
  local service="$1"
  local timeout_sec="${2:-120}"
  if ! stack_manager_ready; then
    echo "WARN: manager not ready, fallback to direct script" >&2
    return 1
  fi
  echo "[stack] manager service call $service"
  timeout "$timeout_sec" ros2 service call "$service" std_srvs/srv/Trigger "{}" 2>&1 | tail -n 5
  return "${PIPESTATUS[0]}"
}

stack_run_script() {
  local script="$1"
  shift
  local path="$STACK_SCRIPT_DIR/$script"
  if [ ! -f "$path" ]; then
    echo "ERROR: missing script $path" >&2
    return 1
  fi
  TRASH_STACK_INTERNAL=1 bash "$path" "$@"
}

stack_component_start() {
  local component="$1"
  shift
  local use_manager="${TRASH_USE_MANAGER:-0}"

  case "$component" in
    base)
      if [ "$use_manager" = "1" ] && stack_manager_call /trash_manager/start_base 90; then return 0; fi
      stack_run_script start_base.sh start "$@"
      ;;
    camera)
      if [ "$use_manager" = "1" ] && stack_manager_call /trash_manager/start_camera 120; then return 0; fi
      stack_run_script start_camera.sh start "$@"
      ;;
    arm)
      if [ "$use_manager" = "1" ] && stack_manager_call /trash_manager/start_arm 180; then return 0; fi
      stack_run_script start_arm.sh start "$@"
      ;;
    mapping)
      if [ "$use_manager" = "1" ] && stack_manager_call /trash_manager/start_mapping 120; then return 0; fi
      stack_run_script start_mapping.sh start "$@"
      ;;
    navigation)
      local map_file=""
      if [ -n "${1:-}" ]; then
        map_file="$1"
      else
        map_file="$(stack_resolve_map)" || return 1
      fi
      if [ "$use_manager" = "1" ] && stack_manager_call /trash_manager/start_navigation 180; then
        return 0
      fi
      stack_run_script start_navigation.sh start "$map_file"
      ;;
    grasp)
      local mode="${1:-dry}"
      case "$mode" in
        dry)
          if [ "$use_manager" = "1" ] && stack_manager_call /trash_manager/start_grasp_vlm_dry 180; then return 0; fi
          stack_run_script start_grasp.sh dry
          ;;
        live)
          if [ "$use_manager" = "1" ] && stack_manager_call /trash_manager/start_grasp_vlm_live 180; then return 0; fi
          stack_run_script start_grasp.sh live
          ;;
        *)
          echo "ERROR: grasp start requires dry or live" >&2
          return 2
          ;;
      esac
      ;;
    video)
      if [ "$use_manager" = "1" ] && stack_manager_call /trash_manager/start_video 120; then return 0; fi
      start_video_stream
      ;;
    handeye)
      if [ "$use_manager" = "1" ] && stack_manager_call /trash_manager/start_handeye 120; then return 0; fi
      stack_run_script start_handeye.sh start "$@"
      ;;
    *)
      echo "ERROR: unknown component $component" >&2
      return 2
      ;;
  esac
}

stack_component_stop() {
  local component="$1"
  local use_manager="${TRASH_USE_MANAGER:-0}"
  case "$component" in
    base)
      [ "$use_manager" = "1" ] && stack_manager_call /trash_manager/stop_base 60 && return 0
      stack_run_script start_base.sh stop
      ;;
    camera)
      [ "$use_manager" = "1" ] && stack_manager_call /trash_manager/stop_camera 60 && return 0
      stack_run_script start_camera.sh stop
      ;;
    arm)
      [ "$use_manager" = "1" ] && stack_manager_call /trash_manager/stop_arm 60 && return 0
      stack_run_script start_arm.sh stop
      ;;
    mapping)
      [ "$use_manager" = "1" ] && stack_manager_call /trash_manager/stop_mapping 60 && return 0
      stack_run_script start_mapping.sh stop
      ;;
    navigation)
      [ "$use_manager" = "1" ] && stack_manager_call /trash_manager/stop_navigation 60 && return 0
      stack_run_script start_navigation.sh stop
      ;;
    grasp)
      [ "$use_manager" = "1" ] && stack_manager_call /trash_manager/stop_grasp 60 && return 0
      stack_run_script start_grasp.sh stop
      ;;
    video)
      [ "$use_manager" = "1" ] && stack_manager_call /trash_manager/stop_video 60 && return 0
      stop_video >/dev/null 2>&1 || true
      ;;
    handeye)
      [ "$use_manager" = "1" ] && stack_manager_call /trash_manager/stop_handeye 60 && return 0
      stack_run_script start_handeye.sh stop
      ;;
    *)
      echo "ERROR: unknown component $component" >&2
      return 2
      ;;
  esac
}

stack_component_status() {
  local component="$1"
  echo "===== stack status: $component ====="
  echo "mode_lock=$(current_mode) runtime_mode=$(current_runtime_mode)"
  case "$component" in
    base) stack_run_script start_base.sh status ;;
    camera) stack_run_script start_camera.sh status ;;
    arm) stack_run_script start_arm.sh status ;;
    mapping) stack_run_script start_mapping.sh status ;;
    navigation) stack_run_script start_navigation.sh status ;;
    grasp) stack_run_script start_grasp.sh status ;;
    handeye) stack_run_script start_handeye.sh status ;;
    video)
      status_pid video || true
      ;;
    *)
      echo "unknown component"
      return 2
      ;;
  esac
}

stack_profile_start() {
  local profile="$1"
  case "$profile" in
    sensors)
      stack_component_start camera
      ;;
    mobility)
      stack_component_start base
      ;;
    perception_dry)
      stack_component_start camera
      stack_component_start grasp dry
      ;;
    patrol)
      stack_component_start base
      stack_component_start camera
      stack_component_start navigation
      ;;
    *)
      echo "ERROR: unknown profile $profile (sensors|mobility|perception_dry|patrol)" >&2
      return 2
      ;;
  esac
}

stack_profile_stop() {
  local profile="$1"
  case "$profile" in
    sensors) stack_component_stop camera ;;
    mobility) stack_component_stop base ;;
    perception_dry)
      stack_component_stop grasp
      stack_component_stop camera
      ;;
    patrol)
      stack_component_stop navigation
      stack_component_stop camera
      stack_component_stop base
      ;;
    *)
      echo "ERROR: unknown profile $profile" >&2
      return 2
      ;;
  esac
}

stack_status_all() {
  local c
  for c in base camera arm mapping navigation grasp video handeye; do
    stack_component_status "$c" || true
    echo ""
  done
  echo "===== perception topics ====="
  for t in /trash_grasp_plan /trash_target_camera_point /trash_target_point_camera /trash_target_point_arm; do
    echo "$t publishers=$(stack_topic_publishers "$t")"
  done
}

stack_dispatch() {
  local target="${1:-}"
  local action="${2:-}"
  shift 2 || true

  case "$target" in
    ""|-h|--help|help)
      stack_usage
      return 0
      ;;
    status-all)
      stack_status_all
      return 0
      ;;
    stop-all)
      if stack_manager_ready; then
        stack_manager_call /trash_manager/stop_all 180 || true
      fi
      stack_run_script stop_all.sh
      return 0
      ;;
    profile)
      local profile="${1:-}"
      local paction="${2:-start}"
      case "$paction" in
        start) stack_profile_start "$profile" ;;
        stop) stack_profile_stop "$profile" ;;
        *)
          echo "usage: trash_stack.sh profile <name> start|stop" >&2
          return 2
          ;;
      esac
      return 0
      ;;
    base|camera|arm|mapping|navigation|grasp|video|handeye)
      case "$action" in
        start) stack_component_start "$target" "$@" ;;
        stop) stack_component_stop "$target" ;;
        restart)
          stack_component_stop "$target"
          stop_pause
          stack_component_start "$target" "$@"
          ;;
        status) stack_component_status "$target" ;;
        *)
          echo "usage: trash_stack.sh $target start|stop|restart|status ..." >&2
          return 2
          ;;
      esac
      ;;
    *)
      echo "ERROR: unknown target '$target'" >&2
      stack_usage
      return 2
      ;;
  esac
}

# Thin entry for legacy start_*.sh wrappers
stack_run() {
  stack_dispatch "$@"
}
