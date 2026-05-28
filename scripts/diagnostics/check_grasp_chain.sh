#!/usr/bin/env bash
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
if [ -f "$ROOT/scripts/source_v3.sh" ]; then
  # shellcheck source=/dev/null
  source "$ROOT/scripts/source_v3.sh" >/dev/null 2>&1 || true
fi

TIMEOUT_SHORT="${TRASH_DIAG_TIMEOUT_SHORT:-6s}"
TIMEOUT_ECHO="${TRASH_DIAG_TIMEOUT_ECHO:-5s}"
MAIN_CAMERA_POINT="/trash_target_camera_point"
LEGACY_CAMERA_POINT="/trash_target_point_camera"
ARM_POINT="/trash_target_point_arm"

run_timeout() {
  timeout -k 1s "$@"
}

topic_list="$(run_timeout "$TIMEOUT_SHORT" ros2 topic list 2>/dev/null || true)"

topic_present() {
  printf '%s\n' "$topic_list" | grep -qx "$1"
}

node_info() {
  run_timeout "$TIMEOUT_SHORT" ros2 node info "$1" 2>/dev/null || true
}

node_param() {
  run_timeout "$TIMEOUT_SHORT" ros2 param get "$1" "$2" 2>/dev/null || true
}

has_subscription() {
  local info="$1"
  local topic="$2"
  printf '%s\n' "$info" | grep -Eq "(^|[[:space:]])${topic}([[:space:]]|:)"
}

bool_is_true() {
  printf '%s\n' "$1" | grep -Eiq '(^|[^A-Za-z])(true|True)([^A-Za-z]|$)'
}

bool_is_false() {
  printf '%s\n' "$1" | grep -Eiq '(^|[^A-Za-z])(false|False)([^A-Za-z]|$)'
}

extract_plan_field() {
  local field="$1"
  local input
  input="$(cat)"
  PLAN_TEXT="$input" python3 - "$field" <<'PY'
import ast
import json
import os
import sys

field = sys.argv[1]
text = os.environ.get('PLAN_TEXT', '')
payload = ''
for line in text.splitlines():
    stripped = line.strip()
    if stripped.startswith('data:'):
        payload = stripped.split(':', 1)[1].strip()
        break
    if stripped.startswith('{') and stripped.endswith('}'):
        payload = stripped
        break
if not payload:
    print('unknown')
    raise SystemExit(0)
try:
    payload = ast.literal_eval(payload)
except Exception:
    pass
try:
    data = json.loads(payload)
except Exception:
    print('unknown')
    raise SystemExit(0)
value = data.get(field, 'unknown')
if isinstance(value, bool):
    print(str(value).lower())
else:
    print(value if value not in (None, '') else 'unknown')
PY
}

plan_present=false
main_present=false
legacy_present=false
arm_present=false
handeye_status_present=false
camera_to_arm_status_present=false
grasp_status_present=false

topic_present /trash_grasp_plan && plan_present=true
topic_present "$MAIN_CAMERA_POINT" && main_present=true
topic_present "$LEGACY_CAMERA_POINT" && legacy_present=true
topic_present "$ARM_POINT" && arm_present=true
topic_present /trash_handeye_status && handeye_status_present=true
topic_present /trash_camera_to_arm_status && camera_to_arm_status_present=true
topic_present /trash_grasp_status && grasp_status_present=true

handeye_info="$(node_info /handeye_target_transformer)"
handeye_node_present=false
handeye_sub_main=false
handeye_sub_legacy=false
if printf '%s\n' "$handeye_info" | grep -q 'Subscribers:'; then
  handeye_node_present=true
  has_subscription "$handeye_info" "$MAIN_CAMERA_POINT" && handeye_sub_main=true
  has_subscription "$handeye_info" "$LEGACY_CAMERA_POINT" && handeye_sub_legacy=true
fi

chain_conflict=false
if [ "$handeye_sub_legacy" = true ]; then
  chain_conflict=true
fi

plan_echo="$(run_timeout "$TIMEOUT_ECHO" ros2 topic echo /trash_grasp_plan --field data --once 2>/dev/null || true)"
camera_echo="$(run_timeout "$TIMEOUT_ECHO" ros2 topic echo "$MAIN_CAMERA_POINT" --once 2>/dev/null || true)"
arm_echo="$(run_timeout "$TIMEOUT_ECHO" ros2 topic echo "$ARM_POINT" --once 2>/dev/null || true)"

depth_ok="$(printf '%s\n' "$plan_echo" | extract_plan_field depth_ok)"
depth_reason="$(printf '%s\n' "$plan_echo" | extract_plan_field depth_reason)"

dry_run_param="$(node_param /roarm_sort_grasper dry_run)"
auto_execute_param="$(node_param /roarm_sort_grasper auto_execute)"
dry_run_safe=false
auto_execute_safe=false
bool_is_true "$dry_run_param" && dry_run_safe=true
bool_is_false "$auto_execute_param" && auto_execute_safe=true

dryrun_ready=false
if [ "$chain_conflict" = false ] \
  && [ "$handeye_sub_main" = true ] \
  && [ "$plan_present" = true ] \
  && [ "$main_present" = true ] \
  && [ "$arm_present" = true ] \
  && [ "$depth_ok" = "true" ] \
  && [ "$dry_run_safe" = true ] \
  && [ "$auto_execute_safe" = true ]; then
  dryrun_ready=true
fi

status=PASS
if [ "$handeye_node_present" = false ] || [ "$chain_conflict" = true ] || [ "$dry_run_safe" = false ] || [ "$auto_execute_safe" = false ]; then
  status=FAIL
elif [ "$dryrun_ready" = false ]; then
  status=WARN
fi

cat <<EOF
GRASP_CHAIN_STATUS=$status
MAIN_CAMERA_POINT=$MAIN_CAMERA_POINT
LEGACY_CAMERA_POINT=$LEGACY_CAMERA_POINT
ARM_POINT=$ARM_POINT
LEGACY_PRESENT=$legacy_present
HANDEYE_NODE_PRESENT=$handeye_node_present
HANDEYE_SUBSCRIBES_MAIN=$handeye_sub_main
HANDEYE_SUBSCRIBES_LEGACY=$handeye_sub_legacy
CHAIN_CONFLICT=$chain_conflict
DEPTH_OK=$depth_ok
DEPTH_REASON=$depth_reason
DRY_RUN_PARAM=${dry_run_param:-unknown}
AUTO_EXECUTE_PARAM=${auto_execute_param:-unknown}
DRYRUN_READY=$dryrun_ready
TOPIC_GRASP_PLAN=$plan_present
TOPIC_CAMERA_POINT=$main_present
TOPIC_LEGACY_CAMERA_POINT=$legacy_present
TOPIC_ARM_POINT=$arm_present
TOPIC_HANDEYE_STATUS=$handeye_status_present
TOPIC_CAMERA_TO_ARM_STATUS=$camera_to_arm_status_present
TOPIC_GRASP_STATUS=$grasp_status_present
EOF

echo
echo "---- /trash_grasp_plan --once ----"
printf '%s\n' "${plan_echo:-NO_MESSAGE}"
echo
echo "---- $MAIN_CAMERA_POINT --once ----"
printf '%s\n' "${camera_echo:-NO_MESSAGE}"
echo
echo "---- $ARM_POINT --once ----"
printf '%s\n' "${arm_echo:-NO_MESSAGE}"
