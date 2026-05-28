#!/usr/bin/env bash
# Collect read-only ROS + system snapshot. No hardware control.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TS="$(date +%Y%m%d_%H%M%S)"
OUT="$ROOT/runtime/snapshots/$TS"
mkdir -p "$OUT"

TIMEOUT_ROS="${TRASH_SNAPSHOT_ROS_TIMEOUT:-8}"
TIMEOUT_HZ="${TRASH_SNAPSHOT_HZ_TIMEOUT:-3}"
TIMEOUT_ECHO="${TRASH_SNAPSHOT_ECHO_TIMEOUT:-3}"
SNAPSHOT_QUICK="${TRASH_SNAPSHOT_QUICK:-0}"

log() { echo "[snapshot] $*"; }

run_timeout() {
  local sec="$1"
  local name="$2"
  shift 2
  mkdir -p "$OUT/commands" "$OUT/errors"
  timeout "$sec" "$@" >"$OUT/commands/${name}.out" 2>"$OUT/errors/${name}.err" || true
}

mkdir -p "$OUT/errors" "$OUT/commands" "$OUT/topics" "$OUT/logs_copy"

log "output=$OUT"

# Optional ROS env (do not fail if missing)
if [ -f "$ROOT/scripts/common.sh" ]; then
  # shellcheck disable=SC1091
  source "$ROOT/scripts/common.sh"
  if declare -f source_runtime >/dev/null 2>&1; then
    set +u
    source_runtime
    set -u
  fi
fi

# Host info
{
  echo "timestamp=$TS"
  echo "hostname=$(hostname 2>/dev/null || echo unknown)"
  echo "user=$(whoami 2>/dev/null || echo unknown)"
  echo "root=$ROOT"
  uname -a 2>/dev/null || true
} >"$OUT/host.txt"

run_timeout 5 free_h free -h
run_timeout 5 df_root df -h "$ROOT"
if command -v top >/dev/null 2>&1; then
  run_timeout 3 top_head top -bn1
  head -n 40 "$OUT/commands/top_head.out" >"$OUT/commands/top_head_trim.out" 2>/dev/null || true
fi

# Git (repo may not be git)
if command -v git >/dev/null 2>&1 && git -C "$ROOT" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  run_timeout 5 git_status git -C "$ROOT" status
  run_timeout 5 git_diff_stat git -C "$ROOT" diff --stat
  run_timeout 5 git_log git -C "$ROOT" log -3 --oneline
else
  echo "not a git repository" >"$OUT/commands/git_status.out"
fi

# ROS discovery
if command -v ros2 >/dev/null 2>&1; then
  run_timeout "$TIMEOUT_ROS" ros2_node_list ros2 node list
  run_timeout "$TIMEOUT_ROS" ros2_topic_list ros2 topic list
  run_timeout "$TIMEOUT_ROS" ros2_service_list ros2 service list
  run_timeout "$TIMEOUT_ROS" ros2_action_list ros2 action list
else
  echo "ros2 CLI not available" >"$OUT/commands/ros2_missing.out"
fi

# TF tree
if command -v ros2 >/dev/null 2>&1; then
  if ros2 pkg list 2>/dev/null | grep -q tf2_tools; then
    run_timeout 15 tf_view_frames ros2 run tf2_tools view_frames
    if [ -f frames.pdf ]; then
      mv -f frames.pdf "$OUT/" 2>/dev/null || true
    fi
    if [ -f frames.yaml ]; then
      mv -f frames.yaml "$OUT/tf_frames.yaml" 2>/dev/null || true
    fi
  fi
fi

# Key topics: hz + once
TOPICS=(
  /cmd_vel
  /odom
  /scan
  /tf
  /tf_static
  /camera/camera/color/image_raw
  /camera/camera/aligned_depth_to_color/image_raw
  /camera/camera/color/camera_info
  /trash_grasp_plan
  /trash_target_camera_point
  /trash_target_point_camera
  /trash_target_point_arm
  /trash_target_arm_point
  /trash_target_pixel
  /trash_vlm_result
  /trash_perception_status
  /arm/status
  /trash_robot/state
  /trash_system_status
)

if command -v ros2 >/dev/null 2>&1; then
  for topic in "${TOPICS[@]}"; do
    safe="$(echo "$topic" | tr '/' '_')"
    if [ "$SNAPSHOT_QUICK" != "1" ]; then
      run_timeout "$TIMEOUT_HZ" "hz${safe}" ros2 topic hz "$topic"
      cp -f "$OUT/commands/hz${safe}.out" "$OUT/topics/hz${safe}.out" 2>/dev/null || true
    fi
    run_timeout "$TIMEOUT_ECHO" "echo${safe}" ros2 topic echo "$topic" --once
    cp -f "$OUT/commands/echo${safe}.out" "$OUT/topics/echo${safe}.out" 2>/dev/null || true
  done
fi

# Recent logs
LOG_SRC="$ROOT/runtime/logs"
if [ -d "$LOG_SRC" ]; then
  find "$LOG_SRC" -type f \( -name '*.log' -o -name '*.jsonl' \) -mmin -120 2>/dev/null | head -n 30 | while read -r f; do
    rel="${f#"$LOG_SRC"/}"
    mkdir -p "$OUT/logs_copy/$(dirname "$rel")"
    tail -n 200 "$f" >"$OUT/logs_copy/$rel.tail" 2>/dev/null || true
  done
fi

# Agent state copy
for f in PROJECT_STATE.md NEXT_ACTION.md SAFETY_RULES.md; do
  [ -f "$ROOT/.agent/$f" ] && cp "$ROOT/.agent/$f" "$OUT/agent_$f" || true
done

# Report stub
cat >"$OUT/snapshot_report.md" <<EOF
# Snapshot Report

- **Timestamp:** $TS
- **Directory:** \`runtime/snapshots/$TS\`
- **ROS available:** $(command -v ros2 >/dev/null 2>&1 && echo yes || echo no)

## Commands run

See \`commands/\` and \`topics/\`. Errors in \`errors/\`.

## Review

\`\`\`bash
python3 $ROOT/scripts/diagnostics/review_last_snapshot.py
\`\`\`

EOF

# Symlink latest
ln -sfn "$OUT" "$ROOT/runtime/snapshots/latest"

log "done -> $OUT"
echo "$OUT"
