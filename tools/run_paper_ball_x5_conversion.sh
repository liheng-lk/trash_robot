#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${TRASH_RDK_X5_TOOLCHAIN_IMAGE:-openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.8}"

docker run --platform linux/amd64 --rm \
  -v "$ROOT_DIR":/workspace \
  -w /workspace \
  "$IMAGE" \
  python3 tools/convert_paper_ball_yolo_x5.py "$@"
