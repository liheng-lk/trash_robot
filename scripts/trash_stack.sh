#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT/scripts/common.sh"
# shellcheck disable=SC1091
source "$ROOT/scripts/lib/stack.sh"
source_debug

stack_dispatch "$@"
