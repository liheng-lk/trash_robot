#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_FILE="$ROOT/config/grasp/trash_sort_params.yaml"

usage() {
  cat <<'EOF'
Usage:
  ./scripts/set_drop_point.sh show
  ./scripts/set_drop_point.sh set <label> <x_mm> <y_mm> <z_mm>
  ./scripts/set_drop_point.sh add <label> <dx_mm> <dy_mm> <dz_mm>
  ./scripts/set_drop_point.sh read <label>

Labels:
  recycle | other | hazard | kitchen
  GARBAGE_RECYCLE | GARBAGE_OTHER | GARBAGE_HAZARD | GARBAGE_KITCHEN

Coordinate convention:
  Uses RoArm base coordinates in millimeters.
  read <label> reads /get_pose_cmd and saves the current arm pose as that bin's drop point.

After changing drop points, restart the grasp pipeline:
  ./scripts/start_grasp.sh restart live
EOF
}

if [ "${1:-}" = "" ] || [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
  usage
  exit 0
fi

python3 - "$ROOT" "$CONFIG_FILE" "$@" <<'PY'
import datetime as _dt
import re
import subprocess
import sys
from pathlib import Path

import yaml

root = Path(sys.argv[1])
path = Path(sys.argv[2])
cmd = sys.argv[3]

ALIASES = {
    'recycle': 'GARBAGE_RECYCLE',
    '可回收': 'GARBAGE_RECYCLE',
    'other': 'GARBAGE_OTHER',
    '其他': 'GARBAGE_OTHER',
    'hazard': 'GARBAGE_HAZARD',
    '有害': 'GARBAGE_HAZARD',
    'kitchen': 'GARBAGE_KITCHEN',
    '厨余': 'GARBAGE_KITCHEN',
    'GARBAGE_RECYCLE': 'GARBAGE_RECYCLE',
    'GARBAGE_OTHER': 'GARBAGE_OTHER',
    'GARBAGE_HAZARD': 'GARBAGE_HAZARD',
    'GARBAGE_KITCHEN': 'GARBAGE_KITCHEN',
}


def label_of(text: str) -> str:
    if text not in ALIASES:
        raise SystemExit(f'unknown label: {text}')
    return ALIASES[text]


def load_config() -> dict:
    if not path.exists():
        raise SystemExit(f'missing config: {path}')
    with path.open('r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise SystemExit(f'invalid yaml: {path}')
    points = data.setdefault('drop_points_mm', {})
    if not isinstance(points, dict):
        raise SystemExit('drop_points_mm must be a mapping')
    return data


def save_config(data: dict) -> None:
    stamp = _dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    backup = path.with_name(path.name + f'.bak_{stamp}')
    backup.write_text(path.read_text(encoding='utf-8'), encoding='utf-8')
    with path.open('w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    print(f'backup: {backup}')
    print(f'updated: {path}')


def values_from_args(argv: list[str]) -> list[float]:
    if len(argv) != 3:
        raise SystemExit('expected 3 values: x_mm y_mm z_mm')
    return [round(float(v), 4) for v in argv]


def current_pose_mm() -> list[float]:
    command = (
        'set +u; '
        f'source "{root}/scripts/source_v3.sh" >/dev/null 2>&1; '
        'set -u; '
        'timeout 8s ros2 service call /get_pose_cmd roarm_moveit/srv/GetPoseCmd "{}"'
    )
    result = subprocess.run(
        ['bash', '-lc', command],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    if result.returncode != 0:
        raise SystemExit(f'failed to read /get_pose_cmd:\n{result.stdout}')
    matches = dict((k, float(v)) for k, v in re.findall(r'^\s*(x|y|z):\s*([-+]?\d+(?:\.\d+)?)\s*$', result.stdout, re.M))
    if not {'x', 'y', 'z'} <= set(matches):
        raise SystemExit(f'could not parse /get_pose_cmd response:\n{result.stdout}')
    return [round(matches['x'] * 1000.0, 4), round(matches['y'] * 1000.0, 4), round(matches['z'] * 1000.0, 4)]


def print_points(data: dict) -> None:
    for label in ('GARBAGE_KITCHEN', 'GARBAGE_OTHER', 'GARBAGE_RECYCLE', 'GARBAGE_HAZARD'):
        value = data.get('drop_points_mm', {}).get(label)
        print(f'{label}: {value}')


data = load_config()
points = data['drop_points_mm']

if cmd == 'show':
    print_points(data)
elif cmd in ('set', 'add'):
    if len(sys.argv) != 8:
        raise SystemExit(f'{cmd} usage: {cmd} <label> <x_mm> <y_mm> <z_mm>')
    label = label_of(sys.argv[4])
    values = values_from_args(sys.argv[5:8])
    if cmd == 'add':
        old = points.get(label)
        if not isinstance(old, list) or len(old) != 3:
            raise SystemExit(f'missing current point for {label}')
        values = [round(float(a) + float(b), 4) for a, b in zip(old, values)]
    points[label] = values
    save_config(data)
    print(f'{label}: {values}')
elif cmd == 'read':
    if len(sys.argv) != 5:
        raise SystemExit('read usage: read <label>')
    label = label_of(sys.argv[4])
    values = current_pose_mm()
    points[label] = values
    save_config(data)
    print(f'{label}: {values}')
else:
    raise SystemExit(f'unknown command: {cmd}')
PY
