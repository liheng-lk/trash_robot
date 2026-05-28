#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OFFSET_FILE="$ROOT/config/grasp/grasp_offset.yaml"

usage() {
  cat <<'EOF'
Usage:
  ./scripts/set_grasp_offset.sh show
  ./scripts/set_grasp_offset.sh set <x_mm> <y_mm> <z_mm>
  ./scripts/set_grasp_offset.sh add <dx_mm> <dy_mm> <dz_mm>

Coordinate convention:
  +x moves the grasp point farther forward from the arm base.
  -x moves the grasp point closer to the arm base.
  +y moves the grasp point left.
  -y moves the grasp point right.
  +z raises the grasp point.
  -z lowers the grasp point.

This only changes config/grasp/grasp_offset.yaml. Restart the grasp pipeline after
changing the offset so roarm_sort_grasper reloads it.
EOF
}

if [ "${1:-}" = "" ] || [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
  usage
  exit 0
fi

mkdir -p "$(dirname "$OFFSET_FILE")"

python3 - "$OFFSET_FILE" "$@" <<'PY'
import datetime as _dt
import re
import sys
from pathlib import Path

path = Path(sys.argv[1])
cmd = sys.argv[2]

def load_offset():
    if not path.exists():
        return [0.0, 0.0, 0.0]
    text = path.read_text(encoding='utf-8')
    inline = re.search(r'offset_mm\s*:\s*\[([^\]]+)\]', text)
    if inline:
        values = [float(v) for v in re.split(r'[,\s]+', inline.group(1).strip()) if v]
        if len(values) == 3:
            return values
    lines = text.splitlines()
    for i, line in enumerate(lines):
        if re.match(r'^\s*offset_mm\s*:\s*$', line):
            values = []
            for item in lines[i + 1 : i + 4]:
                m = re.match(r'^\s*-\s*([-+]?\d+(?:\.\d+)?)\s*$', item)
                if not m:
                    break
                values.append(float(m.group(1)))
            if len(values) == 3:
                return values
    raise SystemExit(f"invalid offset_mm in {path}")

def write_offset(values):
    if path.exists():
        stamp = _dt.datetime.now().strftime('%Y%m%d_%H%M%S')
        backup = path.with_name(path.name + f'.bak_{stamp}')
        backup.write_text(path.read_text(encoding='utf-8'), encoding='utf-8')
        print(f'backup: {backup}')
    rounded = [round(float(v), 4) for v in values]
    path.write_text(
        'offset_mm:\n'
        f'  - {rounded[0]}\n'
        f'  - {rounded[1]}\n'
        f'  - {rounded[2]}\n',
        encoding='utf-8',
    )
    print(f'updated: {path}')
    print('offset_mm:', ' '.join(f'{v:.1f}' for v in rounded))

current = load_offset()
if cmd == 'show':
    print('offset_mm:', ' '.join(f'{v:.1f}' for v in current))
elif cmd in ('set', 'add'):
    if len(sys.argv) != 6:
        raise SystemExit(f'{cmd} requires 3 values: x y z')
    values = [float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5])]
    if cmd == 'add':
        values = [a + b for a, b in zip(current, values)]
    write_offset(values)
else:
    raise SystemExit(f'unknown command: {cmd}')
PY
