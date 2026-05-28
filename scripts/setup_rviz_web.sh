#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/common.sh"

THIRD_PARTY_DIR="$TRASH_ROBOT_RUNTIME/third_party"
NOVNC_DIR="${TRASH_NOVNC_DIR:-$THIRD_PARTY_DIR/noVNC}"
NOVNC_ZIP="$TRASH_ROBOT_RUNTIME/noVNC-master.zip"

mkdir -p "$THIRD_PARTY_DIR"

USER_SITE="$(python3 - <<'PY'
import site
print(site.getusersitepackages())
PY
)"
case ":${PYTHONPATH:-}:" in
  *":$USER_SITE:"*) ;;
  *) export PYTHONPATH="$USER_SITE${PYTHONPATH:+:$PYTHONPATH}" ;;
esac
export PATH="$HOME/.local/bin:$PATH"

if ! python3 - <<'PY' >/dev/null 2>&1
import websockify
PY
then
  echo "Installing websockify into user site..."
  python3 -m pip install --user websockify
fi

if [ ! -f "$NOVNC_DIR/vnc.html" ]; then
  echo "Installing noVNC into $NOVNC_DIR ..."
  rm -rf "$NOVNC_DIR" "$TRASH_ROBOT_RUNTIME/noVNC-master"
  curl -L https://github.com/novnc/noVNC/archive/refs/heads/master.zip -o "$NOVNC_ZIP"
  unzip -q "$NOVNC_ZIP" -d "$TRASH_ROBOT_RUNTIME"
  mv "$TRASH_ROBOT_RUNTIME/noVNC-master" "$NOVNC_DIR"
fi

echo "RViz Web dependencies ready"
echo "noVNC: $NOVNC_DIR"
