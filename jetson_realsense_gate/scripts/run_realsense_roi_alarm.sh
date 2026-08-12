#!/usr/bin/env bash
set -euo pipefail

export DISPLAY="${DISPLAY:-:1}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GATE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

python3 "$GATE_ROOT/realsense_roi_alarm.py" \
  "$@"
