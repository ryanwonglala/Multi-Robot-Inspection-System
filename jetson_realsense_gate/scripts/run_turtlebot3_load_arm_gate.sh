#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
if [[ -f "$HOME/ros2_ws/install/setup.bash" ]]; then
  source "$HOME/ros2_ws/install/setup.bash"
fi

export DISPLAY="${DISPLAY:-:1}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GATE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

python3 "$GATE_ROOT/turtlebot3_load_arm_gate.py" \
  --trigger-mode arrival \
  --arrival-type string \
  --arrival-topic /turtlebot3/load_unload_arrived \
  --arrival-ready-text "Ready, waiting for recognition results" \
  "$@"
