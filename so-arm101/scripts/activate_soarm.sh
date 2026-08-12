#!/usr/bin/env bash

SOARM_ROOT="$HOME/Multi-Robot-Inspection-System/so-arm101"

if [[ ! -x "$SOARM_ROOT/.venv/bin/python" ]]; then
    echo "SO-ARM environment not found: $SOARM_ROOT/.venv" >&2
    return 1 2>/dev/null || exit 1
fi

export SOARM_ROOT
export PATH="$HOME/.local/bin:$PATH"
export PYTHONPATH="$SOARM_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
export SOARM_PORT="${SOARM_PORT:-/dev/ttyACM0}"

source "$SOARM_ROOT/.venv/bin/activate"
cd "$SOARM_ROOT" || return 1 2>/dev/null || exit 1

echo "SO-ARM101 environment active"
echo "  project: $SOARM_ROOT"
echo "  python:  $(python --version 2>&1)"
echo "  port:    $SOARM_PORT"
