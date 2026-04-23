#!/bin/bash
#
# Set up Python virtual environment for cmdrsk_vfd testing.
# Safe to run multiple times — only installs if the venv is missing or
# requirements.txt has changed.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_DIR="$SCRIPT_DIR/.venv"
REQUIREMENTS="$SCRIPT_DIR/requirements.txt"
STAMP="$VENV_DIR/.installed_stamp"

if [ ! -d "$VENV_DIR" ]; then
    echo "Creating virtual environment at $VENV_DIR ..."
    python3 -m venv "$VENV_DIR"
fi

source "$VENV_DIR/bin/activate"

# Only pip install if requirements changed or venv is fresh
if [ ! -f "$STAMP" ] || [ "$REQUIREMENTS" -nt "$STAMP" ]; then
    echo "Installing Python dependencies..."
    pip install -q -r "$REQUIREMENTS"
    touch "$STAMP"
else
    echo "Python venv up to date."
fi

echo "Venv ready: $VENV_DIR"
