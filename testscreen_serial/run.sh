#!/usr/bin/env bash
# Compiles and uploads the Arduino sketch, then runs the touchscreen UI.
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PORT="${1:-/dev/ttyACM0}"
FQBN="esp32:esp32:esp32"

read -rp "Upload sketch to Arduino? [y/N] " answer < /dev/tty
if [[ "$answer" =~ ^[Yy]$ ]]; then
  echo "=== Compiling sketch ==="
  arduino-cli compile --fqbn "$FQBN" "$SCRIPT_DIR"

  echo "=== Uploading to $PORT ==="
  arduino-cli upload --fqbn "$FQBN" -p "$PORT" "$SCRIPT_DIR"
  sleep 2
else
  echo "=== Skipping upload ==="
fi

echo "=== Starting test screen ==="
sudo python3 "$SCRIPT_DIR/testscreen.py" --port "$PORT"
