#!/usr/bin/env bash
# Run the MQTT WiFi test screen
# Optionally compiles & uploads the sketch first.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PORT="/dev/ttyACM0"
FQBN="esp32:esp32:esp32"

read -rp "Upload sketch to Arduino? [y/N] " upload < /dev/tty
if [[ "$upload" =~ ^[Yy]$ ]]; then
  echo "Compiling..."
  arduino-cli compile --fqbn "$FQBN" "$SCRIPT_DIR"
  echo "Uploading to $PORT..."
  arduino-cli upload --fqbn "$FQBN" -p "$PORT" "$SCRIPT_DIR"
  echo "Done. Waiting for board to boot..."
  sleep 3
fi

echo "Starting test screen (MQTT WiFi)..."
cd "$SCRIPT_DIR"
sudo python3 testscreen.py
