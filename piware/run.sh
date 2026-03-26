#!/usr/bin/env bash
# Optionally uploads an ESP32 sketch, then launches the PiWare game.
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PORT="${1:-/dev/ttyACM0}"
FQBN="esp32:esp32:esp32"
FIRMWARE_FILE="$SCRIPT_DIR/.last_firmware"

echo "╔══════════════════════════════════╗"
echo "║          P i W a r e            ║"
echo "║        MICRO  ARCADE            ║"
echo "╚══════════════════════════════════╝"
echo ""

# Show last-flashed firmware
if [ -f "$FIRMWARE_FILE" ]; then
  echo "  Current firmware: $(cat "$FIRMWARE_FILE")"
else
  echo "  Current firmware: unknown"
fi
echo ""
echo "Upload new firmware to ESP32?"
echo "  1) Serial mode      (testscreen_serial)"
echo "  2) MQTT WiFi mode   (testscreen_mqtt_wifi)"
echo "  3) MQTT LTE mode    (testscreen_mqtt)"
echo "  s) Skip — launch game directly (default)"
echo ""
read -rp "Choice [s]: " choice < /dev/tty

case "$choice" in
  1) SKETCH_DIR="$PROJECT_DIR/testscreen_serial";    SKETCH_NAME="testscreen_serial" ;;
  2) SKETCH_DIR="$PROJECT_DIR/testscreen_mqtt_wifi";  SKETCH_NAME="testscreen_mqtt_wifi" ;;
  3) SKETCH_DIR="$PROJECT_DIR/testscreen_mqtt";       SKETCH_NAME="testscreen_mqtt" ;;
  *) SKETCH_DIR="" ;;
esac

if [ -n "$SKETCH_DIR" ]; then
  echo ""
  echo "=== Compiling sketch ==="
  arduino-cli compile --fqbn "$FQBN" "$SKETCH_DIR"

  echo "=== Uploading to $PORT ==="
  arduino-cli upload --fqbn "$FQBN" -p "$PORT" "$SKETCH_DIR"
  echo "$SKETCH_NAME" > "$FIRMWARE_FILE"
  sleep 2
else
  echo "=== Skipping upload ==="
fi

echo ""
echo "=== Starting PiWare ==="
sudo python3 "$SCRIPT_DIR/game.py" --port "$PORT"
