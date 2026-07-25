#!/bin/bash
# OTA upload for VehicleTracker_ESP32C3
# Usage: ./ota_upload.sh [ip-address]
# Default IP: 192.168.1.175

IP="${1:-192.168.1.175}"
SKETCH_DIR="$(dirname "$0")"
BUILD_DIR="/tmp/vehicletracker_build"
ESPOTA="$HOME/.arduino15/packages/esp32/hardware/esp32/3.3.8/tools/espota.py"
FQBN="esp32:esp32:esp32c3"

echo "=== VehicleTracker OTA Upload ==="
echo "Compiling..."
arduino-cli compile --fqbn "$FQBN" \
  --build-property "upload.maximum_size=1310720" \
  "$SKETCH_DIR" --output-dir "$BUILD_DIR" 2>&1 | tail -4

if [ $? -ne 0 ]; then
    echo "ERROR: compilation failed"
    exit 1
fi

echo "Uploading to $IP (make sure the device is awake)..."
python3 "$ESPOTA" -i "$IP" -p 3232 \
  -f "$BUILD_DIR/VehicleTracker_ESP32C3.ino.bin"

echo "Done."
