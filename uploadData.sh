#!/bin/bash
# Builds data/ into a LittleFS image and flashes it to the firmware's "www" partition only - the
# same image ProjectWallLabel ships with each release. Never touches the app, NVS Preferences or the
# LittleFS sensor-data partition. Offset and size come from the partition table of the current
# build, not from anything hardcoded here.
#   ./uploadData.sh [port]        (default /dev/ttyUSB0)
set -euo pipefail
cd "$(dirname "$0")"

PORT="${1:-/dev/ttyUSB0}"
PARTITIONS=build/esp32.esp32.esp32da/Daffodil.ino.partitions.bin
MKLITTLEFS="$(ls -d "$HOME"/.arduino15/packages/esp32/tools/mklittlefs/*/mklittlefs | sort | tail -1)"
ESPTOOL="$HOME/.arduino15/packages/esp32/tools/esptool_py/3.0.0/esptool.py"

read -r OFFSET SIZE < <(python3 - "$PARTITIONS" <<'EOF'
import struct, sys
d = open(sys.argv[1], 'rb').read()
for i in range(0, len(d) - 31, 32):
    if d[i:i+2] != b'\xaa\x50':
        break
    off, size = struct.unpack('<II', d[i+4:i+12])
    if d[i+12:i+28].rstrip(b'\0') == b'www':
        print(hex(off), size)
        sys.exit(0)
sys.exit("no 'www' partition in " + sys.argv[1])
EOF
)

IMAGE=/tmp/Daffodil.www.bin
"$MKLITTLEFS" -c data -p 256 -b 4096 -s "$SIZE" "$IMAGE"
python3 "$ESPTOOL" --chip esp32 --port "$PORT" --baud 921600 --before default_reset --after hard_reset \
  write_flash "$OFFSET" "$IMAGE"
