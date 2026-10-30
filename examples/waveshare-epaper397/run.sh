#!/bin/sh
# Run one of the e-paper demos build.sh made on --board waveshare-epaper397.
#   ./run.sh gxepd2|waveshare [OUT] [extra emulator flags...]
#   ./run.sh gxepd2 --tft-png epd.png --max-seconds 12
#   ./run.sh waveshare --web 8768
# The mask ROM comes from --rom or $ESP32SIM_ROM_DIR (tools/fetch-rom-elfs.sh DIR fetches it).
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
DEMO=${1:?usage: $0 gxepd2|waveshare [OUT] [flags...]}; shift
OUT=$HERE/out
if [ $# -gt 0 ] && [ -d "$1" ]; then OUT=$1; shift; fi
FW=$OUT/$DEMO
exec "$HERE/../../target/release/esp32sim" --board waveshare-epaper397 --boot rom \
  --bootloader "$FW/bootloader.bin" --ptable "$FW/partitions.bin" --app "$FW/app.bin" --console both --no-dump "$@"
