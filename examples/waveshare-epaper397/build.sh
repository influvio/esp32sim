#!/bin/sh
# Build the two e-paper demos for --board waveshare-epaper397 into OUT (default ./out):
#   OUT/gxepd2/      this directory's gxepd2-demo sketch (GxEPD2 over SPI2)
#   OUT/waveshare/   Waveshare's 02_E-Paper_Example (bit-banged SPI), fetched at a pinned commit
#   OUT/mic/         this directory's mic-demo sketch (I2S RX from the ES8311's ADC line)
# each with bootloader.bin, partitions.bin and app.bin.
# Needs arduino-cli (or ARDUINO_CLI=path) with the esp32 core 3.3.x and the GxEPD2 (>= 1.6.5) and
# Adafruit GFX libraries; the Arduino IDE 2 ships arduino-cli inside its app bundle.
set -e
here=$(cd "$(dirname "$0")" && pwd)
OUT=${1:-$here/out}
CLI=${ARDUINO_CLI:-arduino-cli}
WAVESHARE_REPO=https://github.com/waveshareteam/ESP32-S3-ePaper-3.97
WAVESHARE_COMMIT=9b12d40731a80213b927ee8a421cae4082952819
# The board's module: ESP32-S3-WROOM-1-N16R8, 16 MB quad flash, 8 MB octal PSRAM. CDC on boot so
# Serial is the USB Serial/JTAG console, as Waveshare's Arduino settings for this board have it.
FQBN=esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB,CDCOnBoot=cdc

build() {  # SKETCH_DIR NAME
  build_dir=$OUT/$2/build
  "$CLI" compile --fqbn "$FQBN" --build-path "$build_dir" "$1"
  sketch=$(basename "$1")
  cp "$build_dir/$sketch.ino.bootloader.bin" "$OUT/$2/bootloader.bin"
  cp "$build_dir/$sketch.ino.partitions.bin" "$OUT/$2/partitions.bin"
  cp "$build_dir/$sketch.ino.bin" "$OUT/$2/app.bin"
}

mkdir -p "$OUT/gxepd2" "$OUT/waveshare" "$OUT/mic"
build "$here/gxepd2-demo" gxepd2
build "$here/mic-demo" mic

src=$OUT/waveshare/src
if [ ! -d "$src/.git" ]; then
  git clone --quiet --filter=blob:none --no-checkout "$WAVESHARE_REPO" "$src"
  git -C "$src" sparse-checkout set Arduino/examples/02_E-Paper_Example
fi
git -C "$src" checkout --quiet "$WAVESHARE_COMMIT"
build "$src/Arduino/examples/02_E-Paper_Example" waveshare
echo "built into $OUT"
