# Waveshare ESP32-S3-ePaper-3.97 · SSD1677 e-paper in the emulator

    ./build.sh                                  # both demos into ./out (needs arduino-cli, see below)
    ./run.sh gxepd2 --tft-png epd.png --max-seconds 12
    ./run.sh waveshare --tft-png epd.png --max-seconds 25
    ./run.sh gxepd2 --web 8768                  # http://127.0.0.1:8768/

`build.sh` compiles two Arduino-ESP32 sketches for the board's module (ESP32-S3-WROOM-1-N16R8:
16 MB flash, 8 MB octal PSRAM; `FlashSize=16M,PSRAM=opi,CDCOnBoot=cdc`):

- `gxepd2-demo/`, from this directory: GxEPD2's `GxEPD2_397_GDEM0397T81` over SPI2, as in
  Waveshare's E-Paper tutorial. One standard full refresh, three partial refreshes of a counter
  window, then hibernate. Each refresh prints how long GxEPD2 waited for BUSY.
- Waveshare's `Arduino/examples/02_E-Paper_Example`, fetched from
  [waveshareteam/ESP32-S3-ePaper-3.97](https://github.com/waveshareteam/ESP32-S3-ePaper-3.97) at
  commit `9b12d40`. It bit-bangs SPI on the same GPIOs. It clears, shows a picture with the fast
  init, draws shapes and text, runs four partial clock updates, clears and sleeps.

Built with Arduino-ESP32 3.3.12, GxEPD2 1.6.9 and Adafruit GFX 1.13.0. Arduino IDE 2 bundles
arduino-cli in its app (on macOS
`/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli`): pass it
as `ARDUINO_CLI=...`.

What to expect:

- The glass changes only when a refresh ends, which is when BUSY (GPIO 3) falls. GxEPD2 reports
  about 8.7 s for its first full refresh (the initial clear and the frame, 4,278 ms each) and about
  418 ms per partial refresh. These are the BUSY times GxEPD2 measured on this glass.
- The Waveshare drawing shows its top row as a one-pixel line at the bottom edge. The driver sets
  the Y counter to 0 while writing with Y decrementing in a 479..0 window, so after the first row
  the counter wraps to 479. That is what the datasheet's window rules give.
- Waveshare's partial clock (`EPD_3IN97_Display_Partial`) comes out mirrored at y ≈ 156..179, not at
  the requested y = 300. Its digits also smear across updates. The function hardware-resets the
  controller, which in the model restores the register defaults (Y increment), and it never
  rewrites the RED RAM the partial waveform compares against. Neither has been checked on hardware;
  see `docs/boards.md`.

`cli/tests/epaper397.rs` runs both builds (`EPAPER397_FW_DIR=out`; ignored without them).
