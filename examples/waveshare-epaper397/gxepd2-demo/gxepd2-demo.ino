// GxEPD2 on the Waveshare ESP32-S3-ePaper-3.97: one standard full refresh, then partial refreshes
// of a counter window, then hibernate. Pins and driver class as in Waveshare's E-Paper tutorial
// (section 5.2). Every refresh prints how long GxEPD2 waited for BUSY.
#include <SPI.h>
#include <GxEPD2_BW.h>

constexpr int EPD_BUSY = 3;
constexpr int EPD_DC = 9;
constexpr int EPD_CS = 10;
constexpr int EPD_SCLK = 11;
constexpr int EPD_MOSI = 12;
constexpr int EPD_RST = 46;

using EpdDriver = GxEPD2_397_GDEM0397T81;
GxEPD2_BW<EpdDriver, EpdDriver::HEIGHT> display(EpdDriver(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY));

constexpr int WIN_X = 280, WIN_Y = 180, WIN_W = 240, WIN_H = 120;

void drawFrame() {
  display.fillScreen(GxEPD_WHITE);
  display.fillRect(8, 8, 784, 56, GxEPD_BLACK);
  display.setTextColor(GxEPD_WHITE);
  display.setTextSize(3);
  display.setCursor(40, 26);
  display.print("esp32sim SSD1677 800x480");
  display.drawRect(WIN_X - 8, WIN_Y - 8, WIN_W + 16, WIN_H + 16, GxEPD_BLACK);
  display.fillCircle(120, 360, 56, GxEPD_BLACK);
  display.drawLine(0, 479, 799, 0, GxEPD_BLACK);
  display.drawRect(0, 0, 800, 480, GxEPD_BLACK);
}

void drawCounter(uint32_t n) {
  display.setTextColor(GxEPD_BLACK);
  display.setTextSize(7);
  display.setCursor(WIN_X + 24, WIN_Y + 32);
  display.print(n);
}

void setup() {
  Serial.begin(115200);
  SPI.begin(EPD_SCLK, -1, EPD_MOSI, EPD_CS);
  display.init(0);
  display.epd2.selectFastFullUpdate(false);
  display.setRotation(0);
  display.setTextWrap(false);

  uint32_t start = millis();
  display.setFullWindow();
  display.firstPage();
  do { drawFrame(); drawCounter(0); } while (display.nextPage());
  Serial.printf("epd: full refresh %lu ms\n", millis() - start);

  for (uint32_t n = 1; n <= 3; n++) {
    start = millis();
    display.setPartialWindow(WIN_X, WIN_Y, WIN_W, WIN_H);
    display.firstPage();
    do { display.fillScreen(GxEPD_WHITE); drawCounter(n); } while (display.nextPage());
    Serial.printf("epd: partial refresh %lu: %lu ms\n", n, millis() - start);
  }
  display.hibernate();
  Serial.println("epd: done");
}

void loop() { delay(1000); }
