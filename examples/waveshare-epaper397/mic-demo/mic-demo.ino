// Microphone check for --board waveshare-epaper397: records 16 kHz stereo PCM16 from the
// ES8311's ADC line over I2S (MCLK 13, BCLK 14, WS 47, DIN 21, as Waveshare's board_cfg.txt
// has it) and prints, every 100 ms, the level and the frequency estimated from zero crossings.
// In the emulator the ES8311 is a register stub and the samples come from --mic or the page's
// microphone; on the board the codec would need its ADC set up first (Waveshare's es8311 driver).
#include <ESP_I2S.h>

I2SClass i2s;
static int16_t buf[1600 * 2];                // 100 ms of stereo frames

void setup() {
  Serial.begin(115200);
  i2s.setPins(14, 47, -1, 21, 13);           // BCLK, WS, DOUT (unused), DIN, MCLK
  if (!i2s.begin(I2S_MODE_STD, 16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO)) {
    Serial.println("mic: i2s begin failed");
    return;
  }
  Serial.println("mic: recording 16 kHz");
}

void loop() {
  size_t got = i2s.readBytes((char *)buf, sizeof buf) / 4;
  if (got == 0) return;
  double sum = 0;
  int crossings = 0;
  for (size_t i = 0; i < got; i++) {
    int16_t l = buf[2 * i];
    sum += (double)l * l;
    if (i > 0 && (buf[2 * (i - 1)] < 0) != (l < 0)) crossings++;
  }
  double rms = sqrt(sum / got);
  int hz = (int)(crossings * 16000.0 / (2.0 * got) + 0.5);
  Serial.printf("mic: rms %5.0f  ~%d Hz\n", rms, hz);
}
