#include <Wire.h>
#include <math.h>

#define SDA_PIN 4        // D2 on NodeMCU/ESP8266
#define SCL_PIN 5        // D1
#define TLE_ADDR 0x35    // your scan shows 0x35

// 8-bit scale from datasheet: 2.08 mT per LSB
static const float MT_PER_LSB_8 = 2.08f;

bool readBlock(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(TLE_ADDR);
  Wire.write(reg);                          // datasheet: first register address is 0
  if (Wire.endTransmission(false) != 0) return false;  // repeated start
  uint8_t got = Wire.requestFrom(TLE_ADDR, n);
  if (got != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nTLE493D raw dump + 8-bit XYZ (no library)");
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);                   // start conservative
  Wire.setClockStretchLimit(150000L);

  // Small settle time after power
  delay(5);
}

void loop() {
  uint8_t r[8];

  // Read twice; discard first to let sensor shadow/refresh settle
  readBlock(0x00, r, 8);
  delayMicroseconds(200);
  if (readBlock(0x00, r, 8)) {
    // Raw bytes: [Xmsb Ymsb Zmsb Tmsb Xlsb Ylsb Zlsb Tlsb]
    Serial.printf("raw: %02X %02X %02X %02X  %02X %02X %02X %02X  ",
                  r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7]);

    // 8-bit MSBs only (simple, robust)
    int8_t x8 = (int8_t)r[0];
    int8_t y8 = (int8_t)r[1];
    int8_t z8 = (int8_t)r[2];

    float Bx = x8 * MT_PER_LSB_8;
    float By = y8 * MT_PER_LSB_8;
    float Bz = z8 * MT_PER_LSB_8;
    float Bmag = sqrtf(Bx*Bx + By*By + Bz*Bz);

    Serial.printf("MSB8: Bx=%6.2f mT  By=%6.2f mT  Bz=%6.2f mT  |B|=%6.2f mT\n",
                  Bx, By, Bz, Bmag);
  } else {
    Serial.println("read fail");
  }

  delay(100);
}




