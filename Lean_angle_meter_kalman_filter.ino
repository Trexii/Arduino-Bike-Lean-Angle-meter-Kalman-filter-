#include <Wire.h>
#include <U8g2lib.h>
#include <math.h>

// OLED
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// Sensor values
float AccX, AccY, AccZ, GyroX;
float kalmanAngle = 0.0, kalmanBias = 0.0;
float angleOffset = 0.0, smoothedLean = 0.0;
float maxRightLean = 0, maxLeftLean = 0;

bool showAngle = true;
unsigned long lastFlickerTime = 0;
unsigned long lastTime = 0;

// Kalman vars
float P[2][2] = { {1, 0}, {0, 1} };
const float Q_angle = 0.001;
const float Q_bias  = 0.003;
const float R_measure = 0.03;

void setup() {
  Serial.begin(57600);
  Wire.begin();
  Wire.setClock(400000);  // Fast I2C
  u8g2.begin();

  showSplash("Royal", "Enfield");
  showSplash("Hunter", "350");

  // MPU6050 setup
  writeMPU(0x6B, 0x00); // Wake
  writeMPU(0x1C, 0x10); // ±8g
  writeMPU(0x1B, 0x08); // ±500°/s

  lastTime = micros();

  // Calibrate angle offset
  float sum = 0;
  for (int i = 0; i < 100; i++) {
    readMPU();
    sum += kalmanAngle;
    delay(5);  // faster calibration
  }
  angleOffset = sum / 100.0;
}

void loop() {
  readMPU();

  float angle = kalmanAngle - angleOffset;
  smoothedLean = 0.7 * smoothedLean + 0.3 * angle;
  float clamped = constrain(smoothedLean, -65, 65);

  // Polar to screen
  float theta = 180.0 - ((clamped + 65.0) * 1.3846); // precompute factor
  float rad = theta * DEG_TO_RAD;
  int mx = 64 + cos(rad) * 45;
  int my = 50 - sin(rad) * 45;

  // Track extremes
  if (clamped > maxRightLean) maxRightLean = clamped;
  if (clamped < maxLeftLean)  maxLeftLean  = clamped;

  // Flicker on sharp lean
  if (abs(clamped) > 30) {
    if (millis() - lastFlickerTime > 100) {
      showAngle = !showAngle;
      lastFlickerTime = millis();
    }
  } else showAngle = true;

  // Draw
  u8g2.clearBuffer();
  u8g2.drawCircle(64, 50, 45, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
  u8g2.drawDisc(mx, my, 4);

  char buf[10];
  u8g2.setFont(u8g2_font_logisoso24_tr);
  if (showAngle) {
    int val = (int)abs(clamped);
    itoa(val, buf, 10);
    int w = u8g2.getStrWidth(buf);
    u8g2.drawStr(64 - w / 2, 36, buf);
  }

  u8g2.setFont(u8g2_font_6x10_tr);
  const char* name = "Trexii";
  u8g2.drawStr((128 - u8g2.getStrWidth(name)) / 2, 50, name);

  snprintf(buf, sizeof(buf), "L Max: %d", (int)abs(maxLeftLean));
  u8g2.drawStr(0, 64, buf);

  snprintf(buf, sizeof(buf), "%d :R Max", (int)abs(maxRightLean));
  u8g2.drawStr(128 - u8g2.getStrWidth(buf), 64, buf);

  u8g2.sendBuffer();
  delay(16);  // maintain ~60 FPS
}

void showSplash(const char* line1, const char* line2) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_logisoso24_tr);
  u8g2.drawStr((128 - u8g2.getStrWidth(line1)) / 2, 28, line1);
  u8g2.drawStr((128 - u8g2.getStrWidth(line2)) / 2, 56, line2);
  u8g2.sendBuffer();
  delay(1200);
}

void writeMPU(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(0x68);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void readMPU() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14);

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // skip temp
  int16_t gx = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // skip GyroY
  Wire.read(); Wire.read();  // skip GyroZ

  AccX = ax / 4096.0;
  AccY = ay / 4096.0;
  AccZ = az / 4096.0;
  GyroX = gx / 65.5;

  float accRoll = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * 57.2958;  // to deg

  // Kalman filter
  unsigned long now = micros();
  float dt = (now - lastTime) * 1e-6;
  lastTime = now;

  // Predict
  kalmanAngle += dt * (GyroX - kalmanBias);
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  // Update
  float y = accRoll - kalmanAngle;
  float S = P[0][0] + R_measure;
  float K0 = P[0][0] / S;
  float K1 = P[1][0] / S;

  kalmanAngle += K0 * y;
  kalmanBias  += K1 * y;

  float P00 = P[0][0], P01 = P[0][1];
  P[0][0] -= K0 * P00;
  P[0][1] -= K0 * P01;
  P[1][0] -= K1 * P00;
  P[1][1] -= K1 * P01;
}
