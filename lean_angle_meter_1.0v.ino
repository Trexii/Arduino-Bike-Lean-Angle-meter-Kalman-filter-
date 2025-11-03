#include <Wire.h>
#include <U8g2lib.h>
#include <math.h>

// OLED Display
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// MPU6050 raw vars
float AccX, AccY, AccZ;
float AngleRoll;
float angleOffset = 0.0;  // Calibration offset

// Display smoothing
float smoothedLean = 0;
float maxRightLean = 0;
float maxLeftLean = 0;

bool showAngle = true;
unsigned long lastFlickerTime = 0;

void setup() {
  Serial.begin(57600);
  Wire.begin();
  Wire.setClock(400000);
  u8g2.begin();

  // --- Clean Startup Sequence ---
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_logisoso24_tr);
  u8g2.drawStr((128 - u8g2.getStrWidth("Royal")) / 2, 28, "Royal");
  u8g2.drawStr((128 - u8g2.getStrWidth("Enfield")) / 2, 56, "Enfield");
  u8g2.sendBuffer();
  delay(1200);

  u8g2.clearBuffer();
  u8g2.drawStr((128 - u8g2.getStrWidth("Hunter")) / 2, 28, "Hunter");
  u8g2.drawStr((128 - u8g2.getStrWidth("350")) / 2, 56, "350");
  u8g2.sendBuffer();
  delay(1200);

  // Wake up MPU6050
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B); 
  Wire.write(0x00); 
  Wire.endTransmission();

  // Configure accel ±8g
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  // Gyro config (optional)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  // --- Auto Angle Calibration ---
  float sum = 0;
  const int samples = 100;
  for (int i = 0; i < samples; i++) {
    readMPU();
    sum += AngleRoll;
    delay(10);
  }
  angleOffset = sum / samples;
}

void loop() {
  readMPU();

  float correctedAngle = AngleRoll - angleOffset;
  smoothedLean = 0.7 * smoothedLean + 0.3 * correctedAngle;

  float clamped = constrain(smoothedLean, -65, 65);
  float theta = 180.0 - ((clamped + 65.0) * 180.0 / 130.0);
  float rad = radians(theta);

  int cx = 64;
  int cy = 50;
  int r = 45;
  int mx = cx + cos(rad) * r;
  int my = cy - sin(rad) * r;

  // Track max leans
  if (clamped > maxRightLean) maxRightLean = clamped;
  if (clamped < maxLeftLean)  maxLeftLean = clamped;

  // Flicker logic
  if (abs(clamped) > 30) {
    if (millis() - lastFlickerTime > 100) {
      showAngle = !showAngle;
      lastFlickerTime = millis();
    }
  } else {
    showAngle = true;
  }

  // Draw Display
  u8g2.clearBuffer();

  // Semi-circle arc and moving dot
  u8g2.drawCircle(cx, cy, r, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
  u8g2.drawDisc(mx, my, 4);

  // Lean angle number
  int angleY = cy - 14;
  if (showAngle) {
    char buf[10];
    sprintf(buf, "%d", (int)abs(clamped));
    u8g2.setFont(u8g2_font_logisoso24_tr);
    int txtWidth = u8g2.getStrWidth(buf);
    u8g2.drawStr(cx - txtWidth / 2, angleY, buf);
  }

  // "Trexii" - always shown
  u8g2.setFont(u8g2_font_6x10_tr);
  const char* name = "Trexii";
  int nameWidth = u8g2.getStrWidth(name);
  u8g2.drawStr(cx - nameWidth / 2, angleY + 14, name);

  // Max lean stats
  char buf[10];
  sprintf(buf, "L Max: %d", (int)abs(maxLeftLean));
  u8g2.drawStr(0, 64, buf);

  sprintf(buf, "%d :R Max", (int)abs(maxRightLean));
  int rMaxWidth = u8g2.getStrWidth(buf);
  u8g2.drawStr(128 - rMaxWidth, 64, buf);

  u8g2.sendBuffer();
  delay(16);
}

void readMPU() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6);

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();

  AccX = (float)ax / 4096.0;
  AccY = (float)ay / 4096.0;
  AccZ = (float)az / 4096.0;

  AngleRoll = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * 180.0 / PI;
}