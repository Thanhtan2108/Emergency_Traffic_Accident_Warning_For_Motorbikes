#include <Arduino.h>
#include <Wire.h>

/* ===================== CONFIG ===================== */
#define MPU6050_ADDR   0x68

#define I2C_SDA_PIN    21
#define I2C_SCL_PIN    22
#define I2C_FREQ       400000   // 400kHz - Fast mode

/* ===================== RAW DATA ===================== */
int16_t acc_raw[3];    // X Y Z
int16_t gyro_raw[3];   // X Y Z
int16_t temp_raw;

/* ===================== SCALE ===================== */
// Full-scale config:
// Accel ±2g   -> 16384 LSB / g
// Gyro  ±250  -> 131 LSB / (deg/s)
const float ACC_LSB_PER_G   = 16384.0f;
const float GYRO_LSB_PER_DPS = 131.0f;

/* ===================== I2C LOW LEVEL ===================== */
void mpuWrite(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission(true);
}

void mpuRead(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, len, true);

  for (uint8_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
}

/* ===================== INIT ===================== */
void mpuInit() {
  delay(100);

  // Reset device
  mpuWrite(0x6B, 0x80);
  delay(100);

  // Wake up, internal clock
  mpuWrite(0x6B, 0x00);
  delay(10);

  // Gyro full-scale ±250 deg/s
  mpuWrite(0x1B, 0x00);

  // Accel full-scale ±2g
  mpuWrite(0x1C, 0x00);

  // Digital Low Pass Filter ~42Hz
  mpuWrite(0x1A, 0x03);
}

/* ===================== READ ALL ===================== */
void mpuReadAll() {
  uint8_t buf[14];
  mpuRead(0x3B, buf, 14);

  acc_raw[0]  = (buf[0]  << 8) | buf[1];
  acc_raw[1]  = (buf[2]  << 8) | buf[3];
  acc_raw[2]  = (buf[4]  << 8) | buf[5];

  temp_raw    = (buf[6]  << 8) | buf[7];

  gyro_raw[0] = (buf[8]  << 8) | buf[9];
  gyro_raw[1] = (buf[10] << 8) | buf[11];
  gyro_raw[2] = (buf[12] << 8) | buf[13];
}

/* ===================== SETUP ===================== */
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ);
  mpuInit();

  Serial.println("MPU6050 initialized - first raw read");
}

/* ===================== LOOP ===================== */
void loop() {
  mpuReadAll();

  // Convert to physical units (chưa calibrate)
  float ax = acc_raw[0] / ACC_LSB_PER_G;
  float ay = acc_raw[1] / ACC_LSB_PER_G;
  float az = acc_raw[2] / ACC_LSB_PER_G;

  float gx = gyro_raw[0] / GYRO_LSB_PER_DPS;
  float gy = gyro_raw[1] / GYRO_LSB_PER_DPS;
  float gz = gyro_raw[2] / GYRO_LSB_PER_DPS;

  Serial.print("ACC [g]   : ");
  Serial.print(ax, 3); Serial.print("  ");
  Serial.print(ay, 3); Serial.print("  ");
  Serial.println(az, 3);

  Serial.print("GYRO [d/s]: ");
  Serial.print(gx, 2); Serial.print("  ");
  Serial.print(gy, 2); Serial.print("  ");
  Serial.println(gz, 2);

  Serial.println("----------------------------------");
  delay(500);
}
