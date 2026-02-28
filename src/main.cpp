#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_tockn.h>

#include "MPU6050Driver.h"
#include "MotionSensor.h"
#include "MotionNormalizer.h"

MPU6050 mpu(Wire);
MPU6050Driver imuDriver(mpu);
MotionSensor sensor(&imuDriver);
MotionNormalizer normalizer;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  sensor.begin();
  sensor.calibrateGyro(true);

  Serial.println("System ready");
}

void loop() {
  MotionSample raw = sensor.readRaw();
  NormalizedMotionData norm = normalizer.normalize(raw);

  Serial.print("t=");
  Serial.print(norm.timestamp);

  Serial.print(" | |a|=");
  Serial.print(norm.accMagnitude, 2);

  Serial.print(" m/s2 | |w|=");
  Serial.print(norm.gyroMagnitude, 2);

  Serial.println(" rad/s");

  delay(1000);
}
