#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_tockn.h>

#include "MPU6050Driver.h"
#include "MotionSensor.h"

MPU6050 mpu(Wire);
MPU6050Driver imuDriver(mpu);
MotionSensor motion(&imuDriver);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!motion.begin()) {
    Serial.println("MPU6050 init failed!");
    while (1);
  }

  Serial.println("Calibrating gyro...");
  motion.calibrateGyro(true);
  Serial.println("Ready!");
}

void loop() {
  MotionSample s = motion.readRaw();

  Serial.print("t=");
  Serial.print(s.timestamp);
  Serial.print(" | A(m/s2): ");
  Serial.print(s.ax, 2); Serial.print(", ");
  Serial.print(s.ay, 2); Serial.print(", ");
  Serial.print(s.az, 2);

  Serial.print(" | G(deg/s): ");
  Serial.print(s.gx, 2); Serial.print(", ");
  Serial.print(s.gy, 2); Serial.print(", ");
  Serial.println(s.gz, 2);

  delay(1000); // ~1Hz
}
