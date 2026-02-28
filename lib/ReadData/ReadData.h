#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_tockn.h>

struct MPUData {
  float accX, accY, accZ;
  float gyroX, gyroY, gyroZ;
  float angleX, angleY, angleZ;
};

class MPU {
public:
  explicit MPU(TwoWire& wire = Wire);
  void begin();
  void update();
  MPUData getData() const;

private:
  MPU6050 mpu;
  MPUData data;
};

class ReadData {
public:
  explicit ReadData(MPU& mpu);

  void update();
  MPUData getData() const;

  float getTotalAcceleration() const;
  float getTotalGyroscope() const;

private:
  MPU& mpu;
  MPUData currentData;
};
