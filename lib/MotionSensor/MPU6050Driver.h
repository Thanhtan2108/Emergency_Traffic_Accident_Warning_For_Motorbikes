#pragma once
#include <MPU6050_tockn.h>
#include "IMUDriver.h"

/*
  Adapter cho MPU6050_tockn
*/
class MPU6050Driver : public IMUDriver {
public:
  explicit MPU6050Driver(MPU6050& mpu);

  bool begin() override;
  void update() override;

  float getAccX() override;
  float getAccY() override;
  float getAccZ() override;

  float getGyroX() override;
  float getGyroY() override;
  float getGyroZ() override;

  void calibrateGyro(bool blocking) override;

private:
  MPU6050& _mpu;
};
