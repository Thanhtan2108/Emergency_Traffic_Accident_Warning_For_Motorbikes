#include "MPU6050Driver.h"

MPU6050Driver::MPU6050Driver(MPU6050& mpu)
  : _mpu(mpu)
{}

bool MPU6050Driver::begin() {
  _mpu.begin();
  return true;
}

void MPU6050Driver::update() {
  _mpu.update();
}

float MPU6050Driver::getAccX() { return _mpu.getAccX(); }
float MPU6050Driver::getAccY() { return _mpu.getAccY(); }
float MPU6050Driver::getAccZ() { return _mpu.getAccZ(); }

float MPU6050Driver::getGyroX() { return _mpu.getGyroX(); }
float MPU6050Driver::getGyroY() { return _mpu.getGyroY(); }
float MPU6050Driver::getGyroZ() { return _mpu.getGyroZ(); }

void MPU6050Driver::calibrateGyro(bool blocking) {
  _mpu.calcGyroOffsets(blocking);
}
