#include "ReadData.h"
#include <math.h>

/* MPU */
MPU::MPU(TwoWire& wire) : mpu(wire) {}

void MPU::begin() {
  mpu.begin();
  mpu.calcGyroOffsets(true);
}

void MPU::update() {
  mpu.update();
//   data = {
//     mpu.getAccX(), mpu.getAccY(), mpu.getAccZ(),
//     mpu.getGyroX(), mpu.getGyroY(), mpu.getGyroZ(),
//     mpu.getAngleX(), mpu.getAngleY(), mpu.getAngleZ()
//   };
  data.accX = mpu.getAccX();
  data.accY = mpu.getAccY(); 
  data.accZ = mpu.getAccZ();
  
  data.gyroX = mpu.getGyroX();
  data.gyroY = mpu.getGyroY();
  data.gyroZ = mpu.getGyroZ();
  
  data.angleX = mpu.getAngleX();
  data.angleY = mpu.getAngleY();
  data.angleZ = mpu.getAngleZ();
}

MPUData MPU::getData() const {
  return data;
}

/* ReadData */
ReadData::ReadData(MPU& mpu) : mpu(mpu) {}

void ReadData::update() {
  mpu.update();
  currentData = mpu.getData();
}

MPUData ReadData::getData() const {
  return currentData;
}

float ReadData::getTotalAcceleration() const {
  return sqrt(
    currentData.accX * currentData.accX +
    currentData.accY * currentData.accY +
    currentData.accZ * currentData.accZ
  );
}

float ReadData::getTotalGyroscope() const {
  return sqrt(
    currentData.gyroX * currentData.gyroX +
    currentData.gyroY * currentData.gyroY +
    currentData.gyroZ * currentData.gyroZ
  );
}
