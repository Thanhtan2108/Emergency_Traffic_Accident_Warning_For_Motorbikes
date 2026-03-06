#include "ReadDataFromSensor.h"
#include <math.h>

ReadData::ReadData()
: mpu(Wire), accX(0), accY(0), accZ(0),
  accelTotal(0), angleX(0), angleY(0), angleZ(0), tempC(0)
{}

void ReadData::begin() {
    Wire.begin();
    mpu.begin();
    // Calibrate gyro offsets (blocking, the library prints messages)
    mpu.calcGyroOffsets(true);
}

void ReadData::update() {
    // update library (reads sensor and fuses)
    mpu.update();

    accX = mpu.getAccX();
    accY = mpu.getAccY();
    accZ = mpu.getAccZ();

    accelTotal = sqrtf(accX*accX + accY*accY + accZ*accZ);

    angleX = mpu.getAngleX();
    angleY = mpu.getAngleY();
    angleZ = mpu.getAngleZ();

    tempC = mpu.getTemp();
}

float ReadData::getAccX() const { return accX; }
float ReadData::getAccY() const { return accY; }
float ReadData::getAccZ() const { return accZ; }
float ReadData::getAccelTotal() const { return accelTotal; }
float ReadData::getAngleX() const { return angleX; }
float ReadData::getAngleY() const { return angleY; }
float ReadData::getTemp() const { return tempC; }
