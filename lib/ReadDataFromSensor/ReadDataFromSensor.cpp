#include <ReadDataFromSensor.h>

// Constructor
ReadData::ReadData() : mpu(Wire) {}

// Khoi dong cam bien
void ReadData::begin()
{
    Wire.begin();

    mpu.begin();

    // Calibrate gyro
    mpu.calcGyroOffsets(true);
}

// Cap nhat du lieu cam bien
void ReadData::update()
{
    mpu.update();

    // Doc gia toc
    accX = mpu.getAccX();
    accY = mpu.getAccY();
    accZ = mpu.getAccZ();

    // Tinh gia toc tong hop
    accelTotal = sqrt(accX * accX +
                      accY * accY +
                      accZ * accZ);

    // Doc goc nghieng
    angleX = mpu.getAngleX();
    angleY = mpu.getAngleY();
}

// Lay gia toc tung truc
float ReadData::getAccX()
{
    return accX;
}

float ReadData::getAccY()
{
    return accY;
}

float ReadData::getAccZ()
{
    return accZ;
}

// Lay gia toc tong hop
float ReadData::getAccelTotal()
{
    return accelTotal;
}

// Lay goc nghieng
float ReadData::getAngleX()
{
    return angleX;
}

float ReadData::getAngleY()
{
    return angleY;
}
