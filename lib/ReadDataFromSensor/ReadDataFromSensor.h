#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_tockn.h>

class ReadData {
public:
    ReadData();
    void begin();      // init I2C + mpu + calibrate
    void update();     // đọc & tính toán (gọi mpu.update())
    // getters
    float getAccX() const;
    float getAccY() const;
    float getAccZ() const;
    float getAccelTotal() const;
    float getAngleX() const;
    float getAngleY() const;
    float getTemp() const;

private:
    MPU6050 mpu;
    float accX, accY, accZ;
    float accelTotal;
    float angleX, angleY, angleZ;
    float tempC;
};
