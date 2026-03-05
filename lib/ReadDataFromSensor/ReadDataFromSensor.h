#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_tockn.h>

class ReadData {
public:

    // Constructor
    ReadData();

    // Khoi dong cam bien
    void begin();

    // Cap nhat du lieu cam bien
    void update();

    // Lay gia toc tung truc
    float getAccX();
    float getAccY();
    float getAccZ();

    // Gia toc tong hop
    float getAccelTotal();

    // Lay goc nghieng
    float getAngleX();
    float getAngleY();

private:

    // gia toc theo truc
    float accX;
    float accY;
    float accZ;

    // gia toc tong hop
    float accelTotal;

    // goc nghieng
    float angleX;
    float angleY;

    // doi tuong MPU6050
    MPU6050 mpu;
};
