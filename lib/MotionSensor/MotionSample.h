#pragma once
#include <Arduino.h>

/*
  Dữ liệu chuyển động THÔ (raw)
  Không filter, không xử lý
*/
struct MotionSample {
  float ax;   // m/s^2
  float ay;
  float az;

  float gx;   // deg/s
  float gy;
  float gz;

  unsigned long timestamp; // ms (millis)
};
