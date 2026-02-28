#pragma once
#include <Arduino.h>

/*
  Dữ liệu chuyển động đã chuẩn hóa
*/
struct NormalizedMotionData {
  // Gia tốc (m/s^2)
  float ax;
  float ay;
  float az;

  // Vận tốc góc (rad/s)
  float wx;
  float wy;
  float wz;

  // Độ lớn vector
  float accMagnitude;   // |a| (m/s^2)
  float gyroMagnitude;  // |ω| (rad/s)

  unsigned long timestamp;
};
