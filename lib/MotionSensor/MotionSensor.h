#pragma once
#include "IMUDriver.h"
#include "MotionSample.h"

/*
  MotionSensor
  - Thu thập dữ liệu chuyển động thô
  - Không xử lý thuật toán
*/
class MotionSensor {
public:
  explicit MotionSensor(IMUDriver* driver);

  bool begin();
  void calibrateGyro(bool blocking = true);

  MotionSample readRaw();        // đọc 1 mẫu
  MotionSample last() const;     // lấy mẫu gần nhất

private:
  IMUDriver* _driver;
  MotionSample _lastSample;

  static constexpr float G_TO_MSS = 9.80665f;
};
