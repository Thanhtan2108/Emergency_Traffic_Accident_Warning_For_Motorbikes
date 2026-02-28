#pragma once

/*
  Interface cho mọi IMU
  MotionSensor chỉ phụ thuộc abstraction này
*/
class IMUDriver {
public:
  virtual ~IMUDriver() {}

  virtual bool begin() = 0;
  virtual void update() = 0;

  virtual float getAccX() = 0;
  virtual float getAccY() = 0;
  virtual float getAccZ() = 0;

  virtual float getGyroX() = 0;
  virtual float getGyroY() = 0;
  virtual float getGyroZ() = 0;

  virtual void calibrateGyro(bool blocking) {}
};
